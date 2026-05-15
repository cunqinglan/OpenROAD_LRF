#include "Initializer.hh"

#include "lrf/IncreSta.hh"
#include "LocalSta.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "est/EstimateParasitics.h"
#include "sta/EquivCells.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/PortDirection.hh"
#include "sta/Graph.hh"
#include "sta/TimingModel.hh"
#include "sta/TimingRole.hh"
#include "sta/TimingArc.hh"
#include "sta/Scene.hh"
#include "sta/Mode.hh"
#include "sta/Sdc.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Delay.hh"
#include "odb/db.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>

namespace lrf {

using sta::LibertyCell;
using sta::LibertyCellSeq;
using sta::Pin;
using sta::Scene;
using sta::RiseFall;
using sta::MinMax;

Initializer::Initializer(sta::dbSta* sta, IncreSta* incre_sta,
                         rsz::Resizer* resizer, odb::dbBlock* block)
  : incre_sta_(incre_sta),
    resizer_(resizer),
    block_(block)
{
  dbStaState::init(sta);
}

Initializer::~Initializer() = default;

float
Initializer::estimateMaxSlew(sta::LibertyPort* port, float load_cap,
                             const sta::Scene* scene,
                             const sta::MinMax* min_max,
                             sta::Instance* inst)
{
  if (!port) return sta::INF;
  sta::LibertyCell* cell = port->libertyCell();
  sta::Graph* graph = sta_->graph();
  const sta::DcalcAPIndex ap_index = scene->dcalcAnalysisPtIndex(min_max);
  sta::OperatingConditions* op_cond = scene->sdc()->operatingConditions(min_max);
  float max_slew = 0;
  for (sta::TimingArcSet* arc_set : cell->timingArcSets()) {
    if (arc_set->role()->isTimingCheck()) continue;
    for (sta::TimingArc* arc : arc_set->arcs()) {
      if (arc->to() != port) continue;
      sta::GateTimingModel* model =
          dynamic_cast<sta::GateTimingModel*>(arc->model());
      if (!model) continue;
      // Use actual input slew from graph when instance is available
      // (same approach as RepairDesign::checkDriverArcSlew).
      sta::Slew in_slew = 50e-12;  // fallback
      if (inst && graph) {
        sta::Pin* in_pin = network_->findPin(inst, arc->from()->name());
        if (in_pin) {
          sta::Vertex* in_v = graph->pinLoadVertex(in_pin);
          if (in_v) {
            const sta::RiseFall* in_rf = arc->fromEdge()->asRiseFall();
            float s = graph->slew(in_v, in_rf, ap_index);
            if (s > 0) in_slew = s;
          }
        }
      }
      float arc_delay = 0, arc_slew = 0;
      model->gateDelay(op_cond, sta::delayAsFloat(in_slew), load_cap,
                       arc_delay, arc_slew);
      max_slew = std::max(max_slew, arc_slew);
    }
  }
  return max_slew;
}

void
Initializer::run()
{
  auto t0 = std::chrono::steady_clock::now();
  printf("[Initializer] Sharma et al. 3-step initialization\n");
  fflush(stdout);

  resizer_->makeEquivCells();
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();
  LocalSta* local_sta = incre_sta_->localSta();

  // Step 1: Downsize all combinational cells to min-leakage
  downsizeToMinLeakage();

  auto t1 = std::chrono::steady_clock::now();

  // Update parasitics and timing after bulk downsize.
  local_sta->updateGlobalParasiticsAndSync(ep);
  sta_->delaysInvalid();
  sta_->updateTiming(false);

  // Step 2: Fix load violations (reverse topo)
  fixLoadViolations();

  auto t2 = std::chrono::steady_clock::now();

  // Update parasitics and timing after Step 2 upsizes.
  local_sta->updateGlobalParasiticsAndSync(ep);
  sta_->delaysInvalid();
  sta_->updateTiming(false);

  // Step 3: Fix slew violations (forward topo)
  fixSlewViolations();

  auto t3 = std::chrono::steady_clock::now();

  // Update parasitics and timing after Step 3.
  local_sta->updateGlobalParasiticsAndSync(ep);
  sta_->delaysInvalid();
  sta_->updateTiming(false);

  // Step 4: Fix remaining cap violations by buffer insertion
  fixCapByBuffering();

  auto t4 = std::chrono::steady_clock::now();
  printf("[Initializer] Step 1: %.2fs, Step 2: %.2fs, Step 3: %.2fs, "
         "Step 4: %.2fs, Total: %.2fs\n",
         std::chrono::duration<double>(t1 - t0).count(),
         std::chrono::duration<double>(t2 - t1).count(),
         std::chrono::duration<double>(t3 - t2).count(),
         std::chrono::duration<double>(t4 - t3).count(),
         std::chrono::duration<double>(t4 - t0).count());

  // ── Post-init violation summary (same method as get_score) ──
  sta_->ensureGraph();
  sta_->findDelays();
  sta::Graph* graph = sta_->graph();
  sta::dbNetwork* db_network = sta_->getDbNetwork();
  const Scene* corner = sta_->cmdScene();
  const MinMax* max = MinMax::max();
  sta::LibertyLibrary* sum_lib = db_network->defaultLibertyLibrary();
  const sta::DcalcAPIndex sum_ap_index = corner->dcalcAnalysisPtIndex(max);

  int drvr_slew_cnt = 0, load_slew_cnt = 0, cap_cnt = 0;
  float drvr_slew_sum = 0, load_slew_sum = 0, cap_sum = 0;

  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* si = db_network->dbToSta(db_inst);
    LibertyCell* c = db_network->libertyCell(si);
    if (!c || c->hasSequentials()) continue;

    sta::InstancePinIterator* pit = network_->pinIterator(si);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      sta::LibertyPort* lp = network_->libertyPort(p);
      if (!lp) continue;

      // Slew check
      float sl; bool se;
      lp->slewLimit(max, sl, se);
      if (!se && sum_lib) sum_lib->defaultMaxSlew(sl, se);
      if (se) {
        sta::Vertex* v, *bi;
        graph->pinVertices(p, v, bi);
        if (v) {
          for (auto rf : RiseFall::range()) {
            float s = graph->slew(v, rf, sum_ap_index);
            if (s > sl) {
              float excess = (s - sl) * 1e9;
              if (network_->direction(p)->isOutput()) {
                drvr_slew_cnt++; drvr_slew_sum += excess;
              } else {
                load_slew_cnt++; load_slew_sum += excess;
              }
              break;
            }
          }
        }
      }

      // Cap check (output only)
      if (network_->direction(p)->isOutput()) {
        float cl; bool ce;
        lp->capacitanceLimit(max, cl, ce);
        if (!ce && sum_lib) sum_lib->defaultMaxCapacitance(cl, ce);
        if (ce) {
          float lc = sta_->graphDelayCalc()->loadCap(p, corner, max);
          if (lc > cl) {
            cap_cnt++;
            cap_sum += (lc - cl) * 1e15;
          }
        }
      }
    }
    delete pit;
  }

  printf("[Initializer] Post-init violations:\n");
  printf("[Initializer]   Driver slew: %d pins, %.4f ns\n", drvr_slew_cnt, drvr_slew_sum);
  printf("[Initializer]   Load slew:   %d pins, %.4f ns\n", load_slew_cnt, load_slew_sum);
  printf("[Initializer]   Cap:         %d pins, %.4f fF\n", cap_cnt, cap_sum);
  fflush(stdout);
}

// ── Step 1: Downsize to min-leakage ─────────────────────────────
void
Initializer::downsizeToMinLeakage()
{
  sta::dbNetwork* db_network = sta_->getDbNetwork();
  int swap_count = 0;
  int inst_count = 0;

  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* sta_inst = db_network->dbToSta(db_inst);
    LibertyCell* cell = db_network->libertyCell(sta_inst);
    if (!cell || cell->hasSequentials()) continue;

    inst_count++;

    LibertyCellSeq* equiv_cells = sta_->equivCells(cell);
    if (!equiv_cells || equiv_cells->empty()) continue;

    // Find min-leakage cell using resizer's cached leakage.
    LibertyCell* min_cell = cell;
    auto cur_leak = resizer_->cellLeakage(cell);
    float min_leak = cur_leak.value_or(sta::INF);
    for (LibertyCell* ec : *equiv_cells) {
      auto leak = resizer_->cellLeakage(ec);
      if (leak.has_value() && leak.value() < min_leak) {
        min_leak = leak.value();
        min_cell = ec;
      }
    }

    if (min_cell != cell) {
      odb::dbMaster* new_master = db_network->staToDb(min_cell);
      db_inst->swapMaster(new_master);
      swap_count++;
    }
  }

  sta_->updateTiming(false);
  printf("[Initializer] Step 1: Downsized %d / %d gates to min-leakage\n",
         swap_count, inst_count);
  fflush(stdout);
}

// ── Step 2: Fix load violations (reverse topo PO→PI) ────────────
// For each gate, check if actual load cap exceeds:
//   effective_limit = min(maxcap, slew_derived_cap)
// where slew_derived_cap is the max cap this cell can drive
// while keeping output slew ≤ slew_limit.
// If violated, upsize to smallest cell that satisfies both.
void
Initializer::fixLoadViolations()
{
  sta::dbNetwork* db_network = sta_->getDbNetwork();
  const Scene* scene = sta_->cmdScene();
  const MinMax* min_max = MinMax::max();
  sta::LibertyLibrary* default_lib = db_network->defaultLibertyLibrary();
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();
  est::IncrementalParasiticsGuard guard(ep);

  float default_max_slew = sta::INF;
  if (default_lib) {
    bool exists;
    default_lib->defaultMaxSlew(default_max_slew, exists);
    if (!exists) default_max_slew = sta::INF;
  }

  // Use shared IncreSta's sorted instances (no second IncreSta created).
  incre_sta_->resetSortedInstances();
  sta::InstanceSeq& sorted = incre_sta_->getSortedInstances();

  int upsize_count = 0;
  int viol_count = 0;

  // Reverse topo (PO→PI): downstream input caps settled first.
  for (int i = static_cast<int>(sorted.size()) - 1; i >= 0; i--) {
    sta::Instance* inst = const_cast<sta::Instance*>(sorted[i]);
    LibertyCell* cell = network_->libertyCell(inst);
    if (!cell || cell->hasSequentials()) continue;

    // Find output pin
    Pin* out_pin = nullptr;
    sta::InstancePinIterator* pit = network_->pinIterator(inst);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      if (network_->direction(p)->isOutput()) { out_pin = p; break; }
    }
    delete pit;
    if (!out_pin) continue;

    sta::LibertyPort* drvr_port = network_->libertyPort(out_pin);
    if (!drvr_port) continue;

    float load_cap = sta_->graphDelayCalc()->loadCap(out_pin, scene, min_max);

    // Get maxcap limit
    float cap_limit = sta::INF;
    {
      float cl; bool ex;
      drvr_port->capacitanceLimit(min_max, cl, ex);
      if (!ex && default_lib) default_lib->defaultMaxCapacitance(cl, ex);
      if (ex) cap_limit = cl;
    }

    // Get slew limit and convert to cap requirement:
    // max cap this cell can drive while keeping slew ≤ limit.
    float slew_limit = sta::INF;
    {
      float sl; bool ex;
      drvr_port->slewLimit(min_max, sl, ex);
      if (!ex && default_lib) default_lib->defaultMaxSlew(sl, ex);
      if (ex) slew_limit = sl;
    }

    // Check if current cell violates either cap or slew-as-cap
    bool cap_viol = (load_cap > cap_limit);
    bool slew_viol = false;
    if (slew_limit < sta::INF) {
      float est_slew = estimateMaxSlew(drvr_port, load_cap, scene, min_max, inst);
      if (est_slew > slew_limit)
        slew_viol = true;
    }

    if (!cap_viol && !slew_viol) continue;
    viol_count++;

    // Find smallest upsize that satisfies both maxcap AND slew
    LibertyCellSeq* equivs = sta_->equivCells(cell);
    if (!equivs || equivs->empty()) continue;

    std::vector<LibertyCell*> by_area(equivs->begin(), equivs->end());
    std::sort(by_area.begin(), by_area.end(),
              [](LibertyCell* a, LibertyCell* b) { return a->area() < b->area(); });

    const std::string port_name = drvr_port->name();
    LibertyCell* best = nullptr;

    for (LibertyCell* ec : by_area) {
      if (ec->area() <= cell->area() && ec != cell) continue;
      if (ec == cell) continue;
      sta::LibertyPort* ep = ec->findLibertyPort(port_name.c_str());
      if (!ep) continue;

      // Check maxcap
      float cl; bool ex;
      ep->capacitanceLimit(min_max, cl, ex);
      if (!ex && default_lib) default_lib->defaultMaxCapacitance(cl, ex);
      if (ex && load_cap > cl) continue;

      // Check slew-as-cap: can this cell drive load_cap within slew_limit?
      if (slew_limit < sta::INF) {
        float est = estimateMaxSlew(ep, load_cap, scene, min_max, inst);
        if (est > slew_limit) continue;
      }

      best = ec;
      break;
    }

    if (!best) best = by_area.back();
    if (best == cell) continue;

    sta_->replaceCell(inst, best);
    // Incremental parasitic + delay update so subsequent gates see
    // accurate load caps (same pattern as RepairDesign).
    ep->updateParasitics();
    sta::Vertex* drvr_v, *bi_v;
    sta_->graph()->pinVertices(out_pin, drvr_v, bi_v);
    if (drvr_v)
      sta_->findDelays(drvr_v);
    upsize_count++;
  }

  printf("[Initializer] Step 2: Fixed %d load violations, upsized %d gates (PO→PI)\n",
         viol_count, upsize_count);
  fflush(stdout);
}

// ── Step 3: Fix slew violations (forward topo PI→PO) ────────────
// Multi-pass: after step 2, check actual output slew (now with accurate
// input slews from settled upstream). Upsize if needed.
void
Initializer::fixSlewViolations()
{
  sta::dbNetwork* db_network = sta_->getDbNetwork();
  const Scene* scene = sta_->cmdScene();
  const MinMax* min_max = MinMax::max();
  sta::Graph* graph = sta_->graph();
  sta::LibertyLibrary* default_lib = db_network->defaultLibertyLibrary();
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();
  est::IncrementalParasiticsGuard guard(ep);

  float default_max_slew = sta::INF;
  if (default_lib) {
    bool exists;
    default_lib->defaultMaxSlew(default_max_slew, exists);
    if (!exists) default_max_slew = sta::INF;
  }

  // Collect violated driver pins (using actual graph slew after step 2).
  struct ViolatedPin {
    const sta::Pin* drvr_pin;
    float limit;
  };
  std::vector<ViolatedPin> violations;

  sta::VertexIterator viter(graph);
  while (viter.hasNext()) {
    sta::Vertex* vertex = viter.next();
    if (!vertex->isDriver(db_network)) continue;
    const sta::Pin* pin = vertex->pin();
    if (db_network->isTopLevelPort(pin)) continue;
    sta::LibertyPort* port = db_network->libertyPort(pin);
    if (!port) continue;

    float limit = 0.0f;
    bool exists = false;
    port->slewLimit(min_max, limit, exists);
    if (!exists) limit = default_max_slew;

    float worst = 0.0f;
    for (auto rf : RiseFall::range()) {
      float s = graph->slew(vertex, rf, scene->dcalcAnalysisPtIndex(min_max));
      worst = std::max(worst, s);
    }
    if (worst > limit)
      violations.push_back({pin, limit});
  }

  printf("[Initializer] Step 3: Found %zu remaining slew violations (PI→PO)\n",
         violations.size());

  if (violations.empty()) {
    printf("[Initializer] Step 3: No slew violations to fix\n");
    fflush(stdout);
    return;
  }

  // Multi-pass upsize
  int total_upsized = 0;

  for (int pass = 0; pass < 5; pass++) {
    int upsized_this_pass = 0;

    sta_->ensureGraph();
    sta_->findDelays();

    for (auto& v : violations) {
      sta::Vertex* vertex = graph->pinDrvrVertex(v.drvr_pin);
      if (!vertex) continue;

      float worst = 0.0f;
      for (auto rf : RiseFall::range()) {
        float s = graph->slew(vertex, rf, scene->dcalcAnalysisPtIndex(min_max));
        worst = std::max(worst, s);
      }
      if (worst <= v.limit) continue;

      sta::Instance* inst = network_->instance(v.drvr_pin);
      LibertyCell* cur_cell = network_->libertyCell(inst);
      sta::LibertyPort* drvr_port = network_->libertyPort(v.drvr_pin);
      if (!cur_cell || !drvr_port) continue;

      float load_cap = sta_->graphDelayCalc()->loadCap(v.drvr_pin, scene, min_max);

      LibertyCellSeq* equivs = sta_->equivCells(cur_cell);
      if (!equivs) continue;

      std::vector<LibertyCell*> by_area(equivs->begin(), equivs->end());
      std::sort(by_area.begin(), by_area.end(),
                [](LibertyCell* a, LibertyCell* b) { return a->area() < b->area(); });

      LibertyCell* best = nullptr;
      for (LibertyCell* ec : by_area) {
        if (ec == cur_cell) continue;
        if (ec->area() < cur_cell->area()) continue;
        sta::LibertyPort* ep = ec->findLibertyPort(drvr_port->name());
        if (!ep) continue;
        float est = estimateMaxSlew(ep, load_cap, scene, min_max, inst);
        if (est <= v.limit) {
          best = ec;
          break;
        }
      }

      if (!best) {
        best = by_area.back();
        if (best == cur_cell) continue;
      }

      sta_->replaceCell(inst, best);
      // Incremental update so downstream gates see accurate slew.
      ep->updateParasitics();
      sta::Vertex* drvr_v = graph->pinDrvrVertex(v.drvr_pin);
      if (drvr_v)
        sta_->findDelays(drvr_v);
      upsized_this_pass++;
      total_upsized++;
    }

    printf("[Initializer]   Pass %d: upsized %d cells\n", pass, upsized_this_pass);
    fflush(stdout);
    if (upsized_this_pass == 0) break;
  }

  // Final verification with diagnostics for unresolved violations.
  sta_->findDelays();
  int remaining = 0;
  sta::VertexIterator viter2(graph);
  while (viter2.hasNext()) {
    sta::Vertex* vertex = viter2.next();
    if (!vertex->isDriver(db_network)) continue;
    const sta::Pin* pin = vertex->pin();
    if (db_network->isTopLevelPort(pin)) continue;
    sta::LibertyPort* port = db_network->libertyPort(pin);
    if (!port) continue;
    float limit = 0.0f;
    bool exists = false;
    port->slewLimit(min_max, limit, exists);
    if (!exists) limit = default_max_slew;
    float worst = 0.0f;
    for (auto rf : RiseFall::range()) {
      float s = graph->slew(vertex, rf, scene->dcalcAnalysisPtIndex(min_max));
      worst = std::max(worst, s);
    }
    if (worst > limit) {
      remaining++;
      // Diagnostic: why can't we fix this?
      sta::Instance* inst = network_->instance(pin);
      LibertyCell* cur = network_->libertyCell(inst);
      float load_cap = sta_->graphDelayCalc()->loadCap(pin, scene, min_max);
      // Find largest equiv cell and its estimated slew
      LibertyCellSeq* equivs = cur ? sta_->equivCells(cur) : nullptr;
      std::string largest_name = cur ? cur->name() : std::string("?");
      float largest_slew = worst;
      if (equivs && !equivs->empty()) {
        LibertyCell* largest = cur;
        for (LibertyCell* ec : *equivs) {
          if (ec->area() > largest->area()) largest = ec;
        }
        largest_name = largest->name();
        sta::LibertyPort* lp = largest->findLibertyPort(port->name().c_str());
        if (lp)
          largest_slew = estimateMaxSlew(lp, load_cap, scene, min_max, inst);
      }
      printf("[Initializer] Step 3: UNRESOLVED pin %s, cell %s, "
             "slew=%.3fps limit=%.3fps load_cap=%.2ffF, "
             "largest_equiv=%s est_slew=%.3fps\n",
             network_->pathName(pin).c_str(), cur ? cur->name().c_str() : "?",
             worst * 1e12, limit * 1e12, load_cap * 1e15,
             largest_name.c_str(), largest_slew * 1e12);
    }
  }

  printf("[Initializer] Step 3: upsized %d cells, %d remaining violations\n",
         total_upsized, remaining);
  fflush(stdout);
}

// ── Step 4: Fix remaining cap violations by buffer insertion ────
// After Step 2/3, some nets may still violate max_capacitance because
// the driver cell has no larger equivalent (single-size family, e.g.
// OA21x2 in ASAP7). For these nets, insert buffers to split the fanout.
void
Initializer::fixCapByBuffering()
{
  sta::dbNetwork* db_network = sta_->getDbNetwork();
  const Scene* scene = sta_->cmdScene();
  const MinMax* min_max = MinMax::max();
  sta::LibertyLibrary* default_lib = db_network->defaultLibertyLibrary();
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();

  // Collect all output pins with cap violations.
  struct CapViol {
    sta::Pin* drvr_pin;
    sta::Instance* inst;
    float load_cap;
    float max_cap;
  };
  std::vector<CapViol> violations;

  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* inst = db_network->dbToSta(db_inst);
    LibertyCell* cell = network_->libertyCell(inst);
    if (!cell) continue;

    sta::InstancePinIterator* pit = network_->pinIterator(inst);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      if (!network_->direction(p)->isOutput()) continue;
      sta::LibertyPort* port = network_->libertyPort(p);
      if (!port) continue;

      float cap_limit;
      bool exists;
      port->capacitanceLimit(min_max, cap_limit, exists);
      if (!exists && default_lib)
        default_lib->defaultMaxCapacitance(cap_limit, exists);
      if (!exists) continue;

      float load_cap = sta_->graphDelayCalc()->loadCap(p, scene, min_max);
      if (load_cap > cap_limit) {
        violations.push_back({p, inst, load_cap, cap_limit});
      }
    }
    delete pit;
  }

  if (violations.empty()) {
    printf("[Initializer] Step 4: No cap violations to fix by buffering\n");
    fflush(stdout);
    return;
  }

  // Sort by violation magnitude (largest first)
  std::sort(violations.begin(), violations.end(),
            [](const CapViol& a, const CapViol& b) {
              return (a.load_cap - a.max_cap) > (b.load_cap - b.max_cap);
            });

  printf("[Initializer] Step 4: Found %zu cap violations for buffering\n",
         violations.size());

  int total_buffers = 0;

  for (auto& viol : violations) {
    sta::Pin* drvr_pin = viol.drvr_pin;
    float max_cap = viol.max_cap;

    // Collect all load pins on this net (input pins driven by drvr_pin).
    sta::Net* net = network_->net(drvr_pin);
    if (!net) continue;

    struct LoadInfo {
      const sta::Pin* pin;
      float cap;
    };
    std::vector<LoadInfo> loads;

    sta::NetPinIterator* npi = network_->pinIterator(net);
    while (npi->hasNext()) {
      const sta::Pin* p = npi->next();
      if (p == drvr_pin) continue;
      if (!network_->direction(p)->isInput()) continue;
      sta::LibertyPort* lp = network_->libertyPort(p);
      float c = lp ? lp->capacitance() : 0.0f;
      loads.push_back({p, c});
    }
    delete npi;

    if (loads.size() <= 1) continue;

    // Sort loads by cap descending — split off largest loads first.
    std::sort(loads.begin(), loads.end(),
              [](const LoadInfo& a, const LoadInfo& b) {
                return a.cap > b.cap;
              });

    // Recompute actual load cap from STA (may differ slightly from sum of
    // pin caps due to wire cap).
    float remaining_cap = viol.load_cap;
    int inserted = 0;

    while (remaining_cap > max_cap && loads.size() > 1) {
      sta::PinSeq buf_loads;
      float buf_group_cap = 0.0f;
      float target_split = remaining_cap - max_cap;

      // Greedily move loads into the buffer group until we've moved enough
      // capacitance to bring the driver under limit.
      auto it = loads.begin();
      while (it != loads.end() && buf_group_cap < target_split) {
        buf_loads.push_back(const_cast<sta::Pin*>(it->pin));
        buf_group_cap += it->cap;
        it = loads.erase(it);
      }

      if (buf_loads.empty()) break;

      // Pick the smallest buffer cell whose max_capacitance >= group load.
      // We search all liberty libraries for buffer cells.
      sta::LibertyCell* buf_cell = nullptr;
      float best_area = sta::INF;
      sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
      while (lib_iter->hasNext()) {
        sta::LibertyLibrary* lib = lib_iter->next();
        sta::LibertyCellIterator cell_iter(lib);
        while (cell_iter.hasNext()) {
          sta::LibertyCell* cell = cell_iter.next();
          if (!cell->isBuffer()) continue;
          sta::LibertyPort *in, *out;
          cell->bufferPorts(in, out);
          if (!out) continue;
          float cl; bool ce;
          out->capacitanceLimit(min_max, cl, ce);
          if (!ce && default_lib)
            default_lib->defaultMaxCapacitance(cl, ce);
          if (!ce || cl < buf_group_cap) continue;
          if (cell->area() < best_area) {
            best_area = cell->area();
            buf_cell = cell;
          }
        }
      }
      delete lib_iter;
      if (!buf_cell) {
        // Find the largest buffer max_cap in library for diagnostic.
        float max_buf_cap = 0;
        std::string max_buf_name = "none";
        sta::LibertyLibraryIterator* dlib = network_->libertyLibraryIterator();
        while (dlib->hasNext()) {
          sta::LibertyLibrary* lib = dlib->next();
          sta::LibertyCellIterator ci(lib);
          while (ci.hasNext()) {
            sta::LibertyCell* c = ci.next();
            if (!c->isBuffer()) continue;
            sta::LibertyPort *bi, *bo;
            c->bufferPorts(bi, bo);
            if (!bo) continue;
            float cl; bool ce;
            bo->capacitanceLimit(min_max, cl, ce);
            if (ce && cl > max_buf_cap) {
              max_buf_cap = cl;
              max_buf_name = c->name();
            }
          }
        }
        delete dlib;
        printf("[Initializer] Step 4: UNRESOLVED pin %s, "
               "need buffer for group_cap=%.2f fF, "
               "but largest buffer %s has max_cap=%.2f fF (%zu loads in group)\n",
               network_->pathName(drvr_pin).c_str(),
               buf_group_cap * 1e15, max_buf_name.c_str(), max_buf_cap * 1e15,
               buf_loads.size());
        break;
      }

      // Place the buffer at the centroid of its loads.
      int sum_x = 0, sum_y = 0;
      for (const sta::Pin* lp : buf_loads) {
        odb::Point loc = db_network->location(lp);
        sum_x += loc.x();
        sum_y += loc.y();
      }
      odb::Point buf_loc(sum_x / static_cast<int>(buf_loads.size()),
                         sum_y / static_cast<int>(buf_loads.size()));

      sta::Instance* buf_inst = resizer_->insertBufferBeforeLoads(
          net, &buf_loads, buf_cell, &buf_loc, "cap_repair");
      if (!buf_inst) {
        printf("[Initializer] Step 4: insertBufferBeforeLoads failed\n");
        break;
      }
      inserted++;

      // Update remaining cap: remove split loads, add buffer input cap.
      sta::LibertyPort *buf_in, *buf_out;
      buf_cell->bufferPorts(buf_in, buf_out);
      float buf_in_cap = buf_in ? buf_in->capacitance() : 0.0f;
      remaining_cap = remaining_cap - buf_group_cap + buf_in_cap;
    }

    if (inserted > 0) {
      printf("[Initializer] Step 4: pin %s — inserted %d buffer(s), "
             "remaining_cap=%.2f fF (limit=%.2f fF)\n",
             network_->pathName(drvr_pin), inserted,
             remaining_cap * 1e15, max_cap * 1e15);
      total_buffers += inserted;
    }
    if (remaining_cap > max_cap) {
      printf("[Initializer] Step 4: UNRESOLVED pin %s, "
             "remaining_cap=%.2f fF > limit=%.2f fF after %d buffer(s), "
             "%zu loads remaining\n",
             network_->pathName(drvr_pin),
             remaining_cap * 1e15, max_cap * 1e15,
             inserted, loads.size());
    }
  }

  if (total_buffers > 0) {
    // Update parasitics after buffer insertion.
    LocalSta* local_sta = incre_sta_->localSta();
    local_sta->updateGlobalParasiticsAndSync(ep);
    sta_->delaysInvalid();
    sta_->updateTiming(false);
  }

  printf("[Initializer] Step 4: Inserted %d buffer(s) for %zu cap violations\n",
         total_buffers, violations.size());
  fflush(stdout);
}

} // namespace lrf

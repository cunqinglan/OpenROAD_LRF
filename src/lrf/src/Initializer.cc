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
#include "sta/DcalcAnalysisPt.hh"
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
using sta::Corner;
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
                             const sta::DcalcAnalysisPt* dcalc_ap)
{
  if (!port) return sta::INF;
  sta::LibertyCell* cell = port->libertyCell();
  float max_slew = 0;
  for (sta::TimingArcSet* arc_set : cell->timingArcSets()) {
    if (arc_set->role()->isTimingCheck()) continue;
    for (sta::TimingArc* arc : arc_set->arcs()) {
      if (arc->to() != port) continue;
      sta::GateTimingModel* model =
          dynamic_cast<sta::GateTimingModel*>(arc->model());
      if (!model) continue;
      sta::Slew in_slew = 50e-12;  // fixed 50ps
      sta::ArcDelay arc_delay;
      sta::Slew arc_slew;
      model->gateDelay(dcalc_ap->operatingConditions(),
                       in_slew, load_cap, false, arc_delay, arc_slew);
      max_slew = std::max(max_slew, sta::delayAsFloat(arc_slew));
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
  sta_->updateTiming(true);

  // Step 2: Fix load violations (reverse topo)
  fixLoadViolations();

  auto t2 = std::chrono::steady_clock::now();

  // Update parasitics and timing after Step 2 upsizes.
  local_sta->updateGlobalParasiticsAndSync(ep);
  sta_->delaysInvalid();
  sta_->updateTiming(true);

  // Step 3: Fix slew violations (forward topo)
  fixSlewViolations();

  auto t3 = std::chrono::steady_clock::now();
  printf("[Initializer] Step 1: %.2fs, Step 2: %.2fs, Step 3: %.2fs, Total: %.2fs\n",
         std::chrono::duration<double>(t1 - t0).count(),
         std::chrono::duration<double>(t2 - t1).count(),
         std::chrono::duration<double>(t3 - t2).count(),
         std::chrono::duration<double>(t3 - t0).count());
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

  sta_->updateTiming(true);
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
  const Corner* corner = sta_->cmdCorner();
  const MinMax* max = MinMax::max();
  const sta::DcalcAnalysisPt* dcalc_ap = corner->findDcalcAnalysisPt(max);
  sta::LibertyLibrary* default_lib = db_network->defaultLibertyLibrary();

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

    float load_cap = sta_->graphDelayCalc()->loadCap(out_pin, dcalc_ap);

    // Get maxcap limit
    float cap_limit = sta::INF;
    {
      float cl; bool ex;
      drvr_port->capacitanceLimit(max, cl, ex);
      if (!ex && default_lib) default_lib->defaultMaxCapacitance(cl, ex);
      if (ex) cap_limit = cl;
    }

    // Get slew limit and convert to cap requirement:
    // max cap this cell can drive while keeping slew ≤ limit.
    float slew_limit = sta::INF;
    {
      float sl; bool ex;
      drvr_port->slewLimit(max, sl, ex);
      if (!ex && default_lib) default_lib->defaultMaxSlew(sl, ex);
      if (ex) slew_limit = sl;
    }

    // Check if current cell violates either cap or slew-as-cap
    bool cap_viol = (load_cap > cap_limit);
    bool slew_viol = false;
    if (slew_limit < sta::INF) {
      float est_slew = estimateMaxSlew(drvr_port, load_cap, dcalc_ap);
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

    const char* port_name = drvr_port->name();
    LibertyCell* best = nullptr;

    for (LibertyCell* ec : by_area) {
      if (ec->area() <= cell->area() && ec != cell) continue;
      if (ec == cell) continue;
      sta::LibertyPort* ep = ec->findLibertyPort(port_name);
      if (!ep) continue;

      // Check maxcap
      float cl; bool ex;
      ep->capacitanceLimit(max, cl, ex);
      if (!ex && default_lib) default_lib->defaultMaxCapacitance(cl, ex);
      if (ex && load_cap > cl) continue;

      // Check slew-as-cap: can this cell drive load_cap within slew_limit?
      if (slew_limit < sta::INF) {
        float est = estimateMaxSlew(ep, load_cap, dcalc_ap);
        if (est > slew_limit) continue;
      }

      best = ec;
      break;
    }

    if (!best) best = by_area.back();
    if (best == cell) continue;

    sta_->replaceCell(inst, best);
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
  const Corner* corner = sta_->cmdCorner();
  const MinMax* max = MinMax::max();
  const sta::DcalcAnalysisPt* dcalc_ap = corner->findDcalcAnalysisPt(max);
  sta::Graph* graph = sta_->graph();
  sta::LibertyLibrary* default_lib = db_network->defaultLibertyLibrary();

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
    port->slewLimit(max, limit, exists);
    if (!exists) limit = default_max_slew;

    float worst = 0.0f;
    for (auto rf : RiseFall::range()) {
      float s = graph->slew(vertex, rf, dcalc_ap->index());
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
        float s = graph->slew(vertex, rf, dcalc_ap->index());
        worst = std::max(worst, s);
      }
      if (worst <= v.limit) continue;

      sta::Instance* inst = network_->instance(v.drvr_pin);
      LibertyCell* cur_cell = network_->libertyCell(inst);
      sta::LibertyPort* drvr_port = network_->libertyPort(v.drvr_pin);
      if (!cur_cell || !drvr_port) continue;

      float load_cap = sta_->graphDelayCalc()->loadCap(v.drvr_pin, dcalc_ap);

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
        float est = estimateMaxSlew(ep, load_cap, dcalc_ap);
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
      upsized_this_pass++;
      total_upsized++;
    }

    printf("[Initializer]   Pass %d: upsized %d cells\n", pass, upsized_this_pass);
    fflush(stdout);
    if (upsized_this_pass == 0) break;
    sta_->delaysInvalid();
  }

  // Final verification
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
    port->slewLimit(max, limit, exists);
    if (!exists) limit = default_max_slew;
    float worst = 0.0f;
    for (auto rf : RiseFall::range()) {
      float s = graph->slew(vertex, rf, dcalc_ap->index());
      worst = std::max(worst, s);
    }
    if (worst > limit) remaining++;
  }

  printf("[Initializer] Step 3: upsized %d cells, %d remaining violations\n",
         total_upsized, remaining);
  fflush(stdout);
}

} // namespace lrf

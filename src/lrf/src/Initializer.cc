#include "Initializer.hh"

#include "LocalSta.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
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
#include <unordered_map>
#include <vector>

namespace lrf {

using sta::LibertyCell;
using sta::LibertyCellSeq;
using sta::Pin;
using sta::Corner;
using sta::RiseFall;
using sta::MinMax;

Initializer::Initializer(sta::dbSta* sta, rsz::Resizer* resizer,
                         odb::dbBlock* block)
  : resizer_(resizer),
    block_(block),
    local_sta_(nullptr)
{
  dbStaState::init(sta);
  local_sta_ = new LocalSta(sta);
}

Initializer::~Initializer()
{
  delete local_sta_;
}

// ── Helper: estimate max output slew for a candidate port driving load_cap ──
static float
estimateMaxSlew(sta::LibertyPort* port, float load_cap,
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

  printf("[Initializer] Fixing ERC violations (multi-pass upsize)\n");
  fflush(stdout);

  sta::dbNetwork* db_network = sta_->getDbNetwork();
  const Corner* corner = sta_->cmdCorner();
  const MinMax* max = MinMax::max();
  const sta::DcalcAnalysisPt* dcalc_ap = corner->findDcalcAnalysisPt(max);
  sta::Graph* graph = sta_->graph();
  sta::LibertyLibrary* default_lib = db_network->defaultLibertyLibrary();

  // Get default max slew from liberty
  float default_max_slew = sta::INF;
  if (default_lib) {
    bool exists = false;
    default_lib->defaultMaxSlew(default_max_slew, exists);
    if (!exists) default_max_slew = sta::INF;
  }
  printf("[Initializer]   Default max slew limit: %.3f ps\n",
         default_max_slew * 1e12);

  // Ensure timing is up to date
  sta_->ensureGraph();
  sta_->searchPreamble();
  sta_->ensureClkArrivals();
  sta_->findDelays();
  resizer_->makeEquivCells();

  // ── Step 1: Collect all violated driver pins (liberty limits) ──
  // Check both driver output pins AND load input pins.
  // For load pin violations, trace back to the driver of that net.
  struct ViolatedPin {
    const sta::Pin* drvr_pin;
    float worst_slew;
    float limit;
  };
  // Use a set to avoid duplicate driver entries.
  std::unordered_map<const sta::Pin*, ViolatedPin> violation_map;

  sta::VertexIterator viter(graph);
  while (viter.hasNext()) {
    sta::Vertex* vertex = viter.next();
    const sta::Pin* pin = vertex->pin();
    if (db_network->isTopLevelPort(pin))
      continue;
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
    if (worst <= limit) continue;

    // Find the driver pin of this net.
    const sta::Pin* drvr_pin = pin;
    if (!vertex->isDriver(db_network)) {
      // This is a load pin — trace back to the driver.
      sta::Net* net = network_->net(pin);
      if (!net) continue;
      drvr_pin = nullptr;
      sta::NetConnectedPinIterator* nit = network_->connectedPinIterator(net);
      while (nit->hasNext()) {
        const sta::Pin* npin = nit->next();
        if (network_->isDriver(npin) && !db_network->isTopLevelPort(npin)) {
          drvr_pin = npin;
          break;
        }
      }
      delete nit;
      if (!drvr_pin) continue;
    }

    // Use the tightest limit among all pins on this net.
    auto it = violation_map.find(drvr_pin);
    if (it == violation_map.end()) {
      violation_map[drvr_pin] = {drvr_pin, worst, limit};
    } else {
      // Keep worst slew and tightest limit.
      if (worst > it->second.worst_slew)
        it->second.worst_slew = worst;
      if (limit < it->second.limit)
        it->second.limit = limit;
    }
  }

  std::vector<ViolatedPin> violations;
  violations.reserve(violation_map.size());
  for (auto& [pin, vp] : violation_map)
    violations.push_back(vp);

  // Also fix cap violations
  int cap_fix_count = 0;
  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* sta_inst = db_network->dbToSta(db_inst);
    LibertyCell* cell = db_network->libertyCell(sta_inst);
    if (!cell || cell->hasSequentials()) continue;

    sta::InstancePinIterator* it = network_->pinIterator(sta_inst);
    while (it->hasNext()) {
      Pin* pin = it->next();
      if (!network_->direction(pin)->isOutput()) continue;
      sta::LibertyPort* lib_port = network_->libertyPort(pin);
      if (!lib_port) continue;
      float cap_limit;
      bool cap_exists;
      lib_port->capacitanceLimit(max, cap_limit, cap_exists);
      if (!cap_exists && default_lib)
        default_lib->defaultMaxCapacitance(cap_limit, cap_exists);
      if (!cap_exists) continue;
      float load_cap = sta_->graphDelayCalc()->loadCap(pin, dcalc_ap);
      if (load_cap > cap_limit) {
        LibertyCellSeq* equivs = sta_->equivCells(cell);
        if (equivs && !equivs->empty()) {
          LibertyCell* largest = equivs->back();
          if (largest != cell) {
            sta_->replaceCell(sta_inst, largest);
            cap_fix_count++;
          }
        }
      }
    }
    delete it;
  }

  printf("[Initializer]   Found %zu slew violations, %d cap violations\n",
         violations.size(), cap_fix_count);

  if (violations.empty() && cap_fix_count == 0) {
    printf("[Initializer]   No ERC violations to repair.\n");
    fflush(stdout);
    return;
  }

  // Sort by severity (worst first)
  std::sort(violations.begin(), violations.end(),
            [](const ViolatedPin& a, const ViolatedPin& b) {
              return (a.worst_slew - a.limit) > (b.worst_slew - b.limit);
            });

  for (size_t i = 0; i < std::min(violations.size(), size_t(10)); i++) {
    auto& v = violations[i];
    printf("[Initializer]   [%zu] %s: slew=%.1fps limit=%.1fps excess=%.1fps\n",
           i, db_network->pathName(v.drvr_pin),
           v.worst_slew * 1e12, v.limit * 1e12,
           (v.worst_slew - v.limit) * 1e12);
  }

  // ── Step 2: Multi-pass upsize ──
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

      std::vector<LibertyCell*> sorted_equivs(equivs->begin(), equivs->end());
      std::sort(sorted_equivs.begin(), sorted_equivs.end(),
                [](LibertyCell* a, LibertyCell* b) {
                  return a->area() < b->area();
                });

      LibertyCell* best = nullptr;
      for (LibertyCell* ec : sorted_equivs) {
        if (ec == cur_cell) continue;
        if (ec->area() < cur_cell->area()) continue;
        sta::LibertyPort* ep = ec->findLibertyPort(drvr_port->name());
        if (!ep) continue;
        float est_slew = estimateMaxSlew(ep, load_cap, dcalc_ap);
        if (est_slew <= v.limit) {
          best = ec;
          break;
        }
      }

      if (!best) {
        best = sorted_equivs.back();
        if (best == cur_cell) continue;
      }

      sta_->replaceCell(inst, best);
      upsized_this_pass++;
      total_upsized++;
    }

    printf("[Initializer]   Pass %d: upsized %d cells\n",
           pass, upsized_this_pass);
    fflush(stdout);
    if (upsized_this_pass == 0) break;

    sta_->delaysInvalid();
  }

  // ── Step 3: Final verification (all pins, not just drivers) ──
  sta_->findDelays();
  int remaining_drvr = 0, remaining_load = 0;
  sta::VertexIterator viter2(graph);
  while (viter2.hasNext()) {
    sta::Vertex* vertex = viter2.next();
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
    if (worst > limit) {
      bool is_drvr = vertex->isDriver(db_network);
      if (is_drvr) remaining_drvr++;
      else remaining_load++;
    }
  }

  auto t1 = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(t1 - t0).count();
  printf("[Initializer]   Total: upsized %d cells, remaining: %d drvr + %d load violations (%.2f s)\n",
         total_upsized, remaining_drvr, remaining_load, elapsed);
  fflush(stdout);
}

} // namespace lrf

#include "sta/Sdc.hh"
#include "ParallelInitializer.hh"

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
#include "sta/GraphDelayCalc.hh"
#include "sta/Delay.hh"
#include "sta/Bfs.hh"
#include "sta/Search.hh"
#include "sta/SearchPred.hh"
#include "sta/VertexVisitor.hh"
#include "odb/db.h"
#include <tcl.h>

#include <algorithm>
#include <atomic>
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
using sta::Vertex;
using sta::Level;

// ═══════════════════════════════════════════════════════════════════
// FixLoadVisitor — Step 2 (PO→PI) via BfsBkwdIterator
//
// visit():          parallel within level — evaluate + replaceCell (mutex)
// levelFinished():  serial between levels — updateParasitics + findDelays
// ═══════════════════════════════════════════════════════════════════
class FixLoadVisitor : public sta::VertexVisitor
{
public:
  FixLoadVisitor(ParallelInitializer* init,
                 sta::dbSta* sta,
                 sta::dbNetwork* network,
                 sta::Graph* graph,
                 sta::GraphDelayCalc* graph_delay_calc,
                 est::EstimateParasitics* ep,
                 sta::DcalcAPIndex dcalc_ap,
                 sta::LibertyLibrary* default_lib,
                 std::mutex& modify_mutex)
    : init_(init), sta_(sta), network_(network), graph_(graph),
      graph_delay_calc_(graph_delay_calc),
      ep_(ep), dcalc_ap_(dcalc_ap),
      default_lib_(default_lib), modify_mutex_(modify_mutex) {}

  VertexVisitor* copy() const override
  {
    return new FixLoadVisitor(init_, sta_, network_, graph_,
                              graph_delay_calc_, ep_, dcalc_ap_,
                              default_lib_, modify_mutex_);
  }

  void visit(Vertex* vertex) override
  {
    const Pin* rep_pin = vertex->pin();
    if (network_->isTopLevelPort(rep_pin)) return;
    sta::Instance* inst = network_->instance(rep_pin);
    LibertyCell* cell = network_->libertyCell(inst);
    if (!cell || cell->hasSequentials()) return;

    // Collect all output pins of this instance with their load_cap, cap_limit,
    // slew_limit. Aggregate violation across all outputs.
    struct OutInfo {
      sta::LibertyPort* port;
      float load_cap;
      float cap_limit;   // sta::INF if none
      float slew_limit;  // sta::INF if none
    };
    std::vector<OutInfo> outs;
    bool any_cap_viol = false;
    bool any_slew_viol = false;

    sta::InstancePinIterator* pit = network_->pinIterator(inst);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      if (!network_->direction(p)->isOutput()) continue;
      sta::LibertyPort* lp = network_->libertyPort(p);
      if (!lp) continue;

      OutInfo oi;
      oi.port = lp;
      oi.load_cap = graph_delay_calc_->loadCap(p, sta_->cmdScene(), sta::MinMax::max());

      oi.cap_limit = sta::INF;
      {
        float cl; bool ex;
        lp->capacitanceLimit(MinMax::max(), cl, ex);
        if (!ex && default_lib_) default_lib_->defaultMaxCapacitance(cl, ex);
        if (ex) oi.cap_limit = cl * (1.0f - init_->cap_margin_ / 100.0f);
      }

      oi.slew_limit = sta::INF;
      {
        float sl; bool ex;
        lp->slewLimit(MinMax::max(), sl, ex);
        if (!ex && default_lib_) default_lib_->defaultMaxSlew(sl, ex);
        if (ex) oi.slew_limit = sl * (1.0f - init_->slew_margin_ / 100.0f);
      }

      if (oi.load_cap > oi.cap_limit) any_cap_viol = true;
      if (oi.slew_limit < sta::INF) {
        float est = init_->estimateMaxSlew(lp, oi.load_cap, dcalc_ap_, inst);
        if (est > oi.slew_limit) any_slew_viol = true;
      }
      outs.push_back(oi);
    }
    delete pit;

    if (outs.empty()) return;
    if (!any_cap_viol && !any_slew_viol) return;
    viol_count_++;

    LibertyCellSeq* equivs = sta_->equivCells(cell);
    if (!equivs || equivs->empty()) {
      unfixed_no_equiv_++;
      return;
    }

    // Evaluate each candidate by summing violations across all output ports.
    // Candidates missing any output port are skipped.
    using SizeCandidate = std::pair<float, LibertyCell*>;
    std::vector<SizeCandidate> sizes;

    for (LibertyCell* ec : *equivs) {
      if (ec == cell) continue;
      if (ec->area() <= cell->area()) continue;
      float violation = 0.0f;
      bool missing_port = false;
      for (const OutInfo& oi : outs) {
        sta::LibertyPort* ep = ec->findLibertyPort(oi.port->name());
        if (!ep) { missing_port = true; break; }
        float cl; bool ex;
        ep->capacitanceLimit(MinMax::max(), cl, ex);
        if (!ex && default_lib_) default_lib_->defaultMaxCapacitance(cl, ex);
        if (ex && oi.load_cap > cl) violation += (oi.load_cap - cl);
        if (oi.slew_limit < sta::INF) {
          float est = init_->estimateMaxSlew(ep, oi.load_cap, dcalc_ap_, inst);
          if (est > oi.slew_limit) violation += (est - oi.slew_limit);
        }
      }
      if (missing_port) continue;
      sizes.emplace_back(violation, ec);
    }

    if (sizes.empty()) {
      if (any_cap_viol) unfixed_cap_exceeded_++;
      else unfixed_slew_exceeded_++;
      return;
    }

    std::sort(sizes.begin(), sizes.end(),
              [](const SizeCandidate& a, const SizeCandidate& b) {
                if (a.first == 0 && b.first == 0)
                  return a.second->area() < b.second->area();
                return a.first < b.first;
              });

    LibertyCell* best = sizes.front().second;
    if (best == cell) {
      if (any_cap_viol) unfixed_cap_exceeded_++;
      else unfixed_slew_exceeded_++;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(modify_mutex_);
      sta_->replaceCell(inst, best);
    }
    upsize_count_++;
    Level lv = vertex->level();
    Level cur = current_level_.load();
    while (lv > cur && !current_level_.compare_exchange_weak(cur, lv));
  }

  void levelFinished() override
  {
    // Called serially after all threads finish this level.
    Level lv = current_level_.load();
    if (upsize_count_ > prev_upsize_count_) {
      ep_->updateParasitics();
      sta_->findDelays(lv);
    }
    prev_upsize_count_ = upsize_count_;
    current_level_ = 0;
  }

  int violCount() const { return viol_count_; }
  int upsizeCount() const { return upsize_count_; }
  int unfixedNoEquiv() const { return unfixed_no_equiv_; }
  int unfixedCapExceeded() const { return unfixed_cap_exceeded_; }
  int unfixedSlewExceeded() const { return unfixed_slew_exceeded_; }

private:
  ParallelInitializer* init_;
  sta::dbSta* sta_;
  sta::dbNetwork* network_;
  sta::Graph* graph_;
  sta::GraphDelayCalc* graph_delay_calc_;
  est::EstimateParasitics* ep_;
  sta::DcalcAPIndex dcalc_ap_;
  sta::LibertyLibrary* default_lib_;
  std::mutex& modify_mutex_;
  int viol_count_ = 0;
  int upsize_count_ = 0;
  int prev_upsize_count_ = 0;
  std::atomic<Level> current_level_{0};
  // Unfixed violation reasons (thread-local, aggregated after visitParallel).
  int unfixed_no_equiv_ = 0;
  int unfixed_cap_exceeded_ = 0;
  int unfixed_slew_exceeded_ = 0;
};

// ═══════════════════════════════════════════════════════════════════
// FixSlewVisitor — Step 3 (PI→PO) via BfsFwdIterator
// ═══════════════════════════════════════════════════════════════════
class FixSlewVisitor : public sta::VertexVisitor
{
public:
  FixSlewVisitor(ParallelInitializer* init,
                 sta::dbSta* sta,
                 sta::dbNetwork* network,
                 sta::Graph* graph,
                 sta::GraphDelayCalc* graph_delay_calc,
                 est::EstimateParasitics* ep,
                 sta::DcalcAPIndex dcalc_ap,
                 float default_max_slew,
                 std::mutex& modify_mutex)
    : init_(init), sta_(sta), network_(network), graph_(graph),
      graph_delay_calc_(graph_delay_calc),
      ep_(ep), dcalc_ap_(dcalc_ap),
      default_max_slew_(default_max_slew), modify_mutex_(modify_mutex) {}

  VertexVisitor* copy() const override
  {
    return new FixSlewVisitor(init_, sta_, network_, graph_,
                              graph_delay_calc_, ep_, dcalc_ap_,
                              default_max_slew_, modify_mutex_);
  }

  void visit(Vertex* vertex) override
  {
    const Pin* rep_pin = vertex->pin();
    if (network_->isTopLevelPort(rep_pin)) return;
    sta::Instance* inst = network_->instance(rep_pin);
    LibertyCell* cur_cell = network_->libertyCell(inst);
    if (!cur_cell || cur_cell->hasSequentials()) return;

    // Collect all output pins of this instance with their slew limit and
    // actual worst slew. Aggregate across outputs.
    struct OutInfo {
      sta::LibertyPort* port;
      float load_cap;
      float slew_limit;
    };
    std::vector<OutInfo> outs;
    bool any_viol = false;

    sta::InstancePinIterator* pit = network_->pinIterator(inst);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      if (!network_->direction(p)->isOutput()) continue;
      sta::LibertyPort* lp = network_->libertyPort(p);
      if (!lp) continue;
      Vertex* vout = graph_->pinDrvrVertex(p);
      if (!vout) continue;

      float limit = 0.0f; bool exists = false;
      lp->slewLimit(MinMax::max(), limit, exists);
      if (!exists) limit = default_max_slew_;
      limit *= (1.0f - init_->slew_margin_ / 100.0f);

      float worst = 0.0f;
      for (auto rf : RiseFall::range()) {
        float s = graph_->slew(vout, rf, dcalc_ap_);
        worst = std::max(worst, s);
      }
      if (worst > limit) any_viol = true;

      OutInfo oi;
      oi.port = lp;
      oi.load_cap = graph_delay_calc_->loadCap(p, sta_->cmdScene(), sta::MinMax::max());
      oi.slew_limit = limit;
      outs.push_back(oi);
    }
    delete pit;

    if (outs.empty() || !any_viol) return;

    LibertyCellSeq* equivs = sta_->equivCells(cur_cell);
    if (!equivs) {
      unfixed_no_equiv_++;
      return;
    }

    // Evaluate each candidate by summing slew violation across all outputs.
    // Candidates missing any output port are skipped.
    using SizeCandidate = std::pair<float, LibertyCell*>;
    std::vector<SizeCandidate> sizes;

    for (LibertyCell* ec : *equivs) {
      if (ec == cur_cell) continue;
      if (ec->area() < cur_cell->area()) continue;
      float violation = 0.0f;
      bool missing_port = false;
      for (const OutInfo& oi : outs) {
        sta::LibertyPort* ep = ec->findLibertyPort(oi.port->name());
        if (!ep) { missing_port = true; break; }
        float est = init_->estimateMaxSlew(ep, oi.load_cap, dcalc_ap_, inst);
        if (est > oi.slew_limit) violation += (est - oi.slew_limit);
      }
      if (missing_port) continue;
      sizes.emplace_back(violation, ec);
    }

    if (sizes.empty()) {
      unfixed_at_max_size_++;
      return;
    }

    std::sort(sizes.begin(), sizes.end(),
              [](const SizeCandidate& a, const SizeCandidate& b) {
                if (a.first == 0 && b.first == 0)
                  return a.second->area() < b.second->area();
                return a.first < b.first;
              });

    LibertyCell* best = sizes.front().second;
    if (best == cur_cell) {
      unfixed_at_max_size_++;
      return;
    }

    {
      std::lock_guard<std::mutex> lock(modify_mutex_);
      sta_->replaceCell(inst, best);
    }
    upsized_++;
    Level lv = vertex->level();
    Level cur = current_level_.load();
    while (lv > cur && !current_level_.compare_exchange_weak(cur, lv));
  }

  void levelFinished() override
  {
    Level lv = current_level_.load();
    if (upsized_ > prev_upsized_) {
      ep_->updateParasitics();
      sta_->findDelays(lv);
    }
    prev_upsized_ = upsized_;
    current_level_ = 0;
  }

  int upsized() const { return upsized_; }
  int unfixedNoEquiv() const { return unfixed_no_equiv_; }
  int unfixedAtMaxSize() const { return unfixed_at_max_size_; }

private:
  ParallelInitializer* init_;
  sta::dbSta* sta_;
  sta::dbNetwork* network_;
  sta::Graph* graph_;
  sta::GraphDelayCalc* graph_delay_calc_;
  est::EstimateParasitics* ep_;
  sta::DcalcAPIndex dcalc_ap_;
  float default_max_slew_;
  std::mutex& modify_mutex_;
  int upsized_ = 0;
  int prev_upsized_ = 0;
  std::atomic<Level> current_level_{0};
  int unfixed_no_equiv_ = 0;
  int unfixed_at_max_size_ = 0;
};

// ═══════════════════════════════════════════════════════════════════
// ParallelInitializer implementation
// ═══════════════════════════════════════════════════════════════════

ParallelInitializer::ParallelInitializer(
    sta::dbSta* sta, IncreSta* incre_sta,
    rsz::Resizer* resizer, odb::dbBlock* block,
    int thread_count, bool minimize_leakage)
  : incre_sta_(incre_sta),
    resizer_(resizer),
    block_(block),
    thread_count_(thread_count),
    minimize_leakage_(minimize_leakage)
{
  dbStaState::init(sta);
  db_network_ = sta->getDbNetwork();
}

ParallelInitializer::~ParallelInitializer() = default;

float
ParallelInitializer::estimateMaxSlew(sta::LibertyPort* port, float load_cap,
                                     sta::DcalcAPIndex dcalc_ap,
                                     sta::Instance* inst)
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
      float in_slew = 50e-12;
      if (inst && graph_) {
        sta::Pin* in_pin = network_->findPin(inst, arc->from()->name());
        if (in_pin) {
          Vertex* in_v = graph_->pinLoadVertex(in_pin);
          if (in_v) {
            const RiseFall* in_rf = arc->fromEdge()->asRiseFall();
            float s = graph_->slew(in_v, in_rf, dcalc_ap);
            if (s > 0) in_slew = s;
          }
        }
      }
      float arc_delay;
      float arc_slew;
      model->gateDelay(sta_->cmdSdc()->operatingConditions(sta::MinMax::max()),
                       in_slew, load_cap, arc_delay, arc_slew);
      max_slew = std::max(max_slew, arc_slew);
    }
  }
  return max_slew;
}

void
ParallelInitializer::countViolations(int& cap_cnt, int& slew_cnt)
{
  cap_cnt = 0;
  slew_cnt = 0;
  sta::LibertyLibrary* default_lib = network_->defaultLibertyLibrary();
  sta::DcalcAPIndex dcalc_ap
      = sta_->cmdScene()->dcalcAnalysisPtIndex(MinMax::max());

  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* inst = db_network_->dbToSta(db_inst);
    LibertyCell* cell = db_network_->libertyCell(inst);
    if (!cell || cell->hasSequentials()) continue;
    sta::InstancePinIterator* pit = network_->pinIterator(inst);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      if (!network_->direction(p)->isOutput()) continue;
      sta::LibertyPort* port = network_->libertyPort(p);
      if (!port) continue;
      // Cap check (with margin)
      float cl; bool ce;
      port->capacitanceLimit(MinMax::max(), cl, ce);
      if (!ce && default_lib) default_lib->defaultMaxCapacitance(cl, ce);
      if (ce) {
        float cl_m = cl * (1.0f - cap_margin_ / 100.0f);
        if (graph_delay_calc_->loadCap(p, sta_->cmdScene(), sta::MinMax::max()) > cl_m) cap_cnt++;
      }
      // Slew check (with margin)
      float sl; bool se;
      port->slewLimit(MinMax::max(), sl, se);
      if (!se && default_lib) default_lib->defaultMaxSlew(sl, se);
      if (se) {
        float sl_m = sl * (1.0f - slew_margin_ / 100.0f);
        Vertex* v, *bi;
        graph_->pinVertices(p, v, bi);
        if (v) {
          for (auto rf : RiseFall::range()) {
            if (graph_->slew(v, rf, dcalc_ap) > sl_m) {
              slew_cnt++;
              break;
            }
          }
        }
      }
    }
    delete pit;
  }
}

void
ParallelInitializer::run()
{
  auto t0 = std::chrono::steady_clock::now();
  printf("[ParallelInitializer] Sharma 4-step initialization (%d threads%s)\n",
         thread_count_, minimize_leakage_ ? "" : ", no downsize");
  fflush(stdout);

  resizer_->makeEquivCells();
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();
  LocalSta* local_sta = incre_sta_->localSta();

  if (minimize_leakage_) {
    downsizeToMinLeakage();
  } else {
    printf("[ParallelInitializer] Step 1: Skipped (minimize_leakage=false)\n");
    fflush(stdout);
  }
  auto t1 = std::chrono::steady_clock::now();

  local_sta->updateGlobalParasiticsAndSync(ep);
  sta_->delaysInvalid();
  sta_->updateTiming(false);

  fixLoadViolationsParallel();
  auto t2 = std::chrono::steady_clock::now();

  local_sta->updateGlobalParasiticsAndSync(ep);
  sta_->delaysInvalid();
  sta_->updateTiming(false);

  int cap_cnt2 = 0, slew_cnt2 = 0;
  countViolations(cap_cnt2, slew_cnt2);
  printf("[ParallelInitializer] After Step 2: %d cap, %d slew violations\n",
         cap_cnt2, slew_cnt2);
  fflush(stdout);

  auto t3 = t2;
  if (slew_cnt2 > 0) {
    fixSlewViolationsParallel();
    t3 = std::chrono::steady_clock::now();

    local_sta->updateGlobalParasiticsAndSync(ep);
    sta_->delaysInvalid();
    sta_->updateTiming(false);

    int cap_cnt3 = 0, slew_cnt3 = 0;
    countViolations(cap_cnt3, slew_cnt3);
    printf("[ParallelInitializer] After Step 3: %d cap, %d slew violations\n",
           cap_cnt3, slew_cnt3);
    fflush(stdout);
    cap_cnt2 = cap_cnt3;
  } else {
    printf("[ParallelInitializer] Step 3: Skipped (no slew violations)\n");
    fflush(stdout);
  }

  auto t4 = t3;
  if (cap_cnt2 > 0) {
    fixCapByBuffering();
    t4 = std::chrono::steady_clock::now();
  } else {
    printf("[ParallelInitializer] Step 4: Skipped (no cap violations)\n");
    fflush(stdout);
  }

  printf("[ParallelInitializer] Step 1: %.2fs, Step 2: %.2fs, "
         "Step 3: %.2fs, Step 4: %.2fs, Total: %.2fs\n",
         std::chrono::duration<double>(t1 - t0).count(),
         std::chrono::duration<double>(t2 - t1).count(),
         std::chrono::duration<double>(t3 - t2).count(),
         std::chrono::duration<double>(t4 - t3).count(),
         std::chrono::duration<double>(t4 - t0).count());

  // Post-init violation summary
  sta_->ensureGraph();
  sta_->findDelays();
  sta::LibertyLibrary* sum_lib = network_->defaultLibertyLibrary();
  sta::DcalcAPIndex  sum_ap
      = sta_->cmdScene()->dcalcAnalysisPtIndex(MinMax::max());

  int drvr_slew_cnt = 0, load_slew_cnt = 0, cap_cnt = 0;
  float drvr_slew_sum = 0, load_slew_sum = 0, cap_sum = 0;

  // Filter口径与 test/lrf/utils.py:get_score 对齐：
  //   skip CLOCK/POWER/GROUND nets, skip /SETN /RESETN async pins,
  //   不按 hasSequentials 过滤 (FF 的 D pin 也要计入)
  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* si = db_network_->dbToSta(db_inst);
    LibertyCell* c = db_network_->libertyCell(si);
    if (!c) continue;
    sta::InstancePinIterator* pit = network_->pinIterator(si);
    while (pit->hasNext()) {
      Pin* p = pit->next();
      sta::LibertyPort* lp = network_->libertyPort(p);
      if (!lp) continue;
      odb::dbITerm* iterm = nullptr;
      odb::dbBTerm* bterm = nullptr;
      odb::dbModITerm* moditerm = nullptr;
      db_network_->staToDb(p, iterm, bterm, moditerm);
      if (!iterm) continue;
      odb::dbNet* dnet = iterm->getNet();
      if (!dnet) continue;
      auto sig = dnet->getSigType();
      if (sig == odb::dbSigType::POWER || sig == odb::dbSigType::GROUND
          || sig == odb::dbSigType::CLOCK)
        continue;
      const std::string pname = iterm->getName();
      if (pname.size() >= 5
          && pname.compare(pname.size() - 5, 5, "/SETN") == 0)
        continue;
      if (pname.size() >= 7
          && pname.compare(pname.size() - 7, 7, "/RESETN") == 0)
        continue;
      float sl; bool se;
      lp->slewLimit(MinMax::max(), sl, se);
      if (!se && sum_lib) sum_lib->defaultMaxSlew(sl, se);
      if (se) {
        Vertex* v, *bi;
        graph_->pinVertices(p, v, bi);
        if (v) {
          for (auto rf : RiseFall::range()) {
            float s = graph_->slew(v, rf, sum_ap);
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
      if (network_->direction(p)->isOutput()) {
        float cl; bool ce;
        lp->capacitanceLimit(MinMax::max(), cl, ce);
        if (!ce && sum_lib) sum_lib->defaultMaxCapacitance(cl, ce);
        if (ce) {
          float lc = graph_delay_calc_->loadCap(p, sta_->cmdScene(), sta::MinMax::max());
          if (lc > cl) { cap_cnt++; cap_sum += (lc - cl) * 1e15; }
        }
      }
    }
    delete pit;
  }

  printf("[ParallelInitializer] Post-init violations:\n");
  printf("[ParallelInitializer]   Driver slew: %d pins, %.4f ns\n",
         drvr_slew_cnt, drvr_slew_sum);
  printf("[ParallelInitializer]   Load slew:   %d pins, %.4f ns\n",
         load_slew_cnt, load_slew_sum);
  printf("[ParallelInitializer]   Cap:         %d pins, %.4f fF\n",
         cap_cnt, cap_sum);
  fflush(stdout);
}

// ── Step 1: Downsize to min-leakage ─────────────────────────────
void
ParallelInitializer::downsizeToMinLeakage()
{
  int swap_count = 0, inst_count = 0;

  for (odb::dbInst* db_inst : block_->getInsts()) {
    if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
    sta::Instance* sta_inst = db_network_->dbToSta(db_inst);
    LibertyCell* cell = db_network_->libertyCell(sta_inst);
    if (!cell || cell->hasSequentials()) continue;
    inst_count++;
    LibertyCellSeq* equiv_cells = sta_->equivCells(cell);
    if (!equiv_cells || equiv_cells->empty()) continue;
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
      odb::dbMaster* new_master = db_network_->staToDb(min_cell);
      db_inst->swapMaster(new_master);
      swap_count++;
    }
  }

  sta_->updateTiming(false);
  printf("[ParallelInitializer] Step 1: Downsized %d / %d gates\n",
         swap_count, inst_count);
  fflush(stdout);
}

// ── Step 2: Fix load violations PO→PI ───────────────────────────
void
ParallelInitializer::fixLoadViolationsParallel()
{
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();
  est::IncrementalParasiticsGuard guard(ep);
  sta::DcalcAPIndex dcalc_ap
      = sta_->cmdScene()->dcalcAnalysisPtIndex(MinMax::max());
  sta::LibertyLibrary* default_lib = network_->defaultLibertyLibrary();

  sta_->ensureGraph();
  sta_->ensureLevelized();

  sta::SearchPred* search_pred = sta_->search()->searchAdj();
  sta::BfsBkwdIterator bfs(sta::BfsIndex::other, search_pred, sta_);
  bfs.ensureSize();

  // Enqueue one representative output driver vertex per instance so each
  // instance is visited exactly once per level (avoids multi-output races).
  for (odb::dbInst* db_inst : block_->getInsts()) {
    sta::Instance* inst = db_network_->dbToSta(db_inst);
    LibertyCell* cell = db_network_->libertyCell(inst);
    if (!cell || cell->hasSequentials()) continue;
    sta::InstancePinIterator* pit = network_->pinIterator(inst);
    Vertex* rep = nullptr;
    while (pit->hasNext()) {
      Pin* p = pit->next();
      if (!network_->direction(p)->isOutput()) continue;
      rep = graph_->pinDrvrVertex(p);
      if (rep) break;
    }
    delete pit;
    if (rep) bfs.enqueue(rep);
  }

  FixLoadVisitor visitor(this, sta_, db_network_, graph_, graph_delay_calc_,
                         ep, dcalc_ap, default_lib, modify_mutex_);
  bfs.visitParallel(0, &visitor);

  int unfixed = visitor.violCount() - visitor.upsizeCount();
  printf("[ParallelInitializer] Step 2: %d violations, upsized %d gates (PO→PI)",
         visitor.violCount(), visitor.upsizeCount());
  if (unfixed > 0) {
    printf(", unfixed %d (no_equiv=%d, cap_exceeded=%d, slew_exceeded=%d)",
           unfixed, visitor.unfixedNoEquiv(),
           visitor.unfixedCapExceeded(), visitor.unfixedSlewExceeded());
  }
  printf("\n");
  fflush(stdout);
}

// ── Step 3: Fix slew violations PI→PO ───────────────────────────
void
ParallelInitializer::fixSlewViolationsParallel()
{
  est::EstimateParasitics* ep = resizer_->getEstimateParasitics();
  est::IncrementalParasiticsGuard guard(ep);
  sta::DcalcAPIndex dcalc_ap
      = sta_->cmdScene()->dcalcAnalysisPtIndex(MinMax::max());
  sta::LibertyLibrary* default_lib = network_->defaultLibertyLibrary();

  float default_max_slew = sta::INF;
  if (default_lib) {
    bool exists;
    default_lib->defaultMaxSlew(default_max_slew, exists);
    if (!exists) default_max_slew = sta::INF;
  }

  sta_->ensureGraph();
  sta_->ensureLevelized();
  Level max_level = sta::level_max;
  sta::SearchPred* search_pred = sta_->search()->searchAdj();

  int total_upsized = 0;
  int total_no_equiv = 0;
  int total_at_max = 0;

  for (int pass = 0; pass < 5; pass++) {
    sta_->ensureGraph();
    sta_->findDelays();

    sta::BfsFwdIterator bfs(sta::BfsIndex::other, search_pred, sta_);
    bfs.ensureSize();

    // Enqueue one representative output driver vertex per instance.
    for (odb::dbInst* db_inst : block_->getInsts()) {
      if (!db_inst->getMaster()->isCoreAutoPlaceable()) continue;
      sta::Instance* inst = db_network_->dbToSta(db_inst);
      LibertyCell* cell = db_network_->libertyCell(inst);
      if (!cell || cell->hasSequentials()) continue;
      sta::InstancePinIterator* pit = network_->pinIterator(inst);
      Vertex* rep = nullptr;
      while (pit->hasNext()) {
        Pin* p = pit->next();
        if (!network_->direction(p)->isOutput()) continue;
        rep = graph_->pinDrvrVertex(p);
        if (rep) break;
      }
      delete pit;
      if (rep) bfs.enqueue(rep);
    }

    FixSlewVisitor visitor(this, sta_, db_network_, graph_, graph_delay_calc_,
                           ep, dcalc_ap, default_max_slew, modify_mutex_);
    bfs.visitParallel(max_level, &visitor);

    int upsized_this_pass = visitor.upsized();
    total_upsized += upsized_this_pass;
    total_no_equiv += visitor.unfixedNoEquiv();
    total_at_max += visitor.unfixedAtMaxSize();

    printf("[ParallelInitializer]   Pass %d: upsized %d cells",
           pass, upsized_this_pass);
    int unfixed = visitor.unfixedNoEquiv() + visitor.unfixedAtMaxSize();
    if (unfixed > 0) {
      printf(", unfixed %d (no_equiv=%d, at_max_size=%d)",
             unfixed, visitor.unfixedNoEquiv(), visitor.unfixedAtMaxSize());
    }
    printf("\n");
    fflush(stdout);
    if (upsized_this_pass == 0) break;
  }

  // Final verification.
  sta_->findDelays();
  int remaining = 0;
  sta::DcalcAPIndex dcalc_ap2
      = sta_->cmdScene()->dcalcAnalysisPtIndex(MinMax::max());
  sta::VertexIterator viter2(graph_);
  while (viter2.hasNext()) {
    Vertex* vertex = viter2.next();
    if (!vertex->isDriver(network_)) continue;
    const Pin* pin = vertex->pin();
    if (network_->isTopLevelPort(pin)) continue;
    sta::LibertyPort* port = network_->libertyPort(pin);
    if (!port) continue;
    float limit = 0.0f; bool exists = false;
    port->slewLimit(MinMax::max(), limit, exists);
    if (!exists) limit = default_max_slew;
    float worst = 0.0f;
    for (auto rf : RiseFall::range()) {
      float s = graph_->slew(vertex, rf, dcalc_ap2);
      worst = std::max(worst, s);
    }
    if (worst > limit) remaining++;
  }

  printf("[ParallelInitializer] Step 3: upsized %d cells, "
         "%d remaining violations\n", total_upsized, remaining);
  fflush(stdout);
}

// ── Step 4: Fix remaining cap/slew violations via repair_design ──
//
// After Step 1-3 (downsize + resize for cap/slew), delegate remaining
// violations to OpenROAD's repair_design which handles cap, slew, and
// wire length violations via Steiner tree buffer insertion.
void
ParallelInitializer::fixCapByBuffering()
{
  Tcl_Eval(sta_->tclInterp(), "repair_design");
  Tcl_Eval(sta_->tclInterp(), "estimate_parasitics -placement");
}

} // namespace lrf

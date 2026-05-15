#include "lrf/IncreSta.hh"
#include "LocalSta.hh"
#include "LrHelper.hh"
#include "sta/Liberty.hh"
#include "sta/Path.hh"
#include "sta/Corner.hh"
#include "sta/PathExpanded.hh"
#include "sta/Search.hh"
#include "sta/EquivCells.hh"
#include "sta/Sdc.hh"
#include "power/Power.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/PortDirection.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/TimingRole.hh"
#include "lrf/LrfClass.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "TaskArranger.hh"
#include "NetlistTransformation.hh"
#include "LrRebuffer.hh"
#include "TestRebuffer.hh"
#include "rsz/Resizer.hh"
#include "ParallelLibData.hh"
#include "LrSizer.hh"

#include <unordered_map>
#include <chrono>
#include <algorithm>
#include <limits>
#include <vector>

namespace lrf {

namespace {
// Heuristic parser for ASAP7-style names, eg
//   O2A1O1Ixp33_ASAP7_75t_R
//   O2A1O1Ixp33_ASAP7_75t_L
//   O2A1O1Ixp5_ASAP7_75t_SRAM
// We treat everything before the first "_ASAP7_" as the structural prefix.
struct CellNameParts
{
  std::string prefix;  // structure (horizontal axis)
  std::string vt;      // R/L/SL/SRAM/... (vertical axis)
};

CellNameParts parseCellName(const char* name)
{
  CellNameParts parts;
  if (name == nullptr) {
    return parts;
  }
  const std::string s{name};

  // prefix: up to _ASAP7_ if present, else up to last '_' (best-effort)
  const std::string anchor = "_ASAP7_";
  const size_t anchor_pos = s.find(anchor);
  if (anchor_pos != std::string::npos) {
    parts.prefix = s.substr(0, anchor_pos);
  } else {
    const size_t last_us = s.rfind('_');
    parts.prefix = (last_us == std::string::npos) ? s : s.substr(0, last_us);
  }

  // vt: token after last '_'
  const size_t last_us = s.rfind('_');
  if (last_us != std::string::npos && last_us + 1 < s.size()) {
    parts.vt = s.substr(last_us + 1);
  } else {
    parts.vt = "";
  }
  return parts;
}

// A coarse VT ordering (top->bottom). If unknown, keep after known values.
// NOTE: You said index increasing corresponds to VT rising / input cap decreasing.
// We'll compute per-prefix order late using measured input cap, but this provides
// a stable tie-breaker.
int vtRank(const std::string& vt)
{
  if (vt == "SRAM") {
    return 0;
  }
  if (vt == "R") {
    return 1;
  }
  if (vt == "L") {
    return 2;
  }
  if (vt == "SL") {
    return 3;
  }
  // unknown
  return 100;
}

double avgInputCap(const sta::LibertyCell* corner_cell)
{
  if (corner_cell == nullptr) {
    return 0.0;
  }
  double sum = 0.0;
  int cnt = 0;
  sta::LibertyCellPortIterator port_iter(corner_cell);
  while (port_iter.hasNext()) {
    sta::LibertyPort* port = port_iter.next();
    if (!port)
      continue;
    if (port->isPwrGnd() || port->isClock())
      continue;
    if (port->direction() == sta::PortDirection::input()) {
      sum += port->capacitance();
      cnt++;
    }
  }
  return (cnt > 0) ? (sum / cnt) : 0.0;
}

}  // namespace

// All logic is handled via dbStaState base; nothing additional yet.
IncreSta::IncreSta(dbSta *db_sta)
    : local_sta_(nullptr),
      lr_helper_(nullptr)
{
  db_sta->setThreadCount(1);
  dbStaState::init(db_sta);
  makeLocalSta();
  makeLRHelper();
  swappable_cells_cache_.clear();
}

IncreSta::IncreSta(dbSta *db_sta, size_t thread_count)
    : local_sta_(nullptr),
      lr_helper_(nullptr)
{
  dbStaState::init(db_sta);
  if (thread_count != threadCount()) {
    printf("IncreSta: setting STA thread count to %zu\n", thread_count);
    sta_->setThreadCount(thread_count);
  }
  makeLocalSta();
  makeLRHelper();
  swappable_cells_cache_.clear();
}

void
IncreSta::init()
{
  sta_->ensureLevelized();
  local_sta_->copyState(sta_);
  lr_helper_->copyState(sta_);
}

IncreSta::~IncreSta()
{
  delete local_sta_;
  delete lr_helper_;
  for (auto &it : swappable_cells_cache_) {
    delete it.second;
  }
  clearLocalCellInfoMap();
  swappable_cells_cache_.clear();
  sta_->unregisterStaState(this);
}

void
IncreSta::clearLocalCellInfoMap()
{
  delete[] cell_info_vec_;
  cell_info_vec_ = nullptr;
  inst_info_map_.clear();
}

void 
IncreSta::copyState(const dbSta *sta)
{
  dbStaState::copyState(sta);
  local_sta_->copyState(sta);
  lr_helper_->copyState(sta);
}

void 
IncreSta::makeLocalSta()
{
  if (local_sta_)
    delete local_sta_;
  local_sta_ = new LocalSta(sta_);
}

void 
IncreSta::makeLRHelper(std::string method)
{
  std::transform(method.begin(), method.end(), method.begin(),
                 [](unsigned char ch){ return static_cast<char>(std::tolower(ch)); });
  if (lr_helper_)
    delete lr_helper_;
  if (method == "lrhelper")
    lr_helper_ = new LRHelper(sta_);
  else if (method == "rapidlrhelper")
    lr_helper_ = new RapidLrHelper(sta_);
  else
    lr_helper_ = new RapidLrHelper(sta_);
}

InstanceSeq &
IncreSta::getSortedInstances()
{ 
  sta_->ensureLevelized();
  InstanceSet instance_visited(network_);
  
  VertexSeq &vertices = lr_helper_->ensureSorted(sta_);
  printf("Size of sorted vertices: %zu\n", vertices.size());
  fflush(stdout);
  for (Vertex *vertex : vertices) {
    Instance *inst = network_->instance(vertex->pin());
    if (instance_visited.find(inst) == instance_visited.end()) {
      sorted_instances_.push_back(inst);
      instance_visited.insert(inst);
    }
  }
  return sorted_instances_;
}

void 
IncreSta::checkeTopoOrder(InstanceSeq &) {

}

void
IncreSta::initDelayDiff()
{
  sta::Graph *graph = sta_->graph();
  if (!graph || graph->enableDiff()) return;   // already initialized
  graph->setEnableDiff(true);
  // Force a full re-eval: clears the incremental queue and re-seeds roots,
  // so the next findDelays revisits every arc and writes delay_diffs_.
  sta_->delaysInvalid();
  sta_->findDelays();
}

void
IncreSta::delayLmSum(Instance *inst, const MinMax *minmax, float &delay_lambda_sum)
{
  init();
  delay_lambda_sum = 0.0;
  delay_lambda_sum = local_sta_->delayLmSum(inst, minmax);
}

bool
IncreSta::isPowerOptimizationMode() const
{
  return lr_helper_->mode() == "power";
}

void
IncreSta::recordMetrics(double wns_ps, double tns_ps, double leakage)
{
  // wns/tns rolling 4 (used by isTnsPlateau over a 3-iter span).
  wns_history_.push_back(wns_ps);
  tns_history_.push_back(tns_ps);
  while (wns_history_.size() > 4) wns_history_.pop_front();
  while (tns_history_.size() > 4) tns_history_.pop_front();

  // leakage is *unbounded* — power_mode_iters_ acts as the index back to the
  // pre-power baseline (leakage_history_[size-1-power_mode_iters_]). Memory
  // is O(total LR iters), trivially small.
  leakage_history_.push_back(leakage);

  // power_mode_iters_ tracks iters since first entering power mode.
  // Power mode is sticky, so no entry flag needed.
  if (isPowerOptimizationMode()) ++power_mode_iters_;
}

double
IncreSta::tnsImprovementRate() const
{
  if (tns_history_.size() < 4) return 0.0;
  const double front = std::min(tns_history_.front(), 0.0);
  const double back  = std::min(tns_history_.back(),  0.0);
  if (front >= 0.0) return 0.0;            // already met → rate ill-defined
  return std::max(back - front, 0.0) / -front;     // +ve = TNS less negative
}

double
IncreSta::leakageReductionRate() const
{
  // Index back power_mode_iters_ steps from the latest sample to find the
  // pre-power baseline. Requires the baseline to still be in the deque
  // (it is, since leakage_history_ is unbounded).
  if (power_mode_iters_ == 0) return 0.0;
  const size_t n = leakage_history_.size();
  if (n <= power_mode_iters_) return 0.0;            // no pre-power sample
  const double baseline = leakage_history_[n - 1 - power_mode_iters_];
  if (baseline <= 0.0) return 0.0;
  const double cur = leakage_history_.back();
  return (baseline - cur) / baseline / static_cast<double>(power_mode_iters_);
}

bool
IncreSta::isTnsPlateau(double threshold) const
{
  if (tns_history_.size() < 4) return false;
  if (tns_history_.front() >= 0.0) return true;  // already met → treat as plateau
  return tnsImprovementRate() < threshold;
}

bool
IncreSta::isLeakagePlateau(double threshold) const
{
  if (!isPowerOptimizationMode()) return false;
  if (power_mode_iters_ < 3)      return false;   // need ≥3 post-entry samples
  return leakageReductionRate() < threshold;
}

void
IncreSta::lmUpdate()
{
  auto lm_t0 = std::chrono::high_resolution_clock::now();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());
  sta::Slack tns = sta_->totalNegativeSlack(sta::MinMax::max());

  // Power-mode entry: any ONE of the following is sufficient.
  //   (a) |WNS| < 1%  of T_eff  (worst violation small)
  //   (b) |TNS| < 10% of T_eff  (total violation small)
  //   (c) TNS plateau over the rolling history (improvement rate < 10%)
  // T_eff = clock_period * (1 + timing_margin), matching RapidLrHelper.
  // History is fed externally via recordMetrics() at snapshot time.
  if (lr_helper_ && lr_helper_->mode() != "power") {
    float clock_period = 0.0f;
    for (Clock *clock : *sdc_->clocks()) {
      float period = clock->period();
      if (period > clock_period) {
        clock_period = period;
        break;
      }
    }
    if (clock_period > 0.0f) {
      const float t_eff = clock_period * (1.0f + lr_helper_->timingMargin());
      const bool wns_ok     = wns > -0.01 * t_eff;
      const bool tns_ok     = tns > -0.10 * t_eff;
      const bool plateau_ok = isTnsPlateau(0.10);
      if (wns_ok || tns_ok || plateau_ok) {
        lr_helper_->setMode("power");
        printf("Switching to power mode "
               "(WNS=%.3f ps, TNS=%.3f ps, T_eff=%.3f ps; "
               "wns_ok=%d tns_ok=%d plateau_ok=%d, TNS rate=%.4f)\n",
               wns * 1e12, tns * 1e12, t_eff * 1e12,
               wns_ok, tns_ok, plateau_ok, tnsImprovementRate());
        fflush(stdout);
      }
      // Sticky CPS gate — only the timing-clean (a)/(b) thresholds latch
      // it on; (c) plateau alone is not enough. Once enabled, stays on
      // for the rest of the run.
      if (!cps_enabled_ && (wns_ok || tns_ok)) {
        cps_enabled_ = true;
        printf("CPS enabled (WNS=%.3f ps, TNS=%.3f ps cleared "
               "(a)/(b) threshold; T_eff=%.3f ps)\n",
               wns * 1e12, tns * 1e12, t_eff * 1e12);
        fflush(stdout);
      }
    }
  }

  const bool use_parallel = (thread_count_ > 1 && dispatch_queue_);

  auto lm_t_edge_start = std::chrono::high_resolution_clock::now();
  double lm_edge_s = 0.0;
  double lm_kkt_s  = 0.0;
  if (projected_) {
    if (use_parallel)
      lr_helper_->parallelUpdateAllEdgeLms(sta_);
    else
      lr_helper_->updateAllEdgeLms(sta_);
    auto lm_t_edge_end = std::chrono::high_resolution_clock::now();
    lm_edge_s = std::chrono::duration<double>(lm_t_edge_end - lm_t_edge_start).count();

    if (use_parallel)
      lr_helper_->parallelKKTProjection(sta_);
    else
      lr_helper_->KKTProjection(sta_);
    lm_kkt_s = std::chrono::duration<double>(
                 std::chrono::high_resolution_clock::now() - lm_t_edge_end).count();
  } else {
    bool kkt_satisfied = use_parallel
      ? lr_helper_->parallelKKTProjection(sta_)
      : lr_helper_->KKTProjection(sta_);
    lm_kkt_s = std::chrono::duration<double>(
                 std::chrono::high_resolution_clock::now() - lm_t_edge_start).count();
    if (kkt_satisfied)
      projected_ = true;
  }

  double lm_total_s = std::chrono::duration<double>(
                        std::chrono::high_resolution_clock::now() - lm_t0).count();
  printf("[LM_UPDATE] total=%.3f s  edge_lm=%.3f s  kkt_proj=%.3f s  "
         "(parallel=%d, projected=%d)\n",
         lm_total_s, lm_edge_s, lm_kkt_s,
         use_parallel ? 1 : 0, projected_ ? 1 : 0);
  fflush(stdout);
}

bool
IncreSta::saveLmToFile(const std::string &path, const std::string &design_name)
{
  if (!lr_helper_) {
    printf("IncreSta::saveLmToFile: LRHelper not initialized\n");
    return false;
  }
  return lr_helper_->saveLmToFile(path, design_name);
}

int
IncreSta::loadLmFromFile(const std::string &path, const std::string &design_name)
{
  if (!lr_helper_) {
    printf("IncreSta::loadLmFromFile: LRHelper not initialized\n");
    return -1;
  }
  return lr_helper_->loadLmFromFile(path, design_name);
}

bool
IncreSta::checkCapViolated(Pin *pin, const Corner *corner, const MinMax *min_max)
{
  const Corner *corner1;
  const RiseFall *rf;
  float capacitance, limit, slack;
  sta_->checkCapacitance(pin, corner, min_max, corner1, rf, capacitance, limit, slack);
  if (corner1 && slack < 0.0)
    return true;
  return false;
}

float
IncreSta::maxInputSlew(const Pin* input_pin,
                            const Corner* corner) const
{
  return local_sta_->maxInputSlew(input_pin, corner);
}

float
IncreSta::averageDelayOnCritPath() {
  // Ensure timing is up-to-date before querying worst paths.
  sta_->searchPreamble();

  // Find WNS worst endpoint across all corners for setup (max) by default.
  const MinMax *minmax = MinMax::max();
  Slack worst_slack;
  Vertex *worst_vertex = nullptr;
  sta_->worstSlack(minmax, worst_slack, worst_vertex);
  if (worst_vertex == nullptr) {
    return 0.0f;
  }

  // Reconstruct the worst-slack data path ending at the worst endpoint.
  Path *path = sta_->vertexWorstSlackPath(worst_vertex, minmax);
  PathExpanded path_expanded(path, sta_);
  if (path == nullptr) {
    return 0.0f;
  }

  Arrival worst_arrival = sta_->vertexArrival(worst_vertex, MinMax::max());
  size_t path_length = path_expanded.size();

  return (worst_arrival / path_length);
}

float
IncreSta::averageLeakage()
{
  float total_leakage = 0.0;
  int cnt = 0;
  sta::Corner *corner = sta_->corners()->findCorner("default");
  sta::LeafInstanceIterator* inst_iter = network_->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance* inst = inst_iter->next();
    sta::LibertyCell *cell = network_->libertyCell(inst);
    if (cell) {
      sta::PowerResult power_result = sta_->power(inst, corner);
      total_leakage += power_result.leakage();
      cnt++;
    }
  }
  delete inst_iter;
  return total_leakage / cnt;
}

float
IncreSta::averageOutSlew()
{
  sta::Corner *corner = sta_->corners()->findCorner("default");
  sta::DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
  double sum = 0.0;
  int cnt = 0;
  sta::LeafInstanceIterator *inst_iter = network_->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    sta::InstancePinIterator *pin_iter = network_->pinIterator(inst);
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      if (!network_->direction(pin)->isAnyOutput())
        continue;
      sta::Vertex *vtx = sta_->graph()->pinDrvrVertex(pin);
      if (!vtx)
        continue;
      float s = 0.0f;
      for (const RiseFall *rf : RiseFall::range()) {
        float sl = sta_->graph()->slew(vtx, rf, dcalc_ap->index());
        if (sl > s) s = sl;
      }
      sum += s;
      cnt++;
    }
    delete pin_iter;
  }
  delete inst_iter;
  return cnt > 0 ? static_cast<float>(sum / cnt) : 1e-10f;
}

float
IncreSta::averageLoadCap()
{
  sta::Corner *corner = sta_->corners()->findCorner("default");
  sta::DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
  double sum = 0.0;
  int cnt = 0;
  sta::LeafInstanceIterator *inst_iter = network_->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    sta::InstancePinIterator *pin_iter = network_->pinIterator(inst);
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      if (!network_->direction(pin)->isAnyOutput())
        continue;
      float lc = sta_->graphDelayCalc()->loadCap(pin, dcalc_ap);
      sum += lc;
      cnt++;
    }
    delete pin_iter;
  }
  delete inst_iter;
  return cnt > 0 ? static_cast<float>(sum / cnt) : 1e-15f;
}

void
IncreSta::updateErcNormalizers()
{
  avg_out_slew_ = averageOutSlew();
  avg_load_cap_ = averageLoadCap();
  printf("ERC normalizers: avg_slew=%.3e s, avg_cap=%.3e F\n",
         avg_out_slew_, avg_load_cap_);
}

float
IncreSta::totalLeakageFast()
{
  float total_leakage = 0.0f;
  sta::LeafInstanceIterator *inst_iter = network_->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    sta::LibertyCell *cell = network_->libertyCell(inst);
    if (!cell)
      continue;
    auto it = inst_info_map_.find(inst);
    if (it != inst_info_map_.end()) {
      LocalCellInfo *info = it->second;
      if (info->equiv_cells) {
        for (size_t j = 0; j < info->equiv_cells->size(); j++) {
          if ((*(info->equiv_cells))[j] == cell) {
            total_leakage += info->cell_leakages[j];
            break;
          }
        }
      }
    } else {
      total_leakage += local_sta_->cellAvgLeakage(cell);
    }
  }
  delete inst_iter;
  return total_leakage;
}

void
IncreSta::setLocalStaParasiticsEst(est::EstimateParasitics *estimate_parasitics)
{
  local_sta_->setParasiticsEst(estimate_parasitics);
}

void
IncreSta::preSaveLibCellLeakage()
{
  if (!swap_cell_presaved_)
    throw std::runtime_error("IncreSta::preSaveLibCellLeakage called before swappable cells are presaved\n");
  ensureActivities();
  sta::Corner *corner = sta_->corners()->findCorner("default");
  // Clear any previous allocation before creating new one.
  clearLocalCellInfoMap();
  cell_info_vec_ = new LocalCellInfo[int(network_->leafInstanceCount() * 1.4)];
  inst_info_map_.clear();
  inst_info_map_.reserve(int(network_->leafInstanceCount() * 1.4));

  int cnt = 0;
  sta::LeafInstanceIterator* inst_iter = network_->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance* inst = inst_iter->next();
    sta::LibertyCell *cell = network_->libertyCell(inst);
    if (cell) {
      if (swappable_cells_cache_.find(cell) == swappable_cells_cache_.end()) {
        cnt++;
        continue;
      }

      LocalCellInfo *cell_info = &cell_info_vec_[cnt++];
      cell_info->equiv_cells = swappable_cells_cache_[cell];
      
      // Safety check: ensure equiv_cells is not null.
      // Do NOT delete cell_info here — it points into the array cell_info_vec_.
      if (!cell_info->equiv_cells) {
        continue; 
      }

      cell_info->cell_leakages = new float[cell_info->equiv_cells->size()];
      sta::Power *power_calc = sta_->power();
      for (size_t i = 0; i < cell_info->equiv_cells->size(); ++i) {
        sta::LibertyCell *equiv_cell = (*(cell_info->equiv_cells))[i];
        cell_info->cell_leakages[i] = power_calc->leakagePower(inst, equiv_cell, corner);
      }
      inst_info_map_[inst] = cell_info;
    }
  }
  delete inst_iter;
  swap_cell_leakage_presaved_ = true;
}

void
IncreSta::makeParallelLibData(rsz::Resizer *resizer, TaskArranger *task_arranger)
{
  if (parallel_lib_data_)
    delete parallel_lib_data_;
  parallel_lib_data_ = new ParallelLibData(sta_);
  parallel_lib_data_->init(resizer, task_arranger);
}

//////////////////////////////////////////////////////////
// APIs for parasitics estimation
///////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// APIs for swappable cells
///////////////////////////////////////////////////////////
void 
IncreSta::ensureActivities()
{
  sta::Corner *corner = sta_->corners()->findCorner("default");
  LeafInstanceIterator* inst_iter = network_->leafInstanceIterator();
  sta::Instance* inst = nullptr;
  while (inst_iter->hasNext()) {
    inst = inst_iter->next();
    if (!network_->libertyCell(inst))
      continue;
    else
      break;
  }
  delete inst_iter;
  if (inst)
    sta_->power(inst, corner);
  else 
    throw std::runtime_error("IncreSta::ensureActivities no valid instance found to trigger activity calculation\n");
}

void
IncreSta::makeEquivCellArray(bool verbose)
{
  if (equiv_cell_array_built_)
    return;
  equiv_cell_array_.clear();
  equiv_cell_pos_map_.clear();

  sta::dbSta* sta = sta_;
  sta::dbNetwork* network = sta->getDbNetwork();

  sta::Corner* corner = sta->cmdCorner();
  const sta::DcalcAnalysisPt* dcalc_ap
      = corner ? corner->findDcalcAnalysisPt(sta::MinMax::max()) : nullptr;
  const int lib_ap = dcalc_ap ? dcalc_ap->libertyIndex() : 0;

  // Iterate all liberty cells and get their equiv group.
  std::set<sta::LibertyCellSeq*> seen_groups;
  sta::LibertyLibraryIterator* lib_iter = network->libertyLibraryIterator();
  while (lib_iter->hasNext()) {
    sta::LibertyLibrary* lib = lib_iter->next();
    sta::LibertyCellIterator cell_iter(lib);
    while (cell_iter.hasNext()) {
      sta::LibertyCell* any_cell = cell_iter.next();
      if (!any_cell) {
        continue;
      }
      sta::LibertyCellSeq* group = sta->equivCells(any_cell);
      if (!group || group->empty() || seen_groups.find(group) != seen_groups.end()) {
        continue;
      }
      seen_groups.insert(group);

      // Build columns by prefix.
      // column_key = prefix; rows determined by VT order (or cap order).
      std::map<std::string, std::vector<sta::LibertyCell*>> cols;

      // Track per-cell metrics to drive sorting.
      std::map<sta::LibertyCell*, double> cell_incap;
      std::map<sta::LibertyCell*, CellNameParts> cell_parts;

      for (sta::LibertyCell* c : *group) {
        if (!c)
          continue;
        const sta::LibertyCell* corner_cell = c->cornerCell(lib_ap);
        const double incap = avgInputCap(corner_cell);
        cell_incap[c] = incap;
        cell_parts[c] = parseCellName(c->name());
        cols[cell_parts[c].prefix].push_back(c);
      }

      // Horizontal axis: sort structural prefixes by smallest input cap inside
      // that prefix ("input cap smaller on the left").
      std::vector<std::string> prefixes;
      prefixes.reserve(cols.size());
      for (const auto& [prefix, _] : cols) {
        prefixes.push_back(prefix);
      }
      std::sort(prefixes.begin(), prefixes.end(), [&](const std::string& a, const std::string& b) {
        auto best_cap = [&](const std::string& p) {
          double best = std::numeric_limits<double>::infinity();
          for (sta::LibertyCell* c : cols[p]) {
            best = std::min(best, cell_incap[c]);
          }
          return best;
        };
        const double ca = best_cap(a);
        const double cb = best_cap(b);
        if (ca != cb)
          return ca < cb;
        return a < b;
      });

      // For each prefix (column), vertical axis: sort by (vtRank, input cap desc)
      // to encourage higher vt (smaller cap) at larger row index.
      // If the naming doesn't capture VT well, the cap tie-breaker still gives a
      // sensible ordering.
      for (auto& [prefix, vec] : cols) {
        std::sort(vec.begin(), vec.end(), [&](sta::LibertyCell* x, sta::LibertyCell* y) {
          const auto& px = cell_parts[x];
          const auto& py = cell_parts[y];
          const int rx = vtRank(px.vt);
          const int ry = vtRank(py.vt);
          // smaller cap should be lower (larger index), so sort cap descending.
          const double cx = cell_incap[x];
          const double cy = cell_incap[y];
          if (verbose) {
            if (rx < ry && cx > cy) {
              printf("  [Warning: unexpected ranking] %s vs %s: cap wins (%.2e vs %.2e)",
                x->name(), y->name(), cx, cy);
            } else if (rx > ry && cx < cy) {
              printf("  [Warning: unexpected ranking] %s vs %s: cap wins (%.2e vs %.2e)",
                x->name(), y->name(), cx, cy);
            }
          }
          if (rx != ry)
            return rx < ry;
          if (cx != cy)
            return cx > cy;
          return std::string(x->name()) < std::string(y->name());
        });
      }

      // Build a rectangular matrix [rows][cols]. Missing entries are nullptr.
      size_t max_rows = 0;
      for (const auto& p : prefixes) {
        max_rows = std::max(max_rows, cols[p].size());
      }
      std::vector<std::vector<sta::LibertyCell*>> matrix(
          max_rows, std::vector<sta::LibertyCell*>(prefixes.size(), nullptr));

      // Fill matrix and build per-cell position with global row offset.
      const int row_offset = static_cast<int>(equiv_cell_array_.size());
      const int group_end = row_offset + static_cast<int>(max_rows);
      for (size_t col = 0; col < prefixes.size(); col++) {
        const auto& prefix = prefixes[col];
        const auto& vec = cols[prefix];
        for (size_t row = 0; row < vec.size(); row++) {
          matrix[row][col] = vec[row];
          equiv_cell_pos_map_[vec[row]] = {static_cast<int>(row) + row_offset,
                                           static_cast<int>(col),
                                           row_offset,
                                           group_end};
        }
      }

      // Append this group's matrix into the global array.
      equiv_cell_array_.insert(equiv_cell_array_.end(), matrix.begin(), matrix.end());

      // Log one group matrix.
      if (verbose) {
        printf("EquivCellArrayGroup: %s (%zu cells, %zu cols, %zu rows)\n",
          group->front() ? group->front()->name() : "<null>",
          group->size(),
          prefixes.size(),
          max_rows);
        // Print header row (prefixes)
        std::string header = "  col:";
        for (size_t c = 0; c < prefixes.size(); c++) {
          header += (c == 0 ? " " : " | ");
          header += prefixes[c];
        }
        printf("%s\n", header.c_str());
        for (size_t r = 0; r < max_rows; r++) {
          std::string line = fmt::format("  row{:>2d}:", static_cast<int>(r));
          for (size_t c = 0; c < prefixes.size(); c++) {
            line += (c == 0 ? " " : " | ");
            if (matrix[r][c]) {
              const double cap = cell_incap[matrix[r][c]];
              line += fmt::format("{}({:.3g})", matrix[r][c]->name(), cap);
            } else {
              line += "<null>";
            }
          }
          printf("%s\n", line.c_str());
        }
        printf("  --\n");
      }
    }
  }
  delete lib_iter;
  equiv_cell_array_built_ = true;
  if (verbose) {
    printf("makeEquivCellArray: %zu rows, %zu cells in pos_map\n",
           equiv_cell_array_.size(), equiv_cell_pos_map_.size());
    fflush(stdout);
  }
}

void 
IncreSta::makeSwappableCellsCache(rsz::Resizer *resizer)
{
  // This should be used as an ensurance
  resizer->makeEquivCells();
  // First clear the existing cache
  swappable_cells_cache_.clear();
  // prepare the neccesary activities
  ensureActivities();
  const sta::LibertyCellSeq &unique_equiv_cells = sta_->equivCellsRecorder()->uniqueEquivCells();
  for (const sta::LibertyCell* source_cell : unique_equiv_cells) {
    sta::LibertyCellSeq *equive_cells = sta_->equivCells(const_cast<sta::LibertyCell*>(source_cell));
    for (sta::LibertyCell* equiv_cell : *equive_cells) {
      sta::LibertyCellSeq *swappable_cells = resizer->makeSwappableCells(equiv_cell);
      swappable_cells_cache_[equiv_cell] = swappable_cells;
    }
  }
  swap_cell_presaved_ = true;
}


//////////////////////////////////////////////////////////
// APIs for LR resizing
///////////////////////////////////////////////////////////
void
IncreSta::setMaxResizeNum(size_t max_resize_num)
{
  local_sta_->taskArranger()->setMaxResizeNum(max_resize_num);
}

void
IncreSta::parallelResizeByArray(rsz::Resizer *resizer, float avg_delay,
                                  float avg_power, float PT_tradeoff,
                                  float erc_violation_weight,
                                  float erc_limit_scale)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  if (!swap_cell_presaved_)
    makeSwappableCellsCache(resizer);
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();
  makeEquivCellArray();
  updateErcNormalizers();

  auto start_resize = std::chrono::high_resolution_clock::now();

  // Create new-framework visitor with ResizeOperator
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);

  auto resize_op = std::make_unique<ResizeOperator>(sta_, local_sta_);
  resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  resize_op->setPruningControl(&pruning_control_);
  visitor->setOperator(std::move(resize_op));

  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);

  // Pass density map to EvalContext if set.
  if (density_map_) {
    visitor->evalContext().density_map = density_map_;
    visitor->evalContext().density_weight = density_weight_;
    visitor->evalContext().average_area = average_area_;
  }
  // ERC penalty normalizers (cached by updateErcNormalizers()).
  visitor->evalContext().average_slew = avg_out_slew_;
  visitor->evalContext().average_cap = avg_load_cap_;
  visitor->evalContext().erc_violation_weight = erc_violation_weight;
  visitor->evalContext().erc_slew_limit_scale = erc_limit_scale;
  visitor->evalContext().erc_cap_limit_scale = erc_limit_scale;
  visitor->evalContext().debug = debug_;

  local_sta_->taskArranger()->setProgressTag("LRF resize");
  local_sta_->runResize(resizer, visitor);

  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(sta::MinMax::max());
  double wns_after = sta_->worstSlack(sta::MinMax::max());
  printf("After parallel resize, TNS: %e, WNS: %e\n", tns_after, wns_after);
  printf("parallel resize time: %f s\n", diff_resize.count());

  // Record change count for ECO adaptive ratio computation
  pruning_control_.last_change_count = local_sta_->taskArranger()->lastChangeCount();

  // Pruning: update iteration counter (K detection done in TaskArranger)
  pruning_control_.iteration++;
  printf("Pruning: iteration %d, enabled=%d, K=%d\n",
         pruning_control_.iteration, pruning_control_.enabled, pruning_control_.K);
  fflush(stdout);

  if (cps_enabled_) {
    ParallelVisitor *cp_visitor = new ParallelVisitor(sta_, local_sta_, resizer);
    std::unique_ptr<ResizeOperator> cp_resize_op =
        std::make_unique<ResizeOperator>(sta_, local_sta_);
    cp_resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
    cp_resize_op->setPruningControl(&pruning_control_);
    cp_visitor->setOperator(std::move(cp_resize_op));
    cp_visitor->init(avg_delay, avg_power, wns_after, PT_tradeoff, &inst_info_map_);
    cp_visitor->evalContext().average_slew = avg_out_slew_;
    cp_visitor->evalContext().average_cap = avg_load_cap_;
    cp_visitor->evalContext().erc_violation_weight = erc_violation_weight;
    cp_visitor->evalContext().erc_slew_limit_scale = erc_limit_scale;
    cp_visitor->evalContext().erc_cap_limit_scale = erc_limit_scale;
    cp_visitor->evalContext().debug = debug_;
    std::chrono::high_resolution_clock::time_point start_cps =
        std::chrono::high_resolution_clock::now();
    LrSizer lr_sizer(sta_, lr_helper_, cp_visitor);
    lr_sizer.criticalPathSizing();
    std::chrono::high_resolution_clock::time_point end_cps =
        std::chrono::high_resolution_clock::now();
    printf("After critical path sizing, TNS: %e, WNS: %e\n",
           sta_->totalNegativeSlack(MinMax::max()),
           (double)sta_->worstSlack(MinMax::max()));
    printf("critical path sizing time: %f s\n",
           std::chrono::duration<double>(end_cps - start_cps).count());
    delete cp_visitor;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}

void
IncreSta::parallelResizeFFs(rsz::Resizer *resizer, float avg_delay,
                            float avg_power, float PT_tradeoff,
                            float erc_violation_weight,
                            float erc_limit_scale)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  if (!swap_cell_presaved_)
    makeSwappableCellsCache(resizer);
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();
  makeEquivCellArray();
  updateErcNormalizers();

  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  auto ff_op = std::make_unique<FFResizeOperator>(sta_, local_sta_);
  ff_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  visitor->setOperator(std::move(ff_op));
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);

  if (density_map_) {
    visitor->evalContext().density_map = density_map_;
    visitor->evalContext().density_weight = density_weight_;
    visitor->evalContext().average_area = average_area_;
  }
  visitor->evalContext().average_slew = avg_out_slew_;
  visitor->evalContext().average_cap = avg_load_cap_;
  visitor->evalContext().erc_violation_weight = erc_violation_weight;
  visitor->evalContext().erc_slew_limit_scale = erc_limit_scale;
  visitor->evalContext().erc_cap_limit_scale = erc_limit_scale;
  visitor->evalContext().debug = debug_;

  local_sta_->taskArranger()->setProgressTag("LRF FF resize");
  local_sta_->taskArranger()->visitAllFFs(visitor, resizer);
  // visitAllFFs deletes per-thread visitor copies (including the original).

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelResizeFFs total time %f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
  fflush(stdout);
}

void
IncreSta::parallelResizeByArrayWithFF(rsz::Resizer *resizer, float avg_delay,
                                      float avg_power, float PT_tradeoff,
                                      float erc_violation_weight,
                                      float erc_limit_scale)
{
  // FF resize pass (silly parallel) runs first so the combinational pass
  // sees up-to-date FF Q drive / D-cap loads on shared nets.
  parallelResizeFFs(resizer, avg_delay, avg_power, PT_tradeoff,
                    erc_violation_weight, erc_limit_scale);
  parallelResizeByArray(resizer, avg_delay, avg_power, PT_tradeoff,
                        erc_violation_weight, erc_limit_scale);
}

void
IncreSta::parallelBuffering(rsz::Resizer *resizer, float PT_tradeoff,
                              float top_ratio)
{
  printf("IncreSta::parallelBuffering start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  float avg_delay = averageDelayOnCritPath();
  float avg_leakage = averageLeakage();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  local_sta_->initParallel();
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Convert fractional ratio to an absolute top_n (min 1).
  int top_n = std::max<int>(
      1, static_cast<int>(task_arranger->vertexCount() * top_ratio));
  printf("Buffering candidates: top_ratio=%.3f (→ top %d of %zu instances)\n",
         top_ratio, top_n, task_arranger->vertexCount());

  // Screen buffering candidates via sensitivity-based evaluation
  std::vector<size_t> selected = bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, top_n);
  if (selected.empty()) {
    printf("No buffering candidates found. Skipping.\n");
    return;
  }
  task_arranger->markSelectedInstances(selected);

  // Create visitor with BufferOperator only
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  visitor->setTaskArranger(task_arranger);

  auto buffer_op = std::make_unique<BufferOperator>(
      sta_, local_sta_, resizer, &visitor->evalContext());
  visitor->setOperator(std::move(buffer_op));
  visitor->init(avg_delay, avg_leakage, wns, PT_tradeoff, nullptr);
  visitor->evalContext().debug = debug_;

  // Mark all selected instances for buffer-only
  for (size_t vid : selected)
    task_arranger->vertex(vid)->move_mask_ = InstVertex::kMoveBuffer;

  auto start_buf = std::chrono::high_resolution_clock::now();
  local_sta_->taskArranger()->setProgressTag("LRF buffering");
  local_sta_->runResize(resizer, visitor);
  task_arranger->markDirty();
  auto end_buf = std::chrono::high_resolution_clock::now();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(sta::MinMax::max());
  double wns_after = sta_->worstSlack(sta::MinMax::max());
  printf("After buffering, TNS: %.4f ps, WNS: %.4f ps\n",
         tns_after * 1e12, wns_after * 1e12);
  printf("  buffering time: %.3f s\n",
         std::chrono::duration<double>(end_buf - start_buf).count());

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelBuffering total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

void
IncreSta::parallelBufferingSdp(rsz::Resizer *resizer, float PT_tradeoff,
                                float top_ratio, float erc_violation_weight,
                                float erc_limit_scale)
{
  printf("IncreSta::parallelBufferingSdp start (LRF slack-DP rebuffering)\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  float avg_delay = averageDelayOnCritPath();
  float avg_leakage = averageLeakage();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  local_sta_->initParallel();
  TaskArranger *task_arranger = local_sta_->taskArranger();

  int top_n = std::max<int>(
      1, static_cast<int>(task_arranger->vertexCount() * top_ratio));
  printf("SDP buffering candidates: top_ratio=%.3f (→ top %d of %zu instances)\n",
         top_ratio, top_n, task_arranger->vertexCount());

  std::vector<size_t> selected = bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, top_n);
  if (selected.empty()) {
    printf("No buffering candidates found. Skipping.\n");
    return;
  }
  task_arranger->markSelectedInstances(selected);

  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  visitor->setTaskArranger(task_arranger);

  auto buffer_op = std::make_unique<BufferSdpOperator>(
      sta_, local_sta_, resizer, &visitor->evalContext());
  visitor->setOperator(std::move(buffer_op));
  visitor->init(avg_delay, avg_leakage, wns, PT_tradeoff, nullptr);
  visitor->evalContext().average_slew = avg_out_slew_;
  visitor->evalContext().average_cap = avg_load_cap_;
  visitor->evalContext().erc_violation_weight = erc_violation_weight;
  visitor->evalContext().erc_slew_limit_scale = erc_limit_scale;
  visitor->evalContext().erc_cap_limit_scale = erc_limit_scale;
  visitor->evalContext().debug = debug_;

  for (size_t vid : selected)
    task_arranger->vertex(vid)->move_mask_ = InstVertex::kMoveBuffer;

  auto start_buf = std::chrono::high_resolution_clock::now();
  local_sta_->taskArranger()->setProgressTag("LRF buffering (SDP)");
  local_sta_->runResize(resizer, visitor);
  task_arranger->markDirty();
  auto end_buf = std::chrono::high_resolution_clock::now();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(sta::MinMax::max());
  double wns_after = sta_->worstSlack(sta::MinMax::max());
  printf("After SDP buffering: TNS: %.4f ps, WNS: %.4f ps\n",
         tns_after * 1e12, wns_after * 1e12);
  printf("  buffering time: %.3f s\n",
         std::chrono::duration<double>(end_buf - start_buf).count());

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelBufferingSdp total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

void
IncreSta::parallelBufferingRsz(rsz::Resizer *resizer, float PT_tradeoff,
                                int top_n)
{
  printf("IncreSta::parallelBufferingRsz start (rsz-style rebuffering)\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  float avg_delay = averageDelayOnCritPath();
  float avg_leakage = averageLeakage();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  local_sta_->initParallel();
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Screen buffering candidates via sensitivity-based evaluation
  std::vector<size_t> selected = bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, top_n);
  if (selected.empty()) {
    printf("No buffering candidates found. Skipping.\n");
    return;
  }
  task_arranger->markSelectedInstances(selected);

  // Create visitor with BufferRszOperator
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  visitor->setTaskArranger(task_arranger);

  LrRebuffer::initGlobalPreamble(sta_, resizer);
  sta_->findRequireds();  // pre-compute requireds for annotateLoadSlacks

  auto buffer_rsz_op = std::make_unique<BufferRszOperator>(
      sta_, local_sta_, resizer, &visitor->evalContext());
  visitor->setOperator(std::move(buffer_rsz_op));
  visitor->init(avg_delay, avg_leakage, wns, PT_tradeoff, nullptr);
  visitor->evalContext().debug = debug_;

  // Mark all selected instances for buffer-only
  for (size_t vid : selected)
    task_arranger->vertex(vid)->move_mask_ = InstVertex::kMoveBuffer;

  auto start_buf = std::chrono::high_resolution_clock::now();
  local_sta_->taskArranger()->setProgressTag("LRF buffering");
  local_sta_->runResize(resizer, visitor);
  task_arranger->markDirty();
  auto end_buf = std::chrono::high_resolution_clock::now();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(sta::MinMax::max());
  double wns_after = sta_->worstSlack(sta::MinMax::max());
  printf("After rsz buffering: TNS: %.4f ps, WNS: %.4f ps\n",
         tns_after * 1e12, wns_after * 1e12);
  printf("  buffering time: %.3f s\n",
         std::chrono::duration<double>(end_buf - start_buf).count());

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelBufferingRsz total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

void
IncreSta::probeRszBnet(rsz::Resizer *resizer, float PT_tradeoff, int top_n)
{
  printf("IncreSta::probeRszBnet start\n");

  float avg_delay = averageDelayOnCritPath();
  float avg_leakage = averageLeakage();

  local_sta_->initParallel();
  if (!swap_cell_presaved_)
    makeSwappableCellsCache(resizer);

  std::vector<size_t> selected = bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, top_n);
  if (selected.empty()) {
    printf("No buffering candidates. Done.\n");
    return;
  }

  TaskArranger *task_arranger = local_sta_->taskArranger();
  sta::Network *network = network_;
  sta::Graph *graph = sta_->graph();

  EvalContext probe_ctx;
  probe_ctx.arc_delay_calc = sta_->arcDelayCalc();
  probe_ctx.average_delay = avg_delay;
  probe_ctx.average_leakage = avg_leakage;
  probe_ctx.PT_tradeoff = PT_tradeoff;
  std::map<std::string, double> rt;
  probe_ctx.runtime_map = &rt;

  TestRebuffer lr_rebuffer(resizer, local_sta_, &probe_ctx);
  TestRebuffer::initGlobalPreamble(sta_, resizer);
  lr_rebuffer.init();

  sta_->findRequireds();

  int probed = 0;
  for (size_t vid : selected) {
    InstVertex *iv = task_arranger->vertex(vid);
    sta::Instance *inst = iv->inst();

    sta::InstancePinIterator *iter = network->pinIterator(inst);
    while (iter->hasNext()) {
      sta::Pin *pin = iter->next();
      if (!network->isDriver(pin)) continue;
      sta::Vertex *vtx = graph->pinDrvrVertex(pin);
      if (!vtx || sta_->vertexSlack(vtx, sta::MinMax::max()) >= 0.0f) continue;

      // Build PtGraph for this instance (includes parasitic init)
      PtGraph *pt_graph = local_sta_->makePtGraph(inst, true);
      if (!pt_graph) continue;
      probe_ctx.pt_graph = pt_graph;

      // Find driver PtVertex
      PtVertex *drvr_pv = nullptr;
      for (size_t i = 0; i < pt_graph->vertexCount(); i++) {
        PtVertex &pv = pt_graph->ptVertex(i);
        if (pv.vertex() && pv.type() == PtVertexType::RefOutput
            && pv.vertex()->pin() == pin) {
          drvr_pv = &pv;
          break;
        }
      }
      if (!drvr_pv) continue;

      lr_rebuffer.probeRszBnetWithLocalEval(pin, *drvr_pv);
      probed++;
    }
    delete iter;
  }
  printf("probeRszBnet: probed %d pins\n", probed);
}

std::vector<size_t>
IncreSta::precedingResizeCheck(rsz::Resizer *resizer, float avg_delay,
                                 float avg_power, float PT_tradeoff,
                                 float top_ratio)
{
  printf("IncreSta::precedingResizeCheck start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  // Ensure prerequisites
  initDelayDiff();        // populate sta::Edge.delay_diffs_ on first call
  local_sta_->initParallel();
  if (!equiv_cell_array_built_)
    makeEquivCellArray();
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();
  updateErcNormalizers();

  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Pre-allocate results with 1:1 mapping to TaskArranger vertices.
  std::vector<ResizeBenefit> results(task_arranger->vertexCount());
  for (size_t i = 0; i < results.size(); i++)
    results[i] = {nullptr, -std::numeric_limits<float>::infinity(), i};

  // Create ParallelVisitor with ResizePrecheckOperator
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);

  auto precheck_op = std::make_unique<ResizePrecheckOperator>(sta_, local_sta_);
  precheck_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  precheck_op->setInstInfoMap(&inst_info_map_);
  visitor->setOperator(std::move(precheck_op));

  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);
  visitor->setPrecheckResults(&results);

  // Run embarrassingly parallel precheck (no conflict graph)
  task_arranger->visitAll(visitor);

  // Sort by cost_change descending
  std::sort(results.begin(), results.end(),
            [](const ResizeBenefit &a, const ResizeBenefit &b) {
              return a.cost_change > b.cost_change;
            });

  // Count positive before filtering (for logging)
  size_t positive_count = 0;
  for (const auto &r : results) {
    if (r.cost_change > 0.0f)
      positive_count++;
  }
  printf("Precheck: %zu/%zu instances have positive benefit\n",
         positive_count, results.size());

  // Filter: keep top_ratio fraction, remove non-positive
  size_t top_n = static_cast<size_t>(results.size() * top_ratio);
  results.resize(top_n);
  results.erase(
    std::remove_if(results.begin(), results.end(),
                   [](const ResizeBenefit &b) { return b.cost_change <= 0.0f; }),
    results.end());

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;

  // Extract vertex indices
  std::vector<size_t> selected_ids;
  selected_ids.reserve(results.size());

  // Print top results (cap at 20 for display)
  size_t print_n = std::min(results.size(), static_cast<size_t>(20));
  printf("Selected %zu instances (top_ratio=%.2f), top %zu:\n",
         results.size(), top_ratio, print_n);
  for (size_t i = 0; i < results.size(); i++) {
    selected_ids.push_back(results[i].vertex_idx);
    if (i < print_n) {
      printf("  [%zu] %s  cost_change=%.6f  vertex_idx=%zu\n", i,
             network_->pathName(results[i].inst),
             results[i].cost_change, results[i].vertex_idx);
    }
  }
  printf("precedingResizeCheck total time: %f s\n", diff_total.count());
  fflush(stdout);

  pruning_control_.last_selected_count = static_cast<int>(selected_ids.size());
  return selected_ids;
}

void
IncreSta::parallelResizeByArrayWithPrecheck(
    rsz::Resizer *resizer, float avg_delay, float avg_power,
    float PT_tradeoff, float top_ratio, float erc_violation_weight,
    float erc_limit_scale, bool resize_ff)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  // Ensure swappable cells cache and leakage data are populated
  if (!swap_cell_presaved_) {
    makeSwappableCellsCache(resizer);
  }
  if (!swap_cell_leakage_presaved_) {
    preSaveLibCellLeakage();
  }

  // FF parallel resize first (silly-parallel sequential sweep). The precheck
  // below filters combinational vertices only, so FFs need their own pass.
  if (resize_ff) {
    parallelResizeFFs(resizer, avg_delay, avg_power, PT_tradeoff,
                      erc_violation_weight, erc_limit_scale);
    // Re-sync timing so precheck sees the new FF cell choices.
    sta_->updateTiming(true);
    sta_->findRequireds();
  }

  // Phase 1: Precheck — returns filtered top instances sorted by benefit
  // Use adaptive ratio when active (after K detected), otherwise caller's top_ratio
  float effective_ratio = (pruning_control_.adaptive_top_ratio > 0.0f)
      ? pruning_control_.adaptive_top_ratio
      : top_ratio;

  auto t_precheck_start = std::chrono::high_resolution_clock::now();
  auto vertex_ids = precedingResizeCheck(resizer, avg_delay, avg_power,
                                           PT_tradeoff, effective_ratio);
  auto t_precheck_end = std::chrono::high_resolution_clock::now();
  double precheck_sec = std::chrono::duration<double>(t_precheck_end - t_precheck_start).count();

  // Phase 2: Mark selected instances
  TaskArranger *task_arranger = local_sta_->taskArranger();
  task_arranger->markSelectedInstances(vertex_ids);

  // Phase 3: Resize (only selected instances visited in runTask)
  Slack wns = sta_->worstSlack(MinMax::max());
  auto t_resize_start = std::chrono::high_resolution_clock::now();

  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  auto resize_op = std::make_unique<ResizeOperator>(sta_, local_sta_);
  resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  resize_op->setPruningControl(&pruning_control_);
  visitor->setOperator(std::move(resize_op));
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);
  if (density_map_) {
    visitor->evalContext().density_map = density_map_;
    visitor->evalContext().density_weight = density_weight_;
    visitor->evalContext().average_area = average_area_;
  }
  visitor->evalContext().average_slew = avg_out_slew_;
  visitor->evalContext().average_cap = avg_load_cap_;
  visitor->evalContext().erc_violation_weight = erc_violation_weight;
  visitor->evalContext().erc_slew_limit_scale = erc_limit_scale;
  visitor->evalContext().erc_cap_limit_scale = erc_limit_scale;
  visitor->evalContext().debug = debug_;

  local_sta_->taskArranger()->setProgressTag("LRF precheck");
  local_sta_->runResize(resizer, visitor);
  auto t_resize_end = std::chrono::high_resolution_clock::now();
  double resize_sec = std::chrono::duration<double>(t_resize_end - t_resize_start).count();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns = sta_->totalNegativeSlack(MinMax::max());
  double wns_after = sta_->worstSlack(MinMax::max());
  printf("After parallel LR resize with precheck, TNS: %e, WNS: %e\n", tns, wns_after);
  printf("  precheck time: %.3f s, resize time: %.3f s, ratio: %.2f\n",
         precheck_sec, resize_sec,
         resize_sec > 0 ? precheck_sec / resize_sec : 0.0);

  // Pruning: update iteration counter and detect K
  pruning_control_.iteration++;
  printf("Pruning: iteration %d, enabled=%d, K=%d\n",
         pruning_control_.iteration, pruning_control_.enabled, pruning_control_.K);

  // Adaptive instance filtering: store stats for caller to decide activation
  {
    TaskArranger *ta = local_sta_->taskArranger();
    pruning_control_.last_change_count = ta->lastChangeCount();
    printf("InstanceFilter: selected=%d, changed=%d, total=%d, adaptive_ratio=%.4f\n",
           pruning_control_.last_selected_count,
           pruning_control_.last_change_count,
           static_cast<int>(ta->vertexCount()),
           pruning_control_.adaptive_top_ratio);
    fflush(stdout);
  }

  if (cps_enabled_) {
    ParallelVisitor *cp_visitor = new ParallelVisitor(sta_, local_sta_, resizer);
    std::unique_ptr<ResizeOperator> cp_resize_op =
        std::make_unique<ResizeOperator>(sta_, local_sta_);
    cp_resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
    cp_resize_op->setPruningControl(&pruning_control_);
    cp_visitor->setOperator(std::move(cp_resize_op));
    cp_visitor->init(avg_delay, avg_power, wns_after, PT_tradeoff, &inst_info_map_);
    if (density_map_) {
      cp_visitor->evalContext().density_map = density_map_;
      cp_visitor->evalContext().density_weight = density_weight_;
      cp_visitor->evalContext().average_area = average_area_;
    }
    cp_visitor->evalContext().average_slew = avg_out_slew_;
    cp_visitor->evalContext().average_cap = avg_load_cap_;
    cp_visitor->evalContext().erc_violation_weight = erc_violation_weight;
    cp_visitor->evalContext().erc_slew_limit_scale = erc_limit_scale;
    cp_visitor->evalContext().erc_cap_limit_scale = erc_limit_scale;
    cp_visitor->evalContext().debug = debug_;
    std::chrono::high_resolution_clock::time_point start_cps =
        std::chrono::high_resolution_clock::now();
    LrSizer lr_sizer(sta_, lr_helper_, cp_visitor);
    lr_sizer.criticalPathSizing();
    std::chrono::high_resolution_clock::time_point end_cps =
        std::chrono::high_resolution_clock::now();
    printf("After critical path sizing, TNS: %e, WNS: %e\n",
           sta_->totalNegativeSlack(MinMax::max()),
           (double)sta_->worstSlack(MinMax::max()));
    printf("critical path sizing time: %f s\n",
           std::chrono::duration<double>(end_cps - start_cps).count());
    delete cp_visitor;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("parallelResizeByArrayWithPrecheck total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

void
IncreSta::activateInstanceFilter(float max_ratio)
{
  TaskArranger *ta = local_sta_->taskArranger();
  int total = static_cast<int>(ta->vertexCount());
  int change_count = pruning_control_.last_change_count;
  int target_n = static_cast<int>(
      change_count * pruning_control_.instance_filter_multiplier);
  target_n = std::max(target_n, 50);  // absolute floor
  float new_ratio = (total > 0)
      ? static_cast<float>(target_n) / total
      : max_ratio;
  pruning_control_.adaptive_top_ratio = std::min(new_ratio, max_ratio);
  printf("InstanceFilter ACTIVATED: change_count=%d, target_n=%d, "
         "adaptive_ratio=%.4f (max=%.4f)\n",
         change_count, target_n, pruning_control_.adaptive_top_ratio, max_ratio);
  fflush(stdout);
}

// Screen buffering candidates: collect gates with negative late slack,
// sort by Cout/Cin ratio descending, return top_n vertex indices.
std::vector<size_t>
IncreSta::bufferingVerticesCandidate(int top_n)
{
  TaskArranger *task_arranger = local_sta_->taskArranger();
  const size_t total = task_arranger->vertexCount();
  sta::Graph *graph = sta_->graph();
  const sta::Network *network = sta_->network();
  sta::GraphDelayCalc *dcalc = sta_->graphDelayCalc();
  const sta::Corner *corner = sta_->cmdCorner();
  const sta::DcalcAnalysisPt *dcalc_ap
      = corner->findDcalcAnalysisPt(sta::MinMax::max());

  struct BufferCandidate {
    size_t vertex_id;
    float cout_cin_ratio;
  };
  std::vector<BufferCandidate> candidates;
  candidates.reserve(total);

  for (size_t i = 0; i < total; i++) {
    InstVertex *iv = task_arranger->vertex(i);
    if (iv->type() != VertexType::COMBINATIONAL)
      continue;

    sta::Instance *inst = iv->inst();
    sta::LibertyCell *lib_cell = network->libertyCell(inst);
    if (!lib_cell)
      continue;

    // Find worst output slack and output load cap
    float worst_slack = std::numeric_limits<float>::max();
    float cout = 0.0f;
    sta::InstancePinIterator *pin_iter = network->pinIterator(inst);
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      if (network->direction(pin)->isOutput()) {
        sta::Vertex *vertex, *bidirect;
        graph->pinVertices(pin, vertex, bidirect);
        if (vertex) {
          sta::Slack slack = sta_->vertexSlack(vertex, sta::MinMax::max());
          if (slack < worst_slack)
            worst_slack = slack;
          float load = dcalc->loadCap(pin, dcalc_ap);
          if (load > cout)
            cout = load;
        }
      }
    }
    delete pin_iter;

    // Skip gates with non-negative slack
    if (worst_slack >= 0.0f)
      continue;

    // Compute average input capacitance
    float cin = 0.0f;
    int nin = 0;
    sta::LibertyCellPortIterator port_iter(lib_cell);
    while (port_iter.hasNext()) {
      const sta::LibertyPort *port = port_iter.next();
      if (port->direction() == sta::PortDirection::input()) {
        cin += port->capacitance();
        nin++;
      }
    }
    if (nin > 0)
      cin /= nin;

    float ratio = (cin > 0.0f) ? cout / cin : 0.0f;
    candidates.push_back({i, ratio});
  }

  // Sort by Cout/Cin ratio descending
  std::sort(candidates.begin(), candidates.end(),
            [](const BufferCandidate &a, const BufferCandidate &b) {
              return a.cout_cin_ratio > b.cout_cin_ratio;
            });

  // Take top_n
  size_t keep = std::min(static_cast<size_t>(top_n), candidates.size());
  std::vector<size_t> selected;
  selected.reserve(keep);
  for (size_t i = 0; i < keep; i++)
    selected.push_back(candidates[i].vertex_id);

  printf("bufferingVerticesCandidate: %zu negative-slack gates, selected top %zu by Cout/Cin\n",
         candidates.size(), keep);
  if (!candidates.empty()) {
    printf("  Cout/Cin ratio range: [%.2f, %.2f]\n",
           candidates.back().cout_cin_ratio, candidates.front().cout_cin_ratio);
  }

  return selected;
}

std::vector<size_t>
IncreSta::bufferingVerticesCandidateBySensitivity(
    rsz::Resizer *resizer, float avg_delay, float avg_leakage, int top_n)
{
  printf("IncreSta::bufferingVerticesCandidateBySensitivity start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Initialize LrRebuffer global preamble
  LrRebuffer::initGlobalPreamble(sta_, resizer);

  // Pre-allocate results
  std::vector<ResizeBenefit> results(task_arranger->vertexCount());
  for (size_t i = 0; i < results.size(); i++)
    results[i] = {nullptr, -std::numeric_limits<float>::infinity(), i};

  // Create ParallelVisitor with BufferSensitivityOperator.
  // No task_arranger needed: dispatch infers buffer route from operator config.
  ParallelVisitor *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  std::unique_ptr<BufferSensitivityOperator> sens_op =
      std::make_unique<BufferSensitivityOperator>(
          sta_, local_sta_, resizer, &visitor->evalContext());
  visitor->setOperator(std::move(sens_op));
  visitor->init(avg_delay, avg_leakage, wns, 100.0f, nullptr);
  visitor->evalContext().bakoglu_k = bakoglu_k_;
  visitor->evalContext().debug = debug_;
  visitor->setPrecheckResults(&results);

  // Ensure required times are computed before parallel dispatch
  // (vertexSlack inside evaluate may trigger findRequireds which is not thread-safe)
  sta_->findRequireds();

  // Dispatch all instances in parallel (no conflict graph)
  task_arranger->visitAll(visitor);

  // Sort by sensitivity descending
  std::sort(results.begin(), results.end(),
            [](const ResizeBenefit &a, const ResizeBenefit &b) {
              return a.cost_change > b.cost_change;
            });

  // Filter non-positive sensitivity
  results.erase(
      std::remove_if(results.begin(), results.end(),
                     [](const ResizeBenefit &b) { return b.cost_change <= 0.0f; }),
      results.end());

  // Take top_n
  size_t keep = std::min(static_cast<size_t>(top_n), results.size());
  results.resize(keep);

  auto end_total = std::chrono::high_resolution_clock::now();
  double total_sec = std::chrono::duration<double>(end_total - start_total).count();

  std::vector<size_t> selected_ids;
  selected_ids.reserve(keep);
  size_t print_n = std::min(keep, static_cast<size_t>(20));
  printf("Sensitivity screening: %zu instances with positive sensitivity, "
         "selected top %zu (%.3f s)\n", results.size(), keep, total_sec);
  for (size_t i = 0; i < keep; i++) {
    selected_ids.push_back(results[i].vertex_idx);
    if (i < print_n) {
      printf("  [%zu] %s  sensitivity=%.6e  vertex_idx=%zu\n", i,
             network_->pathName(results[i].inst),
             results[i].cost_change, results[i].vertex_idx);
    }
  }

  return selected_ids;
}

void
IncreSta::parallelResizeAndBuffering(rsz::Resizer *resizer, float avg_delay,
                                       float avg_power, float PT_tradeoff,
                                       float buffer_top_ratio)
{
  printf("IncreSta::parallelResizeAndBuffering start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());

  if (!swap_cell_presaved_)
    makeSwappableCellsCache(resizer);
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();
  makeEquivCellArray();
  updateErcNormalizers();

  // Screen buffering candidates by sensitivity, top buffer_top_ratio fraction
  // of total instances (design-size scaled).
  TaskArranger *task_arranger = local_sta_->taskArranger();
  int buffer_top_n = std::max<int>(
      1, static_cast<int>(task_arranger->vertexCount() * buffer_top_ratio));
  printf("Buffering candidates: top_ratio=%.3f (→ top %d of %zu instances)\n",
         buffer_top_ratio, buffer_top_n, task_arranger->vertexCount());
  std::vector<size_t> buf_candidates = bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_power, buffer_top_n);
  printf("Buffer candidates (sensitivity): %zu (of %zu total)\n",
         buf_candidates.size(), task_arranger->vertexCount());

  // Set move masks: resize + buffer, or buffer-only
  if (buffer_only_mode_) {
    // Buffer-only: only buffer candidates get kMoveBuffer, no resize
    for (size_t i = 0; i < task_arranger->vertexCount(); i++)
      task_arranger->vertex(i)->move_mask_ = 0;
    for (size_t vid : buf_candidates)
      task_arranger->vertex(vid)->move_mask_ = InstVertex::kMoveBuffer;
    printf("Buffer-only mode: %zu instances with kMoveBuffer\n", buf_candidates.size());
  } else {
    // Combined: all instances get resize; buffer candidates also get buffer bit
    for (size_t i = 0; i < task_arranger->vertexCount(); i++)
      task_arranger->vertex(i)->move_mask_ = InstVertex::kMoveResize;
    for (size_t vid : buf_candidates)
      task_arranger->vertex(vid)->move_mask_ |= InstVertex::kMoveBuffer;
  }

  // Initialize global STA/Resizer state for buffering (serial preamble)
  LrRebuffer::initGlobalPreamble(sta_, resizer);

  // Create ParallelVisitor with CombinedOperator
  auto start_resize = std::chrono::high_resolution_clock::now();
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  visitor->setTaskArranger(task_arranger);

  auto combined_op = std::make_unique<CombinedOperator>(
      sta_, local_sta_, resizer, &visitor->evalContext());
  combined_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  visitor->setOperator(std::move(combined_op));

  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);
  visitor->evalContext().debug = debug_;

  local_sta_->taskArranger()->setProgressTag("LRF resize+buf");
  local_sta_->runResize(resizer, visitor);

  // If any buffers were inserted, mark graph dirty for next pass
  task_arranger->markDirty();

  auto end_resize = std::chrono::high_resolution_clock::now();
  printf("parallelResizeAndBuffering pass time: %.3f s\n",
         std::chrono::duration<double>(end_resize - start_resize).count());

  sta_->updateTiming(true);
  sta_->findRequireds();

  double tns_after = sta_->totalNegativeSlack(MinMax::max());
  double wns_after = sta_->worstSlack(MinMax::max());
  printf("After parallelResizeAndBuffering, TNS: %e, WNS: %e\n",
         tns_after, wns_after);

  if (cps_enabled_) {
    ParallelVisitor *cp_visitor = new ParallelVisitor(sta_, local_sta_, resizer);
    std::unique_ptr<ResizeOperator> cp_resize_op =
        std::make_unique<ResizeOperator>(sta_, local_sta_);
    cp_resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
    cp_resize_op->setPruningControl(&pruning_control_);
    cp_visitor->setOperator(std::move(cp_resize_op));
    cp_visitor->init(avg_delay, avg_power, wns_after, PT_tradeoff, &inst_info_map_);
    std::chrono::high_resolution_clock::time_point start_cps =
        std::chrono::high_resolution_clock::now();
    LrSizer lr_sizer(sta_, lr_helper_, cp_visitor);
    lr_sizer.criticalPathSizing();
    std::chrono::high_resolution_clock::time_point end_cps =
        std::chrono::high_resolution_clock::now();
    printf("After critical path sizing, TNS: %e, WNS: %e\n",
           sta_->totalNegativeSlack(MinMax::max()),
           (double)sta_->worstSlack(MinMax::max()));
    printf("critical path sizing time: %f s\n",
           std::chrono::duration<double>(end_cps - start_cps).count());
    delete cp_visitor;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelResizeAndBuffering total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

} // namespace lrf

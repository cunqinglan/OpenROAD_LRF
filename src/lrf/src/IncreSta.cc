#include "lrf/IncreSta.hh"
#include "LocalSta.hh"
#include "LrHelper.hh"
#include "sta/Liberty.hh"
#include "sta/Path.hh"
#include "sta/Corner.hh"
#include "sta/PathExpanded.hh"
#include "sta/Search.hh"
#include "sta/EquivCells.hh"
#include "power/Power.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/PortDirection.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/TimingRole.hh"
#include "lrf/LrfClass.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "TaskArranger.hh"
#include "ParallelVisitor.hh"
#include "NetlistTransformation.hh"
#include "LrRebuffer.hh"
#include "LrRebufferV2.hh"
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
    // printf("IncreSta: setting STA thread count to %zu\n", thread_count);
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
    lr_helper_ = new LRHelper(sta_);
}

InstanceSeq &
IncreSta::getSortedInstances()
{ 
  sta_->ensureLevelized();
  InstanceSet instance_visited(network_);
  
  VertexSeq &vertices = lr_helper_->ensureSorted(sta_);
  // printf("Size of sorted vertices: %zu\n", vertices.size());
  // fflush(stdout);
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
IncreSta::lmUpdate()
{
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());
  if (wns >= 0.0) {
    lr_helper_->setMode("power");
    // printf("All timing constraints are met (WNS %e), switching to power optimization mode\n", wns);
  }

  const bool use_parallel = (thread_count_ > 1 && dispatch_queue_);

  if (projected_) {
    if (use_parallel)
      lr_helper_->parallelUpdateAllEdgeLms(sta_);
    else
      lr_helper_->updateAllEdgeLms(sta_);

    if (use_parallel)
      lr_helper_->parallelKKTProjection(sta_);
    else
      lr_helper_->KKTProjection(sta_);
  } else {
    bool kkt_satisfied = use_parallel
      ? lr_helper_->parallelKKTProjection(sta_)
      : lr_helper_->KKTProjection(sta_);
    if (kkt_satisfied)
      projected_ = true;
  }
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
              // printf("  [Warning: unexpected ranking] %s vs %s: cap wins (%.2e vs %.2e)",
                // x->name(), y->name(), cx, cy);
            } else if (rx > ry && cx < cy) {
              // printf("  [Warning: unexpected ranking] %s vs %s: cap wins (%.2e vs %.2e)",
                // x->name(), y->name(), cx, cy);
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
        // printf("EquivCellArrayGroup: %s (%zu cells, %zu cols, %zu rows)\n",
          // group->front() ? group->front()->name() : "<null>",
          // group->size(),
          // prefixes.size(),
          // max_rows);
        // Print header row (prefixes)
        std::string header = "  col:";
        for (size_t c = 0; c < prefixes.size(); c++) {
          header += (c == 0 ? " " : " | ");
          header += prefixes[c];
        }
        // printf("%s\n", header.c_str());
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
          // printf("%s\n", line.c_str());
        }
        // printf("  --\n");
      }
    }
  }
  delete lib_iter;
  equiv_cell_array_built_ = true;
  if (verbose) {
    // printf("makeEquivCellArray: %zu rows, %zu cells in pos_map\n",
           // equiv_cell_array_.size(), equiv_cell_pos_map_.size());
    // fflush(stdout);
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
IncreSta::parallelResize(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff)
{
  auto start_total = std::chrono::high_resolution_clock::now();
  Slack wns = sta_->worstSlack(MinMax::max());

  if (!swap_cell_presaved_) {
    auto start_cache = std::chrono::high_resolution_clock::now();
    makeSwappableCellsCache(resizer);
    auto end_cache = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_cache = end_cache - start_cache;
    // printf("makeSwappableCellsCache took %f s\n", diff_cache.count());
  }
  if (!swap_cell_leakage_presaved_) {
    auto start_presave = std::chrono::high_resolution_clock::now();
    preSaveLibCellLeakage();
    auto end_presave = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_presave = end_presave - start_presave;
    // printf("preSaveLibCellLeakage took %f s\n", diff_presave.count());
  }

  // float average_delay = averageDelayOnCritPath();
  // float average_power = averageLeakage();
  // printf("Average delay: %f, average power: %f\n", avg_delay * 1e12, avg_power * 1e9);
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &swappable_cells_cache_, &inst_info_map_);

  auto start_resize = std::chrono::high_resolution_clock::now();
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;
  // printf("local_sta_->runResize took %f s\n", diff_resize.count());

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  // printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}

void 
IncreSta::parallelResizeV1(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  // We first create a serials of instance visitors
  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  if (parallel_lib_data_ == nullptr) {
    auto start_pld = std::chrono::high_resolution_clock::now();
    makeParallelLibData(resizer, task_arranger);
    auto end_pld = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_pld = end_pld - start_pld;
    // printf("makeParallelLibData took %f s\n", diff_pld.count());
  }

  // float average_delay = averageDelayOnCritPath();
  // float average_power = averageLeakage();
  // printf("Average delay: %f, average power: %f\n", avg_delay * 1e12, avg_power * 1e9);
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, parallel_lib_data_);

  auto start_resize = std::chrono::high_resolution_clock::now();
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;
  // printf("local_sta_->runResize took %f s\n", diff_resize.count());

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  // printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}

void
IncreSta::setMaxResizeNum(size_t max_resize_num)
{
  local_sta_->taskArranger()->setMaxResizeNum(max_resize_num);
}

void 
IncreSta::parallelResizeAdaptive(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff)
{
  // printf("IncreSta::parallelResizeAdaptive start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  // We first create a serials of instance visitors
  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  if (!swap_cell_presaved_) {
    auto start_cache = std::chrono::high_resolution_clock::now();
    makeSwappableCellsCache(resizer);
    auto end_cache = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_cache = end_cache - start_cache;
    // printf("makeSwappableCellsCache took %f s\n", diff_cache.count());
  }
  if (!swap_cell_leakage_presaved_) {
    auto start_presave = std::chrono::high_resolution_clock::now();
    preSaveLibCellLeakage();
    auto end_presave = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_presave = end_presave - start_presave;
    // printf("preSaveLibCellLeakage took %f s\n", diff_presave.count());
  }

  auto start_resize = std::chrono::high_resolution_clock::now();
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, 
      &swappable_cells_cache_, &inst_info_map_);
  visitor->setMoveType(MoveType::Resizing);
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;

  // Use distinct variable names to avoid shadowing Slack
  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after_resize = sta_->totalNegativeSlack(MinMax::max());
  double wns_after_resize = sta_->worstSlack(MinMax::max());
  // printf("After parallel LR resize, TNS: %e, WNS: %e\n", tns_after_resize, wns_after_resize);
  // printf("parallel resize time: %f s\n", diff_resize.count());

  if (isPowerOptimizationMode()) {
    ParallelLrVisitor *critical_path_visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
    critical_path_visitor->init(avg_delay, avg_power, wns_after_resize,
        PT_tradeoff, &swappable_cells_cache_, &inst_info_map_);
    critical_path_visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
    critical_path_visitor->setMoveType(MoveType::Resizing);

    // Time the critical-path sizing phase
    auto start_cps = std::chrono::high_resolution_clock::now();
    LrSizer lr_sizer(sta_, lr_helper_, critical_path_visitor);
    lr_sizer.criticalPathSizing();
    auto end_cps = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_cps = end_cps - start_cps;

    double tns_after_cps = sta_->totalNegativeSlack(MinMax::max());
    double wns_after_cps = sta_->worstSlack(MinMax::max());
    // printf("After critical path sizing, TNS: %e, WNS: %e\n", tns_after_cps, wns_after_cps);
    // printf("critical path sizing time: %f s\n", diff_cps.count());
    delete critical_path_visitor;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  // printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}

void
IncreSta::parallelResizeByArray(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());

  if (!swap_cell_presaved_) {
    makeSwappableCellsCache(resizer);
  }
  if (!swap_cell_leakage_presaved_) {
    preSaveLibCellLeakage();
  }
  makeEquivCellArray();

  auto start_resize = std::chrono::high_resolution_clock::now();
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff,
      &swappable_cells_cache_, &inst_info_map_);
  visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  visitor->setPruningControl(&pruning_control_);
  visitor->setMoveType(MoveType::Resizing);
  // visitor ownership is transferred to TaskArranger::visitOrdered.
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after_resize = sta_->totalNegativeSlack(MinMax::max());
  double wns_after_resize = sta_->worstSlack(MinMax::max());
  // printf("After parallel LR resize, TNS: %e, WNS: %e\n", tns_after_resize, wns_after_resize);
  // printf("parallel resize time: %f s\n", diff_resize.count());

  // --- Pruning: update iteration counter and detect K ---
  pruning_control_.iteration++;
  printf("Pruning: iteration %d, enabled=%d, K=%d\n",
         pruning_control_.iteration, pruning_control_.enabled, pruning_control_.K);
  fflush(stdout);

  if (isPowerOptimizationMode()) {
    ParallelLrVisitor *critical_path_visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
    critical_path_visitor->init(avg_delay, avg_power, wns_after_resize,
        PT_tradeoff, &swappable_cells_cache_, &inst_info_map_);
    critical_path_visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
    critical_path_visitor->setMoveType(MoveType::Resizing);

    // Time the critical-path sizing phase
    auto start_cps = std::chrono::high_resolution_clock::now();
    LrSizer lr_sizer(sta_, lr_helper_, critical_path_visitor);
    lr_sizer.criticalPathSizing();
    auto end_cps = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_cps = end_cps - start_cps;

    double tns_after_cps = sta_->totalNegativeSlack(MinMax::max());
    double wns_after_cps = sta_->worstSlack(MinMax::max());
    // printf("After critical path sizing, TNS: %e, WNS: %e\n", tns_after_cps, wns_after_cps);
    // printf("critical path sizing time: %f s\n", diff_cps.count());
    delete critical_path_visitor;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  // printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}

void
IncreSta::parallelResizeByArrayV2(rsz::Resizer *resizer, float avg_delay,
                                  float avg_power, float PT_tradeoff)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  if (!swap_cell_presaved_)
    makeSwappableCellsCache(resizer);
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();
  makeEquivCellArray();

  auto start_resize = std::chrono::high_resolution_clock::now();

  // Create new-framework visitor with ResizeOperator
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);

  auto resize_op = std::make_unique<ResizeOperator>(sta_, local_sta_);
  resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  visitor->setOperator(std::move(resize_op));

  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);

  local_sta_->runResize(resizer, visitor);

  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(sta::MinMax::max());
  double wns_after = sta_->worstSlack(sta::MinMax::max());
  printf("After V2 parallel resize, TNS: %e, WNS: %e\n", tns_after, wns_after);
  printf("parallel resize time: %f s\n", diff_resize.count());

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  printf("IncreSta::parallelResizeV2 total time %f s\n", diff_total.count());
}

void
IncreSta::parallelBufferingV2(rsz::Resizer *resizer, float PT_tradeoff,
                              int top_n)
{
  printf("IncreSta::parallelBufferingV2 start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  float avg_delay = averageDelayOnCritPath();
  float avg_leakage = averageLeakage();
  sta::Slack wns = sta_->worstSlack(sta::MinMax::max());

  local_sta_->initParallel();
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Screen buffering candidates via sensitivity-based evaluation
  std::vector<size_t> selected = bufferingVerticesCandidateBySensitivityV2(
      resizer, avg_delay, avg_leakage, top_n);
  if (selected.empty()) {
    printf("No buffering candidates found. Skipping.\n");
    return;
  }
  task_arranger->markSelectedInstances(selected);

  // Create V2 visitor with BufferOperator only
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  visitor->setTaskArranger(task_arranger);

  auto buffer_op = std::make_unique<BufferOperator>(
      sta_, local_sta_, resizer, &visitor->evalContext());
  visitor->setOperator(std::move(buffer_op));
  visitor->init(avg_delay, avg_leakage, wns, PT_tradeoff, nullptr);

  // Mark all selected instances for buffer-only
  for (size_t vid : selected)
    task_arranger->vertex(vid)->move_mask_ = InstVertex::kMoveBuffer;

  auto start_buf = std::chrono::high_resolution_clock::now();
  local_sta_->runResize(resizer, visitor);
  task_arranger->markDirty();
  auto end_buf = std::chrono::high_resolution_clock::now();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(sta::MinMax::max());
  double wns_after = sta_->worstSlack(sta::MinMax::max());
  printf("After V2 buffering, TNS: %.4f ps, WNS: %.4f ps\n",
         tns_after * 1e12, wns_after * 1e12);
  printf("  buffering time: %.3f s\n",
         std::chrono::duration<double>(end_buf - start_buf).count());

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelBufferingV2 total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

std::vector<size_t>
IncreSta::precedingResizeCheck(rsz::Resizer *resizer, float avg_delay,
                               float avg_power, float PT_tradeoff,
                               float top_ratio)
{
  // printf("IncreSta::precedingResizeCheck start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  // Ensure prerequisites
  local_sta_->initParallel();
  if (!equiv_cell_array_built_)
    makeEquivCellArray();

  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Pre-allocate results with 1:1 mapping to TaskArranger vertices.
  std::vector<ResizeBenefit> results(task_arranger->vertexCount());
  for (size_t i = 0; i < results.size(); i++)
    results[i] = {nullptr, -std::numeric_limits<float>::infinity(), i};

  // Create PrecheckVisitor — stores results via visitor->visit(), no DB changes.
  PrecheckVisitor *visitor = new PrecheckVisitor(
      sta_, local_sta_, resizer, &results);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff,
                &swappable_cells_cache_, &inst_info_map_);
  visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);

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
  // printf("Precheck: %zu/%zu instances have positive benefit\n",
         // positive_count, results.size());

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
  // printf("Selected %zu instances (top_ratio=%.2f), top %zu:\n",
  //        results.size(), top_ratio, print_n);
  for (size_t i = 0; i < results.size(); i++) {
    selected_ids.push_back(results[i].vertex_idx);
    // if (i < print_n) {
    //   printf("  [%zu] %s  cost_change=%.6f  vertex_idx=%zu\n", i,
    //          network_->pathName(results[i].inst),
    //          results[i].cost_change, results[i].vertex_idx);
    // }
  }
  // printf("precedingResizeCheck total time: %f s\n", diff_total.count());
  // fflush(stdout);

  return selected_ids;
}

void
IncreSta::parallelResizeByArrayWithPrecheck(
    rsz::Resizer *resizer, float avg_delay, float avg_power,
    float PT_tradeoff, float top_ratio)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  // Ensure swappable cells cache and leakage data are populated
  if (!swap_cell_presaved_) {
    makeSwappableCellsCache(resizer);
  }
  if (!swap_cell_leakage_presaved_) {
    preSaveLibCellLeakage();
  }

  // Phase 1: Precheck — returns filtered top instances sorted by benefit
  auto t_precheck_start = std::chrono::high_resolution_clock::now();
  auto vertex_ids = precedingResizeCheck(resizer, avg_delay, avg_power,
                                       PT_tradeoff, top_ratio);
  auto t_precheck_end = std::chrono::high_resolution_clock::now();
  double precheck_sec = std::chrono::duration<double>(t_precheck_end - t_precheck_start).count();

  // Phase 2: Mark selected instances
  TaskArranger *task_arranger = local_sta_->taskArranger();
  task_arranger->markSelectedInstances(vertex_ids);
    
  // Phase 3: Resize (only selected instances visited in runTask)
  Slack wns = sta_->worstSlack(MinMax::max());
  auto t_resize_start = std::chrono::high_resolution_clock::now();
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff,
      &swappable_cells_cache_, &inst_info_map_);
  visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  visitor->setMoveType(MoveType::Resizing);
  local_sta_->runResize(resizer, visitor);
  auto t_resize_end = std::chrono::high_resolution_clock::now();
  double resize_sec = std::chrono::duration<double>(t_resize_end - t_resize_start).count();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns = sta_->totalNegativeSlack(MinMax::max());
  double wns_after = sta_->worstSlack(MinMax::max());
  // printf("After parallel LR resize with precheck, TNS: %e, WNS: %e\n", tns, wns_after);
  // printf("  precheck time: %.3f s, resize time: %.3f s, ratio: %.2f\n",
         // precheck_sec, resize_sec,
         // resize_sec > 0 ? precheck_sec / resize_sec : 0.0);

  if (isPowerOptimizationMode()) {
    ParallelLrVisitor *cp_visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
    cp_visitor->init(avg_delay, avg_power, wns_after,
        PT_tradeoff, &swappable_cells_cache_, &inst_info_map_);
    cp_visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
    cp_visitor->setMoveType(MoveType::Resizing);
    auto start_cps = std::chrono::high_resolution_clock::now();
    LrSizer lr_sizer(sta_, lr_helper_, cp_visitor);
    lr_sizer.criticalPathSizing();
    auto end_cps = std::chrono::high_resolution_clock::now();
    // printf("After critical path sizing, TNS: %e, WNS: %e\n",
           // sta_->totalNegativeSlack(MinMax::max()),
           // (double)sta_->worstSlack(MinMax::max()));
    // printf("critical path sizing time: %f s\n",
           // std::chrono::duration<double>(end_cps - start_cps).count());
    delete cp_visitor;
  }

  auto end_total = std::chrono::high_resolution_clock::now();
  // printf("parallelResizeByArrayWithPrecheck total time %.3f s\n",
         // std::chrono::duration<double>(end_total - start_total).count());
}

std::vector<size_t>
IncreSta::precedingResizeCheckV2(rsz::Resizer *resizer, float avg_delay,
                                 float avg_power, float PT_tradeoff,
                                 float top_ratio)
{
  printf("IncreSta::precedingResizeCheckV2 start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  // Ensure prerequisites
  local_sta_->initParallel();
  if (!equiv_cell_array_built_)
    makeEquivCellArray();
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();

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
  printf("PrecheckV2: %zu/%zu instances have positive benefit\n",
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
  printf("precedingResizeCheckV2 total time: %f s\n", diff_total.count());
  fflush(stdout);

  return selected_ids;
}

void
IncreSta::parallelResizeByArrayWithPrecheckV2(
    rsz::Resizer *resizer, float avg_delay, float avg_power,
    float PT_tradeoff, float top_ratio)
{
  auto start_total = std::chrono::high_resolution_clock::now();

  // Ensure swappable cells cache and leakage data are populated
  if (!swap_cell_presaved_) {
    makeSwappableCellsCache(resizer);
  }
  if (!swap_cell_leakage_presaved_) {
    preSaveLibCellLeakage();
  }

  // Phase 1: Precheck — returns filtered top instances sorted by benefit
  auto t_precheck_start = std::chrono::high_resolution_clock::now();
  auto vertex_ids = precedingResizeCheckV2(resizer, avg_delay, avg_power,
                                           PT_tradeoff, top_ratio);
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
  visitor->setOperator(std::move(resize_op));
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);

  local_sta_->runResize(resizer, visitor);
  auto t_resize_end = std::chrono::high_resolution_clock::now();
  double resize_sec = std::chrono::duration<double>(t_resize_end - t_resize_start).count();

  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns = sta_->totalNegativeSlack(MinMax::max());
  double wns_after = sta_->worstSlack(MinMax::max());
  printf("After V2 parallel LR resize with precheck, TNS: %e, WNS: %e\n", tns, wns_after);
  printf("  precheck time: %.3f s, resize time: %.3f s, ratio: %.2f\n",
         precheck_sec, resize_sec,
         resize_sec > 0 ? precheck_sec / resize_sec : 0.0);

  if (isPowerOptimizationMode()) {
    ParallelVisitor *cp_visitor = new ParallelVisitor(sta_, local_sta_, resizer);
    std::unique_ptr<ResizeOperator> cp_resize_op =
        std::make_unique<ResizeOperator>(sta_, local_sta_);
    cp_resize_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
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
  printf("parallelResizeByArrayWithPrecheckV2 total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
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

  // printf("bufferingVerticesCandidate: %zu negative-slack gates, selected top %zu by Cout/Cin\n",
         // candidates.size(), keep);
  if (!candidates.empty()) {
    // printf("  Cout/Cin ratio range: [%.2f, %.2f]\n",
           // candidates.back().cout_cin_ratio, candidates.front().cout_cin_ratio);
  }

  return selected;
}

// Sensitivity-based buffering candidate screening (parallel).
// Uses the unified sensitivity formula on each net's buffer tree.
std::vector<size_t>
IncreSta::bufferingVerticesCandidateBySensitivity(
    rsz::Resizer *resizer, float avg_delay, float avg_leakage, int top_n)
{
  // printf("IncreSta::bufferingVerticesCandidateBySensitivity start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Initialize LrRebuffer via BufferInsertion move type
  LrRebuffer::initGlobalPreamble(sta_, resizer);

  // Pre-allocate results
  std::vector<ResizeBenefit> results(task_arranger->vertexCount());
  for (size_t i = 0; i < results.size(); i++)
    results[i] = {nullptr, -std::numeric_limits<float>::infinity(), i};

  // Create BufferSensitivityVisitor and dispatch via visitAll
  BufferSensitivityVisitor *visitor = new BufferSensitivityVisitor(
      sta_, local_sta_, resizer, &results);
  visitor->init(avg_delay, avg_leakage, wns, 100.0f, nullptr, nullptr);
  visitor->setMoveType(MoveType::BufferInsertion);
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
  // printf("Sensitivity screening: %zu instances with positive sensitivity, "
         // "selected top %zu (%.3f s)\n", results.size(), keep, total_sec);
  for (size_t i = 0; i < keep; i++) {
    selected_ids.push_back(results[i].vertex_idx);
    if (i < print_n) {
      // printf("  [%zu] %s  sensitivity=%.6e  vertex_idx=%zu\n", i,
             // network_->pathName(results[i].inst),
             // results[i].cost_change, results[i].vertex_idx);
    }
  }

  return selected_ids;
}

std::vector<size_t>
IncreSta::bufferingVerticesCandidateBySensitivityV2(
    rsz::Resizer *resizer, float avg_delay, float avg_leakage, int top_n)
{
  printf("IncreSta::bufferingVerticesCandidateBySensitivityV2 start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Initialize LrRebufferV2 global preamble
  LrRebufferV2::initGlobalPreamble(sta_, resizer);

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
  visitor->setPrecheckResults(&results);

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
  printf("SensitivityV2 screening: %zu instances with positive sensitivity, "
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

// Apply buffering to the top_n most critical vertices (by Cout/Cin ratio
// among negative-slack gates) in parallel.
void
IncreSta::parallelBuffering(rsz::Resizer *resizer, float PT_tradeoff,
                            int top_n)
{
  // printf("IncreSta::parallelBuffering start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  // Compute average delay/leakage for swapCost normalization
  float avg_delay = averageDelayOnCritPath();
  float avg_leakage = averageLeakage();
  // printf("Buffering avg_delay: %e, avg_leakage: %e\n", avg_delay, avg_leakage);

  // Ensure required times are up-to-date (LMs depend on them)
  sta_->findRequireds();

  local_sta_->initParallel();
  TaskArranger *task_arranger = local_sta_->taskArranger();

  // Phase 1: Screen — select top_n candidates by sensitivity
  auto t_screen_start = std::chrono::high_resolution_clock::now();
  std::vector<size_t> selected = bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, top_n);
  auto t_screen_end = std::chrono::high_resolution_clock::now();
  double screen_sec = std::chrono::duration<double>(t_screen_end - t_screen_start).count();

  if (selected.empty()) {
    // printf("No buffering candidates found. Skipping.\n");
    return;
  }

  // Phase 2: Mark selected instances
  task_arranger->markSelectedInstances(selected);

  // Phase 3: Parallel buffer insertion on selected instances
  Slack wns = sta_->worstSlack(MinMax::max());

  // Initialize global STA/Resizer state once in serial before going parallel.
  LrRebuffer::initGlobalPreamble(sta_, resizer);

  auto start_buffer = std::chrono::high_resolution_clock::now();
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_, resizer);
  visitor->init(avg_delay, avg_leakage, wns, PT_tradeoff, nullptr, nullptr);
  visitor->setMoveType(MoveType::BufferInsertion);
  task_arranger->visitOrdered(sta_, local_sta_, resizer, visitor);
  // Buffer insertion changed the netlist; mark dirty so next
  // visitOrdered() rebuilds the graph from updated netlist.
  task_arranger->markDirty();
  auto end_buffer = std::chrono::high_resolution_clock::now();
  double buffer_sec = std::chrono::duration<double>(end_buffer - start_buffer).count();

  // printf("DEBUG: TNS before full update = %.3f ps, WNS = %.3f ps\n",
         // sta_->totalNegativeSlack(MinMax::max()) * 1e12,
         // sta_->worstSlack(MinMax::max()) * 1e12);
  // fflush(stdout);
  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after = sta_->totalNegativeSlack(MinMax::max());
  double wns_after = sta_->worstSlack(MinMax::max());
  // printf("After parallel LR Buffering, TNS: %.4f ps, WNS: %.4f ps\n",
  //        tns_after * 1e12, wns_after * 1e12);
  // printf("  screening time: %.3f s, buffering time: %.3f s\n",
  //        screen_sec, buffer_sec);

  auto end_total = std::chrono::high_resolution_clock::now();
  // printf("IncreSta::parallelBuffering total time %.3f s\n",
  //        std::chrono::duration<double>(end_total - start_total).count());
}

void
IncreSta::parallelResizeAndBuffering(rsz::Resizer *resizer, float avg_delay,
                                     float avg_power, float PT_tradeoff,
                                     int buffer_top_n)
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

  // Screen buffering candidates by sensitivity and annotate on InstVertex
  TaskArranger *task_arranger = local_sta_->taskArranger();
  std::vector<size_t> buf_candidates =
      bufferingVerticesCandidateBySensitivity(resizer, avg_delay, avg_power,
                                             buffer_top_n);
  printf("Buffer candidates (sensitivity): %zu (of %zu total)\n",
         buf_candidates.size(), task_arranger->vertexCount());

  // All instances get resize; buffer candidates also get buffer bit
  for (size_t i = 0; i < task_arranger->vertexCount(); i++)
    task_arranger->vertex(i)->move_mask_ = InstVertex::kMoveResize;
  for (size_t vid : buf_candidates)
    task_arranger->vertex(vid)->move_mask_ |= InstVertex::kMoveBuffer;

  // Initialize global STA/Resizer state for buffering (serial preamble)
  LrRebuffer::initGlobalPreamble(sta_, resizer);

  // Create CombinedVisitor and run single-pass resize + buffering
  auto start_resize = std::chrono::high_resolution_clock::now();
  CombinedVisitor *visitor = new CombinedVisitor(sta_, local_sta_, resizer,
                                                 task_arranger);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff,
                &swappable_cells_cache_, &inst_info_map_);
  visitor->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);

  task_arranger->visitOrdered(sta_, local_sta_, resizer, visitor);

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

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelResizeAndBuffering total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

void
IncreSta::parallelResizeAndBufferingV2(rsz::Resizer *resizer, float avg_delay,
                                       float avg_power, float PT_tradeoff,
                                       int buffer_top_n)
{
  printf("IncreSta::parallelResizeAndBufferingV2 start\n");
  auto start_total = std::chrono::high_resolution_clock::now();

  local_sta_->initParallel();
  Slack wns = sta_->worstSlack(MinMax::max());

  if (!swap_cell_presaved_)
    makeSwappableCellsCache(resizer);
  if (!swap_cell_leakage_presaved_)
    preSaveLibCellLeakage();
  makeEquivCellArray();

  // Screen buffering candidates by sensitivity and annotate on InstVertex
  TaskArranger *task_arranger = local_sta_->taskArranger();
  std::vector<size_t> buf_candidates = bufferingVerticesCandidateBySensitivityV2(
      resizer, avg_delay, avg_power, buffer_top_n);
  printf("Buffer candidates (sensitivity): %zu (of %zu total)\n",
         buf_candidates.size(), task_arranger->vertexCount());

  // All instances get resize; buffer candidates also get buffer bit
  for (size_t i = 0; i < task_arranger->vertexCount(); i++)
    task_arranger->vertex(i)->move_mask_ = InstVertex::kMoveResize;
  for (size_t vid : buf_candidates)
    task_arranger->vertex(vid)->move_mask_ |= InstVertex::kMoveBuffer;

  // Initialize global STA/Resizer state for buffering (serial preamble)
  LrRebufferV2::initGlobalPreamble(sta_, resizer);

  // Create ParallelVisitor with CombinedOperator
  auto start_resize = std::chrono::high_resolution_clock::now();
  auto *visitor = new ParallelVisitor(sta_, local_sta_, resizer);
  visitor->setTaskArranger(task_arranger);

  auto combined_op = std::make_unique<CombinedOperator>(
      sta_, local_sta_, resizer, &visitor->evalContext());
  combined_op->setEquivCellArray(&equiv_cell_array_, &equiv_cell_pos_map_);
  visitor->setOperator(std::move(combined_op));

  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &inst_info_map_);

  local_sta_->runResize(resizer, visitor);

  // If any buffers were inserted, mark graph dirty for next pass
  task_arranger->markDirty();

  auto end_resize = std::chrono::high_resolution_clock::now();
  printf("parallelResizeAndBufferingV2 pass time: %.3f s\n",
         std::chrono::duration<double>(end_resize - start_resize).count());

  sta_->updateTiming(true);
  sta_->findRequireds();

  double tns_after = sta_->totalNegativeSlack(MinMax::max());
  double wns_after = sta_->worstSlack(MinMax::max());
  printf("After parallelResizeAndBufferingV2, TNS: %e, WNS: %e\n",
         tns_after, wns_after);

  auto end_total = std::chrono::high_resolution_clock::now();
  printf("IncreSta::parallelResizeAndBufferingV2 total time %.3f s\n",
         std::chrono::duration<double>(end_total - start_total).count());
}

} // namespace lrf

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
#include "sta/TimingRole.hh"
#include "lrf/LrfClass.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "TaskArranger.hh"
#include "ParallelVisitor.hh"
#include "rsz/Resizer.hh"
#include "ParallelLibData.hh"
#include "LrSizer.hh"

#include <unordered_map>
#include <chrono>

namespace lrf {
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
IncreSta::delayLmSum(Instance *inst, const MinMax *minmax, float &delay_lambda_sum)
{
  init();
  delay_lambda_sum = 0.0;
  delay_lambda_sum = local_sta_->delayLmSum(inst, minmax);
}

// void 
// IncreSta::lmUpdate()
// {
//   printf("DEBUG: IncreSta::lmUpdate start\n");
//   fflush(stdout);
//   init();
//   printf("DEBUG: IncreSta::lmUpdate calling updateAllEdgeLms\n");
//   fflush(stdout);
//   lr_helper_->updateAllEdgeLms(sta_);
//   printf("DEBUG: IncreSta::lmUpdate calling KKTProjection (projected_)\n");
//   fflush(stdout);
//   lr_helper_->KKTProjection(sta_);
//   printf("DEBUG: IncreSta::lmUpdate end\n");
//   fflush(stdout);
// }

// void 
// IncreSta::lmUpdate()
// {
//   printf("DEBUG: IncreSta::lmUpdate start\n");
//   fflush(stdout);
//   init();
//   printf("DEBUG: IncreSta::lmUpdate calling updateAllEdgeLms\n");
//   fflush(stdout);
//   lr_helper_->updateAllEdgeLms(sta_);
//   printf("DEBUG: IncreSta::lmUpdate calling KKTProjection (projected_)\n");
//   fflush(stdout);
//   lr_helper_->KKTProjection(sta_);
//   printf("DEBUG: IncreSta::lmUpdate end\n");
//   fflush(stdout);
// }

void 
IncreSta::lmUpdate()
{
  printf("DEBUG: IncreSta::lmUpdate start\n");
  fflush(stdout);
  init();
  printf("DEBUG: IncreSta::lmUpdate init done\n");
  fflush(stdout);
  if (projected_) {
    printf("DEBUG: IncreSta::lmUpdate calling updateAllEdgeLms\n");
    fflush(stdout);
    lr_helper_->updateAllEdgeLms(sta_);
    printf("DEBUG: IncreSta::lmUpdate calling KKTProjection (projected_)\n");
    fflush(stdout);
    lr_helper_->KKTProjection(sta_);
  } else {
    printf("DEBUG: IncreSta::lmUpdate calling KKTProjection (else)\n");
    fflush(stdout);
    bool kkt_satisfied = lr_helper_->KKTProjection(sta_);
    if (kkt_satisfied)
      projected_ = true;
    else {
      printf("KKT not satisfied, should be checked\n");
      fflush(stdout);
    }
  }
  printf("DEBUG: IncreSta::lmUpdate end\n");
  fflush(stdout);
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
  cell_info_vec_ = new LocalCellInfo[int(network_->leafInstanceCount() * 1.1)];
  inst_info_map_.clear();
  inst_info_map_.reserve(int(network_->leafInstanceCount() * 1.1));
  // clearLocalCellInfoMap();
  // cell_info_vec_ = new LocalCellInfo[network_->leafInstanceCount() + 1];
  // inst_info_map_.clear();
  // inst_info_map_.reserve(network_->leafInstanceCount() * 1.1);

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
    printf("makeSwappableCellsCache took %f s\n", diff_cache.count());
  }
  if (!swap_cell_leakage_presaved_) {
    auto start_presave = std::chrono::high_resolution_clock::now();
    preSaveLibCellLeakage();
    auto end_presave = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_presave = end_presave - start_presave;
    printf("preSaveLibCellLeakage took %f s\n", diff_presave.count());
  }

  // float average_delay = averageDelayOnCritPath();
  // float average_power = averageLeakage();
  printf("Average delay: %f, average power: %f\n", avg_delay * 1e12, avg_power * 1e9);
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, &swappable_cells_cache_, &inst_info_map_);

  auto start_resize = std::chrono::high_resolution_clock::now();
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;
  printf("local_sta_->runResize took %f s\n", diff_resize.count());

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
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
    printf("makeParallelLibData took %f s\n", diff_pld.count());
  }

  // float average_delay = averageDelayOnCritPath();
  // float average_power = averageLeakage();
  printf("Average delay: %f, average power: %f\n", avg_delay * 1e12, avg_power * 1e9);
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, parallel_lib_data_);

  auto start_resize = std::chrono::high_resolution_clock::now();
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;
  printf("local_sta_->runResize took %f s\n", diff_resize.count());

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}

void
IncreSta::setMaxResizeNum(size_t max_resize_num)
{
  local_sta_->taskArranger()->setMaxResizeNum(max_resize_num);
}

void 
IncreSta::parallelResizeCPS(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff)
{
  printf("IncreSta::parallelResizeCPS start\n");
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
    printf("makeSwappableCellsCache took %f s\n", diff_cache.count());
  }
  if (!swap_cell_leakage_presaved_) {
    auto start_presave = std::chrono::high_resolution_clock::now();
    preSaveLibCellLeakage();
    auto end_presave = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff_presave = end_presave - start_presave;
    printf("preSaveLibCellLeakage took %f s\n", diff_presave.count());
  }

  auto start_resize = std::chrono::high_resolution_clock::now();
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta_, local_sta_);
  visitor->init(avg_delay, avg_power, wns, PT_tradeoff, 
      &swappable_cells_cache_, &inst_info_map_);
  local_sta_->runResize(resizer, visitor);
  auto end_resize = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_resize = end_resize - start_resize;

  // Use distinct variable names to avoid shadowing Slack
  sta_->updateTiming(true);
  sta_->findRequireds();
  double tns_after_resize = sta_->totalNegativeSlack(MinMax::max());
  double wns_after_resize = sta_->worstSlack(MinMax::max());
  printf("After parallel LR resize, TNS: %e, WNS: %e\n", tns_after_resize, wns_after_resize);
  printf("parallel resize time: %f s\n", diff_resize.count());

  // Run critical path sizing
  ParallelLrVisitor *critical_path_visitor = new ParallelLrVisitor(sta_, local_sta_);
  critical_path_visitor->init(avg_delay, avg_power, wns_after_resize, 
      PT_tradeoff, &swappable_cells_cache_, &inst_info_map_);

  // Time the critical-path sizing phase
  auto start_cps = std::chrono::high_resolution_clock::now();
  LrSizer lr_sizer(sta_, lr_helper_, critical_path_visitor);
  lr_sizer.criticalPathSizing();
  auto end_cps = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_cps = end_cps - start_cps;

  double tns_after_cps = sta_->totalNegativeSlack(MinMax::max());
  double wns_after_cps = sta_->worstSlack(MinMax::max());
  printf("After critical path sizing, TNS: %e, WNS: %e\n", tns_after_cps, wns_after_cps);
  printf("critical path sizing time: %f s\n", diff_cps.count());
  delete critical_path_visitor;

  auto end_total = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_total = end_total - start_total;
  printf("IncreSta::parallelResize total time %f s\n", diff_total.count());
}


} // namespace lrf

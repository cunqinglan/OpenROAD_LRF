#include "lrf/IncreSta.hh"
#include "LocalSta.hh"
#include "LrHelper.hh"
#include "sta/Liberty.hh"
#include "sta/Path.hh"
#include "sta/Corner.hh"
#include "sta/PathExpanded.hh"
#include "sta/Search.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/TimingRole.hh"
#include "lrf/LrfClass.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "TaskArranger.hh"

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
  db_sta->setThreadCount(thread_count);
  dbStaState::init(db_sta);
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
  sta_->unregisterStaState(this);
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
IncreSta::makeLRHelper()
{
  if (lr_helper_)
    delete lr_helper_;
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

//////////////////////////////////////////////////////////
// APIs for parasitics estimation
///////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
// APIs for swappable cells
///////////////////////////////////////////////////////////
void 
IncreSta::makeSwappableCellsCache()
{

}

void 
IncreSta::parallelResize(rsz::Resizer *resizer)
{
  // We first create a serials of instance visitors
  local_sta_->initParallel();
  float average_delay = averageDelayOnCritPath();
  float average_power = averageLeakage();
  local_sta_->runResize(resizer, average_delay, average_power);
}

void
IncreSta::setMaxResizeNum(size_t max_resize_num)
{
  local_sta_->taskArranger()->setMaxResizeNum(max_resize_num);
}

} // namespace lrf

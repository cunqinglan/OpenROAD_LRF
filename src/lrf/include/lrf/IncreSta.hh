#pragma once

#include "db_sta/dbSta.hh"
#include "sta/Sta.hh"
#include "lrf/LrfClass.hh"
// #include "LocalSta.hh"

namespace rsz {
  class Resizer;
}

namespace sta {
  class ConcreteParasitic;
  class Parasitic; // forward declare individual parasitic object
  class Parasitics;
  class ParasiticAnalysisPt;
}

namespace est {
  class EstimateParasitics;
}

namespace lrf {
using namespace sta;

class LocalSta;
class LRHelper;

class IncreSta : public dbStaState
{
public:
  IncreSta(dbSta *db_sta);
  IncreSta(dbSta *db_sta, size_t thread_count);
  ~IncreSta();
  void init();
  virtual void copyState(const dbSta *sta);

  LocalSta *localSta() { return local_sta_; };
  LRHelper *lrHelper() { return lr_helper_; };
  

  InstanceSeq &getSortedInstances();
  void resetSortedInstances() { sorted_instances_.clear(); }
  void delayLmSum(Instance *inst, const MinMax *minmax, float &delay_lambda_sum);

  // float averageDelayOnCriPath();

  // KKT projection and LM update
  void lmUpdate();

  // Check violations
  bool checkCapViolated(Pin *pin, const Corner *corner, const MinMax *min_max);
  bool checkSlewViolated(Pin *pin, const Corner *corner, const MinMax *min_max);
  float maxInputSlew(const Pin* input_pin,
                            const Corner* corner) const;
  float averageDelayOnCritPath();
  float averageLeakage();

  // APIs for parasitics estimation
  void setLocalStaParasiticsEst(est::EstimateParasitics *estimate_parasitics);
  void makeSwappableCellsCache();
  void getSwappableCells(LibertyCell* source_cell);
                           
  // APIs for gate swapping
  void parallelResize(rsz::Resizer *resizer);
  void setMaxResizeNum(size_t max_resize_num);

protected:
  void makeLocalSta();
  void makeLRHelper();
  void checkeTopoOrder(InstanceSeq &);
  void ensureLocalSta();

  LocalSta *local_sta_;
  LRHelper *lr_helper_;

  InstanceSeq sorted_instances_;
  bool projected_ = false;
  std::unordered_map<LibertyCell*, LibertyCellSeq> swappable_cells_cache_;
};

} // namespace lrf

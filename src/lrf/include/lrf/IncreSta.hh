#pragma once

#include "db_sta/dbSta.hh"
#include "sta/Sta.hh"
#include "lrf/LrfClass.hh"

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
  ~IncreSta();
  void init();
  virtual void copyState(const dbSta *sta);

  LocalSta *localSta() { return local_sta_; };
  LRHelper *lrHelper() { return lr_helper_; };
  void setLocalStaParasiticsEst(est::EstimateParasitics *estimate_parasitics);

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

protected:
  void makeLocalSta();
  void makeLRHelper();
  void checkeTopoOrder(InstanceSeq &);
  void ensureLocalSta();

  LocalSta *local_sta_;
  LRHelper *lr_helper_;

  InstanceSeq sorted_instances_;
  bool projected_ = false;
};

} // namespace lrf

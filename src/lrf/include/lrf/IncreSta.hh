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
class ParallelLibData;
class TaskArranger;

class IncreSta : public dbStaState
{
public:
  IncreSta(dbSta *db_sta);
  IncreSta(dbSta *db_sta, size_t thread_count);
  ~IncreSta();
  void clearLocalCellInfoMap();
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
  float maxInputSlew(const Pin* input_pin, const Corner* corner) const;
  float averageDelayOnCritPath();
  float averageLeakage();

  // APIs for parasitics estimation
  void setLocalStaParasiticsEst(est::EstimateParasitics *estimate_parasitics);
                           
  // APIs for gate swapping
  void parallelResize(rsz::Resizer *resizer, float avg_delay = 1, float avg_power = 1,
                      float PT_tradeoff = 100.0);
  void parallelResizeV1(rsz::Resizer *resizer, float avg_delay = 1, float avg_power = 1,
                        float PT_tradeoff = 100.0);
  void parallelResizeAdaptive(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff);
  void parallelResizeByArray(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff);
  void setMaxResizeNum(size_t max_resize_num);

  void parallelBuffering(rsz::Resizer *resizer, float PT_tradeoff,
                         int top_n = 10);

  // Screen buffering candidates: collect gates with negative late slack,
  // sort by output_cap / input_cap ratio descending, return top_n vertex ids.
  std::vector<size_t> bufferingVerticesCandidate(int top_n);

  // Preceding resize check: evaluate resize benefit for all instances
  // in parallel (no conflict graph). Returns vertex indices of
  // selected top instances (sorted by benefit, filtered by top_ratio).
  std::vector<size_t> precedingResizeCheck(
      rsz::Resizer *resizer, float avg_delay, float avg_power,
      float PT_tradeoff, float top_ratio = 0.3);

  // Resize by array with preceding precheck: only selected top instances are resized.
  void parallelResizeByArrayWithPrecheck(rsz::Resizer *resizer, float avg_delay,
                                         float avg_power, float PT_tradeoff,
                                         float top_ratio = 0.3);

  // APIs for power optimization
  void ensureActivities();  // Access power of one of the instances will trigger global activity calculation
  void makeSwappableCellsCache(rsz::Resizer *resizer);
  void getSwappableCells(LibertyCell* source_cell);
  void preSaveLibCellLeakage();
  void makeParallelLibData(rsz::Resizer *resizer, TaskArranger *task_arranger);
  ParallelLibData *parallelLibData() { return parallel_lib_data_; }
  void makeEquivCellArray(bool verbose = false);
  LibertyCellArray* equivCellArray() { return &equiv_cell_array_; }
  PosMap* equivCellPosMap() { return &equiv_cell_pos_map_; }

  // APIs for LM update
  void makeLRHelper(std::string method = "LRHelper");

  // APIs for Adaptive optimization
  bool isPowerOptimizationMode() const;

protected:
  void makeLocalSta();
  void checkeTopoOrder(InstanceSeq &);
  void ensureLocalSta();

  LocalSta *local_sta_;
  LRHelper *lr_helper_;

  InstanceSeq sorted_instances_;
  bool projected_ = false;
  std::unordered_map<LibertyCell*, LibertyCellSeq*> swappable_cells_cache_;
  std::unordered_map<sta::Instance*, LocalCellInfo*> inst_info_map_;
  LocalCellInfo *cell_info_vec_ = nullptr;
  bool swap_cell_presaved_ = false;
  bool swap_cell_leakage_presaved_ = false;
  ParallelLibData *parallel_lib_data_ = nullptr;
  LibertyCellArray equiv_cell_array_;
  PosMap equiv_cell_pos_map_;
  bool equiv_cell_array_built_ = false;
};

} // namespace lrf

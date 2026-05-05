#pragma once

#include <deque>

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
class PlacementDensityMap;

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

  // Enable per-arc ∂delay/∂in_slew computation in STA dcalc, then force a
  // full graph refresh so delay_diffs_ are populated for every edge.
  // Call once at LR start before precheck. After this, every subsequent
  // STA gateDelay pays a paired perturbed call (≈ 2× dcalc cost).
  void initDelayDiff();

  // float averageDelayOnCriPath();

  // KKT projection and LM update
  void lmUpdate();

  // Check violations
  bool checkCapViolated(Pin *pin, const Corner *corner, const MinMax *min_max);
  bool checkSlewViolated(Pin *pin, const Corner *corner, const MinMax *min_max);
  float maxInputSlew(const Pin* input_pin, const Corner* corner) const;
  float averageDelayOnCritPath();
  float averageLeakage();
  // Average output-pin slew and load cap across all leaf-instance driver
  // pins. Used as normalizers for ERC violation penalty in EvalContext::swapCost.
  // Returns ~1e-10 / ~1e-15 for empty designs to avoid div-by-zero.
  float averageOutSlew();
  float averageLoadCap();
  // Recompute and cache avg_out_slew_ / avg_load_cap_ for ERC penalty normalization.
  void updateErcNormalizers();
  float avgOutSlew() const { return avg_out_slew_; }
  float avgLoadCap() const { return avg_load_cap_; }
  // Fast total leakage using pre-computed inst_info_map_ (avoids sta->power()).
  float totalLeakageFast();

  // APIs for parasitics estimation
  void setLocalStaParasiticsEst(est::EstimateParasitics *estimate_parasitics);
                           
  // APIs for gate swapping
  // erc_violation_weight: forwarded to ParallelVisitor's EvalContext.
  //   <0 hard reject, ==0 ignore, >0 soft penalty (default -1.0 matches
  //   EvalContext default → existing callers unaffected).
  // erc_limit_scale: multiplier on lib slew/cap limits (default 0.95 =
  //   5% headroom; matches EvalContext default).
  void parallelResizeByArray(rsz::Resizer *resizer, float avg_delay, float avg_power,
                      float PT_tradeoff, float erc_violation_weight = 1e6f,
                      float erc_limit_scale = 0.95f);
  void setMaxResizeNum(size_t max_resize_num);
  void setBufferOnlyMode(bool mode) { buffer_only_mode_ = mode; }
  void setBakogluK(float k) { bakoglu_k_ = k; }
  void setDebug(bool d) { debug_ = d; }

  // top_ratio: fraction of total instances to screen (0.01 = top 1%).
  void parallelBuffering(rsz::Resizer *resizer, float PT_tradeoff,
                         float top_ratio = 0.01f);
  // Same sensitivity screening, but apply rsz-style rebuffering (repair_timing)
  // instead of LRF LrRebuffer.
  void parallelBufferingRsz(rsz::Resizer *resizer, float PT_tradeoff,
                             int top_n = 100);
  // LRF slack-DP rebuffering — uses BufferSdpOperator which invokes
  // LrRebuffer::prepareSlackDpBnet (bufferForTimingSlackDp + recoverLrCost).
  // Mirrors parallelBuffering (cost-DP) signature; only operator differs.
  void parallelBufferingSdp(rsz::Resizer *resizer, float PT_tradeoff,
                             float top_ratio = 0.01f,
                             float erc_violation_weight = 1e6f,
                             float erc_limit_scale = 0.95f);
  void probeRszBnet(rsz::Resizer *resizer, float PT_tradeoff = 10.0f,
                    int top_n = 100);

  // Single-pass resize + buffering using CombinedOperator + ParallelVisitor.
  // buffer_top_ratio: fraction of total instances to screen (0.01 = top 1%).
  void parallelResizeAndBuffering(rsz::Resizer *resizer, float avg_delay,
                                    float avg_power, float PT_tradeoff,
                                    float buffer_top_ratio = 0.01f);

  // Screen buffering candidates: collect gates with negative late slack,
  // sort by output_cap / input_cap ratio descending, return top_n vertex ids.
  std::vector<size_t> bufferingVerticesCandidate(int top_n);

  // Sensitivity screening via ParallelVisitor + BufferSensitivityOperator.
  std::vector<size_t> bufferingVerticesCandidateBySensitivity(
      rsz::Resizer *resizer, float avg_delay, float avg_leakage, int top_n);

  // Preceding resize check: evaluate resize benefit for all instances
  // in parallel (no conflict graph). Returns vertex indices of
  // selected top instances (sorted by benefit, filtered by top_ratio).
  std::vector<size_t> precedingResizeCheck(
      rsz::Resizer *resizer, float avg_delay, float avg_power,
      float PT_tradeoff, float top_ratio = 0.3);

  // Resize with precheck: precedingResizeCheck + parallelResizeByArray.
  void parallelResizeByArrayWithPrecheck(rsz::Resizer *resizer, float avg_delay,
                                           float avg_power, float PT_tradeoff,
                                           float top_ratio = 0.3,
                                           float erc_violation_weight = 1e6f,
                                           float erc_limit_scale = 0.95f);

  // Adaptive instance-level filtering control.
  // Call activateInstanceFilter() when timing regression is detected.
  // It computes target_N = last_change_count * multiplier, then sets adaptive_top_ratio.
  void activateInstanceFilter(float max_ratio = 0.3f);
  void setAdaptiveTopRatio(float ratio) { pruning_control_.adaptive_top_ratio = ratio; }
  float adaptiveTopRatio() const { return pruning_control_.adaptive_top_ratio; }
  int lastChangeCount() const { return pruning_control_.last_change_count; }

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

  // LM snapshot file I/O — save/load LM vector for deterministic experiments.
  // design_name is validated on load to prevent mismatched checkpoint restore.
  bool saveLmToFile(const std::string &path, const std::string &design_name);
  int loadLmFromFile(const std::string &path, const std::string &design_name);

  // APIs for LM update
  void makeLRHelper(std::string method = "LRHelper");

  // Placement density map for density-aware swap cost.
  void setDensityMap(const PlacementDensityMap *map, float weight, float avg_area) {
    density_map_ = map;
    density_weight_ = weight;
    average_area_ = avg_area;
  }

  // APIs for Adaptive optimization
  bool isPowerOptimizationMode() const;

  // ── Iteration metric history ───────────────────────────────────
  // Single push entry. Caller invokes once per "iteration completion"
  // (typically right after IterationHelper::snapshot()). Buffering
  // passes / final post-loop snapshots should NOT push, so the rolling
  // window stays one-sample-per-LR-iter.
  void recordMetrics(double wns_ps, double tns_ps, double leakage);

  // Power-mode entry plateau predicate (consumed by lmUpdate).
  // True when ≥ 4 samples and 3-iter-span TNS improvement (front→back,
  // relative to |front|) is below `threshold`. Default 0.10 = 10%.
  bool isTnsPlateau(double threshold = 0.10) const;

  // Power-mode termination plateau predicate (consumed by EcoController).
  // True when in power mode AND ≥ 3 recordMetrics calls since entering power
  // mode AND avg per-iter reduction (vs the frozen pre-power-mode baseline)
  // is below `threshold`. Default 0.01 = 1% per iter.
  bool isLeakagePlateau(double threshold = 0.01) const;

  // Diagnostics / future-flexibility accessors. Return 0 if window not full.
  size_t historyDepth() const { return tns_history_.size(); }
  double tnsImprovementRate() const;     // (back - front) / |front|, +ve = better
  double leakageReductionRate() const;   // avg/iter vs power-mode-entry baseline

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
  PruningControl pruning_control_;
  const PlacementDensityMap *density_map_ = nullptr;
  float density_weight_ = 0.0f;
  float average_area_ = 1.0f;
  // ERC penalty normalizers — computed lazily via updateErcNormalizers()
  float avg_out_slew_ = 1e-10f;
  float avg_load_cap_ = 1e-15f;
  bool buffer_only_mode_ = false;
  float bakoglu_k_ = 2.5f;
  bool debug_ = false;
  // Per-iteration metric history (one entry per recordMetrics() call).
  // Rolling window of 4. Pushed by callers right after snapshot(); used by
  // isTnsPlateau / isLeakagePlateau / future predicates.
  std::deque<double> wns_history_;
  std::deque<double> tns_history_;
  std::deque<double> leakage_history_;

  // Counts recordMetrics() calls since the helper first entered power mode.
  // Power mode is sticky, so this monotonically advances once started.
  // Used as both:
  //   (1) a guard for isLeakagePlateau (need ≥ 3 post-entry samples), and
  //   (2) an index offset back into leakage_history_ to recover the pre-power
  //       baseline (leakage_history_[size-1-power_mode_iters_]).
  size_t power_mode_iters_ = 0;

  // Sticky gate for criticalPathSizing — set true the first lmUpdate() in
  // which |WNS| < 1% × T_eff OR |TNS| < 10% × T_eff (the (a)/(b) entry
  // criteria for power mode; we deliberately exclude the (c) plateau path).
  // Once true, stays true for the rest of the run. Designs whose timing
  // never gets close enough to the (a)/(b) thresholds skip CPS entirely.
  bool cps_enabled_ = false;

};

} // namespace lrf

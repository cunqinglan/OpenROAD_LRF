



#pragma once

#include <atomic>
#include <memory>

#include "lrf/LrfClass.hh"
#include "sta/SearchPred.hh"
#include "db_sta/dbSta.hh"
#include "LmHistory.hh"

namespace lrf {
using namespace sta;

typedef std::map<DcalcAnalysisPt const*, LMValue> DcalcAPToLMValueMap;
typedef std::map<DcalcAnalysisPt const*, LMValueSeq> DcalcAPToLMValueSeqMap;

class LRHelper: public dbStaState
{
public:
  LRHelper(dbSta *sta);
  ~LRHelper();

  virtual void copyState(const StaState *sta);

  VertexSeq &ensureSorted(Sta *sta);
  bool KKTProjection(Sta *sta);
  virtual void updateAllEdgeLms(Sta *sta);
  void enqueueVertex(Vertex *vertex);
  void setRatcons(bool ratcons) { RATCONS_ = ratcons; }
  void setTimingMargin(float margin) { timing_margin_ = margin; }
  float timingMargin() const { return timing_margin_; }
  virtual std::string strategyName() const { return "Base LRHelper"; }
  virtual bool updateCriticalPathLms(sta::Path *path_end) { return false; };
  virtual void setMode(std::string mode) {};
  virtual std::string mode() const { return ""; }

  // --- Parallel KKT projection and LM update ---
  // Uses ref-count based parallel dispatch (similar to TaskArranger pattern).
  // dispatch_queue_ and thread_count_ are inherited from StaState.
  bool parallelKKTProjection(Sta *sta);
  void parallelUpdateAllEdgeLms(Sta *sta);

  // --- LM History: snapshot & rollback ---
  // Record current LM values of all edges. Returns frame id.
  int recordLM();
  // Restore LM values from a previously recorded frame.
  // Returns the number of edges successfully restored.
  int restoreLM(int frame_id);
  // Number of recorded LM frames.
  int lmFrameCount() const;
  // Clear all recorded LM frames.
  void clearLmHistory();
  // Save the last recorded LM frame to a binary file.
  // design_name is stored in the header for validation on load.
  bool saveLmToFile(const std::string &path, const std::string &design_name);
  // Load an LM frame from a binary file, restore it, and return frame id.
  // Validates design_name and vertex count against the file header.
  int loadLmFromFile(const std::string &path, const std::string &design_name);

protected:
  void distributeLmOutToIn(Vertex *vertex,
                            LMValueSeq &out_lm_sums,
                            DcalcAPToLMValueSeqMap &in_lm_seq_map,
                            size_t in_sum_index);

  LMValueSeq computeOutLmSum(Vertex *vertex) const;
  size_t computeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_seq_map);
  bool checkKKTForAllVertices();
  bool parallelCheckKKTForAllVertices();
  void parallelComputeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_seq_map);
  bool isBeforeReg(Vertex *vertex) const;
  void topoSort(LRHelper *lr_helper, VertexSeq &sorted_vertices);
  virtual void updateEdgeLms(Edge *edge, Sta *sta);
  virtual void updateArcLms(Edge *edge, TimingArc *arc, Sta *sta,
                            DcalcAnalysisPt const *dcalc_ap);
  virtual void updateEndPointArcLms(Edge *edge, TimingArc *arc, Sta *sta,
                                    DcalcAnalysisPt const *dcalc_ap);
  // Debug: print pins covered by set_false_path exceptions (from/to)
  void dumpFalsePathPins(Sta *sta);
  void BFSSort();
  void levelSort(Sta *sta);
  void clearLms(Edge *edge);

  SearchPredNonLatch2* search_pred_;
  BfsFwdIterator* iter_;
  VertexSeq sorted_lm_vertices_;
  bool levelized_valid_;
  bool RATCONS_ = false;
  float timing_margin_ = 0.01f;  // ratio; T_eff = T*(1+margin). Negative = tighter (higher LM, GRT-robust); positive = looser.
  LmHistory lm_history_;

  // vertex_to_sorted_idx_[vertex_id] -> index in sorted_lm_vertices_
  // Used by parallel KKT backward pass to map Vertex* to ap_lm_seq_map index.
  std::unordered_map<VertexId, size_t> vertex_to_sorted_idx_;

private:
  friend class Graph;
  friend class SortVertexVisitor;
  friend class KKTBackwardVisitor;
};

class RapidLrHelper : public LRHelper {
public:
  RapidLrHelper(sta::dbSta *sta);
  ~RapidLrHelper() override = default;
  virtual std::string strategyName() const override { return "Rapid LRHelper: LM=path delay/T"; }
  virtual bool updateCriticalPathLms(sta::Path *path_end) override;
  virtual void setMode(std::string mode) override;
  virtual std::string mode() const override { return power_mode_ ? "power" : "timing"; }
  void setPowerMode();
  void setTimingMode();
  bool isPowerMode() const { return power_mode_; }
  bool isTimingMode() const { return !power_mode_; }

protected:
  virtual void updateArcLms(Edge *edge, TimingArc *arc, Sta *sta, 
                    DcalcAnalysisPt const *dcalc_ap) override;
  virtual void updateEndPointArcLms(Edge *edge, TimingArc *arc, Sta *sta, 
                                    DcalcAnalysisPt const *dcalc_ap) override;
  // This can only be invoked by RapidLrHelper's own methods
  float getMultiplier(Slack arc_slack);

private:
  int critical_arc_k_ = 4;
  int non_critical_arc_k_ = 1;
  bool power_mode_ = false;
};

class AdaptiveLrHelper : public LRHelper {
public:
  AdaptiveLrHelper(sta::dbSta *sta) : LRHelper(sta) {}
  ~AdaptiveLrHelper() override = default;
  virtual std::string strategyName() const override;
  virtual void updateArcLms(Edge *edge, TimingArc *arc, Sta *sta, 
                    DcalcAnalysisPt const *dcalc_ap) override;
  // This can only be invoked by AdaptiveLrHelper's own methods
  float getMultiplier(Slack arc_slack);
  
};

} // namespace lrf

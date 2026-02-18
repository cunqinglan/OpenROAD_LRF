



#pragma once

#include "lrf/LrfClass.hh"
#include "sta/SearchPred.hh"
#include "db_sta/dbSta.hh"


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
  virtual std::string strategyName() const { return "Base LRHelper"; }
  virtual bool updateCriticalPathLms(sta::Path *path_end) { return false; };
  virtual void setMode(std::string mode) {};
  virtual std::string mode() const { return ""; }

protected:
  void distributeLmOutToIn(Vertex *vertex,
                            LMValueSeq &out_lm_sums,
                            DcalcAPToLMValueSeqMap &in_lm_seq_map,
                            size_t in_sum_index);

  LMValueSeq computeOutLmSum(Vertex *vertex) const;
  size_t computeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_seq_map);
  bool checkKKTForAllVertices();
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

private:
  friend class Graph;
  friend class SortVertexVisitor;
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

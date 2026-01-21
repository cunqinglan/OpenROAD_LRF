



#pragma once

#include "lrf/LrfClass.hh"
#include "sta/SearchPred.hh"
#include "db_sta/dbSta.hh"


namespace lrf {
using namespace sta;



typedef std::map<DcalcAnalysisPt const*, LMValue> DcalcAPToLMValueMap;
typedef std::map<DcalcAnalysisPt const*, LMValueSeq> DcalcAPToLMValueSeqMap;

class LRHelper: public StaState
{
public:
  LRHelper(StaState *sta);
  ~LRHelper();

  virtual void copyState(const StaState *sta);

  VertexSeq &ensureSorted(Sta *sta);
  bool KKTProjection(Sta *sta);
  void updateAllEdgeLms(Sta *sta);
  void enqueueVertex(Vertex *vertex);
  void setRatcons(bool ratcons) { RATCONS_ = ratcons; }

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
  void updateEdgeLms(Edge *edge, Sta *sta);
  void updateArcLms(Edge *edge, TimingArc *arc, Sta *sta);
  void updateEndPointArcLms(Edge *edge, TimingArc *arc, Sta *sta);
  void BFSSort();
  void levelSort(Sta *sta);

  SearchPredNonLatch2* search_non_latch_pred_;
  BfsFwdIterator* iter_;
  VertexSeq sorted_lm_vertices_;
  bool levelized_valid_;
  bool RATCONS_ = false;
  bool strict_constraint_ = false;

private:
  friend class Graph;
  friend class SortVertexVisitor;
};

} // namespace lrf





#pragma once

#include "lrf/LrfClass.hh"
#include "sta/SearchPred.hh"


namespace lrf
{
class Graph;
class Vertex;
class Edge;
class Sta;
class SortVertexVisitor;

typedef std::map<DcalcAnalysisPt const*, LMValue> DcalcAPToLMValueMap;
typedef std::map<DcalcAnalysisPt const*, LMValueSeq> DcalcAPToLMValueSeqMap;

using sta::SearchPredNonLatch2;
using sta::BfsFwdIterator;

class LRHelper: public dbStaState
{
public:
  LRHelper(dbStaState *sta);
  ~LRHelper();

  VertexSeq &ensureSorted();
  bool KKTProjection();
  void updateAllEdgeLms(dbSta *sta);
  void enqueueVertex(Vertex *vertex);

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

  SearchPredNonLatch2* search_non_latch_pred_;
  BfsFwdIterator* iter_;
  VertexSeq sorted_lm_vertices_;
  bool levelized_;

  
private:
  friend class sta::Graph;
  friend class SortVertexVisitor;
};

} // namespace sta


#include "sta/Bfs.hh"
#include "LrHelper.hh"
#include "SearchPred.hh"
#include "search/Levelize.hh"
#include "sta/Corner.hh"
#include "DcalcAnalysisPt.hh"
#include "TimingArc.hh"
#include "VertexVisitor.hh"
#include "Sta.hh"


namespace lrf {
using sta::ClockEdge;
using sta::VertexVisitor;

static const sta::ClockEdge *clk_edge_wildcard = reinterpret_cast<ClockEdge*>(1);

//////////////////////////////////////////////////////////////////////
// SortVertexVisitor (define before use)
//////////////////////////////////////////////////////////////////////

class SortVertexVisitor : public VertexVisitor {
public:
  explicit SortVertexVisitor(LRHelper *lr_helper);
  ~SortVertexVisitor() override;
  void visit(Vertex *vertex) override;
  VertexVisitor *copy() const override;
protected:
  LRHelper *lr_helper_;
};

SortVertexVisitor::SortVertexVisitor(LRHelper *lr_helper) :
  lr_helper_(lr_helper)
{
}

SortVertexVisitor::~SortVertexVisitor()
{
}

void
SortVertexVisitor::visit(Vertex *vertex)
{
  lr_helper_->enqueueVertex(vertex);
}

VertexVisitor *
SortVertexVisitor::copy() const
{
  printf("SortVertexVisitor::copy() called - not implemented\n");
  return nullptr;
}

//////////////////////////////////////////////////////////////////////
// LRHelper

LRHelper::LRHelper(sta* sta) :
  StaState(sta),
  search_non_latch_pred_(new SearchPredNonLatch2(sta)),
  iter_(new BfsFwdIterator(BfsIndex::topo, search_non_latch_pred_, sta)),
  levelized_(false)
{
}

LRHelper::~LRHelper() {
  delete search_non_latch_pred_;
  delete iter_;
}

VertexSeq &
LRHelper::ensureSorted() {
  if (levelized_)
    return sorted_lm_vertices_;
  
  levelize_->ensureLevelized();
  iter_->clear();

  Level max_level = levelize_->maxLevel();
  
  SortVertexVisitor visitor(this);
  for (Vertex* root : levelize_->roots()) {
    iter_->enqueue(root);
  }
  iter_->visit(max_level, &visitor);
  levelized_ = true;
  return sorted_lm_vertices_;
}

// KKTProjection performs the Karush-Kuhn-Tucker projection step.
bool
LRHelper::KKTProjection() {
  printf("LRHelper::KKTProjection()\n");
  fflush(stdout);
  // Ensure vertices are sorted in topological order
  const VertexSeq &sorted_vertices = ensureSorted();

  // Map from analysis point to in LM sums of early/late
  DcalcAPToLMValueSeqMap ap_lm_seq_map;
  for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
    ap_lm_seq_map[dcalc_ap] = LMValueSeq();
  }
  size_t in_sum_index = computeInLmSums(ap_lm_seq_map);

  // From outputs to inputs compute output LM sums
  for (auto vertex_it = sorted_vertices.rbegin(); 
       vertex_it != sorted_vertices.rend(); ++vertex_it) {
    in_sum_index--;
    Vertex *vertex = *vertex_it;
    LMValueSeq out_lm_sums = computeOutLmSum(vertex);
    distributeLmOutToIn(vertex, out_lm_sums,
                        ap_lm_seq_map, in_sum_index);
  }
  bool kkt_satisfied = checkKKTForAllVertices();

  return kkt_satisfied;
}

bool
LRHelper::checkKKTForAllVertices() {
  printf("LRHelper::checkKKTForAllVertices()\n");
  fflush(stdout);
  bool all_satisfied = true;
  const VertexSeq &ordered = sorted_lm_vertices_;
  for (auto vertex_it = ordered.begin(); 
    vertex_it != ordered.end(); ++vertex_it) {
    DcalcAPToLMValueMap out_lm_map;
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();

      LMValue out_lm_sum = 0.0;
      size_t out_edge_count = 0;
      VertexOutEdgeIterator out_edge_iter(*vertex_it, graph_);
      while (out_edge_iter.hasNext()) {
        Edge *out_edge = out_edge_iter.next();
        // printf("LRHelper::checkKKTForAllVertices: vertex %s processing out edge %s\n",
        //        (*vertex_it)->to_string(graph_).c_str(),
        //        out_edge->to_string(graph_).c_str());
        //        fflush(stdout);
        LMValue const *lms = out_edge->arcLms();

        for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          LMValue arc_lm = lms[lm_idx];
          out_lm_sum += arc_lm;
        }
        out_edge_count++;
      }
      if (out_edge_count == 0) {
        out_lm_sum = -1.0; // Indicate no outputs
      }
      out_lm_map[dcalc_ap] = out_lm_sum;
    }

    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();

      LMValue in_lm_sum = 0.0;
      VertexInEdgeIterator in_edge_iter(*vertex_it, graph_);
      size_t in_edge_count = 0;
      while (in_edge_iter.hasNext()) {
        Edge *in_edge = in_edge_iter.next();
        LMValue const *lms = in_edge->arcLms();
        // printf("LRHelper::checkKKTForAllVertices: vertex %s processing in edge %s\n",
        //        (*vertex_it)->to_string(graph_).c_str(),
        //        in_edge->to_string(graph_).c_str());
        //        fflush(stdout);
        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          LMValue arc_lm = lms[lm_idx];
          in_lm_sum += arc_lm;
        }
        in_edge_count++;
      }

      LMValue out_lm_sum = out_lm_map[dcalc_ap];
      const float epsilon = 1e-4;
      if ((std::abs(out_lm_sum - in_lm_sum) > epsilon) && !(in_edge_count == 0) &&
          !(out_lm_sum == -1.0)) {
        all_satisfied = false;
        printf("LRHelper::checkKKTForAllVertices: vertex %s KKT not satisfied for AP corner %s, delay min/max %s, slew min/max %s: out LM sum %.6f != in LM sum %.6f\n",
                (*vertex_it)->to_string(graph_).c_str(),
                dcalc_ap->corner()->name(),
                dcalc_ap->delayMinMax()->to_string().c_str(),
                dcalc_ap->slewMinMax()->to_string().c_str(),
                out_lm_sum, in_lm_sum);
        fflush(stdout);
      }
    }
  }
  return all_satisfied;
}

void
LRHelper::distributeLmOutToIn(Vertex *vertex,
                              LMValueSeq &out_lm_sums,
                              DcalcAPToLMValueSeqMap &in_lm_seq_map,
                              size_t in_sum_index)
{
  VertexInEdgeIterator in_edge_iter(vertex, graph_);
  while (in_edge_iter.hasNext()) {
    Edge *in_edge = in_edge_iter.next();
    LMValue *lms = in_edge->arcLms();
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();
      LMValue out_lm_sum = out_lm_sums[ap_index];
      LMValue in_lm_sum = in_lm_seq_map[dcalc_ap][in_sum_index];
      if (in_lm_sum == 0.0) {
        printf("LRHelper::distributeLmOutToIn: vertex %s has zero in LM sum, skipping distribution\n",
               vertex->to_string(graph_).c_str());
               fflush(stdout);
        continue;
      }
      for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
        size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
        lms[lm_idx] = out_lm_sum * (lms[lm_idx] / in_lm_sum);
      }
    }
  }
}

size_t
LRHelper::computeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_map)
{
  size_t in_sum_index = 0;
  const VertexSeq &ordered = sorted_lm_vertices_;
  for (auto vertex_it = ordered.begin(); 
  vertex_it != ordered.end(); ++vertex_it) {
      Vertex *vertex = *vertex_it;
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();

      LMValue in_lm_sum = 0.0;
      VertexInEdgeIterator in_edge_iter(vertex, graph_);
      size_t in_edge_count = 0;
      while (in_edge_iter.hasNext()) {
        Edge *in_edge = in_edge_iter.next();
        LMValue const *lms = in_edge->arcLms();

        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          // Sum early and late LMs separately
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          in_lm_sum += lms[lm_idx];
        }
        in_edge_count++;
      }
      if (in_lm_sum == 0.0 || in_edge_count == 0) {
        printf("LRHelper::computeInLmSums: vertex %s has no in LM edges, setting in LM sum to 1.0\n",
               vertex->to_string(graph_).c_str());
               fflush(stdout);
        in_lm_sum = 1.0;
      }
      ap_lm_map[dcalc_ap].push_back(in_lm_sum);
    }
    in_sum_index++;
  }
  return in_sum_index;
}

LMValueSeq
LRHelper::computeOutLmSum(Vertex *vertex) const
{
  LMValueSeq out_lm_sums(graph_->apCount(), 0.0);
  VertexOutEdgeIterator out_edge_iter(vertex, graph_);
  while (out_edge_iter.hasNext()) {
    Edge *out_edge = out_edge_iter.next();
    LMValue const *lms = out_edge->arcLms();
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();
      for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
        size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
        LMValue arc_lm = lms[lm_idx];
        out_lm_sums[ap_index] += arc_lm;
      }
    }
  }
  return out_lm_sums;
}

void
LRHelper::updateAllEdgeLms(Sta *sta) {
  printf("LRHelper::updateAllEdgeLms()\n");
  fflush(stdout);
  for (auto vertex_it = sorted_lm_vertices_.begin(); 
       vertex_it != sorted_lm_vertices_.end(); ++vertex_it) {
    Vertex *vertex = *vertex_it;
    VertexOutEdgeIterator out_edge_iter(vertex, graph_);
    while (out_edge_iter.hasNext()) {
      Edge *out_edge = out_edge_iter.next();
      updateEdgeLms(out_edge, sta);
    }
  }
}

void 
LRHelper::updateEdgeLms(Edge *edge, Sta *sta) {
  for (TimingArc *arc : edge->timingArcSet()->arcs()) {
    updateArcLms(edge, arc, sta);
  }
}

void 
LRHelper::updateArcLms(Edge *edge, TimingArc *arc, Sta *sta) {
  for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
    const size_t ap_index = dcalc_ap->index();
    size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
    Vertex *from_vertex = edge->from(graph_);;
    Vertex *to_vertex = edge->to(graph_);
    
    RiseFall const *from_rf = arc->fromEdge()->asRiseFall();
    RiseFall  const *to_rf = arc->toEdge()->asRiseFall();
    MinMax const *delay_minmax = dcalc_ap->delayMinMax();
    Arrival aat = sta->vertexArrival(from_vertex, from_rf, 
        clk_edge_wildcard, nullptr,  delay_minmax);
    Arrival rat = sta->vertexArrival(to_vertex, to_rf,
        clk_edge_wildcard, nullptr,  delay_minmax);
    Delay delay = sta->arcDelay(edge, arc, dcalc_ap);
    LMValue *lms = edge->arcLms();
    lms[lm_idx] = (delay_minmax == MinMax::max()) ?
                  (lms[lm_idx] * (aat + delay) / rat) :
                  (lms[lm_idx] * rat / (aat + delay));
  }
}

void
LRHelper::enqueueVertex(Vertex *vertex) {
  sorted_lm_vertices_.push_back(vertex);
  iter_->enqueueAdjacentVertices(vertex);
}


}

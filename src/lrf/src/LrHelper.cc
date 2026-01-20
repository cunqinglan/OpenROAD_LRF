
// Simplified helper implementation for initial lrf build.
#include "lrf/LrfClass.hh"
#include "LrHelper.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/TimingArc.hh"
#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "sta/Network.hh"
#include "sta/Bfs.hh"
#include "search/Levelize.hh"
#include "sta/SearchPred.hh"
#include "sta/Corner.hh"
#include "sta/SdcClass.hh"
#include "sta/Clock.hh"
#include "sta/TimingRole.hh"
#include "sta/Sdc.hh"

namespace lrf {
using namespace sta;

static const ClockEdge *clk_edge_wildcard = reinterpret_cast<ClockEdge*>(1);

class VertexLevelLess
{
 public:
  VertexLevelLess(const Network* network);
  bool operator()(const Vertex* vertex1, const Vertex* vertex2) const;

 protected:
  const Network* network_;
};

VertexLevelLess::VertexLevelLess(const Network* network) : network_(network)
{
}

bool VertexLevelLess::operator()(const Vertex* vertex1,
                                 const Vertex* vertex2) const
{
  Level level1 = vertex1->level();
  Level level2 = vertex2->level();
  return (level1 < level2)
         || (level1 == level2
             // Break ties for stable results.
             && stringLess(network_->pathName(vertex1->pin()),
                           network_->pathName(vertex2->pin())));
}

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

LRHelper::LRHelper(StaState* sta) :
  StaState(sta),
  search_non_latch_pred_(new SearchPredNonLatch2(sta)),
  iter_(new BfsFwdIterator(BfsIndex::topo, search_non_latch_pred_, sta)),
  levelized_valid_(false)
{
}

LRHelper::~LRHelper() {
  delete search_non_latch_pred_;
  delete iter_;
}

void
LRHelper::copyState(const StaState *sta) {
  StaState::copyState(sta);
  // Notify sub-components.
  iter_->copyState(sta);
}

VertexSeq &
LRHelper::ensureSorted(Sta *sta) {
  sta->ensureLevelized();
  copyState(sta);
  levelSort(sta);
  return sorted_lm_vertices_;
}

void
LRHelper::levelSort(Sta *sta) {
  sta->ensureLevelized();
  sorted_lm_vertices_.clear();
  VertexIterator iter(sta->graph());
  while (iter.hasNext()) {
    Vertex *vertex = iter.next();
    sorted_lm_vertices_.push_back(vertex);
  }
  sort(sorted_lm_vertices_, VertexLevelLess(sta->network()));
}

void
LRHelper::BFSSort() {
  sorted_lm_vertices_.clear();
  levelize_->ensureLevelized();
  iter_->clear();

  Level max_level = levelize_->maxLevel();
  
  SortVertexVisitor visitor(this);
  for (Vertex* root : levelize_->roots()) {
    iter_->enqueue(root);
  }
  iter_->visit(max_level, &visitor);
  levelized_valid_ = true;
}

void 
LRHelper::computeInLmSums(std::vector<LMValue> &in_lm_sums) {
  int ap_count = graph_->corners()->dcalcAnalysisPtCount();
  in_lm_sums.resize(sorted_lm_vertices_.size() * ap_count, 1.0);
  for (size_t vertex_idx = 0; vertex_idx < sorted_lm_vertices_.size(); ++vertex_idx) {
    Vertex *vertex = sorted_lm_vertices_[vertex_idx];
    if (vertex->isRoot() || !vertex->hasFanin()) {
      continue;
    }
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();
      size_t in_lm_index = vertex_idx * ap_count + ap_index;
      VertexInEdgeIterator in_edge_iter(vertex, graph_);
      LMValue in_lm_sum = 0.0;
      size_t in_edge_count = 0;
      while (in_edge_iter.hasNext()) {
        Edge *in_edge = in_edge_iter.next();
        if (!search_non_latch_pred_->searchThru(in_edge)) {
          continue;
        }
        LMValue const *lms = in_edge->arcLms();
        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          size_t arc_lm_index = lmIndex(arc, graph_->apCount(), ap_index);
          in_lm_sum += lms[arc_lm_index];
        }
        in_edge_count++;
      }
      if (in_edge_count > 0) {
        in_lm_sums[in_lm_index] = in_lm_sum;
        if (in_lm_sum <= 0.0) {
          printf("LRHelper::computeInLmSums: vertex %s has non-positive in LM sum %.6f for ap %s\n",
                 vertex->to_string(graph_).c_str(),
                 in_lm_sum,
                 dcalc_ap->corner()->name());
          fflush(stdout);
        }
      } else {
        printf("LRHelper::computeInLmSums: vertex %s has no inputs for ap %s\n",
               vertex->to_string(graph_).c_str(),
               dcalc_ap->corner()->name());
        fflush(stdout);
        in_lm_sums[in_lm_index] = 1.0; // Indicate no inputs
      }
    }
  }
}

LMValue
LRHelper::computeOutVertexLmSum(Vertex *vertex,
                         DcalcAnalysisPt const* dcalc_ap) const 
{
  LMValue out_lm_sum = 0.0;
  const size_t ap_index = dcalc_ap->index();
  VertexOutEdgeIterator out_edge_iter(vertex, graph_);
  size_t out_edge_count = 0;
  while (out_edge_iter.hasNext()) {
    Edge *out_edge = out_edge_iter.next();
    if (!search_non_latch_pred_->searchThru(out_edge) || 
        !search_non_latch_pred_->searchTo(out_edge->to(graph_))) {
      continue;
    }
    const LMValue *lms = out_edge->arcLms();
    for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
      size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
      out_lm_sum += lms[lm_index];
    }
    out_edge_count++;
  }
  if (out_edge_count == 0 || out_lm_sum < 0.0) {
    // printf("ERROR: LRHelper::computeOutVertexLmSum: vertex %s has no out edges for ap %s\n",
    //        vertex->to_string(graph_).c_str(),
    //        dcalc_ap->corner()->name());
    // fflush(stdout);
    // return -1.0;
    throw std::runtime_error("LRHelper::computeOutVertexLmSum: no out edges");
  }
  return out_lm_sum;
}

void
LRHelper::distributeLmOutToIn(Vertex *vertex)
{
  for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
    const size_t ap_index = dcalc_ap->index();
    
    // Compute out LM sum on-the-fly to ensure freshness
    LMValue out_lm_sum = computeOutVertexLmSum(vertex, dcalc_ap);
    
    // Compute in LM sum on-the-fly to ensure freshness
    LMValue in_lm_sum = computeInVertexLmSum(vertex, dcalc_ap);
    
    float ratio = out_lm_sum / in_lm_sum;
    
    // Distribute out LM sum to in edges
    VertexInEdgeIterator in_edge_iter(vertex, graph_);
    while (in_edge_iter.hasNext()) {
      Edge *in_edge = in_edge_iter.next();
      if (!search_non_latch_pred_->searchThru(in_edge) || 
          !search_non_latch_pred_->searchFrom(in_edge->from(graph_))) {
        continue;
      }
      LMValue *lms = in_edge->arcLms();
      for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
        size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
        if (lms[lm_index] < 0.0) {
          printf("Error: LRHelper::distributeLmOutToIn (before update): edge %s AP corner %s: negative LM %.6f\n",
                 in_edge->to_string(graph_).c_str(),
                 dcalc_ap->corner()->name(),
                 lms[lm_index]);
          fflush(stdout);
          throw std::runtime_error("LRHelper::distributeLmOutToIn: negative LM");
        }
        lms[lm_index] = lms[lm_index] * ratio;
      }
    }
  }
}

void 
LRHelper::computeOutLmSumsAndDistributeToIn(const VertexSeq &sorted_vertices,
                                      std::vector<LMValue> & /* unused */) {
  int ap_count = graph_->corners()->dcalcAnalysisPtCount();
  for (size_t vertex_idx = sorted_vertices.size(); vertex_idx-- > 0; ) {
    Vertex *vertex = sorted_vertices[vertex_idx];
    if (vertex->isRoot() || !vertex->hasFanin() || !vertex->hasFanout()
  || search_non_latch_pred_->searchTo(vertex)
  || search_non_latch_pred_->searchFrom(vertex)) {
      continue;
    }
    distributeLmOutToIn(vertex);
  }
}



// KKTProjection performs the Karush-Kuhn-Tucker projection step.
bool 
LRHelper::KKTProjection(Sta *sta)
{
  printf("LRHelper::KKTProjection()\n");
  fflush(stdout);
  // Ensure vertices are sorted in topological order
  const VertexSeq &sorted_vertices = ensureSorted(sta);
  std::vector<LMValue> in_lm_sums;
  // computeInLmSums(in_lm_sums);
  computeOutLmSumsAndDistributeToIn(sorted_vertices, in_lm_sums);

  bool kkt_satisfied = checkKKTForAllVertices();
  if (kkt_satisfied) {
    printf("LRHelper::KKTProjection(): KKT conditions satisfied\n");
  } else {
    printf("LRHelper::KKTProjection(): KKT conditions NOT satisfied\n");
  }
  fflush(stdout);

  return kkt_satisfied;
}

// // KKTProjection performs the Karush-Kuhn-Tucker projection step.
bool
LRHelper::KKTProjection1(Sta *sta) {
  printf("LRHelper::KKTProjection()\n");
  fflush(stdout);
  // Ensure vertices are sorted in topological order
  const VertexSeq &sorted_vertices = ensureSorted(sta);

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
    if (vertex->isRoot()) {
      continue;
    }
    LMValueSeq out_lm_sums = computeOutLmSum(vertex);
    distributeLmOutToIn(vertex, out_lm_sums,
                        ap_lm_seq_map, in_sum_index);
  }
  bool kkt_satisfied = checkKKTForAllVertices();
  if (kkt_satisfied) {
    printf("LRHelper::KKTProjection(): KKT conditions satisfied\n");
  } else {
    printf("LRHelper::KKTProjection(): KKT conditions NOT satisfied\n");
  }
  fflush(stdout);

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
    if ((*vertex_it)->isRoot() || !(*vertex_it)->hasFanin() || !(*vertex_it)->hasFanout()
  || search_non_latch_pred_->searchTo(*vertex_it)
  || search_non_latch_pred_->searchFrom(*vertex_it)) {
      continue;
    }
    DcalcAPToLMValueMap out_lm_map;
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();

      LMValue out_lm_sum = 0.0;
      size_t out_edge_count = 0;
      VertexOutEdgeIterator out_edge_iter(*vertex_it, graph_);
      while (out_edge_iter.hasNext()) {
        Edge *out_edge = out_edge_iter.next();
        if (!search_non_latch_pred_->searchThru(out_edge) || 
            !search_non_latch_pred_->searchTo(out_edge->to(graph_))) {
          continue;
        }
        // printf("LRHelper::checkKKTForAllVertices: vertex %s processing out edge %s\n",
        //        (*vertex_it)->to_string(graph_).c_str(),
        //        out_edge->to_string(graph_).c_str());
        //        fflush(stdout);
        LMValue const *lms = out_edge->arcLms();

        for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
          size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
          LMValue arc_lm = lms[lm_index];
          if (arc_lm < 0.0) {
            printf("Error: LRHelper::checkKKTForAllVertices: vertex %s out edge %s AP corner %s: negative LM %.6f\n",
                   (*vertex_it)->to_string(graph_).c_str(),
                   out_edge->to_string(graph_).c_str(),
                   dcalc_ap->corner()->name(),
                   arc_lm);
            fflush(stdout);
            throw std::runtime_error("LRHelper::checkKKTForAllVertices: negative LM");
          }
          out_lm_sum += arc_lm;
        }
        out_edge_count++;
      }
      if (out_edge_count == 0) {
        throw std::runtime_error("LRHelper::checkKKTForAllVertices: no out edges");
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
        if (!search_non_latch_pred_->searchThru(in_edge) || 
            !search_non_latch_pred_->searchFrom(in_edge->from(graph_))) {
          continue;
        }
        LMValue const *lms = in_edge->arcLms();
        // printf("LRHelper::checkKKTForAllVertices: vertex %s processing in edge %s\n",
        //        (*vertex_it)->to_string(graph_).c_str(),
        //        in_edge->to_string(graph_).c_str());
        //        fflush(stdout);
        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
          LMValue arc_lm = lms[lm_index];
          in_lm_sum += arc_lm;
        }
        in_edge_count++;
      }

      LMValue out_lm_sum = out_lm_map[dcalc_ap];
      const float epsilon = 1e-17;
      if ((std::abs(out_lm_sum - in_lm_sum) > epsilon) && !(in_edge_count == 0) &&
          !(out_lm_sum == -1.0)) {
        all_satisfied = false;
        printf("LRHelper::checkKKTForAllVertices: vertex %s KKT not satisfied for AP corner %s, level = %d\n",
                (*vertex_it)->to_string(graph_).c_str(),
                dcalc_ap->corner()->name(),
                (*vertex_it)->level());
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
    if (in_edge->role()->isTimingCheck()) {
      continue;
    }
    LMValue *lms = in_edge->arcLms();
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();
      LMValue out_lm_sum = out_lm_sums[ap_index];
      LMValue in_lm_sum = in_lm_seq_map[dcalc_ap][in_sum_index];
      if (out_lm_sum == 0.0) {
        // No output LM to distribute
        continue;
      }
      if (in_lm_sum == 0.0) {
        printf("LRHelper::distributeLmOutToIn: vertex %s edge %s AP corner %s, delay min/max %s: in LM sum is zero, skipping distribution\n",
               vertex->to_string(graph_).c_str(),
               in_edge->to_string(graph_).c_str(),
               dcalc_ap->corner()->name(),
               dcalc_ap->delayMinMax()->to_string().c_str());
               fflush(stdout);
        continue;
      }
      for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
        size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
        lms[lm_index] = out_lm_sum * (lms[lm_index] / in_lm_sum);
      }
    }
  }
}

LMValue
LRHelper::computeInVertexLmSum(Vertex *vertex, DcalcAnalysisPt const* dcalc_ap) const
{
  VertexInEdgeIterator in_edge_iter(vertex, graph_);
  size_t in_edge_count = 0;
  size_t ap_index = dcalc_ap->index();
  LMValue in_lm_sum = 0.0;
  while (in_edge_iter.hasNext()) {
    Edge *in_edge = in_edge_iter.next();
    if (!search_non_latch_pred_->searchThru(in_edge) || 
        !search_non_latch_pred_->searchFrom(in_edge->from(graph_))) {
      continue;
    }
    LMValue const *lms = in_edge->arcLms();

    for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
      // Sum early and late LMs separately
      size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
      in_lm_sum += lms[lm_index];
    }
    in_edge_count++;
  }
  if (in_edge_count == 0 || in_lm_sum < 0.0) {
    printf("LRHelper::computeInLmSums: vertex %s has lmSum %f, in_edge_count %zu, setting in LM sum to 1.0\n",
            vertex->to_string(graph_).c_str(),
            in_lm_sum,
            in_edge_count);
            fflush(stdout);
    throw std::runtime_error("LRHelper::computeInLmSum: zero in LM sum or no in edges");
  }
  return in_lm_sum;
}

LMValue
LRHelper::computeInLmSum(Vertex *vertex, DcalcAnalysisPt const* dcalc_ap) const
{
  VertexInEdgeIterator in_edge_iter(vertex, graph_);
  size_t in_edge_count = 0;
  size_t ap_index = dcalc_ap->index();
  LMValue in_lm_sum = 0.0;
  while (in_edge_iter.hasNext()) {
    const Edge *in_edge = in_edge_iter.next();
    if (in_edge->role()->isTimingCheck()) {
      continue;
    }
    LMValue const *lms = in_edge->arcLms();

    for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
      // Sum early and late LMs separately
      size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
      in_lm_sum += lms[lm_index];
    }
    in_edge_count++;
  }
  if (in_edge_count == 0 || in_lm_sum < 0.0) {
    printf("LRHelper::computeInLmSums: vertex %s has lmSum %f, in_edge_count %zu, setting in LM sum to 1.0\n",
            vertex->to_string(graph_).c_str(),
            in_lm_sum,
            in_edge_count);
            fflush(stdout);
    throw std::runtime_error("LRHelper::computeInLmSum: zero in LM sum or no in edges");
  }
  return in_lm_sum;
}


size_t
LRHelper::computeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_map)
{
  size_t in_sum_index = 0;
  const VertexSeq &ordered = sorted_lm_vertices_;
  for (auto vertex_it = ordered.begin(); 
  vertex_it != ordered.end(); ++vertex_it) {
    Vertex *vertex = *vertex_it;

    if (vertex->isRoot() || !vertex->hasFanin()) {
      for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
        ap_lm_map[dcalc_ap].push_back(1.0);
      }
      in_sum_index++;
      continue;
    }

    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      LMValue in_lm_sum = computeInLmSum(vertex, dcalc_ap);
      
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
  if (!vertex->hasFanout()) {
    // No outputs, return zero sums。 In fact, if no outputs, the out_lm_sums
    // will not be used.
    return out_lm_sums;
  }
  while (out_edge_iter.hasNext()) {
    Edge *out_edge = out_edge_iter.next();
    if (out_edge->role()->isTimingCheck()) {
      continue;
    }
    LMValue const *lms = out_edge->arcLms();
    for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
      const size_t ap_index = dcalc_ap->index();
      for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
        size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
        LMValue arc_lm = lms[lm_index];
        out_lm_sums[ap_index] += arc_lm;
      }
    }
  }
  return out_lm_sums;
}

float 
LRHelper::updateLmMultiplier(Edge *edge,
                        TimingArc *arc,
                        DcalcAnalysisPt const *dcalc_ap,
                        Sta *sta)
{
  float multiplier = 1.0;
  Vertex *from_vertex = edge->from(graph_);
  RiseFall  const *from_rf = arc->fromEdge()->asRiseFall();
  MinMax const *delay_minmax = dcalc_ap->delayMinMax();
  double clock_period = 0.0;
  for (Clock const* clk : *(sdc_->clocks())) {
    if (clk->period() > clock_period) {
      clock_period = clk->period();
      break;
    }
  }
  if (clock_period == 0.0) {
    throw std::runtime_error("LRHelper::updateLmValue: zero clock period");
  }
  if (delay_minmax == MinMax::max()) {
    Arrival from_aat = sta->pinArrival(from_vertex->pin(), from_rf,
        delay_minmax);
    ArcDelay delay = sta->arcDelay(edge, arc, dcalc_ap);
    Required to_rat = sta->vertexRequired(from_vertex, from_rf,
        delay_minmax);
    Slack arc_slack = to_rat - (from_aat + delay);
    
    multiplier = 1.0 - (arc_slack / clock_period);
    // printf("LRHelper::updateLmValue: edge %s AP corner %s delay min/max %s: to_slack %.6f ps, clock_period %.6f ps, updated LM %.6f\n",
    //        edge->to_string(graph_).c_str(),
    //        dcalc_ap->corner()->name(),
    //        delay_minmax->to_string().c_str(),
    //        arc_slack * 1.0e12,
    //        clock_period * 1.0e12,
    //        multiplier);
    // fflush(stdout);
  } else {
    Arrival from_aat = sta->pinArrival(from_vertex->pin(), from_rf,
        delay_minmax);
    ArcDelay delay = sta->arcDelay(edge, arc, dcalc_ap);
    Required to_rat = sta->vertexRequired(from_vertex, from_rf,
        delay_minmax);
    Slack arc_slack = (from_aat + delay) - to_rat;
    multiplier = 1.0 - (arc_slack / clock_period);
  }
  return pow(multiplier, 2);
}

void
LRHelper::updateEndpointsArcLms(Edge *edge, TimingArc *arc, Sta *sta) 
{
  for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
    const size_t ap_index = dcalc_ap->index();
    size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
    Vertex *from_vertex = edge->from(graph_);;
    Vertex *to_vertex = edge->to(graph_);
    
    RiseFall const *from_rf = arc->fromEdge()->asRiseFall();
    RiseFall  const *to_rf = arc->toEdge()->asRiseFall();
    MinMax const *delay_minmax = dcalc_ap->delayMinMax();
    Arrival from_aat = sta->pinArrival(from_vertex->pin(), from_rf,
        delay_minmax);
    Arrival to_rat = sta->vertexRequired(to_vertex, to_rf,
        delay_minmax);
    Delay delay = sta->arcDelay(edge, arc, dcalc_ap);
    LMValue *lms = edge->arcLms();
    lms[lm_index] = lms[lm_index] * updateLmMultiplier(edge, arc, dcalc_ap, sta);
  }
}

void
LRHelper::updateEndpointLms(Vertex *vertex, Sta *sta) 
{
  VertexInEdgeIterator in_edge_iter(vertex, graph_);
  while (in_edge_iter.hasNext()) {
    Edge *in_edge = in_edge_iter.next();
    for (sta::TimingArc *arc : in_edge->timingArcSet()->arcs()) {
      updateEndpointsArcLms(in_edge, arc, sta);
    }
  }
}

void
LRHelper::updateAllEdgeLms(Sta *sta) {
  sta->findRequireds();
  printf("Size of sorted_lm_vertices_: %zu\n", sorted_lm_vertices_.size());
  fflush(stdout);
  VertexSet *endpoints = sta->endpoints();
  for (Vertex *vertex : *endpoints) {
    vertex->setIsEndpoint(true);
    updateEndpointLms(vertex, sta);
  }
  copyState(sta);
  for (auto vertex_it = sorted_lm_vertices_.begin(); 
       vertex_it != sorted_lm_vertices_.end(); ++vertex_it) {
    Vertex *vertex = *vertex_it;
    VertexInEdgeIterator in_edge_iter(vertex, graph_);
    while (in_edge_iter.hasNext()) {
      Edge *in_edge = in_edge_iter.next();
      // if (out_edge->role()->isTimingCheck()) {
      if (!search_non_latch_pred_->searchThru(in_edge)) {
        continue;
      }
      updateEdgeLms(in_edge, sta);
    }
  }
}

void 
LRHelper::updateEdgeLms(Edge *edge, Sta *sta) {
  for (TimingArc *arc : edge->timingArcSet()->arcs()) {
    if (search_non_latch_pred_->searchThru(edge) && 
        search_non_latch_pred_->searchFrom(edge->from(graph_)) &&
        search_non_latch_pred_->searchTo(edge->to(graph_))) {
      updateArcLms(edge, arc, sta);
    }
  }
}

void 
LRHelper::updateArcLms(Edge *edge, TimingArc *arc, Sta *sta) {
  for (DcalcAnalysisPt const *dcalc_ap : graph_->corners()->dcalcAnalysisPts()) {
    const size_t ap_index = dcalc_ap->index();
    size_t lm_index = lmIndex(arc, graph_->apCount(), ap_index);
    Vertex *from_vertex = edge->from(graph_);;
    Vertex *to_vertex = edge->to(graph_);
    
    RiseFall const *from_rf = arc->fromEdge()->asRiseFall();
    RiseFall  const *to_rf = arc->toEdge()->asRiseFall();
    MinMax const *delay_minmax = dcalc_ap->delayMinMax();
    Arrival from_aat = sta->pinArrival(from_vertex->pin(), from_rf,
        delay_minmax);
    Arrival to_aat = sta->pinArrival(to_vertex->pin(), to_rf,
        delay_minmax);
    Delay delay = sta->arcDelay(edge, arc, dcalc_ap);
    LMValue *lms = edge->arcLms();
    // LMValue origin = lms[lm_index];
    lms[lm_index] = lms[lm_index] * updateLmMultiplier(edge, arc, dcalc_ap, sta);
  }
}

void
LRHelper::enqueueVertex(Vertex *vertex) {
  sorted_lm_vertices_.push_back(vertex);
  iter_->enqueueAdjacentVertices(vertex);
}


} // namespace lrf

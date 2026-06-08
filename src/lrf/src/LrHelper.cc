
// Simplified helper implementation for initial lrf build.
#include "lrf/LrfClass.hh"
#include "LrHelper.hh"
#include "LrfUtil.hh"
#include "LmHistory.hh"
#include "sta/TimingArc.hh"
#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "sta/Network.hh"
#include "sta/Bfs.hh"
#include "search/Levelize.hh"
#include "sta/SearchPred.hh"
#include "sta/Scene.hh"
#include "sta/TimingRole.hh"
#include "sta/Clock.hh"
#include "sta/Sdc.hh"
#include "sta/DispatchQueue.hh"

namespace lrf {
using namespace sta;

static const LMValue MAX_LM_VALUE = 40.0;
static const LMValue MIN_LM_VALUE = 1e-16;

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
             && (network_->pathName(vertex1->pin()) < network_->pathName(vertex2->pin())));
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

LRHelper::LRHelper(dbSta* sta) :
  search_pred_(new SearchPred1(sta)),
  iter_(new BfsFwdIterator(BfsIndex::topo, search_pred_, sta)),
  levelized_valid_(false),
  lm_history_(sta->graph())
{
  dbStaState::init(sta);
}

LRHelper::~LRHelper() {
  delete search_pred_;
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

// KKTProjection performs the Karush-Kuhn-Tucker projection step.
bool
LRHelper::KKTProjection(Sta *sta) {
  printf("LRHelper::KKTProjection()\n");
  fflush(stdout);
  // Ensure vertices are sorted in topological order
  const VertexSeq &sorted_vertices = ensureSorted(sta);

  // Map from analysis point to in LM sums of early/late
  DcalcAPToLMValueSeqMap ap_lm_seq_map;
  for (Scene *scene : scenes()) {
    for (const MinMax *min_max : MinMax::range()) {
      ap_lm_seq_map[scene->dcalcAnalysisPtIndex(min_max)] = LMValueSeq();
    }
  }
  size_t in_sum_index = computeInLmSums(ap_lm_seq_map);

  // From outputs to inputs compute output LM sums
  for (auto vertex_it = sorted_vertices.rbegin(); 
       vertex_it != sorted_vertices.rend(); ++vertex_it) {
    in_sum_index--;
    Vertex *vertex = *vertex_it;
    if (!hasFanin(vertex, search_pred_, graph_, modes()[0]) ||
        !hasFanout(vertex, search_pred_, graph_, modes()[0])) {
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
  LMValue max_lm = MIN_LM_VALUE;
  sta::Edge *max_lm_edge = nullptr;
  LMValue min_lm = MAX_LM_VALUE;
  sta::Edge *min_lm_edge = nullptr;
  for (auto vertex_it = ordered.begin(); 
    vertex_it != ordered.end(); ++vertex_it) {
    if (!hasFanin(*vertex_it, search_pred_, graph_, modes()[0]) || 
        !hasFanout(*vertex_it, search_pred_, graph_, modes()[0]) ||
        network_->isRegClkPin((*vertex_it)->pin())) {
      continue;
    }
    std::vector<LMValue> out_lm_vec(graph_->apCount(), 0.0);
    for (Scene *scene : scenes()) {
     for (const MinMax *min_max : MinMax::range()) {
      const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);

      LMValue out_lm_sum = 0.0;
      size_t out_edge_count = 0;
      VertexOutEdgeIterator out_edge_iter(*vertex_it, graph_);
      while (out_edge_iter.hasNext()) {
        Edge *out_edge = out_edge_iter.next();
        if (out_edge->role()->isTimingCheck()) {
          continue;
        }
        // printf("LRHelper::checkKKTForAllVertices: vertex %s processing out edge %s\n",
        //        (*vertex_it)->to_string(graph_).c_str(),
        //        out_edge->to_string(graph_).c_str());
        //        fflush(stdout);
        LMValue const *lms = out_edge->arcLms();

        for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          LMValue arc_lm = lms[lm_idx];
          out_lm_sum += arc_lm;
          ///////////////////
          if (arc_lm > max_lm) {
            max_lm = arc_lm;
            max_lm_edge = out_edge;
          }
          if (arc_lm < min_lm) {
            min_lm = arc_lm;
            min_lm_edge = out_edge;
            if (min_lm <= 0.0) {
              printf("LRHelper::checkKKTForAllVertices(): encountered zero LM value on vertex %s, edge %s, arc %s\n",
                     (*vertex_it)->to_string(graph_).c_str(),
                     out_edge->to_string(graph_).c_str(),
                     arc->to_string().c_str());
              fflush(stdout);
            }
          }
          ///////////////////
        }
        out_edge_count++;
      }
      if (out_edge_count == 0) {
        out_lm_sum = -1.0; // Indicate no outputs
      }
      out_lm_vec[ap_index] = out_lm_sum;
     }
    }

    for (Scene *scene : scenes()) {
     for (const MinMax *min_max : MinMax::range()) {
      const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);

      LMValue in_lm_sum = 0.0;
      VertexInEdgeIterator in_edge_iter(*vertex_it, graph_);
      size_t in_edge_count = 0;
      while (in_edge_iter.hasNext()) {
        Edge *in_edge = in_edge_iter.next();
        if (in_edge->role()->isTimingCheck()) {
          continue;
        }
        LMValue const *lms = in_edge->arcLms();
        // printf("LRHelper::checkKKTForAllVertices: vertex %s processing in edge %s\n",
        //        (*vertex_it)->to_string(graph_).c_str(),
        //        in_edge->to_string(graph_).c_str());
        //        fflush(stdout);
        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          LMValue arc_lm = lms[lm_idx];
          in_lm_sum += arc_lm;
          ///////////////////
          if (arc_lm > max_lm) {
            max_lm = arc_lm;
            max_lm_edge = in_edge;
          }
          if (arc_lm < min_lm) {
            min_lm = arc_lm;
            min_lm_edge = in_edge;
            if (min_lm <= 0.0) {
              printf("LRHelper::checkKKTForAllVertices(): encountered zero LM value on vertex %s, edge %s, arc %s\n",
                     (*vertex_it)->to_string(graph_).c_str(),
                     in_edge->to_string(graph_).c_str(),
                     arc->to_string().c_str());
              fflush(stdout);
            }
          }
          ///////////////////
        }
        in_edge_count++;
      }

      LMValue out_lm_sum = out_lm_vec[ap_index];
      const float epsilon = 1e-4;
      if (!(out_lm_sum == 0.0 && in_lm_sum == 0.0)
            && !(out_lm_sum == 0.0)   // All output edges disabled for this AP
            && !(in_lm_sum == 0.0)    // All input edges disabled for this AP
            && (std::abs(out_lm_sum - in_lm_sum)/out_lm_sum > epsilon)
            && !(in_edge_count == 0)
            && !(out_lm_sum == -1.0)) {
        all_satisfied = false;
        printf("LRHelper::checkKKTForAllVertices: vertex %s KKT not satisfied for AP corner %s, delay min/max %s, slew min/max %s: out LM sum %e != in LM sum %e\n",
                (*vertex_it)->to_string(graph_).c_str(),
                scene->name().c_str(),
                min_max->to_string().c_str(),
                min_max->to_string().c_str(),
                out_lm_sum, in_lm_sum);
        fflush(stdout);
      }
     }
    }
  }
  printf("LRHelper::checkKKTForAllVertices(): max LM (%s) & min LM (%s) value encountered: %.6f, %.6f\n", 
         max_lm_edge ? max_lm_edge->to_string(graph_).c_str() : "N/A",
         min_lm_edge ? min_lm_edge->to_string(graph_).c_str() : "N/A",
         max_lm, min_lm);
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
    for (Scene *scene : scenes()) {
     for (const MinMax *min_max : MinMax::range()) {
      const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
      LMValue out_lm_sum = out_lm_sums[ap_index];
      LMValue in_lm_sum = in_lm_seq_map[ap_index][in_sum_index];
      if (out_lm_sum == 0.0) {
        // All output edges disabled for this AP; propagate to input edges.
        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          lms[lm_idx] = 0.0;
        }
        continue;
      }
      if (in_lm_sum == 0.0) {
        printf("LRHelper::distributeLmOutToIn: vertex %s edge %s AP corner %s, delay min/max %s: in LM sum is zero, skipping distribution\n",
               vertex->to_string(graph_).c_str(),
               in_edge->to_string(graph_).c_str(),
               scene->name().c_str(),
               min_max->to_string().c_str());
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
}

size_t
LRHelper::computeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_map)
{
  size_t in_sum_index = 0;
  const VertexSeq &ordered = sorted_lm_vertices_;
  for (auto vertex_it = ordered.begin(); 
  vertex_it != ordered.end(); ++vertex_it) {
    Vertex *vertex = *vertex_it;

    if (!hasFanin(vertex, search_pred_, graph_, modes()[0])) {
      for (Scene *scene : scenes()) {
       for (const MinMax *min_max : MinMax::range()) {
        ap_lm_map[scene->dcalcAnalysisPtIndex(min_max)].push_back(0.0);
       }
      }
      in_sum_index++;
      continue;
    }

    for (Scene *scene : scenes()) {
     for (const MinMax *min_max : MinMax::range()) {
      const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);

      LMValue in_lm_sum = 0.0;
      VertexInEdgeIterator in_edge_iter(vertex, graph_);
      size_t in_edge_count = 0;
      while (in_edge_iter.hasNext()) {
        const Edge *in_edge = in_edge_iter.next();
        if (in_edge->role()->isTimingCheck()) {
          continue;
        }
      //   if (strict_constraint_ && (search_pred_->searchThru(in_edge)
      // || search_pred_->searchTo(in_edge->to(graph_))
      // || search_pred_->searchFrom(in_edge->from(graph_)))) {
      //     // Do not include edges through latches
      //     continue;
      //   }
        LMValue const *lms = in_edge->arcLms();

        for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
          // Sum early and late LMs separately
          size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
          in_lm_sum += lms[lm_idx];
        }
        in_edge_count++;
      }
      if (in_lm_sum == 0.0 || in_edge_count == 0) {
        // printf("LRHelper::computeInLmSums: vertex %s has no in LM edges, setting in LM sum to 1.0\n",
        //        vertex->to_string(graph_).c_str());
        //        fflush(stdout);
        in_lm_sum = 1.0;
      }
      ap_lm_map[ap_index].push_back(in_lm_sum);
     }
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
  if (!hasFanout(vertex, search_pred_, graph_, modes()[0])) {
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
    for (Scene *scene : scenes()) {
     for (const MinMax *min_max : MinMax::range()) {
      const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
      for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
        size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
        LMValue arc_lm = lms[lm_idx];
        out_lm_sums[ap_index] += arc_lm;
      }
     }
    }
  }
  return out_lm_sums;
}

void
LRHelper::clearLms(Edge *edge) {
  for (TimingArc *arc : edge->timingArcSet()->arcs()) {
    LMValue *lms = edge->arcLms();
    size_t ap_count = graph_->apCount();
    for (size_t i = 0; i < ap_count; i++) {
      lms[arc->index() * ap_count + i] = 0;
    }
  }
}

void
LRHelper::updateAllEdgeLms(Sta *sta) {
  if (lrfVerbose()) printf("Size of sorted_lm_vertices_: %zu\n", sorted_lm_vertices_.size());
  if (lrfVerbose()) printf("Using LRHelper strategy: %s\n", strategyName().c_str());
  fflush(stdout);
  sta->findRequireds();
  copyState(sta);
  for (auto vertex_it = sorted_lm_vertices_.begin(); 
       vertex_it != sorted_lm_vertices_.end(); ++vertex_it) {
    Vertex *vertex = *vertex_it;
    VertexOutEdgeIterator out_edge_iter(vertex, graph_);
    while (out_edge_iter.hasNext()) {
      Edge *out_edge = out_edge_iter.next();
      if (out_edge->role()->isTimingCheck()) {
        continue;
      }
      updateEdgeLms(out_edge, sta);
    }
  }
}

void
LRHelper::updateEndPointArcLms(Edge *edge, TimingArc *arc, Sta *sta, Scene *scene, const MinMax *min_max) {
  const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
  const RiseFall *from_rf = arc->fromEdge()->asRiseFall();
  const RiseFall *to_rf = arc->toEdge()->asRiseFall();
  const MinMax *delay_minmax = min_max;
  Delay delay = sta->arcDelay(edge, arc, ap_index);
  size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
  Vertex *from_vertex = edge->from(graph_);
  Vertex *to_vertex = edge->to(graph_);
  Arrival from_aat = sta->arrival(from_vertex, from_rf->asRiseFallBoth(), sta->scenes(), delay_minmax);
  Required to_rat = sta->required(to_vertex, to_rf->asRiseFallBoth(), sta->scenes(), delay_minmax);
  LMValue *lms = edge->arcLms();

  // Disabled edge: unconstrained timing values.
  if ((from_aat < 0.0 && to_rat < 0.0)
      || from_aat == INF || to_rat == INF
      || from_aat == -INF || to_rat == -INF) {
    lms[lm_idx] = 0.0;
    return;
  }
  from_aat = std::max(delayAsFloat(from_aat), 1e-17f);
  to_rat = std::max(delayAsFloat(to_rat), 1e-17f);

  if (delay_minmax == MinMax::max()) {
    // printf("LRHelper::updateEndPointArcLms: edge %s AP corner %s, delay min/max %s: aat %f, rat %f, delay %f, original LM %f\n",
    //         edge->to_string(graph_).c_str(),
    //         dcalc_ap->corner()->name(),
    //         dcalc_ap->delayMinMax()->to_string().c_str(),
    //         from_aat * 1.0e12, to_rat * 1.0e12, delay * 1.0e12,
    //         lms[lm_idx]);
    // fflush(stdout);
    lms[lm_idx] = lms[lm_idx] * (from_aat + delay) / to_rat;
    // if (lms[lm_idx] > MAX_LM_VALUE) lms[lm_idx] = MAX_LM_VALUE;
    // if (lms[lm_idx] < MIN_LM_VALUE) lms[lm_idx] = MIN_LM_VALUE;
  } else {
    lms[lm_idx] = lms[lm_idx] * to_rat / (from_aat + delay);
    // if (lms[lm_idx] > MAX_LM_VALUE) lms[lm_idx] = MAX_LM_VALUE;
    // if (lms[lm_idx] < MIN_LM_VALUE) lms[lm_idx] = MIN_LM_VALUE;
  }
  if (lms[lm_idx] < 0.0) {
    printf("LRHelper::updateEndPointArcLms: edge %s AP corner %s delay min/max %s: computed negative LM %.6f with aat %.6f, rat %.6f, delay %.6f\n",
            edge->to_string(graph_).c_str(),
            scene->name().c_str(),
            delay_minmax->to_string().c_str(),
            lms[lm_idx],
            from_aat * 1.0e12, to_rat * 1.0e12, delay * 1.0e12);
    fflush(stdout);
    lms[lm_idx] = 0.0;
  }
}

void 
LRHelper::updateEdgeLms(Edge *edge, Sta *sta) {
  // First annotate endpoints
  for (Vertex *vertex : sta->endpoints()) {
    vertex->setIsEndpoint(true);
  }
  for (Scene *scene : sta->scenes()) {
    for (const MinMax *min_max : MinMax::range()) {
      for (TimingArc *arc : edge->timingArcSet()->arcs()) {
        if (edge->to(graph_)->isEndPoint() && RATCONS_) {
          updateEndPointArcLms(edge, arc, sta, scene, min_max);
        } else
        updateArcLms(edge, arc, sta, scene, min_max);
      }
    }
  }
}

void 
LRHelper::updateArcLms(Edge *edge, TimingArc *arc, Sta *sta, Scene *scene, const MinMax *min_max) {
  const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
  size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
  Vertex *from_vertex = edge->from(graph_);
  Vertex *to_vertex = edge->to(graph_);

  RiseFall const *from_rf = arc->fromEdge()->asRiseFall();
  RiseFall  const *to_rf = arc->toEdge()->asRiseFall();
  MinMax const *delay_minmax = min_max;
  // Wildcard over clock edges: master dropped the clk_edge_wildcard escape
  // (Sta::arrival(...,clk_edge,...) now requires an exact clkEdge() match), so
  // use the RiseFallBoth overload which has no clk_edge filter and excludes
  // gen-clk source paths — identical to the old vertexArrival(clk_edge_wildcard)
  // semantics and consistent with RapidLrHelper::updateArcLms.
  Arrival from_aat = sta->arrival(from_vertex, from_rf->asRiseFallBoth(), sta->scenes(), delay_minmax);
  Arrival to_aat = sta->arrival(to_vertex, to_rf->asRiseFallBoth(), sta->scenes(), delay_minmax);
  Delay delay = sta->arcDelay(edge, arc, ap_index);
  LMValue *lms = edge->arcLms();
  LMValue origin = lms[lm_idx];

  // Disabled edge: unconstrained timing values.
  // max: both arrivals negative/uninitialized
  // min: arrival == +INF (MinMax::min initValue) when unconstrained
  if ((from_aat <= 0.0 && to_aat <= 0.0)
      || from_aat == INF || to_aat == INF
      || from_aat == -INF || to_aat == -INF) {
    lms[lm_idx] = 0.0;
    return;
  }
  from_aat = std::max(delayAsFloat(from_aat), 0.0f);
  to_aat = std::max(delayAsFloat(to_aat), 0.0f);

  if (delay_minmax == MinMax::max()) {
    if (to_aat == 0.0) to_aat = 1.0e-12;
    lms[lm_idx] = lms[lm_idx] * (from_aat + delay) / to_aat;
    // if (lms[lm_idx] > MAX_LM_VALUE) lms[lm_idx] = MAX_LM_VALUE;
    // if (lms[lm_idx] < MIN_LM_VALUE) lms[lm_idx] = MIN_LM_VALUE;
  } else {
    if (from_aat + delay == 0.0) from_aat = 1.0e-12;
    lms[lm_idx] = lms[lm_idx] * to_aat / (from_aat + delay);
    // if (lms[lm_idx] > MAX_LM_VALUE) lms[lm_idx] = MAX_LM_VALUE;
    // if (lms[lm_idx] < MIN_LM_VALUE) lms[lm_idx] = MIN_LM_VALUE;
  }
  if (lms[lm_idx] < 0.0) {
    printf("LRHelper::updateArcLms: edge %s AP corner %s delay min/max %s: computed negative LM %.6f from origin %.6f with aat %.6f, rat %.6f, delay %.6f\n",
            edge->to_string(graph_).c_str(),
            scene->name().c_str(),
            delay_minmax->to_string().c_str(),
            lms[lm_idx], origin,
            from_aat * 1.0e12, to_aat * 1.0e12, delay * 1.0e12);
    fflush(stdout);
    lms[lm_idx] = 0.0;
  }
}

void
LRHelper::enqueueVertex(Vertex *vertex) {
  sorted_lm_vertices_.push_back(vertex);
  iter_->enqueueAdjacentVertices(vertex);
}

//////////////////////////////////////////////////////////////////////
// LM History forwarding methods

int
LRHelper::recordLM() {
  // Ensure the history uses the current graph pointer
  lm_history_.setGraph(graph_);
  return lm_history_.recordLM();
}

int
LRHelper::restoreLM(int frame_id) {
  lm_history_.setGraph(graph_);
  return lm_history_.restoreLM(frame_id);
}

int
LRHelper::lmFrameCount() const {
  return lm_history_.frameCount();
}

void
LRHelper::clearLmHistory() {
  lm_history_.clear();
}

bool
LRHelper::saveLmToFile(const std::string &path,
                        const std::string &design_name) {
  // Ensure there is at least one frame to save.
  // If no frame exists yet, record the current state first.
  if (lm_history_.frameCount() == 0) {
    lm_history_.setGraph(graph_);
    lm_history_.recordLM();
  }
  uint32_t vertex_count = static_cast<uint32_t>(graph_->vertexCount());
  return lm_history_.saveToFile(path, design_name, vertex_count);
}

int
LRHelper::loadLmFromFile(const std::string &path,
                          const std::string &design_name) {
  lm_history_.setGraph(graph_);
  uint32_t vertex_count = static_cast<uint32_t>(graph_->vertexCount());
  int frame_id = lm_history_.loadFromFile(path, design_name, vertex_count);
  if (frame_id >= 0) {
    int restored = lm_history_.restoreLM(frame_id);
    printf("LRHelper::loadLmFromFile: restored %d edges from %s\n",
           restored, path.c_str());
  }
  return frame_id;
}

//////////////////////////////////////////////////////////////////////
// Parallel KKT Projection and LM Update
//
// - Forward pass (computeInLmSums): embarrassingly parallel (read-only on arc LMs)
// - Backward pass (distributeLmOutToIn): uses BfsBkwdIterator::visitParallel()
//   to dispatch same-level vertices in parallel, level-by-level from high to low.
// - checkKKT: embarrassingly parallel (read-only)
// - updateAllEdgeLms: embarrassingly parallel (each edge writes only its own arc LMs)
//
// dispatch_queue_ and thread_count_ are inherited from StaState.

// KKTBackwardVisitor: VertexVisitor for reverse-topo KKT projection.
// Used with BfsBkwdIterator::visitParallel() to process vertices level-by-level
// from high level to low level. Same-level vertices are processed in parallel.
// Thread safety: same-level vertices have disjoint input edges, so
// distributeLmOutToIn writes to different Edge::arcLms() arrays.
class KKTBackwardVisitor : public sta::VertexVisitor
{
public:
  KKTBackwardVisitor(LRHelper *helper,
                     DcalcAPToLMValueSeqMap &ap_lm_seq_map)
    : helper_(helper),
      ap_lm_seq_map_(ap_lm_seq_map) {}

  VertexVisitor *copy() const override {
    return new KKTBackwardVisitor(helper_, ap_lm_seq_map_);
  }

  void visit(Vertex *vertex) override {
    if (!hasFanin(vertex, helper_->search_pred_, helper_->graph_, helper_->modes()[0]) ||
        !hasFanout(vertex, helper_->search_pred_, helper_->graph_, helper_->modes()[0]))
      return;

    VertexId vid = helper_->graph_->id(vertex);
    auto it = helper_->vertex_to_sorted_idx_.find(vid);
    if (it == helper_->vertex_to_sorted_idx_.end())
      return;
    size_t idx = it->second;

    LMValueSeq out_lm_sums = helper_->computeOutLmSum(vertex);
    helper_->distributeLmOutToIn(vertex, out_lm_sums, ap_lm_seq_map_, idx);
  }

private:
  LRHelper *helper_;
  DcalcAPToLMValueSeqMap &ap_lm_seq_map_;
};

void
LRHelper::parallelComputeInLmSums(DcalcAPToLMValueSeqMap &ap_lm_map)
{
  const size_t n = sorted_lm_vertices_.size();

  // Pre-allocate vectors for each analysis point
  for (Scene *scene : scenes()) {
   for (const MinMax *min_max : MinMax::range()) {
    ap_lm_map[scene->dcalcAnalysisPtIndex(min_max)].resize(n, 0.0);
   }
  }

  if (thread_count_ <= 1 || !dispatch_queue_) {
    printf("[ERROR] parallelComputeInLmSums called with thread_count_=%d, dispatch_queue_=%p\n",
           thread_count_, (void*)dispatch_queue_);
  }

  // Chunk vertices across threads.
  // This is safe because each vertex reads only its own input edges' arc LMs
  // (which are not being modified concurrently) and writes to its own index.
  const size_t chunk = (n + thread_count_ - 1) / thread_count_;
  for (size_t t = 0; t < (size_t)thread_count_; t++) {
    const size_t start = t * chunk;
    const size_t end = std::min(start + chunk, n);
    if (start >= end) break;
    dispatch_queue_->dispatch([this, &ap_lm_map, start, end](int) {
      for (size_t i = start; i < end; i++) {
        Vertex *vertex = sorted_lm_vertices_[i];
        if (!hasFanin(vertex, search_pred_, graph_, modes()[0])) {
          for (auto &[ap, seq] : ap_lm_map)
            seq[i] = 0.0;
          continue;
        }
        for (Scene *scene : scenes()) {
         for (const MinMax *min_max : MinMax::range()) {
          const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
          LMValue in_lm_sum = 0.0;
          size_t in_edge_count = 0;
          VertexInEdgeIterator in_edge_iter(vertex, graph_);
          while (in_edge_iter.hasNext()) {
            const Edge *in_edge = in_edge_iter.next();
            if (in_edge->role()->isTimingCheck())
              continue;
            LMValue const *lms = in_edge->arcLms();
            for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
              size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
              in_lm_sum += lms[lm_idx];
            }
            in_edge_count++;
          }
          if (in_lm_sum == 0.0 || in_edge_count == 0)
            in_lm_sum = 1.0;
          ap_lm_map[ap_index][i] = in_lm_sum;
         }
        }
      }
    });
  }
  dispatch_queue_->finishTasks();
}

bool
LRHelper::parallelCheckKKTForAllVertices()
{
  if (lrfVerbose()) printf("LRHelper::parallelCheckKKTForAllVertices()\n");
  fflush(stdout);

  const size_t n = sorted_lm_vertices_.size();
  if (thread_count_ <= 1 || !dispatch_queue_) {
    printf("[ERROR] parallelCheckKKTForAllVertices called with thread_count_=%d, dispatch_queue_=%p\n",
           thread_count_, (void*)dispatch_queue_);
  }

  std::atomic<bool> all_satisfied(true);
  std::mutex stats_mutex;
  LMValue global_max_lm = MIN_LM_VALUE;
  LMValue global_min_lm = MAX_LM_VALUE;
  sta::Edge *global_max_lm_edge = nullptr;
  sta::Edge *global_min_lm_edge = nullptr;

  const size_t chunk = (n + thread_count_ - 1) / thread_count_;
  for (size_t t = 0; t < (size_t)thread_count_; t++) {
    const size_t start = t * chunk;
    const size_t end = std::min(start + chunk, n);
    if (start >= end) break;
    dispatch_queue_->dispatch([this, start, end, &all_satisfied,
                               &stats_mutex, &global_max_lm, &global_min_lm,
                               &global_max_lm_edge, &global_min_lm_edge](int) {
      LMValue local_max_lm = MIN_LM_VALUE;
      LMValue local_min_lm = MAX_LM_VALUE;
      sta::Edge *local_max_edge = nullptr;
      sta::Edge *local_min_edge = nullptr;

      for (size_t idx = start; idx < end; idx++) {
        Vertex *vertex = sorted_lm_vertices_[idx];
        if (!hasFanin(vertex, search_pred_, graph_, modes()[0]) ||
            !hasFanout(vertex, search_pred_, graph_, modes()[0]) ||
            network_->isRegClkPin(vertex->pin()))
          continue;

        std::vector<LMValue> out_lm_vec(graph_->apCount(), 0.0);
        for (Scene *scene : scenes()) {
         for (const MinMax *min_max : MinMax::range()) {
          const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
          LMValue out_lm_sum = 0.0;
          size_t out_edge_count = 0;
          VertexOutEdgeIterator out_edge_iter(vertex, graph_);
          while (out_edge_iter.hasNext()) {
            Edge *out_edge = out_edge_iter.next();
            if (out_edge->role()->isTimingCheck())
              continue;
            LMValue const *lms = out_edge->arcLms();
            for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
              size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
              LMValue arc_lm = lms[lm_idx];
              out_lm_sum += arc_lm;
              if (arc_lm > local_max_lm) { local_max_lm = arc_lm; local_max_edge = out_edge; }
              if (arc_lm < local_min_lm) { local_min_lm = arc_lm; local_min_edge = out_edge; }
            }
            out_edge_count++;
          }
          if (out_edge_count == 0) out_lm_sum = -1.0;
          out_lm_vec[ap_index] = out_lm_sum;
         }
        }

        for (Scene *scene : scenes()) {
         for (const MinMax *min_max : MinMax::range()) {
          const size_t ap_index = scene->dcalcAnalysisPtIndex(min_max);
          LMValue in_lm_sum = 0.0;
          size_t in_edge_count = 0;
          VertexInEdgeIterator in_edge_iter(vertex, graph_);
          while (in_edge_iter.hasNext()) {
            Edge *in_edge = in_edge_iter.next();
            if (in_edge->role()->isTimingCheck())
              continue;
            LMValue const *lms = in_edge->arcLms();
            for (TimingArc *arc : in_edge->timingArcSet()->arcs()) {
              size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
              LMValue arc_lm = lms[lm_idx];
              in_lm_sum += arc_lm;
              if (arc_lm > local_max_lm) { local_max_lm = arc_lm; local_max_edge = in_edge; }
              if (arc_lm < local_min_lm) { local_min_lm = arc_lm; local_min_edge = in_edge; }
            }
            in_edge_count++;
          }
          LMValue out_lm_sum = out_lm_vec[ap_index];
          const float epsilon = 1e-4;
          if (!(out_lm_sum == 0.0 && in_lm_sum == 0.0)
                && !(out_lm_sum == 0.0)
                && !(in_lm_sum == 0.0)
                && (std::abs(out_lm_sum - in_lm_sum)/out_lm_sum > epsilon)
                && !(in_edge_count == 0)
                && !(out_lm_sum == -1.0)) {
            all_satisfied.store(false);
          }
         }
        }
      }

      // Merge local stats
      std::lock_guard<std::mutex> lock(stats_mutex);
      if (local_max_lm > global_max_lm) { global_max_lm = local_max_lm; global_max_lm_edge = local_max_edge; }
      if (local_min_lm < global_min_lm) { global_min_lm = local_min_lm; global_min_lm_edge = local_min_edge; }
    });
  }
  dispatch_queue_->finishTasks();

  if (lrfVerbose()) printf("LRHelper::parallelCheckKKTForAllVertices(): max LM (%s) & min LM (%s) value encountered: %.6f, %.6f\n",
         global_max_lm_edge ? global_max_lm_edge->to_string(graph_).c_str() : "N/A",
         global_min_lm_edge ? global_min_lm_edge->to_string(graph_).c_str() : "N/A",
         global_max_lm, global_min_lm);
  fflush(stdout);
  return all_satisfied.load();
}

bool
LRHelper::parallelKKTProjection(Sta *sta)
{
  if (lrfVerbose()) printf("LRHelper::parallelKKTProjection() with %d threads\n", thread_count_);
  fflush(stdout);

  // Fallback to serial if single-threaded
  if (thread_count_ <= 1 || !dispatch_queue_)
    return KKTProjection(sta);

  const VertexSeq &sorted_vertices = ensureSorted(sta);
  const size_t n = sorted_vertices.size();

  // Step 1: Build vertex-to-sorted-index mapping (needed for distributeLmOutToIn)
  vertex_to_sorted_idx_.clear();
  vertex_to_sorted_idx_.reserve(n);
  for (size_t i = 0; i < n; i++) {
    VertexId vid = graph_->id(sorted_lm_vertices_[i]);
    vertex_to_sorted_idx_[vid] = i;
  }

  // Step 2: Parallel compute in LM sums (read-only on arc LMs)
  DcalcAPToLMValueSeqMap ap_lm_seq_map;
  parallelComputeInLmSums(ap_lm_seq_map);

  // Step 3: Backward pass using BfsBkwdIterator::visitParallel().
  // Pre-enqueue all sorted vertices; visitParallel processes them
  // level-by-level from high to low with parallel dispatch within each level.
  // Same-level vertices have disjoint input edges, so writes are thread-safe.
  {
    BfsBkwdIterator bkwd_iter(BfsIndex::other, search_pred_, sta);
    bkwd_iter.ensureSize();
    for (Vertex *vertex : sorted_lm_vertices_) {
      bkwd_iter.enqueue(vertex);
    }

    KKTBackwardVisitor visitor(this, ap_lm_seq_map);
    bkwd_iter.visitParallel(0, &visitor);
  }

  // Step 4: Parallel KKT check
  bool kkt_satisfied = parallelCheckKKTForAllVertices();
  if (kkt_satisfied) {
    if (lrfVerbose()) printf("LRHelper::parallelKKTProjection(): KKT conditions satisfied\n");
  } else {
    if (lrfVerbose()) printf("LRHelper::parallelKKTProjection(): KKT conditions NOT satisfied\n");
  }
  fflush(stdout);
  return kkt_satisfied;
}

void
LRHelper::parallelUpdateAllEdgeLms(Sta *sta)
{
  if (lrfVerbose()) printf("Size of sorted_lm_vertices_: %zu\n", sorted_lm_vertices_.size());
  if (lrfVerbose()) printf("Using LRHelper strategy: %s (parallel, %d threads)\n",
         strategyName().c_str(), thread_count_);
  fflush(stdout);

  if (thread_count_ <= 1 || !dispatch_queue_) {
    printf("[ERROR] parallelUpdateAllEdgeLms called with thread_count_=%d, dispatch_queue_=%p\n",
           thread_count_, (void*)dispatch_queue_);
  }

  sta->findRequireds();
  copyState(sta);

  // Pre-mark endpoints on main thread (not thread-safe to do in parallel)
  for (Vertex *vertex : sta->endpoints()) {
    vertex->setIsEndpoint(true);
  }

  // Partition vertices into chunks and dispatch.
  // Each vertex's output edges are independent — writes go to different
  // Edge::arcLms() arrays, and timing queries are read-only after findRequireds().
  const size_t n = sorted_lm_vertices_.size();
  const size_t chunk = (n + thread_count_ - 1) / thread_count_;
  for (size_t t = 0; t < (size_t)thread_count_; t++) {
    const size_t start = t * chunk;
    const size_t end = std::min(start + chunk, n);
    if (start >= end) break;
    dispatch_queue_->dispatch([this, sta, start, end](int) {
      for (size_t i = start; i < end; i++) {
        Vertex *vertex = sorted_lm_vertices_[i];
        VertexOutEdgeIterator out_edge_iter(vertex, graph_);
        while (out_edge_iter.hasNext()) {
          Edge *out_edge = out_edge_iter.next();
          if (out_edge->role()->isTimingCheck())
            continue;
          // updateEdgeLms skips endpoint marking (already done above)
          for (Scene *scene : scenes()) {
           for (const MinMax *min_max : MinMax::range()) {
            for (TimingArc *arc : out_edge->timingArcSet()->arcs()) {
              if (out_edge->to(graph_)->isEndPoint() && RATCONS_) {
                updateEndPointArcLms(out_edge, arc, sta, scene, min_max);
              } else {
                updateArcLms(out_edge, arc, sta, scene, min_max);
              }
            }
           }
          }
        }
      }
    });
  }
  dispatch_queue_->finishTasks();
}

//////////////////////////////////////////////////////////////////////

std::string
AdaptiveLrHelper::strategyName() const
{
  return "Adaptive LRHelper:  on basis of base LRHelper, we adaptively control the step of LM update\n"; 
}

void
AdaptiveLrHelper::updateArcLms(sta::Edge *edge, sta::TimingArc *arc, Sta *sta, sta::Scene *scene, const sta::MinMax *min_max)
{
}

float
AdaptiveLrHelper::getMultiplier(Slack arc_slack)
{
  return 1.0f;
}


} // namespace lrf

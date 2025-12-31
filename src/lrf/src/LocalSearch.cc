

#include "sta/Search.hh"
#include "search/TagGroup.hh"
#include "search/Tag.hh"
#include "sta/SearchPred.hh"
#include "search/ClkInfo.hh"
#include "sta/Network.hh"
#include "sta/Sdc.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/Debug.hh"


#include "PtGraph.hh"
#include "LocalSta.hh"
#include "LocalSearch.hh"



namespace lrf {
size_t ptPathIndex(PtVertex &pt_vertex, Path *path)
{
  Path *paths = pt_vertex.paths();
  return path - paths;
}

LocalPathVisitor::LocalPathVisitor(StaState *state, PtGraph *pt_graph)
  : PathVisitor(state),
    pt_graph_(pt_graph)
{
}

LocalPathVisitor::LocalPathVisitor(StaState *state, PtGraph *pt_graph, const std::string &debug_label)
  : PathVisitor(state),
    pt_graph_(pt_graph),
    debug_label_(debug_label)
{
}

LocalPathVisitor::~LocalPathVisitor()
{
}

LocalArrivalVisitor::LocalArrivalVisitor(StaState *state, PtGraph *pt_graph)
  : LocalPathVisitor(state, pt_graph)
{
  // In this initialization, a loop searchPred is created,
  // but not used in this local class.
  LocalArrivalVisitor::init0();
  LocalArrivalVisitor::init();
}

LocalArrivalVisitor::LocalArrivalVisitor(StaState *state, PtGraph *pt_graph, const std::string &debug_label)
  : LocalPathVisitor(state, pt_graph, debug_label)
{
  // In this initialization, a loop searchPred is created,
  // but not used in this local class.
  LocalArrivalVisitor::init0();
  LocalArrivalVisitor::init();
}

VertexVisitor *
LocalPathVisitor::copy() const
{
  throw std::runtime_error("LocalPathVisitor::copy: Not implemented yet");
  return nullptr;
}

LocalArrivalVisitor::~LocalArrivalVisitor()
{
}

void LocalArrivalVisitor::init0()
{
  tag_bldr_ = new TagGroupBldr(true, this);
}

void 
LocalArrivalVisitor::init()
{
  pred_ = search_ ? search_->evalPred() : nullptr;
}

void
LocalArrivalVisitor::findLocalArrivals()
{
  for (VertexId vertex_id : pt_graph_->sortedVertexIds()) {
    findVertexArrival(vertex_id);
  }
}

void 
LocalArrivalVisitor::visit(Vertex *vertex)
{
  PtVertex &pt_vertex = pt_graph_->ptVertex(vertex);
  findVertexArrival(pt_vertex);
}
  
void
LocalArrivalVisitor::findVertexArrival(VertexId vertex_id)
{
  PtVertex &pt_vertex = pt_graph_->ptVertex(vertex_id);
  if (!pt_vertex.hasFanin())
    seedLocalRootArrivals(pt_vertex);
  else
    findVertexArrival(pt_vertex);
}

void
LocalArrivalVisitor::seedLocalRootArrivals(PtVertex &pt_vertex)
{
  // Since we have copy paths from the original timing graph
  // to the local graph, we can just skip this seeding process.
  return;
}

void
LocalArrivalVisitor::findVertexArrival(PtVertex &pt_vertex)
{
  // We cannot consider crpr, so we neglect it here.
  // Also, for all judgments, using Vertex is compltely the
  // same as using Ptvertex. Since PtVertex just wraps Vertex.
  Pin *pin = pt_vertex.pin();
  Vertex *vertex = pt_vertex.vertex();

  // If error occurs, we don't rewrite the arrival.
  bool arrival_changed = true;

  tag_bldr_->init(vertex);
  has_fanin_one_ = graph_->hasFaninOne(vertex);
  
  // Ensure this vertex isn't broke by set path delay.
  if (!sdc_->isPathDelayInternalFromBreak(pin)) {
    // Using pt_vertex can be gotten by map, so it's ok.
    localVisitFaninPaths(pt_vertex);
  }

  // Warn if unsupported SDC constructs are used.
  // We should support these in the future.
  // if (!network_->isTopLevelPort(pin)
  //     && sdc_->hasInputDelay(pin))
  //   printf("WARNING: Local arrival analysis does not support input delays on pin %s\n",
  //          network_->name(pin));
  // if (sdc_->isPathDelayInternalFrom(pin))
  //   printf("WARNING: Local arrival analysis does not support path delay breaks on pin %s\n",
  //          network_->name(pin));
  // if (sdc_->isLeafPinClock(pin))
  //   printf("WARNING: Local arrival analysis does not support leaf pin clocks on pin %s\n",
  //          network_->name(pin));
  if (network_->isLatchData(pin)) {
    printf("WARNING: Local arrival analysis does not support latch data pins %s\n",
           network_->name(pin));
    fflush(stdout);
    arrival_changed = false;
  }
    
  bool is_clk = tag_bldr_->hasClkTag();
  if (vertex->isRegClk() && !is_clk) {
    printf("WARNING: Local arrival analysis found reg clk vertex %s without clk tag\n",
           network_->name(pin));
    fflush(stdout);
    arrival_changed = false;
    // search_->makeUnclkedPaths(vertex, true, false, tag_bldr_);
  }

  // We don't do arrival change judgement, cause it will definitely
  // change in along with gate sizing.
  if (arrival_changed)
    localSetVertexArrivals(pt_vertex, tag_bldr_);
}

void
LocalPathVisitor::localVisitFaninPaths(PtVertex &to_pt_vertex)
{
  // Skip vertices with preset input delays.
  if (pred_->searchTo(to_pt_vertex.vertex())) {
    PtVertexInEdgeIterator pt_edge_iter(to_pt_vertex.objectIdx(), pt_graph_);
    while (pt_edge_iter.hasNext()) {
      PtEdge &pt_edge = pt_edge_iter.next();
      PtVertex &from_pt_vertex = pt_graph_->ptVertex(pt_edge.ptFromId());
      // Pin is not necessary here.
      // Ensure from vertex is not constant and disabled.
      if (pred_->searchFrom(from_pt_vertex.vertex())
    && pred_->searchThru(pt_edge.edge())) {
        // If arrival update fails, do nothing.
        if (!localVisitEdge(from_pt_vertex, pt_edge, to_pt_vertex))
          break;
      }
    }
  }
}

void 
LocalPathVisitor::localVisitFanoutPaths(PtVertex &from_pt_vertex)
{
  if (pred_->searchFrom(from_pt_vertex.vertex())) {
    PtVertexOutEdgeIterator edge_iter(from_pt_vertex.objectIdx(), pt_graph_);
    while (edge_iter.hasNext()) {
      PtEdge &pt_edge = edge_iter.next();
      PtVertex &to_pt_vertex = pt_graph_->ptVertex(pt_edge.ptToId());
      if (pred_->searchTo(to_pt_vertex.vertex()) &&
          pred_->searchThru(pt_edge.edge())) {
        // If required update fails, do nothing.
        if (!localVisitEdge(from_pt_vertex, pt_edge, to_pt_vertex))
          break;
      }
    }
  }
}

bool
LocalPathVisitor::localVisitEdge(PtVertex &from_pt_vertex, 
                        PtEdge &pt_edge, PtVertex &to_pt_vertex)
{
  TagGroup *from_tag_group = 
              search_->tagGroup(from_pt_vertex.tagGroupIndex());
  if (from_tag_group) {
    TimingArcSet *arc_set = pt_edge.timingArcSet();
    PtVertexPathIterator from_iter(from_pt_vertex, search_);
    while (from_iter.hasNext()) {
      Path *from_path = from_iter.next();
      PathAnalysisPt *from_path_ap = from_path->pathAnalysisPt(this);
      const MinMax *min_max = from_path_ap->pathMinMax();
      const RiseFall *from_rf = from_path->transition(this);
      TimingArc *arc1, *arc2;
      arc_set->arcsFrom(from_rf, arc1, arc2);
      if (!localVisitArc(from_pt_vertex, from_rf, from_path, pt_edge,
                         arc1, to_pt_vertex, min_max, from_path_ap))
        return false;
      if (!localVisitArc(from_pt_vertex, from_rf, from_path, pt_edge,
                         arc2, to_pt_vertex, min_max, from_path_ap))
        return false;
    }
  } else {
    throw std::runtime_error("Local arrival analysis found from vertex without tag group");
  }
  return true;
}

bool 
LocalPathVisitor::localVisitArc(PtVertex &from_pt_vertex, 
                                 const RiseFall *from_rf,
                                 Path *from_path,
                                 PtEdge &edge,
                                 TimingArc *arc,
                                 PtVertex &to_pt_vertex,
                                 const MinMax *min_max,
                                 const PathAnalysisPt *path_ap)
{
  if (arc) {
    const RiseFall *to_rf = arc->toEdge()->asRiseFall();
    if (searchThru(from_pt_vertex.vertex(), from_rf,
                  edge.edge(), to_pt_vertex.vertex(), 
                  to_rf)) 
      return localVisitFromPath(from_pt_vertex.pin(),
                                from_pt_vertex,
                                from_rf,
                                from_path,
                                edge,
                                arc,
                                to_pt_vertex.pin(),
                                to_pt_vertex,
                                to_rf,
                                min_max,
                                path_ap);
    
  }
  return true;
}

bool 
LocalPathVisitor::localVisitFromPath(const Pin *from_pin,
                                  PtVertex &from_pt_vertex,
                                  const RiseFall *from_rf,
                                  Path *from_path,
                                  PtEdge &pt_edge,
                                  TimingArc *arc,
                                  const Pin *to_pin,
                                  PtVertex &to_pt_vertex,
                                  const RiseFall *to_rf,
                                  const MinMax *min_max,
                                  const PathAnalysisPt *path_ap)
{
  // Vertex *from_vertex = from_pt_vertex.vertex();
  // Vertex *to_vertex = to_pt_vertex.vertex();
  Edge *edge = pt_edge.edge();

  const TimingRole *role = edge->role();
  Tag *from_tag = from_path->tag(this);
  Tag *to_tag = nullptr;
  // This from arrival should load local arrival of to_path.
  Arrival from_arrival = from_path->arrival();
  ArcDelay arc_delay = 0.0;
  Arrival to_arrival;
  
  // if (from_clk_info->isGenClkSrcPath()) {
  //   printf("ERROR: Local arrival analysis does not support gen clk src paths yet.\n");
  //   fflush(stdout);
  //   continue;
  // }else 
  if (role->genericRole() == TimingRole::regClkToQ()) {
    // reg clk to q
    // printf("ERROR: Local arrival analysis does not support reg clk to q paths yet.\n");
    // fflush(stdout);
    return true;

  } else if (role == TimingRole::latchDtoQ()) {
    // latch clk to q
    printf("ERROR: Local arrival analysis does not support latch clk to q paths yet.\n");
    fflush(stdout);
    return true;
  } else if (from_tag->isClock()) {
    // clk to ff/dl/comb
    // printf("ERROR: Local arrival analysis does not support clk to ff/dl/comb paths yet.\n");
    // fflush(stdout);
    return true;
  } else {
    if (!(sdc_->isPathDelayInternalFromBreak(to_pin)
          || sdc_->isPathDelayInternalToBreak(from_pin))) {
      to_tag = search_->thruTag(from_tag, edge, to_rf, min_max, path_ap, tag_cache_);
      // Derate delay with search
      arc_delay = pt_graph_->arcDelay(pt_edge, arc, path_ap->dcalcAnalysisPt()->index());
      float derate = search_->timingDerate(from_pt_vertex.vertex(), arc, pt_edge.edge(), false, path_ap);
      arc_delay *= derate;

      if (!delayInf(arc_delay)) {
        to_arrival = from_arrival + arc_delay;
      }
    }
  }
  if (to_tag) {
    return localVisitFromToPath(from_pt_vertex, from_rf,
                                      from_tag, from_path, from_arrival,
                                      pt_edge, arc, arc_delay,
                                      to_pt_vertex, to_rf, to_tag, to_arrival,
                                      min_max, path_ap);
  }
  else {
    printf("Error: Local arrival analysis found to vertex without tag\n");
    fflush(stdout);
    return true;
  }
}

bool
LocalArrivalVisitor::localVisitFromToPath(
                    PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    Tag *from_tag,
                    Path *from_path,
                    const Arrival &from_arrival,
                    PtEdge &pt_edge,
                    TimingArc *arc,
                    ArcDelay arc_delay,
                    PtVertex &to_pt_vertex,
                    const RiseFall *to_rf,
                    Tag *to_tag,
                    Arrival &to_arrival,
                    const MinMax *min_max,
                    const PathAnalysisPt *path_ap)
{
  Path *match;
  size_t path_index;
  tag_bldr_->tagMatchPath(to_tag, match, path_index);
  if (match == nullptr || delayGreater(to_arrival, match->arrival(), min_max, this)) {
    tag_bldr_->setMatchPath(match, path_index, to_tag, to_arrival, from_path, pt_edge.edge(), arc);
  }
  return true;
}

void
LocalArrivalVisitor::localSetVertexArrivals(PtVertex &pt_vertex, TagGroupBldr *tag_bldr)
{
  if (tag_bldr->empty())
    return;
  TagGroup *prev_tag_group = 
              search_->tagGroup(pt_vertex.tagGroupIndex());
  Path *prev_paths = pt_vertex.paths();
  TagGroup *tag_group = search_->findExistingTagGroup(tag_bldr);
  if (tag_group == prev_tag_group) {
    // Even if tag_group is the same, we need to ensure prev_paths is not null
    if (prev_paths == nullptr) {
      size_t path_count = tag_bldr->pathCount();
      Path *paths = pt_graph_->makePaths(pt_vertex.objectIdx(), path_count);
      tag_bldr->copyPaths(tag_group, paths);
    } else {
      tag_bldr->copyPaths(tag_group, prev_paths);
    }
  } else {
    if (prev_tag_group) {
      pt_graph_->deletePaths(pt_vertex.objectIdx());
    }
    size_t path_count = tag_bldr->pathCount();
    Path *paths = pt_graph_->makePaths(pt_vertex.objectIdx(), path_count);
    tag_bldr->copyPaths(tag_group, paths);
    pt_vertex.setTagGroupIndex(tag_group->index());
  }
  // We don't consider filtered paths since we don't consider
  // false path in the local graph (we can prevent it from the
  // Local graph extraction phase).
}

void 
LocalArrivalVisitor::printArrivals()
{
  for (auto& pt_vertex : pt_graph_->ptVertices()) {
    PtVertexPathIterator path_iter(pt_vertex, search_);
    size_t path_num = 0;
    while (path_iter.hasNext()) {
      Path *path = path_iter.next();
      Arrival arrival = path->arrival();
      printf("Vertex %s Path %zu Arrival: %f\n",
             network_->name(pt_vertex.pin()),
             path_num,
             arrival);
      path_num++;
    }
  }
}

////////////////////////////////////////////////////
// Functions of LocalRequiredCmp and LocalRequiredVisitor
////////////////////////////////////////////////////
LocalRequiredCmp::LocalRequiredCmp() : have_requireds_(false)
{
}

void 
LocalRequiredCmp::requiredsInit(PtVertex &pt_vertex,
                                 const StaState *sta)
{
  Search *sta_search = sta->search();
  TagGroup *tag_group = sta_search->tagGroup(pt_vertex.tagGroupIndex());
  if (tag_group) {
    size_t path_count = tag_group->pathCount();
    requireds_.resize(path_count);
    for (auto const [tag, path_index] : *tag_group->pathIndexMap()) {
      PathAnalysisPt *path_ap = tag->pathAnalysisPt(sta);
      const MinMax *min_max = path_ap->pathMinMax();
      requireds_[path_index] = delayInitValue(min_max->opposite());
    }
  } else {
    throw std::runtime_error("LocalRequiredCmp::requiredsInit: No tag group found");
    return;
  }
  have_requireds_ = false;
}

void
LocalRequiredCmp::requiredSet(size_t path_index,
			 Required &required,
			 const MinMax *min_max,
			 const StaState *sta)
{
  if (delayGreater(required, requireds_[path_index], min_max, sta)) {
    requireds_[path_index] = required;
    have_requireds_ = true;
  }
}

Required
LocalRequiredCmp::required(size_t path_index)
{
  return requireds_[path_index];
}

bool
LocalRequiredCmp::requiredsSave(PtVertex &pt_vertex,
			   const StaState *sta)
{
  bool requireds_changed = false;
  PtVertexPathIterator path_iter(pt_vertex, sta);
  while (path_iter.hasNext()) {
    Path *path = path_iter.next();
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // This should be resivesed, since path index in local graph
    // may be different from that in original graph.
    // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    size_t path_index = ptPathIndex(pt_vertex, path);
    Required req = requireds_[path_index];
    Required &prev_req = path->required();
    bool changed = !delayEqual(prev_req, req);
    requireds_changed |= changed;
    path->setRequired(req);
  }
  return requireds_changed;
}


LocalRequiredVisitor::LocalRequiredVisitor(StaState *state, PtGraph *pt_graph)
  : LocalPathVisitor(state, pt_graph),
    required_cmp_(new LocalRequiredCmp())
{
}

LocalRequiredVisitor::~LocalRequiredVisitor()
{
  delete required_cmp_;
}

void 
LocalRequiredVisitor::findLocalRequireds()
{
  std::vector<size_t> vertex_ids = pt_graph_->sortedVertexIds();
  for (size_t i = vertex_ids.size(); i > 0; --i) {
    findVertexRequired(vertex_ids[i - 1]);
  }
}

void 
LocalRequiredVisitor::findVertexRequired(VertexId vertex_id)
{
  PtVertex &pt_vertex = pt_graph_->ptVertex(vertex_id);
  if (!pt_vertex.hasFanout())
    seedLocalRootRequireds(pt_vertex);
  else
    findVertexRequired(pt_vertex);
}

void
LocalRequiredVisitor::seedLocalRootRequireds(PtVertex &pt_vertex)
{
  // Since we have copy paths from the original timing graph
  // to the local graph, we can just skip this seeding process.
  return;
}

void
LocalRequiredVisitor::findVertexRequired(PtVertex &pt_vertex)
{
  required_cmp_->requiredsInit(pt_vertex, this);
  localVisitFanoutPaths(pt_vertex);

  // Save requireds in cmp back to paths
  required_cmp_->requiredsSave(pt_vertex, this);
}

void
LocalRequiredVisitor::visit(Vertex *vertex)
{
  PtVertex &pt_vertex = pt_graph_->ptVertex(vertex);
  findVertexRequired(pt_vertex);
}

bool LocalRequiredVisitor::localVisitFromToPath(
                    PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    Tag *from_tag,
                    Path *from_path,
                    const Arrival &from_arrival,
                    PtEdge &pt_edge,
                    TimingArc *arc,
                    ArcDelay arc_delay,
                    PtVertex &to_pt_vertex,
                    const RiseFall *to_rf,
                    Tag *to_tag,
                    Arrival &to_arrival,
                    const MinMax *min_max,
                    const PathAnalysisPt *path_ap)
{
  // Don't propagate required times through latch D->Q edges.
  if (pt_edge.role() != TimingRole::latchDtoQ()) {
    size_t path_index = ptPathIndex(from_pt_vertex, from_path);
    const MinMax *req_min = min_max->opposite();
    TagGroup *to_tag_group = search_->tagGroup(to_pt_vertex.tagGroupIndex());
    if (to_tag_group && to_tag_group->hasTag(to_tag)) {
      size_t to_path_index = to_tag_group->pathIndex(to_tag);
      Path &to_path = to_pt_vertex.paths()[to_path_index];
      Required &to_required = to_path.required();
      Required from_required = to_required - arc_delay;
      required_cmp_->requiredSet(path_index, from_required, req_min, this);
    }
    else {
      // we don't consider crpr. So this should not happen.
      throw std::runtime_error("Local required analysis found to vertex without tag");
    }
  } else {
    printf("WARNING: Local required analysis does not propagate through latch D->Q edges.\n");
    fflush(stdout);
  }
  return true;
}

void
LocalRequiredVisitor::printRequireds()
{
  for (auto& pt_vertex : pt_graph_->ptVertices()) {
    PtVertexPathIterator path_iter(pt_vertex, search_);
    size_t path_num = 0;
    while (path_iter.hasNext()) {
      Path *path = path_iter.next();
      Required required = path->required();
      printf("Vertex %s Path %zu Required: %f\n",
             network_->name(pt_vertex.pin()),
             path_num,
             required);
      path_num++;
    }
  }
}

size_t
PtVertexPathIterator::pathIndex() const
{
  return std::min(path_index_ - 1, size_t(0)); 
}


} // namespace lrf
#include "PtGraph.hh"
#include "Sta.hh"
#include <algorithm>
#include <cstdio>
#include <numeric>
#include <deque>
#include <vector>
#include "DcalcAnalysisPt.hh"
#include "Corner.hh"
#include "TimingRole.hh"
#include "sta/Liberty.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "search/TagGroup.hh"
#include "LocalSta.hh"

namespace lrf {

using sta::ArcDelay;
using sta::DcalcAPIndex;
using sta::Edge;
using sta::EdgeId;
using sta::Graph;
using sta::Instance;
using sta::InstanceSet;
using sta::Level;
using sta::MinMax;
using sta::RiseFall;
using sta::VertexId;
using sta::Vertex;

const char *ptVertexTypeName(PtVertexType type)
{
  switch (type) {
    case PtVertexType::Sentinel:
      return "Sentinel";
    case PtVertexType::RefDriver:
      return "RefDriver";
    case PtVertexType::RefInput:
      return "RefInput";
    case PtVertexType::RefOutput:
      return "RefOutput";
    case PtVertexType::VirtualInput:
      return "VirtualInput";
    case PtVertexType::VirtualOutput:
      return "VirtualOutput";
    case PtVertexType::None:
      return "None";
  }
  return "Unknown";
}

static const char *ptEdgeTypeName(PtEdgeType type)
{
  switch (type) {
    case PtEdgeType::Sentinel:
      return "Sentinel";
    case PtEdgeType::RefInstEdge:
      return "RefInstEdge";
    case PtEdgeType::VirtualGateEdge:
      return "VirtualGateEdge";
    case PtEdgeType::VirtualWireEdge:
      return "VirtualWireEdge";
    case PtEdgeType::None:
      return "None";
  }
  return "Unknown";
}

static std::string dotEscape(const std::string &text)
{
  std::string escaped = text;
  for (char &ch : escaped) {
    if (ch == '\"') {
      ch = '\'';
    } else if (ch == '\n' || ch == '\r') {
      ch = ' ';
    }
  }
  return escaped;
}

class PtVertexIdLevelLess
{
 public:
  PtVertexIdLevelLess(const PtGraph* pt_graph);
  bool operator()(const VertexId &pt_vertex1, const VertexId &pt_vertex2) const;

 private:
  const PtGraph *pt_graph_;
};

PtVertexIdLevelLess::PtVertexIdLevelLess(const PtGraph* pt_graph) : pt_graph_(pt_graph)
{
}

bool PtVertexIdLevelLess::operator()(const VertexId &pt_vertex1,
                                      const VertexId &pt_vertex2) const
{
  float level1 = pt_graph_->ptVertex(pt_vertex1).level();
  float level2 = pt_graph_->ptVertex(pt_vertex2).level();
  return (level1 < level2)
         || (level1 == level2
             && pt_vertex1 < pt_vertex2);
}

PtGraph::PtGraph(sta::Sta *sta) :
  sta_(sta),
  pt_edges_(1),           // reserve sentinel at index 0
  pt_vertices_(1),
  ap_count_(sta->graph()->apCount()),
  slew_rf_count_(sta::Vertex::transitionCount())
{
  pt_vertices_[0].setType(PtVertexType::Sentinel);
  pt_edges_[0].setType(PtEdgeType::Sentinel);
  graph_made_ = false;
  sorted_ = false;
  sorted_vertex_ids_.clear();
  roots_.clear();
  vertex_map_.clear();
}

PtGraph::~PtGraph()
{
}

PtPiElmore*
PtGraph::findPtParasitic(VertexId drvr_id,
                          const sta::RiseFall *rf,
                          int ap_index)
{
  auto it = pt_parasitics_.find(drvr_id);
  if (it == pt_parasitics_.end())
    return nullptr;
  int idx = rf->index() * ap_count_ + ap_index;
  auto &vec = it->second;
  if (idx < 0 || idx >= (int)vec.size())
    return nullptr;
  return &vec[idx];
}

PtPiElmore&
PtGraph::makePtParasitic(VertexId drvr_id,
                          const sta::RiseFall *rf,
                          int ap_index)
{
  auto &vec = pt_parasitics_[drvr_id];
  size_t total = sta::RiseFall::index_count * ap_count_;
  if (vec.size() < total)
    vec.resize(total);
  int idx = rf->index() * ap_count_ + ap_index;
  return vec[idx];
}

void
PtGraph::clearPtParasitics()
{
  pt_parasitics_.clear();
}

void
PtGraph::clearPtParasitics(VertexId drvr_id)
{
  pt_parasitics_.erase(drvr_id);
}

sta::Level
PtGraph::vertexLevel(VertexId vertex_id) const
{
  if (vertex_id == sta::object_id_null)
    return -1;
  const PtVertex &pt_vertex = pt_vertices_[vertex_id];
  return static_cast<sta::Level>(pt_vertex.level());
}

void
PtGraph::makeGraph(sta::InstanceSet &inst_seq, sta::Instance *ref_inst)
{
  ref_inst_ = ref_inst;
  ref_lib_cell_ = sta_->network()->libertyCell(ref_inst);
  if (ref_lib_cell_ == nullptr) {
    // printf("Warning: PtGraph::makeGraph: ref_inst %s has no liberty cell\n",
           // sta_->network()->name(ref_inst));
    // fflush(stdout);
  }
  makePtVertexAndPtEdge(inst_seq);
  setGraphMade(true);
  initVertexAndEdges();
  createParasiticsNetworks();
  annotateVerticesType();
  annotateEdgesType();
}

void 
PtGraph::makeGraph(sta::VertexSet &vertex_set, sta::Instance *ref_inst) 
{
  ref_inst_ = ref_inst;
  ref_lib_cell_ = sta_->network()->libertyCell(ref_inst);
  if (ref_lib_cell_ == nullptr) {
    throw std::runtime_error("PtGraph::makeGraph: ref_inst has no liberty cell");
  }
  makePtVertexAndPtEdge(vertex_set);
  setGraphMade(true);
  initVertexAndEdges();
  createParasiticsNetworks();
  annotateVerticesType();
  annotateEdgesType();
}

void 
PtGraph::createParasiticsNetworks()
{
  // Placeholder
}

void
PtGraph::makePtVertexAndPtEdge(sta::VertexSet &vertex_set)
{
  sta::Graph *graph = sta_->graph();
  sta::Network *network = sta_->network();
  for (sta::Vertex *vertex : vertex_set) {
    VertexId pt_vertex_id = makeVertex(vertex);
    vertex_map_[vertex] = pt_vertex_id;
  }

  for (auto const &pair : vertex_map_) {
    sta::Vertex *vertex = const_cast<sta::Vertex*>(pair.first);
    VertexId pt_vertex_id = pair.second;
    if (network->isDriver(vertex->pin())) {
      makePtInstEdge(vertex, pt_vertex_id);
      makePtWireEdge(vertex, pt_vertex_id);
    }
  }
}

void
PtGraph::makePtVertexAndPtEdge(sta::InstanceSet &inst_seq)
{
  sta::Graph *graph = sta_->graph();
  sta::Network *network = sta_->network();
  for (const sta::Instance *inst : inst_seq) {
    sta::InstancePinIterator *pin_iter = network->pinIterator(const_cast<sta::Instance*>(inst));
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      sta::Vertex *vertex, *bidirect_vertex;
      graph->pinVertices(pin, vertex, bidirect_vertex);
      if (bidirect_vertex) {
        throw std::runtime_error("PtGraph::makePtVertexAndPtEdge: bidirect vertex not supported");
      }
      if (vertex) {
        VertexId pt_vertex_id = makeVertex(vertex);
        vertex_map_[vertex] = pt_vertex_id;
      }
    }
    delete pin_iter;
  }

  for (auto const &pair : vertex_map_) {
    sta::Vertex *vertex = const_cast<sta::Vertex*>(pair.first);
    VertexId pt_vertex_id = pair.second;
    if (network->isDriver(vertex->pin())) {
      makePtInstEdge(vertex, pt_vertex_id);
      makePtWireEdge(vertex, pt_vertex_id);
    }
  }
}

void 
PtGraph::makePtInstEdge(sta::Vertex *drvr_vertex, VertexId drvr_pt_id)
{
  sta::Network *network = sta_->network();
  sta::Instance *drvr_inst = network->instance(drvr_vertex->pin());
  if (!network->libertyCell(drvr_inst)) {
    return;
  }
  sta::Graph *graph = sta_->graph();
  sta::VertexInEdgeIterator in_edge_iter(drvr_vertex, graph);
  while (in_edge_iter.hasNext()) {
    sta::Edge *in_edge = in_edge_iter.next();
    sta::Vertex *from_vertex = in_edge->from(graph);
    
    auto it_from = vertex_map_.find(from_vertex);
    if (it_from != vertex_map_.end()) {
      VertexId from_pt_id = it_from->second;
      makeEdge(in_edge, from_pt_id, drvr_pt_id);
    }
  }
}

void 
PtGraph::makePtWireEdge(sta::Vertex *drvr_vertex, VertexId drvr_pt_id)
{
  sta::Graph *graph = sta_->graph();
  sta::VertexOutEdgeIterator out_edge_iter(drvr_vertex, graph);
  while (out_edge_iter.hasNext()) {
    sta::Edge *out_edge = out_edge_iter.next();
    if (out_edge->isWire()) {
      sta::Vertex *load_vertex = out_edge->to(graph);
      auto it_to = vertex_map_.find(load_vertex);
      if (it_to != vertex_map_.end()) {
        VertexId to_pt_id = it_to->second;
        makeEdge(out_edge, drvr_pt_id, to_pt_id);
      }
    }
  }
}

VertexId
PtGraph::makeVertex(sta::Vertex *vertex)
{
  sta::Network *network = sta_->network();
  pt_vertices_.emplace_back();
  PtVertex &pt_vertex = pt_vertices_.back();
  VertexId vertex_id = static_cast<VertexId>(pt_vertices_.size() - 1);
  pt_vertex.setObjectIdx(vertex_id);
  pt_vertex.init(vertex);
  if (network->isDriver(vertex->pin())) {
    pt_vertex.setIsDriver(true);
  }
  if (network->isLoad(vertex->pin())) {
    pt_vertex.setIsLoad(true);
  }
  return vertex_id;
}

EdgeId
PtGraph::makeEdge(sta::Edge *edge, 
  VertexId pt_from, 
  VertexId pt_to)  
{
  pt_edges_.emplace_back();
  PtEdge &pt_edge = pt_edges_.back();
  EdgeId edge_id = static_cast<EdgeId>(pt_edges_.size() - 1);
  if (edge_id == pt_edge_id_null) {
    // printf("PtGraph::makeEdge: edge id overflow\n");
    // fflush(stdout);
  }
  pt_edge.setObjectIdx(edge_id);
  pt_edge.init(edge, pt_from, pt_to);

  EdgeId next = pt_vertices_[pt_from].out_edges_;
  pt_edge.vertex_out_next_ = next;
  pt_edge.vertex_out_prev_ = pt_edge_id_null;
  if (next != pt_edge_id_null)
    pt_edges_[next].vertex_out_prev_ = edge_id;
  pt_vertices_[pt_from].out_edges_ = edge_id;

  pt_edge.vertex_in_link_ = pt_vertices_[pt_to].in_edges_;
  pt_vertices_[pt_to].in_edges_ = edge_id;

  return edge_id;
}

VertexId
PtGraph::makeVirtualVertex(sta::LibertyCell *cell, sta::LibertyPort *port,
                           bool is_driver, bool is_load, PtVertexType type)
{
  pt_vertices_.emplace_back();
  PtVertex &pt_vertex = pt_vertices_.back();
  VertexId vertex_id = static_cast<VertexId>(pt_vertices_.size() - 1);
  pt_vertex.setObjectIdx(vertex_id);
  pt_vertex.initVirtual(cell, port, is_driver, is_load);
  pt_vertex.setType(type);
  sorted_ = false;
  return vertex_id;
}

EdgeId
PtGraph::makeVirtualEdge(VertexId pt_from, VertexId pt_to,
                         sta::TimingArcSet *arc_set, bool is_wire)
{
  pt_edges_.emplace_back();
  PtEdge &pt_edge = pt_edges_.back();
  EdgeId edge_id = static_cast<EdgeId>(pt_edges_.size() - 1);
  pt_edge.setObjectIdx(edge_id);
  pt_edge.initVirtual(pt_from, pt_to, arc_set, is_wire);
  pt_edge.setType(is_wire ? PtEdgeType::VirtualWireEdge : PtEdgeType::VirtualGateEdge);

  // Allocate arc delays
  if (is_wire) {
    // Wire edges: delays indexed by rf * ap_count + ap_index
    size_t delay_count = slew_rf_count_ * ap_count_;
    pt_edge.setArcDelays(nullptr, delay_count);
  } else if (arc_set) {
    size_t delay_count = arc_set->arcCount() * ap_count_;
    pt_edge.setArcDelays(nullptr, delay_count);
  }

  // Link into adjacency lists
  EdgeId next = pt_vertices_[pt_from].out_edges_;
  pt_edge.vertex_out_next_ = next;
  pt_edge.vertex_out_prev_ = pt_edge_id_null;
  if (next != pt_edge_id_null)
    pt_edges_[next].vertex_out_prev_ = edge_id;
  pt_vertices_[pt_from].out_edges_ = edge_id;

  pt_edge.vertex_in_link_ = pt_vertices_[pt_to].in_edges_;
  pt_vertices_[pt_to].in_edges_ = edge_id;

  sorted_ = false;
  return edge_id;
}

void
PtGraph::setVirtualEdgeLms(PtEdge &pt_edge, const std::vector<LMValue> &lms)
{
  pt_edge.setArcLms(lms);
}

void
PtGraph::initVirtualPaths(PtVertex &virtual_vertex, PtVertex &source_vertex)
{
  int src_tg_idx = source_vertex.tagGroupIndex();
  if (src_tg_idx == static_cast<int>(sta::tag_group_index_max)) {
    virtual_vertex.setPaths(nullptr);
    virtual_vertex.setTagGroupIndex(sta::tag_group_index_max);
    return;
  }
  virtual_vertex.setTagGroupIndex(src_tg_idx);
  // Do NOT pre-allocate paths here. localSetVertexArrivals will allocate via
  // makePaths + copyPaths when it sees paths_ == nullptr, ensuring full Path
  // initialization (including Tag*). Pre-allocating uninitialized Path objects
  // and then using ptCopyPaths leaves Tag* garbage, causing crashes in tagIndex().
  virtual_vertex.setPaths(nullptr);
}

void
PtGraph::deleteOutEdge(VertexId from_id, EdgeId edge_id)
{
  PtEdge &pt_edge = pt_edges_[edge_id];
  EdgeId next = pt_edge.vertex_out_next_;
  EdgeId prev = pt_edge.vertex_out_prev_;
  if (prev != pt_edge_id_null)
    pt_edges_[prev].vertex_out_next_ = next;
  else
    pt_vertices_[from_id].out_edges_ = next;
  if (next != pt_edge_id_null)
    pt_edges_[next].vertex_out_prev_ = prev;
}

void
PtGraph::deleteInEdge(VertexId to_id, EdgeId edge_id)
{
  EdgeId prev = pt_edge_id_null;
  for (EdgeId i = pt_vertices_[to_id].in_edges_;
       i != pt_edge_id_null && i != edge_id;
       i = pt_edges_[i].vertex_in_link_)
    prev = i;
  if (prev != pt_edge_id_null)
    pt_edges_[prev].vertex_in_link_ = pt_edges_[edge_id].vertex_in_link_;
  else
    pt_vertices_[to_id].in_edges_ = pt_edges_[edge_id].vertex_in_link_;
}

void
PtGraph::deleteEdge(EdgeId edge_id)
{
  PtEdge &pt_edge = pt_edges_[edge_id];
  deleteOutEdge(pt_edge.pt_from_, edge_id);
  deleteInEdge(pt_edge.pt_to_, edge_id);
  pt_edge.setType(PtEdgeType::Sentinel);
  sorted_ = false;
}

void
PtGraph::deleteVertex(VertexId vertex_id)
{
  PtVertex &pt_vertex = pt_vertices_[vertex_id];
  // Delete edges to vertex.
  EdgeId edge_id, next_id;
  for (edge_id = pt_vertex.in_edges_; edge_id != pt_edge_id_null; edge_id = next_id) {
    PtEdge &pt_edge = pt_edges_[edge_id];
    next_id = pt_edge.vertex_in_link_;
    deleteOutEdge(pt_edge.pt_from_, edge_id);
    pt_edge.setType(PtEdgeType::Sentinel);
  }
  // Delete edges from vertex.
  for (edge_id = pt_vertex.out_edges_; edge_id != pt_edge_id_null; edge_id = next_id) {
    PtEdge &pt_edge = pt_edges_[edge_id];
    next_id = pt_edge.vertex_out_next_;
    deleteInEdge(pt_edge.pt_to_, edge_id);
    pt_edge.setType(PtEdgeType::Sentinel);
  }
  pt_vertex.out_edges_ = pt_edge_id_null;
  pt_vertex.in_edges_ = pt_edge_id_null;
  pt_vertex.setType(PtVertexType::Sentinel);
  // Reset tagGroupIndex so PtVertexPathIterator doesn't see tagGroupIndex
  // set but paths_ == nullptr on a deleted (Sentinel) virtual vertex.
  pt_vertex.setTagGroupIndex(sta::tag_group_index_max);
  sorted_ = false;
}

sta::Path *
PtGraph::makePaths(sta::VertexId vertex_id, size_t path_count)
{
  if (vertex_id >= pt_vertices_.size()) {
    throw std::out_of_range("PtGraph::makePaths: invalid vertex id");
    return nullptr;
  }
  sta::Path *paths = new sta::Path[path_count];
  pt_vertices_[vertex_id].setPaths(paths);
  return paths;
}

void 
PtGraph::deletePaths(sta::VertexId vertex_id)
{
  if (vertex_id >= pt_vertices_.size()) {
    throw std::out_of_range("PtGraph::deletePaths: invalid vertex id");
    return;
  }
  pt_vertices_[vertex_id].setPaths(nullptr);
   pt_vertices_[vertex_id].tag_group_index_ = sta::tag_group_index_max;
}

void
PtGraph::initPaths(sta::VertexId vertex_id)
{
  if (vertex_id >= pt_vertices_.size()) {
    throw std::out_of_range("PtGraph::initPaths: invalid vertex id");
    return;
  }
  PtVertex &pt_vertex = pt_vertices_[vertex_id];
  initPaths(pt_vertex);
}

void 
PtGraph::initPaths(PtVertex &pt_vertex)
{
  sta::Vertex *vertex = pt_vertex.vertex();
  sta::TagGroup *tag_group = sta_->search()->tagGroup(vertex);
  if (tag_group == nullptr) {
    // printf("PtGraph::initPaths: vertex %s has no tag group\n",
    //        sta_->network()->name(vertex->pin()));
    // fflush(stdout);
    pt_vertex.setPaths(nullptr);
    pt_vertex.setTagGroupIndex(sta::tag_group_index_max);
    return;
  }
  size_t path_count = tag_group->pathCount();
  sta::Path *paths = vertex->paths();
  sta::Path *new_paths = new sta::Path[path_count];
  for (size_t i = 0; i < path_count; i++) {
    new_paths[i] = paths[i];
    new_paths[i].setIsEnum(false);
  }
  pt_vertex.setPaths(new_paths);
  pt_vertex.setTagGroupIndex(tag_group->index());
}

void
PtGraph::updateTimingArcSets()
{
  if (ref_lib_cell_ == nullptr) {
    // printf("PtGraph::findRefTimingArcSet: ref_lib_cell_ is nullptr\n");
    // fflush(stdout);
    return;
  }

  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel)
      continue;
    if (pt_edge.type() == PtEdgeType::RefInstEdge) {
      sta::TimingArcSet *ref_arc_set = pt_edge.timingArcSet();
      if (ref_arc_set == nullptr) {
        // printf("PtGraph::updateTimingArcSets: PtEdge %u has no timing arc set\n",
               // pt_edge.objectIdx());
        // fflush(stdout);
        continue;
      }
      sta::TimingArcSet *new_arc_set = 
                ref_lib_cell_->findTimingArcSet(ref_arc_set);
      if (new_arc_set == nullptr) {
        // Fallback: find an unconditional arc set for the same
        // from/to port pair.  This handles cases where the lib
        // vendor used different conditional-arc granularity
        // between drive strengths (e.g. ASAP7).
        sta::LibertyPort *old_from = ref_arc_set->from();
        sta::LibertyPort *old_to   = ref_arc_set->to();
        if (old_from && old_to) {
          sta::LibertyPort *new_from =
              ref_lib_cell_->findLibertyPort(old_from->name());
          sta::LibertyPort *new_to =
              ref_lib_cell_->findLibertyPort(old_to->name());
          if (new_from && new_to) {
            const sta::TimingArcSetSeq &candidates =
                ref_lib_cell_->timingArcSets(new_from, new_to);
            // Prefer the unconditional arc set (cond == nullptr).
            for (sta::TimingArcSet *candidate : candidates) {
              if (candidate->cond() == nullptr) {
                new_arc_set = candidate;
                break;
              }
            }
            // If no unconditional arc set, use the first available.
            if (new_arc_set == nullptr && !candidates.empty()) {
              new_arc_set = candidates[0];
            }
          }
        }
      }
      if (new_arc_set == nullptr) {
        // printf("PtGraph::updateTimingArcSets: no matching timing arc set in ref_lib_cell_ %s for PtEdge %u of edge %s\n",
        //        ref_lib_cell_->name(),
        //        pt_edge.objectIdx(),
        //        pt_edge.edge()->to_string(sta_->graph()).c_str());
        // fflush(stdout);
        continue;
      }
      pt_edge.setTimingArcSet(new_arc_set);
    }
  }
}

sta::TagGroup *
PtGraph::tagGroup(const PtVertex &pt_vertex)
{
  return sta_->search()->tagGroup(pt_vertex.tagGroupIndex());
}

void
PtGraph::writeSlewToGraph(const PtVertex &pt_vertex, sta::Vertex *sta_vertex)
{
  sta::Graph *sta_graph = sta_->graph();
  for (const sta::RiseFall *rf : sta::RiseFall::range()) {
    for (size_t ap = 0; ap < ap_count_; ap++) {
      sta::Slew s = slew(pt_vertex, rf, ap);
      sta_graph->setSlew(sta_vertex, rf, ap, s);
    }
  }
}

void
PtGraph::writePathsToGraph(const PtVertex &pt_vertex, sta::Vertex *sta_vertex)
{
  sta::Path *pt_paths = pt_vertex.paths();
  sta::Path *sta_paths = sta_vertex->paths();
  if (!pt_paths || !sta_paths)
    return;
  sta::TagGroup *pt_tg = tagGroup(pt_vertex);
  sta::TagGroup *sta_tg = sta_->search()->tagGroup(sta_vertex);
  if (!pt_tg || !sta_tg || pt_tg->index() != sta_tg->index())
    return;
  size_t count = pt_tg->pathCount();
  for (size_t i = 0; i < count; i++) {
    sta_paths[i].setArrival(pt_paths[i].arrival());
    sta_paths[i].setRequired(pt_paths[i].required());
  }
}

bool
PtGraph::topoSortVertices()
{
  if (sorted_)
    return sorted_;
  if (graph_made_) {
    size_t n = (pt_vertices_.size() > 1) ? pt_vertices_.size() - 1 : 0;
    sorted_vertex_ids_.resize(n);
    if (n > 0)
      std::iota(sorted_vertex_ids_.begin(), sorted_vertex_ids_.end(), static_cast<VertexId>(1));
    for (auto &pt_vertex : pt_vertices_) {
      if (pt_vertex.type() == PtVertexType::Sentinel)
        continue;
      std::stable_sort(sorted_vertex_ids_.begin(), sorted_vertex_ids_.end(),
                       PtVertexIdLevelLess(this));
    }
    sorted_ = true;
  }
  return sorted_;
}

std::vector<size_t> &
PtGraph::sortedVertexIds()
{
  topoSortVertices();
  return sorted_vertex_ids_;
}

void
PtGraph::setSlew(PtVertex &pt_vertex, const RiseFall *rf,
               DcalcAPIndex ap_index, const sta::Slew &slew)
{
  const size_t slew_rf_count = 2;
  if (pt_vertex.slewCount() == 0) {
    size_t slew_count = slew_rf_count * ap_count_;
    pt_vertex.resizeSlews(slew_count);
  }
  sta::Slew *slews = pt_vertex.slews();
  size_t slew_index = ap_index * slew_rf_count + rf->index();
  slews[slew_index] = slew;
}


void
PtGraph::initLoadSlews(PtVertex &drvr_pt_vertex)
{
  PtVertexOutEdgeIterator out_iter(drvr_pt_vertex.objectIdx(), this);
  while (out_iter.hasNext()) {
    PtEdge &pt_edge = out_iter.next();
    if (pt_edge.isWire()) {
      VertexId to_id = pt_edge.ptToId();
      PtVertex &load_pt_vertex = pt_vertices_[to_id];
      for (const sta::DcalcAnalysisPt *dcalc_ap : sta_->corners()->dcalcAnalysisPts()) {
        const sta::MinMax *slew_min_max = dcalc_ap->slewMinMax();
        sta::Slew slew_init_value(slew_min_max->initValue());
        sta::DcalcAPIndex ap_index = dcalc_ap->index();
        for (const sta::RiseFall *rf : sta::RiseFall::range()) {
          setSlew(load_pt_vertex, rf, ap_index, slew_init_value);
        }
      }
    }
  }
}

void
PtGraph::initWireDelays(PtVertex &drvr_pt_vertex)
{
  PtVertexOutEdgeIterator out_iter(drvr_pt_vertex.objectIdx(), this);
  while (out_iter.hasNext()) {
    PtEdge &out_pt_edge = out_iter.next();
    if (out_pt_edge.isWire()
        && out_pt_edge.type() != PtEdgeType::VirtualWireEdge) {
      for (const sta::DcalcAnalysisPt * dcalc_ap : sta_->corners()->dcalcAnalysisPts()) {
        const sta::MinMax *delay_min_max = dcalc_ap->delayMinMax();
        sta::Delay delay_init_value(delay_min_max->initValue());
        sta::DcalcAPIndex ap_index = dcalc_ap->index();
        for (const sta::RiseFall *rf : sta::RiseFall::range()) {
          setWireArcDelay(out_pt_edge, rf, ap_index, delay_init_value);
        }
      }
    }
  }
}

void 
PtGraph::setWireArcDelay(PtEdge &pt_edge, 
                         const sta::RiseFall *rf,
                         sta::DcalcAPIndex ap_index,
                         const sta::ArcDelay &delay)
{
  ArcDelay *arc_delays = pt_edge.arcDelays();
  size_t index = rf->index() * ap_count_ + ap_index;
  arc_delays[index] = delay;
}

const sta::Slew &
PtGraph::slew(const PtVertex &pt_vertex,
                   const sta::RiseFall *rf,
                   sta::DcalcAPIndex ap_index)
{
  if (!slew_rf_count_) {
    static sta::Slew zero_slew(0.0);
    return zero_slew;
  }

  const sta::Slew *slews = pt_vertex.slews();
  size_t slew_index = ap_index * slew_rf_count_ + rf->index();
  return slews[slew_index];
}

sta::ArcDelay
PtGraph::arcDelay(const PtEdge &pt_edge,
                  const sta::TimingArc *arc,
                  sta::DcalcAPIndex ap_index) const
{
  const ArcDelay *arc_delays = pt_edge.arcDelays();
  size_t index = arc->index() * ap_count_ + ap_index;
  return arc_delays[index];
}

const sta::ArcDelay &
PtGraph::wireArcDelay(const PtEdge &pt_edge,
                         const sta::RiseFall *rf,
                         sta::DcalcAPIndex ap_index)
{
  const ArcDelay *arc_delays = pt_edge.arcDelays();
  size_t index = rf->index() * ap_count_ + ap_index;
  return arc_delays[index];
}

float
PtGraph::arcLm(const PtEdge &pt_edge,
                     const sta::TimingArc *timing_arc,
                     sta::DcalcAPIndex ap_index) const
{
  size_t lm_index = lmIndex(timing_arc, ap_index, ap_count_);
  const LMValue *lms = pt_edge.arcLms();
  if (lms == nullptr) {
    throw std::runtime_error("PtGraph::arcLm: pt_edge has no lm values");
  }
  return lms[lm_index];
}


void
PtGraph::setArcDelay(PtEdge &pt_edge, 
                    const sta::TimingArc *arc,
                    sta::DcalcAPIndex ap_index,
                    const sta::ArcDelay &delay)
{
  ArcDelay *arc_delays = pt_edge.arcDelays();
  size_t index = arc->index() * ap_count_ + ap_index;
  arc_delays[index] = delay;
}

std::string
PtGraph::to_string()
{
  std::string graph_descri;
  graph_descri += "Showing PtGraph:\n";
  for (size_t vid = 1; vid < pt_vertices_.size(); vid++) {
    PtVertex &pt_vertex = pt_vertices_[vid];
    graph_descri += "PtVertex " + std::to_string(vid) + " (Vertex " + std::to_string(pt_vertex.objectIdx()) + "):\n";
    PtVertexOutEdgeIterator out_iter(vid, this);
    while (out_iter.hasNext()) {
      PtEdge &pt_edge = out_iter.next();
      Edge *edge = pt_edge.edge();
      std::string edge_str = edge ? edge->to_string(sta_->graph())
                                  : ("virtual_" + std::to_string(pt_edge.objectIdx()));
      graph_descri += "  PtEdge " + std::to_string(pt_edge.objectIdx()) + " (Edge " + edge_str + ")\n";
    }
  }
  graph_descri += "End of PtGraph\n";
  return graph_descri;
}

void
PtGraph::printGraph(bool dot_format)
{
  printGraph(nullptr, dot_format);
}

void
PtGraph::printGraph(const char *output_path, bool dot_format)
{
  sta::Network *network = sta_->network();
  const char *ref_name = ref_inst_ ? network->name(ref_inst_) : "nullptr";
  size_t vertex_count = pt_vertices_.size() > 0 ? pt_vertices_.size() - 1 : 0;
  size_t edge_count = pt_edges_.size() > 0 ? pt_edges_.size() - 1 : 0;
  FILE *out = stdout;

  if (output_path && output_path[0] != '\0') {
    out = fopen(output_path, "w");
    if (!out) {
      // fprintf(stderr,
              // "PtGraph::printGraph: failed to open %s, using stdout\n",
              // output_path);
      out = stdout;
    }
  }

  if (dot_format) {
    std::string ref_label = dotEscape(ref_name);
    // fprintf(out, "digraph PtGraph {\n");
    // fprintf(out, "  label=\"PtGraph ref %s\";\n", ref_label.c_str());
    // fprintf(out, "  labelloc=\"t\";\n");
    // fprintf(out, "  node [shape=box];\n");
    for (size_t vid = 1; vid < pt_vertices_.size(); vid++) {
      const PtVertex &pt_vertex = pt_vertices_[vid];
      const Vertex *vertex = pt_vertex.vertex();
      std::string vertex_name = vertex ? vertex->to_string(sta_)
                                       : ("virtual_" + std::to_string(vid));
      std::string label = dotEscape(vertex_name);
      // fprintf(out, "  v%zu [label=\"%zu: %s\"];\n", vid, vid, label.c_str());
    }
    for (size_t eid = 1; eid < pt_edges_.size(); eid++) {
      const PtEdge &pt_edge = pt_edges_[eid];
      const Edge *edge = pt_edge.edge();
      std::string edge_label = pt_edge.isWire() ? "wire" : "inst";
      if (pt_edge.type() != PtEdgeType::None) {
        edge_label += "/";
        edge_label += ptEdgeTypeName(pt_edge.type());
      }
      edge_label = dotEscape(edge_label);
      // fprintf(out, "  v%u -> v%u [label=\"%s\"];\n",
              // pt_edge.ptFromId(),
              // pt_edge.ptToId(),
              // edge_label.c_str());
    }
    // fprintf(out, "}\n");
  } else {
    // fprintf(out, "PtGraph for ref inst %s: %zu vertices, %zu edges\n",
            // ref_name,
            // vertex_count,
            // edge_count);
    for (size_t vid = 1; vid < pt_vertices_.size(); vid++) {
      PtVertex &pt_vertex = pt_vertices_[vid];
      const Vertex *vertex = pt_vertex.vertex();
      std::string vertex_name = vertex ? vertex->to_string(sta_)
                                       : ("virtual_" + std::to_string(vid));
      // fprintf(out,
              // "PtVertex %zu: %s, level %.1f, type %s, driver %s, load %s, fanin %s, fanout %s\n",
              // vid,
              // vertex_name.c_str(),
              // pt_vertex.level(),
              // ptVertexTypeName(pt_vertex.type()),
              // pt_vertex.isDriver() ? "yes" : "no",
              // pt_vertex.isLoad() ? "yes" : "no",
              // pt_vertex.hasFanin() ? "yes" : "no",
              // pt_vertex.hasFanout() ? "yes" : "no");
      PtVertexOutEdgeIterator out_iter(vid, this);
      while (out_iter.hasNext()) {
        PtEdge &pt_edge = out_iter.next();
        const Edge *edge = pt_edge.edge();
        std::string edge_name = edge ? edge->to_string(sta_->graph())
                                     : ("virtual_" + std::to_string(pt_edge.objectIdx()));
        const char *edge_kind = pt_edge.isWire() ? "wire" : "inst";
        // fprintf(out, "  PtEdge %u -> PtVertex %u: %s, type %s, %s\n",
                // pt_edge.objectIdx(),
                // pt_edge.ptToId(),
                // edge_name.c_str(),
                // ptEdgeTypeName(pt_edge.type()),
                // edge_kind);
      }
    }
  }

  // fflush(out);
  if (out != stdout) {
    fclose(out);
  }
}

void
PtGraph::printDelays()
{
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() != PtEdgeType::Sentinel) {
      Edge *edge = pt_edge.edge();
      std::string edge_str = edge ? edge->to_string(sta_->graph())
                                  : ("virtual_" + std::to_string(pt_edge.objectIdx()));
      // printf("PtEdge %u (Edge %s) delays:\n",
             // pt_edge.objectIdx(),
             // edge_str.c_str());
      size_t delay_count = slew_rf_count_ * ap_count_;
      ArcDelay *arc_delays = pt_edge.arcDelays();
      for (size_t i = 0; i < delay_count; i++) {
        // printf("  Delay[%zu]: %f\n", i, arc_delays[i] * 1e12);
      }
    }
  }
  // fflush(stdout);
}

void PtGraph::initVertexAndEdges()
{
  if (!graph_made_) {
    // printf("PtGraph::initVertexAndEdges: graph not made yet\n");
    // fflush(stdout);
    return;
  }

  for (PtVertex &pt_vertex : pt_vertices_) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    pt_vertex.copyInfoFromVertex(ap_count_, slew_rf_count_);
    initPaths(pt_vertex);
  }
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel)
      continue;
    pt_edge.copyInfoFromEdge(ap_count_);
    pt_edge.timing_arc_set_ = pt_edge.edge()->timingArcSet();
  }
}

void 
PtGraph::setAllArcDelaysZero()
{
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.objectIdx() == pt_edge_id_null)
      continue;

    size_t delay_count = slew_rf_count_ * ap_count_;
    if (pt_edge.arcDelays() == nullptr)
      continue;
    for (size_t i = 0; i < delay_count; i++) {
      pt_edge.arcDelays()[i] = ArcDelay(0.0f);
    }
  }
}

void
PtGraph::delayLmSum(const sta::MinMax *minmax, float &delay_lambda_sum, bool avoid_check)
{
  delay_lambda_sum = 0.0f;
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel)
      continue;
    sta::TimingArcSet *arc_set = pt_edge.timingArcSet();
    if (arc_set == nullptr)
      continue;
    for (sta::DcalcAnalysisPt *dcalc_ap : sta_->corners()->dcalcAnalysisPts()) {
      sta::DcalcAPIndex ap_index = dcalc_ap->index();
      const sta::MinMax *delay_min_max = dcalc_ap->delayMinMax();
      if (delay_min_max != minmax)
        continue;
      for (sta::TimingArc *timing_arc : arc_set->arcs()) {
        size_t lm_index = lmIndex(timing_arc, ap_index, ap_count_);
        const ArcDelay &arc_delay = arcDelay(pt_edge, timing_arc, ap_index);
        if (avoid_check && (pt_edge.role()->isTimingCheck()))
          continue;
        const LMValue *lms = pt_edge.arcLms();
        if (lms == nullptr) {
          // printf("PtGraph::delayLmSum: pt_edge %u has no lm values\n",
                  // pt_edge.objectIdx());
          // fflush(stdout);
          continue;
        }
        LMValue arc_lm = lms[lm_index];
        delay_lambda_sum += arc_delay * arc_lm;
      }
    }
  }
}

void
PtGraph::delayLmSum(const sta::DcalcAnalysisPt *dcalc_ap,
                    float &delay_lambda_sum,
                    bool avoid_check)
{
  const sta::DcalcAPIndex ap_index = dcalc_ap->index();
  delay_lambda_sum = 0.0f;
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel
        || (!pt_edge.hasBase() && !pt_edge.isVirtual()))
      continue;
    sta::TimingArcSet *arc_set = pt_edge.timingArcSet();
    if (arc_set == nullptr)
      continue;
    for (sta::TimingArc *timing_arc : arc_set->arcs()) {
      size_t lm_index = lmIndex(timing_arc, ap_index, ap_count_);
      const ArcDelay &arc_delay = arcDelay(pt_edge, timing_arc, ap_index);
      if (avoid_check && (pt_edge.role()->isTimingCheck()))
        continue;
      const LMValue *lms = pt_edge.arcLms();
      if (lms == nullptr) {
        // printf("PtGraph::delayLmSum: pt_edge %u has no lm values\n",
        //        pt_edge.objectIdx());
        // fflush(stdout);
        continue;
      }
      LMValue arc_lm = lms[lm_index];
      delay_lambda_sum += arc_delay * arc_lm;
    }
  }
}

void
PtGraph::delayLmSum(const sta::DcalcAnalysisPt *dcalc_ap,
                    DelayLmSumResult *result,
                    bool collect_vecs)
{
  if (result == nullptr)
    return;

  const sta::DcalcAPIndex ap_index = dcalc_ap->index();
  result->delay_lm_sum = 0.0f;
  if (collect_vecs) {
    result->vec_lms.clear();
    result->vec_delays.clear();
  }
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel
        || (!pt_edge.hasBase() && !pt_edge.isVirtual()))
      continue;
    sta::TimingArcSet *arc_set = pt_edge.timingArcSet();
    if (arc_set == nullptr)
      continue;
    for (sta::TimingArc *timing_arc : arc_set->arcs()) {
      size_t lm_index = lmIndex(timing_arc, ap_index, ap_count_);
      const ArcDelay &arc_delay = arcDelay(pt_edge, timing_arc, ap_index);
      const LMValue *lms = pt_edge.arcLms();
      if (lms == nullptr) {
        // printf("PtGraph::delayLmSum: pt_edge %u has no lm values\n",
        //        pt_edge.objectIdx());
        // fflush(stdout);
        continue;
      }
      LMValue arc_lm = lms[lm_index];
      result->delay_lm_sum += arc_delay * arc_lm;
      if (collect_vecs) {
        result->vec_lms.push_back(arc_lm);
        result->vec_delays.push_back(arc_delay);
      }
    }
  }
}

void
PtGraph::refgateDelayLmSum(float &delay_lambda_sum, sta::DcalcAnalysisPt *dcalc_ap)
{
  if (dcalc_ap == nullptr) dcalc_ap = dcalc_ap_;
  delay_lambda_sum = 0.0f;
  sta::DcalcAPIndex ref_ap_index = dcalc_ap->index();
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel)
      continue;
    if (pt_edge.type() == PtEdgeType::RefInstEdge) {
      sta::TimingArcSet *arc_set = pt_edge.timingArcSet();
      if (arc_set == nullptr)
        continue;
      for (sta::TimingArc *timing_arc : arc_set->arcs()) {
        const ArcDelay &arc_delay = arcDelay(pt_edge, timing_arc, ref_ap_index);
        LMValue arc_lm = arcLm(pt_edge, timing_arc, ref_ap_index);
        delay_lambda_sum += arc_delay * arc_lm;
      }
    }
  }
}

sta::Level
PtGraph::topVertexLevel()
{
  sta::Level max_level = -1;
  for (auto &pair : vertex_map_) {
    sta::Vertex *vertex = const_cast<sta::Vertex*>(pair.first);
    sta::Level level = vertex->level();
    if (level > max_level)
      max_level = level;
  }
  return max_level;
}

void
PtGraph::annotateVerticesType()
{
  if (ref_inst_ == nullptr)
    return;

  sta::InstancePinIterator *pin_iter = sta_->network()->pinIterator(ref_inst_);
  while(pin_iter->hasNext()) {
    sta::Pin *pin = pin_iter->next();
    if (sta_->network()->isLoad(pin)) {
      sta::Vertex *load_vertex = sta_->graph()->pinLoadVertex(pin);
      if (load_vertex == nullptr)
        continue;
      auto it = vertex_map_.find(load_vertex);
      if (it == vertex_map_.end()) {
        // Commonly these vertices are !searchFrom vertices
        // printf("PtGraph::annotateRefFaninVertices: load vertex %s not found in vertex_map_\n",
        //        load_vertex->to_string(sta_).c_str());
        // fflush(stdout);
        continue;
      }
      ptVertex(it->second).setType(PtVertexType::RefInput);

      PtVertexInEdgeIterator in_edge_iter(it->second, this);
      while (in_edge_iter.hasNext()) {
        PtEdge &pt_edge = in_edge_iter.next();
        VertexId from_pt_id = pt_edge.ptFromId();
        ptVertex(from_pt_id).setType(PtVertexType::RefDriver);
      }
    } 
    if (sta_->network()->isDriver(pin)) {
      sta::Vertex *drvr_vertex = sta_->graph()->pinDrvrVertex(pin);
      if (drvr_vertex == nullptr)
        continue;
      auto it = vertex_map_.find(drvr_vertex);
      if (it == vertex_map_.end()) {
        // printf("PtGraph::annotateRefFaninVertices: drvr vertex not found in vertex_map_\n");
        // fflush(stdout);
        continue;
      }
      ptVertex(it->second).setType(PtVertexType::RefOutput);
    }
  }
  delete pin_iter;
}

void 
PtGraph::annotateEdgesType()
{
  for (PtEdge &pt_edge : pt_edges_) {
    if (pt_edge.type() == PtEdgeType::Sentinel)
      continue;
    PtVertex &from_pt_vertex = pt_vertices_[pt_edge.ptFromId()];
    PtVertex &to_pt_vertex = pt_vertices_[pt_edge.ptToId()];
    if (from_pt_vertex.type() == PtVertexType::RefInput &&
        to_pt_vertex.type() == PtVertexType::RefOutput) {
      pt_edge.setType(PtEdgeType::RefInstEdge);
    }
  }
}

const PtVertex &
PtGraph::pinToPtVertex(const sta::Pin *pin) const
{
  Vertex *vertex, *bidirect_vertex;
  sta_->graph()->pinVertices(pin, vertex, bidirect_vertex);
  if (vertex) {
    auto it = vertex_map_.find(vertex);
    if (it != vertex_map_.end()) {
      VertexId pt_vertex_id = it->second;
      return ptVertex(pt_vertex_id);
    }
    else {
      // printf("PtGraph::pinToPtVertex: vertex %s not found in vertex_map_\n",
             // vertex->to_string(sta_->graph()).c_str());
      // fflush(stdout);
      throw std::out_of_range("PtGraph::pinToPtVertex: vertex not found in vertex_map_");
    }
  } else {
    throw std::out_of_range("PtGraph::pinToPtVertex: vertex is nullptr for pin");
  }
  return pt_vertices_[0]; // should not reach here
}

float
PtGraph::getRefPinCapacitance(const PtVertex &pt_vertex,
                              const sta::RiseFall *rf,
                              const sta::Corner *corner,
                              const sta::MinMax *min_max) const
{
  float port_cap = 0.0f;
  if (pt_vertex.type() != PtVertexType::RefInput && 
      pt_vertex.type() != PtVertexType::RefOutput) {
    // printf("PtGraph::getRefPinCapacitance: pt_vertex is not RefInput type\n");
    // fflush(stdout);
    return 0.0f;
  }
  
  if (ref_lib_cell_ == nullptr) {
    // printf("PtGraph::getRefPinCapacitance: ref_lib_cell_ is nullptr\n");
    // fflush(stdout);
    return 0.0f;
  }
  
  const sta::Pin *pin = pt_vertex.vertex()->pin();
  if (pin == nullptr) {
    // printf("PtGraph::getRefPinCapacitance: pin is nullptr\n");
    // fflush(stdout);
    return 0.0f;
  }
  
  const char* pin_name = sta_->network()->portName(pin);
  if (pin_name == nullptr) {
    // printf("PtGraph::getRefPinCapacitance: pin_name is nullptr for pin\n");
    // fflush(stdout);
    return 0.0f;
  }
  
  sta::LibertyPort *lib_port = ref_lib_cell_->findLibertyPort(pin_name);
  if (lib_port == nullptr) {
    // printf("PtGraph::getRefPinCapacitance: no lib port for pin %s in cell %s\n", 
           // pin_name, ref_lib_cell_->name());
    // fflush(stdout);
    return 0.0f;
  }
  
  port_cap = sta_->sdc()->portCapacitance(ref_inst_,
                                          lib_port,
                                          rf,
                                          corner,
                                          min_max);
  return port_cap;
}

void 
PtGraph::printSlews()
{
  // Placeholder
}

void
PtGraph::printCapacitances()
{
  // Placeholder
}

void 
PtGraph::printArrivals()
{
  // Placeholder
}

void 
PtGraph::printRequireds()
{
  // Placeholder
}
  

////////////////////////////////////////////////////////////////
// PtEdge
////////////////////////////////////////////////////////////////
PtEdge::PtEdge() :
  edge_(nullptr),
  vertex_out_next_(pt_edge_id_null),
  vertex_out_prev_(pt_edge_id_null),
  vertex_in_link_(pt_edge_id_null)
{
  object_idx_ = pt_edge_id_null;
}

void
PtEdge::setObjectIdx(EdgeId idx)
{
  object_idx_ = idx;
}

void
PtEdge::init(sta::Edge *edge, 
            VertexId pt_from, 
            VertexId pt_to)
{
  edge_ = edge;
  pt_from_ = pt_from;
  pt_to_ = pt_to;
}

void
PtEdge::setArcDelays(ArcDelay *arc_delay, size_t delay_count)
{
  if (delay_count > 0) {
    if (arc_delay)
      arc_delays_.assign(arc_delay, arc_delay + delay_count);
    else
      arc_delays_.resize(delay_count, 0);
  } else {
    arc_delays_.clear();
  }
}

void
PtEdge::copyInfoFromEdge(size_t ap_count)
{
  size_t delay_count = edge_->timingArcSet()->arcCount() * ap_count;
  ArcDelay *src_arc_delays = edge_->arcDelays();
  if (src_arc_delays && delay_count > 0)
    arc_delays_.assign(src_arc_delays, src_arc_delays + delay_count);
  else
    arc_delays_.clear();
}

void
PtEdge::initVirtual(VertexId pt_from, VertexId pt_to,
                    sta::TimingArcSet *arc_set, bool is_wire)
{
  edge_ = nullptr;
  pt_from_ = pt_from;
  pt_to_ = pt_to;
  timing_arc_set_ = arc_set;
  is_wire_ = is_wire;
}

const sta::TimingRole *
PtEdge::role() const
{
  // Use cached timing_arc_set_ instead of edge_->role() to avoid
  // dereferencing a potentially dangling sta::Edge* during parallel
  // resize (replaceCell on another thread may delete/recreate edges).
  if (timing_arc_set_)
    return timing_arc_set_->role();
  return sta::TimingRole::combinational();
}

LMValue *
PtEdge::arcLms()
{
  if (edge_)
    return edge_->arcLms();
  return arc_lms_.empty() ? nullptr : arc_lms_.data();
}

const LMValue *
PtEdge::arcLms() const
{
  if (edge_)
    return edge_->arcLms();
  return arc_lms_.empty() ? nullptr : arc_lms_.data();
}

void
PtEdge::setArcLms(const std::vector<LMValue> &lms)
{
  arc_lms_ = lms;
}

//////////////////////////////////////////////////////////////////
// PtVertex
//////////////////////////////////////////////////////////////////
PtVertex::PtVertex() :
  vertex_(nullptr),
  out_edges_(pt_edge_id_null),
  in_edges_(pt_edge_id_null),
  is_root_(false)
{
  object_idx_ = pt_vertex_id_null;
}

PtVertex::~PtVertex()
{
  delete[] paths_;
}

PtVertex::PtVertex(PtVertex &&other) noexcept
  : vertex_(other.vertex_),
    proxy_vertex_(other.proxy_vertex_),
    liberty_port_(other.liberty_port_),
    liberty_cell_(other.liberty_cell_),
    level_(other.level_),
    arrivals_(std::move(other.arrivals_)),
    object_idx_(other.object_idx_),
    out_edges_(other.out_edges_),
    in_edges_(other.in_edges_),
    slews_(std::move(other.slews_)),
    is_root_(other.is_root_),
    type_(other.type_),
    tag_group_index_(other.tag_group_index_),
    paths_(other.paths_),
    is_driver_(other.is_driver_),
    is_load_(other.is_load_)
{
  other.paths_ = nullptr;
}

PtVertex &PtVertex::operator=(PtVertex &&other) noexcept
{
  if (this != &other) {
    delete[] paths_;
    vertex_ = other.vertex_;
    proxy_vertex_ = other.proxy_vertex_;
    liberty_port_ = other.liberty_port_;
    liberty_cell_ = other.liberty_cell_;
    level_ = other.level_;
    arrivals_ = std::move(other.arrivals_);
    object_idx_ = other.object_idx_;
    out_edges_ = other.out_edges_;
    in_edges_ = other.in_edges_;
    slews_ = std::move(other.slews_);
    is_root_ = other.is_root_;
    type_ = other.type_;
    tag_group_index_ = other.tag_group_index_;
    paths_ = other.paths_;
    is_driver_ = other.is_driver_;
    is_load_ = other.is_load_;
    other.paths_ = nullptr;
  }
  return *this;
}

void
PtVertex::init(sta::Vertex *vertex)
{
  vertex_ = vertex;
  level_ = static_cast<float>(vertex->level());
  out_edges_ = pt_edge_id_null;
  in_edges_ = pt_edge_id_null;
  is_root_ = false;
  arrivals_.clear();
  slews_.clear();
}

void
PtVertex::initVirtual(sta::LibertyCell *cell, sta::LibertyPort *port,
                      bool is_driver, bool is_load)
{
  vertex_ = nullptr;
  liberty_cell_ = cell;
  liberty_port_ = port;
  is_driver_ = is_driver;
  is_load_ = is_load;
  out_edges_ = pt_edge_id_null;
  in_edges_ = pt_edge_id_null;
  is_root_ = false;
  arrivals_.clear();
  slews_.clear();
}

bool
PtVertex::hasFanin() const
{
  return in_edges_ != pt_edge_id_null;
}

bool 
PtVertex::isRoot() const
{
  return !hasFanin();
}

bool
PtVertex::hasFanout() const
{
  return out_edges_ != pt_edge_id_null;
}

void
PtVertex::setObjectIdx(VertexId idx)
{
  object_idx_ = idx;
}

void
PtVertex::resizeSlews(size_t slew_count)
{
  if (slew_count == 0)
    slews_.clear();
  else
  slews_.assign(slew_count, sta::Slew());
}

void
PtVertex::copyInfoFromVertex(size_t ap_count, size_t slew_rf_count)
{
  level_ = static_cast<float>(vertex_->level());
  sta::Slew *src_slews = vertex_->slews();
  size_t slew_count = slew_rf_count * ap_count;
  if (src_slews) {
    slews_.assign(src_slews, src_slews + slew_count);
  } else {
    slews_.clear();
  }
}

void
PtVertex::setPaths(sta::Path *paths)
{
  delete[] paths_;
  paths_ = paths;
}

//////////////////////////////////////////////////////////////////
// PtVertexInEdgeIterator
//////////////////////////////////////////////////////////////////
PtVertexInEdgeIterator::PtVertexInEdgeIterator(VertexId vertex_id,
                                                PtGraph *pt_graph) :
  pt_graph_(pt_graph)
{
  if (pt_graph_ && pt_graph_->pt_vertices_.size() > vertex_id) {
    next_ = pt_graph_->ptVertex(vertex_id).in_edges_;
  } else {
    // printf("PtVertexInEdgeIterator: invalid vertex id %u\n", vertex_id);
    next_ = pt_edge_id_null;
  }
}

PtEdge &
PtVertexInEdgeIterator::next()
{
  EdgeId current = next_;
  if (current != pt_edge_id_null) {
    next_ = pt_graph_->edge(current).vertex_in_link_;
  }
  return pt_graph_->edge(current);
}

bool 
PtVertexInEdgeIterator::hasNext()
{
  return next_ != pt_edge_id_null;
}


//////////////////////////////////////////////////////////////////
// PtVertexOutEdgeIterator
//////////////////////////////////////////////////////////////////
PtVertexOutEdgeIterator::PtVertexOutEdgeIterator(VertexId vertex_id,
                                               PtGraph *pt_graph) :
  vertex_id_(vertex_id),
  pt_graph_(pt_graph)
{
  next_ = pt_graph_->ptVertex(vertex_id_).out_edges_;
}

PtVertexOutEdgeIterator::PtVertexOutEdgeIterator(PtVertex &pt_vertex,
                                               PtGraph *pt_graph) :
  vertex_id_(pt_vertex.objectIdx()),
  pt_graph_(pt_graph)
{
  next_ = pt_graph_->ptVertex(vertex_id_).out_edges_;
}

PtEdge &
PtVertexOutEdgeIterator::next()
{
  EdgeId current = next_;
  if (current != pt_edge_id_null) {
    next_ = pt_graph_->pt_edges_[current].vertex_out_next_;
  }
  return pt_graph_->pt_edges_[current];
}

bool
PtVertexOutEdgeIterator::hasNext()
{
  return next_ != pt_edge_id_null;
}


} // namespace lrf

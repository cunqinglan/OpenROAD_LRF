
#include <atomic>

#include "TaskArranger.hh"
#include "search/Levelize.hh"
#include "sta/ObjectTable.hh"
#include "sta/Search.hh"
#include "sta/Network.hh"
#include "sta/ConcreteNetwork.hh"
#include "sta/Graph.hh"



namespace lrf {

class InstVertexLevelLess
{
 public:
  InstVertexLevelLess();
  bool operator()(const InstVertex* vertex1, const InstVertex* vertex2) const;

 protected:
};

InstVertexLevelLess::InstVertexLevelLess()
{
}

bool InstVertexLevelLess::operator()(const InstVertex* vertex1,
                                 const InstVertex* vertex2) const
{
  Level level1 = vertex1->level();
  Level level2 = vertex2->level();
  return (level1 < level2)
         || (level1 == level2
             // Break ties for stable results.
             && (vertex1->objectIdx() < vertex2->objectIdx()));
}

void 
InstVertex::init(sta::Instance* instance, sta::StaState* sta)
{
  inst_ = instance;
  Level level = 0;
  sta::InstancePinIterator *pin_iter = sta->network()->pinIterator(instance);
  sta::Vertex *vertex, *bidirect_vertex;
  while (pin_iter->hasNext()) {
    sta::Pin *pin = pin_iter->next();
    sta->graph()->pinVertices(pin, vertex, bidirect_vertex);
    if (vertex) {
      level = vertex->level();
      break;
    }
  }
  delete pin_iter;
  level_ = level;
}

TaskArranger::TaskArranger(sta::StaState *sta)
  : sta::StaState(sta),
    pred_(new sta::SearchPredNonReg2(sta))
{
  vertices_.clear();
  edges_.clear();
  levelize_->ensureLevelized();
}

TaskArranger::~TaskArranger()
{
  vertices_.clear();
  edges_.clear();
}

void
TaskArranger::init()
{
  vertices_.clear();
  edges_.clear();
  makeGraph();
}

VertexId
TaskArranger::id(const InstVertex *vertex) const
{
  return vertex->objectIdx();
}

VertexId
TaskArranger::instToVertexId(const sta::Instance* inst) const
{
  const sta::ConcreteInstance* cinst = reinterpret_cast<const sta::ConcreteInstance*>(inst);
  VertexId vid = cinst->id1();
  return vid;
}

const InstVertex*
TaskArranger::vertex(VertexId id) const
{
  return &(vertices_[id]);
}

InstVertex*
TaskArranger::vertex(VertexId id)
{
  return &(vertices_[id]);
}

const InstVertex*
TaskArranger::vertex(const sta::Instance* inst) const
{
  VertexId vid = instToVertexId(inst);
  return vertex(vid);
}

InstVertex*
TaskArranger::vertex(const sta::Instance* inst)
{
  VertexId vid = instToVertexId(inst);
  return vertex(vid);
}

const InstEdge*
TaskArranger::edge(EdgeId id) const
{
  if (id >= edges_.size())
    return nullptr;
  return &(edges_[id]);
}

InstEdge*
TaskArranger::edge(EdgeId id)
{
  if (id >= edges_.size())
    return nullptr;
  return &(edges_[id]);
}

void
TaskArranger::makeGraph()
{
  // We first make vertices for all instances One vertex
  // per instance. We first count the number of instances
  makeVertices();
  makeEdges();
}

void 
TaskArranger::makeVertices()
{
  sta::LeafInstanceIterator *inst_iter = network_->leafInstanceIterator();
  int num_insts = 0;
  while (inst_iter->hasNext()) {
    num_insts++;
  }
  delete inst_iter;
  vertices_.resize(num_insts);
  inst_iter = network_->leafInstanceIterator();
  num_insts = 0;
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    vertices_[num_insts].init(inst, this);
    setInstanceId1(inst, num_insts);
    num_insts++;
  }
  delete inst_iter;
  edges_.reserve(4 * num_insts); // rough estimate
}

void 
TaskArranger::setInstanceId1(sta::Instance* inst, VertexId id)
{
  sta::ConcreteInstance* cinst = reinterpret_cast<sta::ConcreteInstance*>(inst);
  cinst->setId1(id);
}

void 
TaskArranger::makeEdges()
{
  sta::PinSet visited_drvrs(network_);
  sta::LeafInstanceIterator *inst_iter = network_->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    makeInstDrvrWireEdges(inst);
  }
  delete inst_iter;
  makeRootEdges();
  // make exclusive MEE edges to inst and its sibling fanouts.
  makeSiblingFanoutsEdges();
}

void 
TaskArranger::makeRootEdges()
{
  
}

void 
TaskArranger::makeInstDrvrWireEdges(sta::Instance* inst)
{
  sta::InstancePinIterator *pin_iter = network_->pinIterator(inst);
  while (pin_iter->hasNext()) {
    sta::Pin *pin = pin_iter->next();
    if (network_->isDriver(pin)) {
      sta::Vertex *drvr_vertex, *bidirect_vertex;
      graph_->pinVertices(pin, drvr_vertex, bidirect_vertex);
      if (bidirect_vertex != nullptr) {
        printf("Bidirect pin found, not supported yet.\n");
        return;
      }
      if (drvr_vertex != nullptr) {
        InstVertex* from_inst_vertex = vertex(inst);
        sta::VertexOutEdgeIterator edge_iter(drvr_vertex, graph_);
        sta::InstanceSet fanout_insts(network_);
        while (edge_iter.hasNext()) {
          sta::Edge* base_edge = edge_iter.next();
          sta::Vertex* base_to_vertex = graph_->vertex(base_edge->to());
          sta::Instance* to_inst = network_->instance(base_to_vertex->pin());
          fanout_insts.insert(to_inst);
        }
        // Create edges to all fanout instances.
        // First sort fanout instances with their levels.
        // This ensures the one with lower level comes first.
        // And the last one is the farthest from the driver.
        std::vector<InstVertex*> fanout_inst_vertices;
        for (auto *fanout_inst : fanout_insts) {
          fanout_inst_vertices.push_back(vertex(fanout_inst));
        }
        sort(fanout_inst_vertices.begin(),
             fanout_inst_vertices.end(),
             InstVertexLevelLess());
        for (auto *fanout_inst : fanout_inst_vertices) {
          makeEdge(from_inst_vertex, fanout_inst);
        }
        // Create edges among sibling instances.
        for (int i = 0; i < fanout_inst_vertices.size() - 1; i++) {
          makeEdge(fanout_inst_vertices[i], fanout_inst_vertices[i+1]);
        }
      }
    }
  }
  delete pin_iter;
}

void 
TaskArranger::makeSiblingFanoutsEdges()
{
  for (auto &inst_vertex : vertices_) {
    InstVertexOutEdgeIterator last_edge_iter(inst_vertex, this);
    InstVertex* last_inst_vertex = nullptr;
    // Get the last fanout instance during iteration.
    while (last_edge_iter.hasNext()) {
      EdgeId last_edge_id = last_edge_iter.next();
      last_inst_vertex = vertex(edges_[last_edge_id].to());
    }
    if (last_inst_vertex == nullptr)
      throw std::runtime_error("No fanout instances found.");
    // Now iterate again to create edges among this instance to
    // all sibling fanouts.
    InstVertexOutEdgeIterator edge_iter(inst_vertex, this);
    while (edge_iter.hasNext()) {
      EdgeId edge_id = edge_iter.next();
      InstVertex* to_inst_vertex = vertex(edges_[edge_id].to());
      InstVertexOutEdgeIterator to_edge_iter(to_inst_vertex, this);
      // Collect all sibling fanout instances.
      while (to_edge_iter.hasNext()) {
        EdgeId to_edge_id = to_edge_iter.next();
        InstVertex* sibling_fanout_vertex = vertex(edges_[to_edge_id].to());
        makeEdge(last_inst_vertex, sibling_fanout_vertex);
      }
    }
  }
}

EdgeId 
TaskArranger::makeEdge(InstVertex* from_vertex,
                        InstVertex* to_vertex)
{
  InstEdge &edge = edges_.emplace_back();
  edge.init(id(from_vertex), id(to_vertex));
  EdgeId edge_id = edges_.size() - 1;
  edge.setObjectIdx(edge_id);
  EdgeId out_edge_next = from_vertex->out_edges_;
  edge.vertex_out_next_ = out_edge_next;
  edge.vertex_out_prev_ = edge_id_null;
  from_vertex->out_edges_ = edge_id;
  if (out_edge_next < edges_.size()) {
    InstEdge &next_edge = edges_[out_edge_next];
    next_edge.vertex_out_prev_ = edge_id;
  }

  edge.vertex_in_link_ = to_vertex->in_edges_;
  to_vertex->in_edges_ = edge_id;
  to_vertex->temp_ref_num_++;
  return edge_id;
}


InstVertexOutEdgeIterator::InstVertexOutEdgeIterator(InstVertex* vertex,
                                               const TaskArranger* arranger)
  : next_(vertex->out_edges_),
    arranger_(arranger)
{ 
}

InstVertexOutEdgeIterator::InstVertexOutEdgeIterator(InstVertex &vertex,
                                               const TaskArranger* arranger)
  : next_(vertex.out_edges_),
    arranger_(arranger)
{ 
}

EdgeId
InstVertexOutEdgeIterator::next()
{
  EdgeId next = next_;
  if (next_ < arranger_->edges_.size()) {
    next_ = arranger_->edge(next_)->vertex_out_next_;
  } else {
    next_ = edge_id_null;
  }
  return next;
}

bool 
InstVertexOutEdgeIterator::hasNext() const
{
  return next_ < arranger_->edges_.size();
}

} // namespace lrf
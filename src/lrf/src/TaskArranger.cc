#include <atomic>
#include <thread>

#include "TaskArranger.hh"
#include "TopologyChecker.hh"
#include "search/Levelize.hh"
#include "sta/ObjectTable.hh"
#include "sta/Search.hh"
#include "sta/Network.hh"
#include "sta/ConcreteNetwork.hh"
#include "sta/Graph.hh"
#include "sta/Liberty.hh"
#include "sta/TimingRole.hh"
#include "sta/DispatchQueue.hh"
#include "ParallelVisitor.hh"
#include "db_sta/dbSta.hh"
#include "LocalSta.hh"
#include "rsz/Resizer.hh"


namespace lrf {

extern std::mutex g_odb_sta_access_mutex;

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
      if (level < vertex->level())
        level = vertex->level();
    }
  }
  delete pin_iter;
  level_ = level;
}

bool InstVertex::hasFanins() const
{
  return in_edges_ != edge_id_null;
}

void
InstEdge::init(VertexId from, VertexId to)
{
  from_ = from;
  to_ = to;
}


TaskArranger::TaskArranger(sta::StaState *sta)
  : sta::StaState(sta),
    pred_(new SearchMEEPred(sta))
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
  if (vertices_.empty()) {
    printf("TaskArranger::init making graph...\n");
    makeGraph();
    initVertexRefCounts(true);
    ensureGraphVertices();
  }
}

void 
TaskArranger::reinit()
{
  printf("TaskArranger::reinit checking graph consistency...\n");
  // This number is completely wrong, need to double check
  // if (network_->instanceCount() != vertices_.size() + 1) { // +1 for TOP instance
  //   init();
  // } else {
    initVertexRefCounts(false);
    ensureGraphVertices();
  // }
}

void
TaskArranger::ensureGraphVertices()
{
  // Pre-warm the graph by accessing all relevant vertices.
  // This forces OpenSTA to allocate vertices in the graph, preventing
  // reallocation/pointer invalidation during parallel execution.
  printf("TaskArranger::ensureGraphVertices pre-warming graph...\n");
  fflush(stdout);
  
  sta::Graph *graph = graph_;
  sta::Network *network = network_;
  sta::Vertex *vertex, *bidirect_vertex;

  for (InstVertex &inst_vertex : vertices_) {
    sta::Instance *inst = inst_vertex.inst();
    sta::InstancePinIterator *pin_iter = network->pinIterator(inst);
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      // Force creation of vertex for this pin
      graph->pinVertices(pin, vertex, bidirect_vertex);
      
      // Also pre-warm connected pins (fanin/fanout) as LocalSta will access them
      if (network->isDriver(pin)) {
        // Fanout
        if (vertex) {
          sta::VertexOutEdgeIterator edge_iter(vertex, graph);
          while (edge_iter.hasNext()) {
            sta::Edge *out_edge = edge_iter.next();
            sta::Vertex *load_vertex = out_edge->to(graph);
            // Accessing load_vertex ensures it exists
            (void)load_vertex; 
          }
        }
      } else if (network->isLoad(pin)) {
        // Fanin siblings logic involves visiting connected pins
        // We simulate what LocalSta::collectLocalFaninSiblings does
        sta::Net *net = network->net(pin);
        if (net) {
          sta::NetPinIterator *net_pin_iter = network->pinIterator(net);
          while (net_pin_iter->hasNext()) {
            const sta::Pin *net_pin = net_pin_iter->next();
            graph->pinVertices(net_pin, vertex, bidirect_vertex);
          }
          delete net_pin_iter;
        }
      }
    }
    delete pin_iter;
  }
  for (InstVertex &inst_vertex : vertices_) {
    if (inst_vertex.type() == VertexType::SEQUENTIAL)
    {
      if (vertex_ref_counts_[inst_vertex.objectIdx()] != 0) {
        throw std::runtime_error("Sequential vertex has non-zero ref count in ensureGraphVertices.");
      }
    }
  }
  printf("TaskArranger::ensureGraphVertices done.\n");
  fflush(stdout);
}

VertexId
TaskArranger::id(const InstVertex *vertex) const
{
  return vertex->objectIdx();
}

VertexId
TaskArranger::instToVertexId(const sta::Instance* inst) const
{
  auto it = inst_to_vid_.find(inst);
  if (it == inst_to_vid_.end()) {
    throw std::runtime_error("Instance not registered in inst_to_vid_ map");
  }
  return it->second;
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
    throw std::runtime_error("Edge ID out of bounds");
  return &(edges_[id]);
}

InstEdge*
TaskArranger::edge(EdgeId id)
{
  if (id >= edges_.size())
    throw std::runtime_error("Edge ID out of bounds");
  return &(edges_[id]);
}

void
TaskArranger::makeGraph()
{
  // We first make vertices for all instances One vertex
  // per instance. We first count the number of instances
  makeVertices();
  makeEdges();
  checkGraph();
  reduceEdgeFromRoots();
}

void 
TaskArranger::checkGraph() const
{
  printf("Checking all vertices in the graph...\n");
  fflush(stdout);
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    const InstVertex &inst_vertex = vertices_[vid];
    if (inst_vertex.type() == VertexType::NONE) {
      printf("ERROR: Vertex %zu has type NONE in checkGraph.\n", vid);
      fflush(stdout);
      continue;
    }
    if (inst_vertex.objectIdx() == object_idx_null) {
      throw std::runtime_error("Vertex object idx is null in checkGraph.");
    }
    if (instToVertexId(inst_vertex.inst()) != vid) {
      throw std::runtime_error("Instance to VertexId mapping incorrect in checkGraph.");
    }
  }
  printf("All vertices checked successfully in checkGraph.\n");
  fflush(stdout);

  printf("Checking all edges in the graph...\n");
  fflush(stdout);
  for (size_t eid = 0; eid < edges_.size(); eid++) {
    const InstEdge &inst_edge = edges_[eid];
    if (inst_edge.objectIdx() == object_idx_null) {
      throw std::runtime_error("Edge object idx is null in checkGraph.");
    }
    if (vertex(inst_edge.to()) == nullptr) {
      continue;
    } else {
      const InstVertex* to_vertex = vertex(inst_edge.to());
      sta::LibertyCell* to_cell = network_->libertyCell(to_vertex->inst());
      if (to_cell == nullptr) {
        throw std::runtime_error("Edge to_vertex has null LibertyCell in checkGraph.");
      }
      if (to_cell->hasSequentials()) {
        throw std::runtime_error("Edge points to sequential vertex in checkGraph.");
      }
    }
  }
  printf("All edges checked successfully in checkGraph.\n");
  fflush(stdout);

  printf("Checking ref counts of vertices in the graph...\n");
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    const InstVertex &inst_vertex = vertices_[vid];
    if (inst_vertex.tempRefNum() == 0) {
      InstVertexOutEdgeIterator out_edge_iter(&inst_vertex, this);
      if (out_edge_iter.hasNext()) {
        const InstVertex* to_inst_vertex = vertex(edge(out_edge_iter.next())->to());
        if (to_inst_vertex->tempRefNum() == 0) {
          throw std::runtime_error("Vertex with zero tempRefNum has outgoing edge to vertex with zero tempRefNum in checkGraph.");
        }
      }
    }
  }
}

void getInstanceNum(sta::StaState* sta, int& com_count, int& root_count)
{
  com_count = 0;
  root_count = 0;
  sta::LeafInstanceIterator *inst_iter = sta->network()->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    if (!inst) {
      continue;
    }
    sta::LibertyCell *cell = sta->network()->libertyCell(inst);
    if (cell) {
      if (cell->hasSequentials()) {
        root_count++;
      } else {
        com_count++;
      }
    }
  }
  delete inst_iter;
}

void 
TaskArranger::makeVertices()
{
  int num_com_insts = 0;
  int num_root_insts = 0;
  getInstanceNum(this, num_com_insts, num_root_insts);
  vertices_.resize(num_com_insts + num_root_insts + 1);
  // Pre-reserve mapping capacity to keep unordered_map lookups O(1) without rehash.
  inst_to_vid_.reserve(vertices_.size());
  num_com_ = num_com_insts;
  printf("Making vertices: %d combinational, %d sequential, %zu numcom\n", 
         num_com_insts, num_root_insts, num_com_);
  fflush(stdout);
  sta::LeafInstanceIterator *inst_iter = network_->leafInstanceIterator();
  num_com_insts = 0;
  num_root_insts = 0;
  while (inst_iter->hasNext()) {
    sta::Instance *inst = inst_iter->next();
    sta::LibertyCell *cell = network_->libertyCell(inst);
    if (cell) {
      if (cell->hasSequentials()) {
        VertexId vid = num_com_ + num_root_insts;
        InstVertex &vertex = vertices_[vid];
        vertex.init(inst, this);
        vertex.setObjectIdx(vid);  
        setInstanceId1(inst, vid);
        vertex.setType(VertexType::SEQUENTIAL);
        // const char *inst_name = network_->name(inst);
        // printf("  Initialized sequential vertex %d: inst=%p, name=%s\n", 
        //        vid, (void*)inst, inst_name);
        // fflush(stdout);
        
        num_root_insts++;
      } else {
        VertexId vid = num_com_insts;
        InstVertex &vertex = vertices_[vid];
        vertex.init(inst, this);
        vertex.setObjectIdx(vid);  
        setInstanceId1(inst, vid);
        vertex.setType(VertexType::COMBINATIONAL);
        // const char* inst_name = network_->name(inst);
        // printf("  Initialized combinational vertex %d: inst=%p, name=%s\n", 
        //        vid, (void*)inst, inst_name);
        // fflush(stdout);
        
        num_com_insts++;
      }
    }
  }
  delete inst_iter;
  printf("Double check size: num_root_insts=%d, num_com_insts=%d\n", 
         num_root_insts, num_com_insts);
  fflush(stdout);

  // Create vertex for top instance
  {
    sta::Instance* top_inst = network_->topInstance();
    // Put TOP in the extra slot reserved by the +1 resize above.
    // The leaf loop fills [0, num_com_ + num_root_insts - 1].
    VertexId vid = static_cast<VertexId>(vertices_.size() - 1);
    InstVertex &vertex = vertices_[vid];
    vertex.init(top_inst, this);
    vertex.setObjectIdx(vid);  
    setInstanceId1(top_inst, vid);
    vertex.setType(VertexType::TOP);
    const char *inst_name = network_->name(top_inst);
    printf("  Initialized top instance vertex %d: inst=%p, name=%s\n", 
           vid, (void*)top_inst, inst_name);
    fflush(stdout);
  }
  
  edges_.reserve(4 * vertices_.size()); // rough estimate
}

void 
TaskArranger::setInstanceId1(sta::Instance* inst, VertexId id)
{
  // Do NOT mutate the STA instance internals. Maintain our own mapping.
  inst_to_vid_[inst] = id;
}

void 
TaskArranger::makeEdges()
{
  // ✅ 先验证所有 vertices 的 inst_ 指针
  printf("Verifying all vertices before makeEdges...\n");
  fflush(stdout);
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    InstVertex &inst_vertex = vertices_[vid];
    if (!inst_vertex.inst()) {
      throw std::runtime_error("Vertex has null inst_ before makeEdges.");
    }
  }
  printf("Verification complete. Starting makeEdges...\n");
  fflush(stdout);
  
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    InstVertex &inst_vertex = vertices_[vid];
    if (inst_vertex.inst())
      makeInstDrvrWireMEE(inst_vertex.inst());
  }
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    InstVertex& inst_vertex = vertices_[vid];
    if (inst_vertex.inst())
      makeSiblingFanoutsMEE(&inst_vertex);
  }
}

void 
TaskArranger::initVertexRefCounts(bool reset)
{
  // Initialize all reference counts
  if (reset || vertex_ref_counts_ == nullptr)
    vertex_ref_counts_ = std::make_unique<std::atomic<size_t>[]>(vertices_.size());

  // Set all to its vertex's temp_ref_num_
  // Initialize all vertices to match the size of allocated array.
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    vertex_ref_counts_[vid].store(vertices_[vid].temp_ref_num_);
  }
}

sta::InstanceSet
TaskArranger::FanoutsInstances(const sta::Pin *drvr_pin)
{
  sta::Vertex *drvr_vertex, *bidirect_vertex;
  graph_->pinVertices(drvr_pin, drvr_vertex, bidirect_vertex);
  sta::InstanceSet fanout_insts(network_);
  if (bidirect_vertex == nullptr) {
    if (drvr_vertex != nullptr) {
      sta::VertexOutEdgeIterator edge_iter(drvr_vertex, graph_);
      while (edge_iter.hasNext()) {
        sta::Edge* base_edge = edge_iter.next();
        // Filter out sequential outputs.
        if (!pred_->searchThru(base_edge))
          continue;
        sta::Vertex* base_to_vertex = graph_->vertex(base_edge->to());
        if (!network_->isLoad(base_to_vertex->pin()))
          continue;
        sta::Instance* to_inst = network_->instance(base_to_vertex->pin());
        // Make sure not to add port.
        if (to_inst)
          fanout_insts.insert(to_inst);
      }
    }
  } else {
    // Bidirect pin case, not supported yet.
    printf("Bidirect pin found, not supported yet.\n");
    fflush(stdout);
  }
  
  return fanout_insts;
}

void 
TaskArranger::makeInstDrvrWireMEE(sta::Instance* inst)
{
  if (inst == nullptr) {
    printf("Instance is nullptr in makeInstDrvrWireMEE.\n");
    fflush(stdout);
    return;
  }
  InstVertex* from_inst_vertex = vertex(inst);
  sta::InstancePinIterator *pin_iter = network_->pinIterator(inst);
  while (pin_iter->hasNext()) {
    const sta::Pin *pin = pin_iter->next();
    if (network_->isDriver(pin)) {
      // Only pick combinational fanout instances.
      sta::InstanceSet fanout_insts = FanoutsInstances(pin);
      if (fanout_insts.size() == 0)
        continue;
      // Create edges to all fanout instances.
      // First sort fanout instances with their levels.
      // This ensures the one with lower level comes first.
      // And the last one is the farthest from the driver.
      // std::vector<InstVertex*> fanout_inst_vertices;
      // for (auto *fanout_inst : fanout_insts) {
      //   InstVertex* inst_vertex = vertex(fanout_inst);
      //   if (inst_vertex == nullptr) {
      //     throw std::runtime_error("Fanout instance vertex is nullptr.");
      //   }
      //   fanout_inst_vertices.push_back(inst_vertex);
      // }
      // sort(fanout_inst_vertices.begin(),
      //       fanout_inst_vertices.end(),
      //       InstVertexLevelLess());
      for (auto *fanout_inst : fanout_insts) {
        if (vertex(fanout_inst))
          makeEdge(from_inst_vertex, vertex(fanout_inst));
        // printf("  Created edge of level %u from instance %s to fanout instance %s\n",
        //        fanout_inst->level(),
        //        network_->name(inst),
        //        network_->name(fanout_inst->inst()));
        // fflush(stdout);
      }
    }
  }
  delete pin_iter;
}

void 
TaskArranger::makeSiblingFanoutsMEE(InstVertex* inst_vertex)
{
  // debug print
  if (!inst_vertex) {
    printf("ERROR: inst_vertex is nullptr in makeSiblingFanoutsMEE\n");
    fflush(stdout);
    return;
  }
  
  sta::Instance* inst = inst_vertex->inst();
  if (!inst) {
    printf("ERROR: inst_vertex->inst() is nullptr in makeSiblingFanoutsMEE\n");
    fflush(stdout);
    return;
  }
  // First, get all fanout instances of this instance.
  // Finally, connect the last one to all others' fanout instances.
  // Then, sort them by level. And create MEE among them.
  std::vector<InstVertex*> fanout_inst_vertices;
  InstVertexOutEdgeIterator edge_iter_init(inst_vertex, this);
  while (edge_iter_init.hasNext()) {
    InstEdge *inst_edge = edge(edge_iter_init.next());
    InstVertex* to_inst_vertex = vertex(inst_edge->to());
    fanout_inst_vertices.push_back(to_inst_vertex);
  }
  if (fanout_inst_vertices.empty()) {
    // printf("  No fanout instances found for instance %s, skipping sibling MEE creation.\n",
    //        network_->name(inst));
    //        fflush(stdout);
    return;
  }
  sort(fanout_inst_vertices.begin(),
        fanout_inst_vertices.end(),
        InstVertexLevelLess());

  InstVertex* last_inst_vertex = fanout_inst_vertices[fanout_inst_vertices.size() - 1];
  
  std::set<VertexId> sib_fanout_ids_set;
  for (size_t i = 0; i < fanout_inst_vertices.size() - 1; i++) {
    InstVertexOutEdgeIterator edge_iter_temp(fanout_inst_vertices[i], this);
    while (edge_iter_temp.hasNext()) {
      EdgeId temp_edge_id = edge_iter_temp.next();
      InstEdge* temp_edge = edge(temp_edge_id);
      sib_fanout_ids_set.insert(temp_edge->to());
    }
  }
  for (VertexId fid : sib_fanout_ids_set) {
    InstVertex* sibling_fanout_vertex = vertex(fid);
    // In makeEdge, it will check for self-loop and duplicates.
    makeEdge(last_inst_vertex, sibling_fanout_vertex);
  }
  for (size_t i = 0; i < fanout_inst_vertices.size() - 1; i++) {
    makeEdge(fanout_inst_vertices[i], fanout_inst_vertices[i+1]);
  }
}

EdgeId 
TaskArranger::makeEdge(InstVertex* from_vertex,
                        InstVertex* to_vertex)
{
  if (from_vertex == nullptr || to_vertex == nullptr) {
    throw std::runtime_error("from_vertex or to_vertex is nullptr in makeEdge.");
  }
  if (from_vertex == to_vertex) {
    return edge_id_null; // Disallow self-loop.
  }
  if (from_vertex->inst() == to_vertex->inst()) {
    throw std::runtime_error("from_vertex and to_vertex have the same instance in makeEdge.");
  }
  InstVertexLevelLess level_less;
  if (!level_less(from_vertex, to_vertex)) 
    return edge_id_null;
  if (to_vertex->type() == VertexType::SEQUENTIAL
      || to_vertex->type() == VertexType::TOP) {
    // Disallow incoming edges to SEQ/TOP: do not create * -> (SEQ/TOP).
    const char* from_name = from_vertex->inst() ? network_->name(from_vertex->inst()) : "<null>";
    const char* to_name = to_vertex->inst() ? network_->name(to_vertex->inst()) : "<null>";
    printf("Disallow edge into %s (SEQ/TOP): from=%s -> to=%s\n", to_name, from_name, to_name);
    fflush(stdout);
    return edge_id_null;
  }
  // Check for duplicate edge and skip if exists.
  {
    EdgeId e = from_vertex->out_edges_;
    while (e < edges_.size()) {
      const InstEdge* existing = edge(e);
      if (existing->to() == id(to_vertex)) {
        return e; // Edge already exists.
      }
      e = existing->vertex_out_next_;
    }
  }
  edges_.emplace_back();
  InstEdge &edge = edges_.back();
  edge.init(id(from_vertex), id(to_vertex));
  EdgeId edge_id = edges_.size() - 1;
  edge.setObjectIdx(edge_id);
  EdgeId out_edge_next = from_vertex->out_edges_;
  edge.vertex_out_next_ = out_edge_next;
  edge.vertex_out_prev_ = edge_id_null;
  from_vertex->out_edges_ = edge_id;
  if (out_edge_next < edges_.size()) {
    InstEdge *next_edge = this->edge(out_edge_next);
    next_edge->vertex_out_prev_ = edge_id;
  }

  edge.vertex_in_link_ = to_vertex->in_edges_;
  to_vertex->in_edges_ = edge_id;
  to_vertex->temp_ref_num_++;
  return edge_id;
}

void
TaskArranger::printFailed() const
{
  InstVertexSet inst_vertex_set;
  for (const InstVertex *inst_vertex : visited_inst_vertices_) {
    inst_vertex_set.insert(const_cast<InstVertex*>(inst_vertex));
  }
  int com_without_zero_fanout = 0;
  // If the vertex is visited, its fanout inst should be visited
  // if its ref count is not zero, report it.
  int com_visited_but_has_nonezero_fanout = 0;
  bool zero_ref_not_visited = false;
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    if (vertices_[vid].type() != VertexType::COMBINATIONAL)
      continue;

    if (vertex_ref_counts_[vid].load() == 0) {
      com_without_zero_fanout++;
      if (inst_vertex_set.find(const_cast<InstVertex*>(&vertices_[vid])) == inst_vertex_set.end()) {
        printf("ERROR: Com vertex %s with zero ref is not visited\n",
               network_->name(vertices_[vid].inst()));
        fflush(stdout);
        zero_ref_not_visited = true;
      }
      InstVertexOutEdgeIterator edge_iter(
          (vertices_[vid]), this);
      while (edge_iter.hasNext()) {
        const InstVertex* out_inst_vertex = vertex((edge(edge_iter.next()))->to());
        VertexId to_vid = id(out_inst_vertex);
        if (vertex_ref_counts_[to_vid].load() != 0) {
          com_visited_but_has_nonezero_fanout++;
          printf("WARNING: Com vertex %s has non-zero ref fanout: %s \n",
                 network_->name(vertices_[vid].inst()),
                 network_->name(out_inst_vertex->inst()));
          fflush(stdout);
        }
      }
    }
  }
  printf("Total combinational vertices without zero ref fanout: %d\n",
         com_without_zero_fanout);
  printf("Total combinational vertices visited but has non-zero ref fanout: %d\n",
         com_visited_but_has_nonezero_fanout);
  if (zero_ref_not_visited)
    printf("Some zero-ref combinational vertices were not visited!\n");
  fflush(stdout);
}

void
TaskArranger::printGraph() const
{
  for (size_t vid = 0; vid < vertices_.size(); vid++) {
    if (vertices_[vid].type() != VertexType::COMBINATIONAL)
      continue;
    const InstVertex &inst_vertex = vertices_[vid];
    const sta::Instance* inst = inst_vertex.inst();
    printf("Vertex %zu: Instance %s, Level %u, TempRefNum %zu, atomicRefCount %zu, type %s\n",
           vid,
           inst ? network_->name(inst) : "nullptr",
           inst_vertex.level(),
           inst_vertex.tempRefNum(),
           vertex_ref_counts_[vid].load(),
           inst_vertex.type() == VertexType::COMBINATIONAL ? "COMBINATIONAL" :
           inst_vertex.type() == VertexType::SEQUENTIAL ? "SEQUENTIAL" :
           inst_vertex.type() == VertexType::TOP ? "TOP" : "NONE");
           fflush(stdout);
    InstVertexOutEdgeIterator edge_iter(
        const_cast<InstVertex*>(&inst_vertex), this);
    while (edge_iter.hasNext()) {
      EdgeId edge_id = edge_iter.next();
      const InstEdge *edge_ptr = edge(edge_id);
      const InstVertex* to_vertex = vertex(edge_ptr->to());
      const sta::Instance* to_inst = to_vertex->inst();
      printf("  Edge to : Instance %s\n",
             to_inst ? network_->name(to_inst) : "nullptr");
             fflush(stdout);
    }
  }
}

void
TaskArranger::reduceEdgeFromRoots()
{
  for (InstVertex& inst_vertex : vertices_) {
    if (inst_vertex.type() == VertexType::SEQUENTIAL
  || inst_vertex.type() == VertexType::TOP) {
      if (inst_vertex.hasFanins()) {
        if (inst_vertex.type() == VertexType::SEQUENTIAL) {
          printf("Sequential vertex of %s has fanins in reduceEdgeFromReg, skipping.\n", network_->name(inst_vertex.inst()));
          fflush(stdout);
        } else {
          printf("Top vertex of %s has fanins in reduceEdgeFromReg, skipping.\n", network_->name(inst_vertex.inst()));
          fflush(stdout);
        }
        throw std::runtime_error("Sequential or Top vertex has fanins in reduceEdgeFromReg.");
      }
      InstVertexOutEdgeIterator edge_iter(&inst_vertex, this);
      while (edge_iter.hasNext()) {
        EdgeId edge_id = edge_iter.next();
        InstEdge* inst_edge = edge(edge_id);
        // Decrease ref count of fanout vertices
        InstVertex* to_vertex = vertex(inst_edge->to());
        to_vertex->temp_ref_num_--;
      
      }
    }
  }
}

void 
TaskArranger::getZeroRefComInstVertices(std::vector<InstVertex*>& zero_ref_vertices)
{
  for (InstVertex& inst_vertex : vertices_) {
    if (inst_vertex.type() == VertexType::COMBINATIONAL) {
      VertexId vid = id(&inst_vertex);
      if (vertex_ref_counts_[vid].load() == 0) {
        zero_ref_vertices.push_back(&inst_vertex);
      }
    }
  }
}

std::set<VertexId>
TaskArranger::decreOutRefCount(InstVertex *inst_vertex)
{
  if (!inst_vertex) {
    throw std::runtime_error("inst_vertex is nullptr in decreRefCount.");
  }
  return decreOutRefCount(*inst_vertex);
}

std::set<VertexId>
TaskArranger::decreOutRefCount(InstVertex &inst_vertex)
{
  std::set<VertexId> zero_ref_set;
  InstVertexOutEdgeIterator edge_iter(&inst_vertex, this);
  while (edge_iter.hasNext()) {
    EdgeId edge_id = edge_iter.next();
    InstEdge* inst_edge = edge(edge_id);
    if (decreRefCount(inst_edge->to()) == 0) {
       zero_ref_set.insert(inst_edge->to());
    }
  }
  return zero_ref_set;
}

size_t
TaskArranger::decreRefCount(VertexId vid)
{
  if (vid >= num_com_) {
    throw std::runtime_error("Attempting to decreRefCount on non-combinational vertex.");
  }
  size_t old = vertex_ref_counts_[vid].fetch_sub(1);
  if (old == 0) {
    throw std::runtime_error("Reference count underflow in decreRefCount.");
  }
  return old - 1;
}

void
TaskArranger::finishTasks()
{
  if (dispatch_queue_) {
    dispatch_queue_->finishTasks();
  }
}

void 
TaskArranger::visitParallel(sta::dbSta *sta, LocalSta *local_sta, rsz::Resizer *resizer, 
                            ParallelLrVisitor *visitor) 
{
  if (incremental_)
    reinit();
  // Clear previous visit records
  clearVisitedInstVertices();
  resizer_ = resizer;
  
  // Initialize topology checker if enabled
  if (enable_topology_check_) {
    printf("Topology check enabled.\n");
    topology_checker_ = std::make_unique<TopologyChecker>(this);
  }
  
  // Clean up old visitors if any
  for (auto v : visitors_) delete v;
  visitors_.clear();

  std::vector<InstVertex*> zero_ref_vertices;
  getZeroRefComInstVertices(zero_ref_vertices);
  
  visitors_.reserve(thread_count_);
  visitors_.push_back(visitor);
  printf("Visit with %u threads\n", thread_count_);
  fflush(stdout);
  for (size_t i = 1; i < thread_count_; i++) {
    visitors_.emplace_back(visitor->copy());
  }
  for (size_t i = 0; i < zero_ref_vertices.size(); i++) {
    createTask(zero_ref_vertices[i]);
  }
  finishTasks();
  
  int cnt = 0;
  for (auto v : visitors_) {
    // printf("Visitor %d runtime profile:\n", cnt);
    v->printRuntimeProfile();
    // v->printVisitedInstNames();
    delete v;
    cnt++;
  }
  visitors_.clear();
  
  // Print topology violations if any
  if (enable_topology_check_ && topology_checker_) {
    topology_checker_->printViolations();
    topology_checker_.reset();
  }

  // Next time we visit, reuse the graph.
  incremental_ = true;
}

void
TaskArranger::visitParallelPrecheck(sta::dbSta *sta, LocalSta *local_sta,
                                    rsz::Resizer *resizer,
                                    ParallelLrVisitor *visitor,
                                    std::vector<ResizeBenefit> &results)
{
  resizer_ = resizer;

  // Clean up old visitors if any
  for (auto v : visitors_) delete v;
  visitors_.clear();

  printf("Precheck: %zu combinational instances out of %zu total, %u threads\n",
         num_com_, vertices_.size(), thread_count_);
  fflush(stdout);

  // Pre-allocate results with 1:1 mapping to vertices_.
  // Non-combinational slots are left with cost_change=0.
  const size_t total = vertices_.size();
  results.resize(total);
  for (size_t i = 0; i < total; i++)
    results[i] = {vertices_[i].inst_, -std::numeric_limits<float>::infinity(), i};

  // Create visitor copies for each thread
  visitors_.reserve(thread_count_);
  visitors_.push_back(visitor);
  for (size_t i = 1; i < thread_count_; i++) {
    visitors_.emplace_back(visitor->copy());
  }

  // Dispatch only combinational vertices (no conflict graph)
  for (size_t i = 0; i < total; i++) {
    if (vertices_[i].type() != VertexType::COMBINATIONAL)
      continue;
    InstVertex *iv = &vertices_[i];
    if (!dispatch_queue_) {
      float cost_change = visitors_[0]->trySwapPrecheck(iv->inst());
      results[i] = {iv->inst(), cost_change, i};
    } else {
      dispatch_queue_->dispatch([this, iv, &results, i](int thread_id) {
        float cost_change = visitors_[thread_id]->trySwapPrecheck(iv->inst());
        results[i] = {iv->inst(), cost_change, i};
      });
    }
  }
  finishTasks();

  // Cleanup visitors
  for (auto v : visitors_) {
    v->printRuntimeProfile();
    delete v;
  }
  visitors_.clear();
}

void
TaskArranger::markSelectedInstances(const std::vector<size_t> &vertex_ids)
{
  // Reset all vertices to unselected
  for (auto &v : vertices_)
    v.selected_ = false;
  // Mark only the top instances from precheck as selected
  for (size_t idx : vertex_ids)
    vertices_[idx].selected_ = true;
}

void
TaskArranger::createTask(InstVertex* inst_vertex)
{
  // Record the instance name being visited
  if (verbose_)
  {
    std::lock_guard<std::mutex> lock(visited_inst_names_mutex_);
    if (visited_inst_vertices_.size() > max_resize_num_) {
      return;
    }
    visited_inst_vertices_.push_back(inst_vertex);
  }
  
  if (!dispatch_queue_) {
    if (thread_count_ == 1) {
      // Single-threaded execution
      runTask(visitors_[0], inst_vertex);
      return;
    } else {
      throw std::runtime_error("Dispatch queue is null in multi-threaded mode.");
    }
  }
  dispatch_queue_->dispatch([inst_vertex, this](int id) {
    runTask(visitors_[id], inst_vertex);
  });
}

void
TaskArranger::runTask(ParallelLrVisitor *visitor, InstVertex* inst_vertex)
{
  // Only visit selected instances; unselected ones just cascade dependencies
  if (inst_vertex->selected_) {
    // Topology validation: check if this vertex is ready to visit
    if (enable_topology_check_ && topology_checker_) {
      topology_checker_->onVisit(inst_vertex, std::this_thread::get_id());
    }

    if (visitor->visit(inst_vertex->inst()))
    {
      // Topology validation: mark before modification
      if (enable_topology_check_ && topology_checker_) {
        topology_checker_->onBeforeModify(inst_vertex, std::this_thread::get_id());
      }

      // Use the global mutex to protect DB/STA modification
      // ensuring exclusive access against other readers and writers.
      visitor->applyChangesToDb(resizer_);

      // Topology validation: mark after modification
      if (enable_topology_check_ && topology_checker_) {
        topology_checker_->onAfterModify(inst_vertex, std::this_thread::get_id());
      }
    }
  }
  // Always cascade dependencies regardless of selected_
  std::set<VertexId> zero_ref_vertices = decreOutRefCount(inst_vertex);
  for (VertexId zero_ref_id : zero_ref_vertices) {
    InstVertex* zero_ref_vertex = vertex(zero_ref_id);
    createTask(zero_ref_vertex);
  }
}

InstVertexOutEdgeIterator::InstVertexOutEdgeIterator(const InstVertex* vertex,
                                               const TaskArranger* arranger)
  : next_(vertex->out_edges_),
    arranger_(arranger)
{ 
}

InstVertexOutEdgeIterator::InstVertexOutEdgeIterator(const InstVertex &vertex,
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

InstVertexInEdgeIterator::InstVertexInEdgeIterator(const InstVertex* vertex,
                                                   const TaskArranger* arranger)
  : next_(vertex->in_edges_),
    arranger_(arranger)
{
}

InstVertexInEdgeIterator::InstVertexInEdgeIterator(const InstVertex &vertex,
                                                   const TaskArranger* arranger)
  : next_(vertex.in_edges_),
    arranger_(arranger)
{
}

EdgeId
InstVertexInEdgeIterator::next()
{
  EdgeId next = next_;
  if (next_ < arranger_->edges_.size()) {
    next_ = arranger_->edge(next_)->vertex_in_link_;
  } else {
    next_ = edge_id_null;
  }
  return next;
}

bool
InstVertexInEdgeIterator::hasNext() const
{
  return next_ < arranger_->edges_.size();
}

SearchMEEPred::SearchMEEPred(sta::StaState* sta)
  : sta::SearchPred2(sta)
{
}

bool
SearchMEEPred::searchThru(sta::Edge* edge)
{
  const sta::TimingRole *role = edge->role();
  sta::Network* network_ = sta_->network();
  sta::Graph* graph_ = sta_->graph();
  sta::Instance *to_inst = network_->instance(graph_->vertex(edge->to())->pin());
  if (to_inst == sta_->network()->topInstance()) {
    return false;
  }
  sta::LibertyCell* to_cell = network_->libertyCell(to_inst);
  if (!to_cell) return false;
  return (SearchPred2::searchThru(edge)
          && role->isWire()
          && !to_cell->hasSequentials());
}

bool 
SearchMEEPred::searchFrom(const sta::Vertex* from_vertex)
{
  sta::Instance *from_inst = sta_->network()->instance(from_vertex->pin());
  if (from_inst == sta_->network()->topInstance()) {
    return false;
  }
  sta::LibertyCell* from_cell = sta_->network()->libertyCell(from_inst);
  if (!from_cell) return false;
  return (!from_cell->hasSequentials()
          && SearchPred2::searchFrom(from_vertex));
}

bool
SearchMEEPred::searchTo(const sta::Vertex* to_vertex)
{
  sta::Instance *to_inst = sta_->network()->instance(to_vertex->pin());
  if (to_inst == sta_->network()->topInstance()) {
    return false;
  }
  sta::LibertyCell* to_cell = sta_->network()->libertyCell(to_inst);
  if (!to_cell) return false;
  return (!to_cell->hasSequentials()
          && SearchPred2::searchTo(to_vertex));
}

void
TaskArranger::printVisitedInstNames() const
{
  printf("=== Visited Instance Names (in order) ===\n");
  printf("Total visited: %zu instances\n", visited_inst_vertices_.size());
  for (size_t i = 0; i < visited_inst_vertices_.size(); i++) {
    printf("[%zu] %s\n", i, network_->name(visited_inst_vertices_[i]->inst()));
  }
  printf("=========================================\n");
  fflush(stdout);
}

void
TaskArranger::printTopologyViolations() const
{
  if (topology_checker_) {
    topology_checker_->printViolations();
  } else {
    printf("Topology checker not initialized. Call enableTopologyCheck(true) before visitParallel().\n");
  }
}

} // namespace lrf

#pragma once

#include <vector>
#include <atomic>
#include <memory>
#include <unordered_map>
#include <mutex>

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/Delay.hh"
#include "sta/StaState.hh"
#include "sta/NetworkClass.hh"
#include "GraphBase.hh"
#include "sta/SearchPred.hh"
#include "lrf/LrfClass.hh"

namespace rsz {
  class Resizer;
}

namespace sta {
  class Graph;
  class Instance;
  class SearchPred;
  class SearchPredNonReg2;
  class dbSta;
  class PwrActivity;
}

namespace lrf {
using sta::EdgeId;
using sta::VertexId;
using sta::Level;
using sta::ObjectIdx;
using rsz::Resizer;

class ParallelLrVisitor;
class LocalSta;
class InstVertex;
class InstEdge;
class TopologyChecker;

enum class VertexType {
  COMBINATIONAL,
  SEQUENTIAL,
  NONE,
  TOP
};

class InstVertexLevelLess
{
 public:
  InstVertexLevelLess();
  bool operator()(const InstVertex* vertex1, const InstVertex* vertex2) const;

 protected:
};
using InstVertexSet = std::set<InstVertex*, InstVertexLevelLess>;

enum class EdgeType {
  MEE,
  NORMAL
};

struct InstVertex {
  void init(sta::Instance* instance, sta::StaState* sta);
  void setObjectIdx(ObjectIdx idx) { object_idx_ = idx; }
  ObjectIdx objectIdx() const { return object_idx_; }
  const sta::Instance* inst() const { return inst_; }
  sta::Instance* inst() { return inst_; }
  Level level() const { return level_; }
  void setLevel(Level level) { level_ = level; }
  size_t tempRefNum() const { return temp_ref_num_; }
  void setType(VertexType type) { type_ = type; }
  VertexType type() const { return type_; }
  bool hasFanins() const;

  sta::Instance* inst_ = nullptr;
  sta::Level level_ = 0;
  size_t temp_ref_num_ = 0;
  
  EdgeId in_edges_ = edge_id_null;
  EdgeId out_edges_ = edge_id_null;
  ObjectIdx object_idx_ = object_idx_null;
  VertexType type_ = VertexType::NONE;
  bool selected_ = true;  // default true: all instances participate in resize
  std::vector<sta::PwrActivity> activities_;
};

struct InstEdge {
  void init(VertexId from_vertex,
            VertexId to_vertex);
  void setObjectIdx(ObjectIdx idx) { object_idx_ = idx; }
  ObjectIdx objectIdx() const { return object_idx_; }      
  VertexId from() const { return from_; }
  VertexId to() const { return to_; }

  VertexId from_ = vertex_idx_null;
  VertexId to_ = vertex_idx_null;
  EdgeId vertex_in_link_ = edge_id_null;
  EdgeId vertex_out_prev_ = edge_id_null;
  EdgeId vertex_out_next_ = edge_id_null;
  ObjectIdx object_idx_ = object_idx_null;
};

typedef sta::ObjectTable<InstVertex> InstVertexTable;
typedef sta::ObjectTable<InstEdge> InstEdgeTable;


class TaskArranger: public sta::StaState
{
public:
  TaskArranger(sta::StaState *sta);
  ~TaskArranger();
  void init();
  void reinit();
  // Rebuild the entire graph from scratch (clears old vertices/edges).
  void rebuild();
  // Mark graph as dirty (e.g. after buffer insertion changes the netlist).
  void markDirty() { dirty_ = true; }

  // Functions of making graph
  void makeGraph();
  void makeVertices();
  void makeEdges();
  void checkGraph() const;
  void ensureGraphVertices();

  // Record activities for all combinational instances.
  void recordComInstActivities();

  // Functions of parallelization
  void reduceEdgeFromRoots();
  void visitParallel(sta::dbSta *sta, LocalSta *local_sta, rsz::Resizer *resizer,
                    ParallelLrVisitor *visitor);
  // Precheck: dispatch all combinational vertices in parallel (no dependency graph).
  // Collects resize benefit for each instance into results.
  void visitParallelPrecheck(sta::dbSta *sta, LocalSta *local_sta,
                             rsz::Resizer *resizer,
                             ParallelLrVisitor *visitor,
                             std::vector<ResizeBenefit> &results);
  std::set<VertexId> decreOutRefCount(InstVertex *inst_vertex);
  std::set<VertexId> decreOutRefCount(InstVertex &inst_vertex);
  size_t decreRefCount(VertexId vid);
  void getZeroRefComInstVertices(std::vector<InstVertex*>& zero_ref_vertices);
  void createTask(InstVertex* inst_vertex);
  void runTask(ParallelLrVisitor *visitor, InstVertex* inst_vertex);
  void finishTasks();

  // Assign MEE edges among sibling fanout instances of inst.
  // Assign MEE edges to all fanout instances of inst.
  void makeInstDrvrWireMEE(sta::Instance *inst);
  // Assign MEE edges among sibling fanout instances of inst.
  void makeSiblingFanoutsMEE(InstVertex* inst_vertex);
  // Make fanouts MEE
  sta::InstanceSet FanoutsInstances(const sta::Pin *pin);
  void makeFanoutsMEEWithLevelSort(sta::Instance* inst);
  // Initialize atomic reference counts for all vertices.
  void initVertexRefCounts(bool reset);

  EdgeId makeEdge(InstVertex *from_vertex,
                InstVertex *to_vertex);
  VertexId id(const InstVertex *vertex) const;
  VertexId instToVertexId(const sta::Instance* inst) const;
  const InstVertex *vertex(VertexId id) const;
  InstVertex *vertex(VertexId id);
  const InstVertex* vertex(const sta::Instance* inst) const;
  InstVertex* vertex(const sta::Instance* inst);
  const InstEdge* edge(EdgeId id) const;
  InstEdge* edge(EdgeId id);

  void setInstanceId1(sta::Instance* inst, VertexId id);
  // Control whether to write id directly into ConcreteInstance (risky) or use mapping (safe).
  void setUseDirectInstanceId1(bool enable) { use_direct_inst_id1_ = enable; }

  void printGraph() const;
  void printFailed() const;
  
  // Access to visited instance names during parallel visit
  const std::vector<const InstVertex*>& getVisitedInstVertices() const { return visited_inst_vertices_; }
  void clearVisitedInstVertices() { visited_inst_vertices_.clear(); }
  void printVisitedInstNames() const;

  // Mark top instances as selected based on precheck results.
  // All vertices are first reset to unselected, then only those in vertex_ids are marked.
  void markSelectedInstances(const std::vector<size_t> &vertex_ids);

  void setMaxResizeNum(size_t max_resize_num) { max_resize_num_ = max_resize_num; }
  size_t vertexCount() const { return vertices_.size(); }
  const std::unordered_map<const sta::Instance*, VertexId> *instToVidMap() const { return &inst_to_vid_; }
  
  // Topology validation
  void enableTopologyCheck(bool enable) { enable_topology_check_ = enable; }
  void printTopologyViolations() const;
  
  // Flag of verbose printing
  bool verbose_ = false;

protected:
  // Vertices before num_com_ are combinational.
  std::vector<InstVertex> vertices_;
  std::vector<InstEdge> edges_;
  sta::SearchPred *pred_;
  std::unique_ptr<std::atomic<size_t>[]> vertex_ref_counts_;

  rsz::Resizer *resizer_ = nullptr;

  // Number of combinational vertices.
  size_t num_com_ = 0;
  // Safe mapping from Instance* to VertexId to avoid mutating STA internals.
  std::unordered_map<const sta::Instance*, VertexId> inst_to_vid_;
  bool use_direct_inst_id1_ = false;
  // Record the instance names visited during parallel visit in order
  std::vector<const InstVertex*> visited_inst_vertices_;
  // Mutex to protect visited_inst_names_ in multi-threaded environment
  std::mutex visited_inst_names_mutex_;
  // Mutex removed: apply_change_to_db_mutex_ is replaced by g_odb_sta_access_mutex
  // Visitors for each thread
  std::vector<ParallelLrVisitor *> visitors_;
  // Maximum resize number allowed in one iteration
  size_t max_resize_num_ = 1000000;
  // Flag of if the first time visitParallel
  bool incremental_ = false;
  // Flag set after netlist-modifying operations (e.g. buffer insertion)
  bool dirty_ = false;
  
  // Topology validation
  bool enable_topology_check_ = false;
  std::unique_ptr<TopologyChecker> topology_checker_;

private:
  friend class InstVertexOutEdgeIterator;
  friend class InstVertexInEdgeIterator;
};

class InstVertexOutEdgeIterator {
public:
  InstVertexOutEdgeIterator(const InstVertex* vertex,
                            const TaskArranger* arranger);
  InstVertexOutEdgeIterator(const InstVertex &vertex,
                            const TaskArranger* arranger);
  bool hasNext() const;
  EdgeId next();
protected:
  EdgeId next_;
  const TaskArranger* arranger_;
private:
  friend class TaskArranger;
};

class InstVertexInEdgeIterator {
public:
  InstVertexInEdgeIterator(const InstVertex* vertex,
                           const TaskArranger* arranger);
  InstVertexInEdgeIterator(const InstVertex &vertex,
                           const TaskArranger* arranger);
  bool hasNext() const;
  EdgeId next();
protected:
  EdgeId next_;
  const TaskArranger* arranger_;
private:
  friend class TaskArranger;
};

class SearchMEEPred : public sta::SearchPred2
{
public:
  explicit SearchMEEPred(sta::StaState* sta);
  virtual bool searchThru(sta::Edge* edge) override;
  virtual bool searchFrom(const sta::Vertex* from_vertex) override;
  virtual bool searchTo(const sta::Vertex* to_vertex) override;
};


} // namespace lrf
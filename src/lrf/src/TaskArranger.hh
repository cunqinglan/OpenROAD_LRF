
#pragma once

#include <vector>

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/Delay.hh"
#include "sta/StaState.hh"
#include "sta/NetworkClass.hh"
#include "GraphBase.hh"


namespace sta {
  class Graph;
  class Instance;
  class SearchPred;
  class SearchPredNonReg2;
}

namespace lrf {
using sta::EdgeId;
using sta::VertexId;
using sta::Level;
using sta::ObjectIdx;

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

  sta::Instance* inst_ = nullptr;
  sta::Level level_ = 0;
  size_t temp_ref_num_ = 0;
  
  EdgeId in_edges_ = edge_id_null;
  EdgeId out_edges_ = edge_id_null;
  ObjectIdx object_idx_ = object_idx_null;
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
  virtual void assignMee();

  void makeGraph();
  void makeVertices();
  void makeEdges();
  // Assign MEE edges among sibling fanout instances of inst.
  // Assign MEE edges to all fanout instances of inst.
  void makeInstDrvrWireEdges(sta::Instance *inst);
  // Assign MEE edges among sibling fanout instances of inst.
  void makeSiblingFanoutsEdges();
  // Assign MEE edges from root driver to its fanout instances.
  void makeRootEdges();
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

protected:
  std::vector<InstVertex> vertices_;
  std::vector<InstEdge> edges_;
  sta::SearchPred *pred_;

private:
  friend class InstVertexOutEdgeIterator;
};

class InstVertexOutEdgeIterator {
public:
  InstVertexOutEdgeIterator(InstVertex* vertex,
                            const TaskArranger* arranger);
  InstVertexOutEdgeIterator(InstVertex &vertex,
                            const TaskArranger* arranger);
  bool hasNext() const;
  EdgeId next();
protected:
  EdgeId next_;
  const TaskArranger* arranger_;
private:
  friend class TaskArranger;
};




} // namespace lrf
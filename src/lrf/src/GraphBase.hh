#pragma once

#include "sta/Graph.hh"
#include "sta/ObjectId.hh"
#include "sta/ObjectTable.hh"
#include "sta/GraphClass.hh"

namespace lrf {
using sta::ObjectId;
using sta::ObjectIdx;
using sta::VertexId;
using sta::EdgeId;

static constexpr ObjectId object_id_null = std::numeric_limits<ObjectId>::max();
static constexpr ObjectIdx object_idx_null = std::numeric_limits<ObjectIdx>::max();
static constexpr EdgeId edge_id_null = object_id_null;
static constexpr ObjectIdx edge_idx_null = object_id_null;
static constexpr ObjectIdx vertex_idx_null = object_id_null;

class VertexBase {
public:
  VertexBase();
  virtual ~VertexBase() {};
  virtual void init() {};
  void setObjectIdx(ObjectIdx idx) { object_idx_ = idx; }
  ObjectIdx objectIdx() const { return object_idx_; }
  EdgeId inEdges() const { return in_edges_; }
  EdgeId outEdges() const { return out_edges_; }

  ObjectIdx object_idx_;
  EdgeId in_edges_;
  EdgeId out_edges_;
};

class EdgeBase {
public:
  EdgeBase();
  virtual ~EdgeBase() {};
  virtual void init(VertexId from, VertexId to);
  void setObjectIdx(ObjectIdx idx) { object_idx_ = idx; }
  ObjectIdx objectIdx() const { return object_idx_; }
  VertexId from() const { return from_; }
  VertexId to() const { return to_; }


  ObjectIdx object_idx_;
  VertexId from_;
  VertexId to_;
  EdgeId vertex_in_link_;
  EdgeId vertex_out_prev_;
  EdgeId vertex_out_next_;
};


// template <typename VertexT, typename EdgeT>
// class GraphBase {
// public:
//   GraphBase();
//   virtual ~GraphBase();
//   virtual void init();
//   VertexT* vertex(VertexId id);
//   EdgeT* edge(EdgeId id);

// protected:
//   ObjectTable<VertexT> vertices_;
//   ObjectTable<EdgeT> edges_;
// };


}
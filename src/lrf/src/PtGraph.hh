#pragma once

#include <vector>
#include <map>

#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "sta/Network.hh"
#include "sta/Delay.hh"
#include "sta/TimingArc.hh"
#include "sta/Map.hh"

namespace sta {
class Sta;
class LibertyCell;
}

namespace lrf {

// Use sta:: typedefs/types explicitly inside lrf namespace.
using sta::VertexId;
using sta::EdgeId;

// Sentinel ids matching sta usage.
static constexpr EdgeId pt_edge_id_null = 0;
static constexpr VertexId pt_vertex_id_null = 0;

class PtEdge;
class PtVertex;

typedef std::vector<PtEdge> PtEdgeSeq;
typedef std::vector<PtVertex> PtVertexSeq;
typedef std::map<sta::Vertex*, sta::VertexId> VertexPtToIdMap;


enum class PtVertexType : uint8_t {
  RefDriver, // driver vertex of the reference instance, should update parasitics of the these vertices
  RefInput,  // fanin vertices of the reference instance
  RefOutput, // fanout vertices of the reference instance
  None
};


class PtGraph {
public:
  explicit PtGraph(sta::Sta *sta);
  ~PtGraph();

  const PtVertexSeq &ptVertices() const { return pt_vertices_; }
  const PtEdgeSeq &ptEdges() const { return pt_edges_; }

  void makeGraph(sta::InstanceSet &inst_seq, sta::Instance *ref_inst);
  void makePtVertexAndPtEdge(sta::InstanceSet &inst_seq);
  void makePtInstEdge(sta::Vertex *drvr_vertex, sta::VertexId drvr_pt_id);
  void makePtWireEdge(sta::Vertex *drvr_vertex, sta::VertexId drvr_pt_id);

  sta::EdgeId makeEdge(sta::Edge *edge, sta::VertexId pt_from, sta::VertexId pt_to);
  sta::VertexId makeVertex(sta::Vertex *vertex);
  PtEdge &edge(sta::EdgeId edge_id) { return pt_edges_[edge_id]; }
  PtVertex &ptVertex(sta::VertexId vertex_id) { return pt_vertices_[vertex_id]; }
  const PtVertex &ptVertex(sta::VertexId vertex_id) const {
     return pt_vertices_[vertex_id];
  }

  void setGraphMade(bool made) { graph_made_ = made; }
  bool topoSortVertices();
  bool sorted() const { return sorted_; }
  void setSlew(PtVertex &pt_vertex, const sta::RiseFall *rf,
               sta::DcalcAPIndex ap_index, const sta::Slew &slew);
  void initLoadSlews(PtVertex &pt_vertex);
  std::vector<size_t> &sortedVertexIds();
  void initWireDelays(PtVertex &drvr_pt_vertex);
  void setWireArcDelay(PtEdge &pt_edge,
                       const sta::RiseFall *rf,
                       sta::DcalcAPIndex ap_index,
                       const sta::ArcDelay &delay);
  void setArcDelay(PtEdge &pt_edge,
                   const sta::TimingArc *arc,
                   sta::DcalcAPIndex ap_index,
                   const sta::ArcDelay &delay);
  const sta::ArcDelay &arcDelay(const PtEdge &pt_edge,
                                const sta::TimingArc *arc,
                                sta::DcalcAPIndex ap_index);
  const sta::Slew &slew(const PtVertex &pt_vertex,
                        const sta::RiseFall *rf,
                        sta::DcalcAPIndex ap_index);
  const sta::ArcDelay &wireArcDelay(const PtEdge &pt_edge,
                                    const sta::RiseFall *rf,
                                    sta::DcalcAPIndex ap_index);
  std::string to_string();
  void printDelays();
  void setAllArcDelaysZero();

  void delayLmSum(const sta::MinMax *minmax, float &delay_lambda_sum,
                  bool avoid_check = true);
  void delayLmSum(const sta::DcalcAnalysisPt *dcalc_ap, float &delay_lambda_sum,
                  bool avoid_check = true);
  sta::Level vertexLevel(sta::VertexId vertex_id) const;
  sta::Level topVertexLevel();
  void createParasiticsNetworks();
  const PtVertex &pinToPtVertex(const sta::Pin *pin) const;
  float getRefPinCapacitance(const PtVertex &pt_vertex,
                             const sta::RiseFall *rf,
                             const sta::Corner *corner,
                             const sta::MinMax *min_max) const;
                             
  // For virtual cell swap
  void setRefGate(sta::LibertyCell *lib_cell) { ref_lib_cell_ = lib_cell; }
  const sta::LibertyCell *refGate() const { return ref_lib_cell_; }

protected:
  void initVertexAndEdges();
  void annotateRefFaninVertices();

  sta::Sta *sta_;
  PtEdgeSeq pt_edges_;
  PtVertexSeq pt_vertices_;
  size_t ap_count_{};
  bool graph_made_ = false;
  bool sorted_ = false;
  std::vector<size_t> sorted_vertex_ids_;
  std::vector<size_t> roots_;
  VertexPtToIdMap vertex_map_;
  size_t slew_rf_count_{};
  sta::Instance *ref_inst_ = nullptr;
  sta::LibertyCell *ref_lib_cell_ = nullptr;

private:
  friend class PtEdge;
  friend class PtVertex;
  friend class PtVertexInEdgeIterator;
  friend class PtVertexOutEdgeIterator;
  friend class LocalSta;
};

class PtEdge {
public:
  PtEdge();
  ~PtEdge() = default;

  void init(sta::Edge *edge,
            sta::VertexId pt_from,
            sta::VertexId pt_to);

  sta::ArcDelay *arcDelays() { return arc_delays_.empty() ? nullptr : arc_delays_.data(); }
  const sta::ArcDelay *arcDelays() const { return arc_delays_.empty() ? nullptr : arc_delays_.data(); }
  sta::Edge *edge() { return edge_; }
  const sta::Edge *edge() const { return edge_; }
  sta::VertexId ptFromId() const { return pt_from_; }
  sta::VertexId ptToId() const { return pt_to_; }

  sta::EdgeId objectIdx() const { return object_idx_; }
  void setObjectIdx(sta::EdgeId idx);

protected:
  void setArcDelays(sta::ArcDelay *arc_delay, size_t delay_count);
  void copyInfoFromEdge(size_t ap_count);

  sta::Edge *edge_{};
  std::vector<sta::ArcDelay> arc_delays_;
  sta::EdgeId vertex_out_next_{};
  sta::EdgeId vertex_out_prev_{};
  sta::EdgeId vertex_in_link_{};
  sta::EdgeId object_idx_{};
  sta::VertexId pt_from_{};
  sta::VertexId pt_to_{};

private:
  friend class PtGraph;
  friend class PtVertex;
  friend class PtVertexInEdgeIterator;
  friend class PtVertexOutEdgeIterator;
};

class PtVertex {
public:
  PtVertex();
  ~PtVertex() = default;

  void init(sta::Vertex *vertex);

  sta::VertexId objectIdx() const { return object_idx_; }
  void setObjectIdx(sta::VertexId idx);
  sta::Vertex *vertex() { return vertex_; }
  sta::Vertex *vertex() const { return vertex_; }
  sta::Slew *slews() { return slews_.empty() ? nullptr : slews_.data(); }
  const sta::Slew *slews() const { return slews_.empty() ? nullptr : slews_.data(); }
  size_t slewCount() const { return slews_.size(); }
  void resizeSlews(size_t slew_count);
  bool isRoot() const { return is_root_; }
  void copyInfoFromVertex(size_t ap_count, size_t slew_rf_count);
  void setType(PtVertexType type) { type_ = type; }
  PtVertexType type() const { return type_; }

protected:
  sta::Vertex *vertex_{};
  std::vector<sta::Arrival> arrivals_;
  sta::VertexId object_idx_{};
  sta::EdgeId out_edges_{};
  sta::EdgeId in_edges_{};
  std::vector<sta::Slew> slews_;
  bool is_root_{};
  PtVertexType type_{PtVertexType::None};

private:
  friend class PtGraph;
  friend class PtEdge;
  friend class PtVertexInEdgeIterator;
  friend class PtVertexOutEdgeIterator;
};

class PtVertexInEdgeIterator {
public:
  PtVertexInEdgeIterator(sta::VertexId vertex_id,
                         PtGraph *pt_graph);

  bool hasNext();
  PtEdge &next();

protected:
  sta::VertexId vertex_id_{};
  sta::EdgeId next_{};
  PtGraph *pt_graph_{};

private:
  friend class PtGraph;
};

class PtVertexOutEdgeIterator {
public:
  PtVertexOutEdgeIterator(sta::VertexId vertex_id,
                          PtGraph *pt_graph);
  PtVertexOutEdgeIterator(PtVertex &pt_vertex,
                          PtGraph *pt_graph);

  PtEdge &next();
  bool hasNext();

protected:
  sta::VertexId vertex_id_{};
  sta::EdgeId next_{};
  PtGraph *pt_graph_{};

private:
  friend class PtGraph;
};

} // namespace lrf

#pragma once

#include <vector>
#include <map>
#include <unordered_map>

#include "sta/Graph.hh"
#include "sta/Sta.hh"
#include "sta/Network.hh"
#include "sta/Delay.hh"
#include "sta/TimingArc.hh"
#include "sta/Map.hh"
#include "lrf/LrfClass.hh"
#include "PtPiElmore.hh"
#include <stdexcept>

namespace sta {
class Sta;
class LibertyCell;
class DcalcAnalysisPt;
}

namespace lrf {

// Use sta:: typedefs/types explicitly inside lrf namespace.
using sta::VertexId;
using sta::EdgeId;

// Sentinel ids matching sta usage.
static constexpr EdgeId pt_edge_id_null = 0;
static constexpr VertexId pt_vertex_id_null = 0;


class PtGraph {
public:
  explicit PtGraph(sta::Sta *sta);
  ~PtGraph();

  const PtVertexSeq &ptVertices() const { return pt_vertices_; }
  PtVertexSeq &ptVertices() { return pt_vertices_; }
  const PtEdgeSeq &ptEdges() const { return pt_edges_; }

  void makeGraph(sta::InstanceSet &inst_seq, sta::Instance *ref_inst);
  void makeGraph(sta::VertexSet &vertex_set, sta::Instance *ref_inst);
  void makePtVertexAndPtEdge(sta::InstanceSet &inst_seq);
  void makePtVertexAndPtEdge(sta::VertexSet &vertex_set);
  void makePtInstEdge(sta::Vertex *drvr_vertex, sta::VertexId drvr_pt_id);
  void makePtWireEdge(sta::Vertex *drvr_vertex, sta::VertexId drvr_pt_id);
  // When do virtual ref cell swap, update timing arc sets of all edges of
  // the ref instance.
  void updateTimingArcSets();
  void updateRefPorts();
  sta::TagGroup *tagGroup(const PtVertex &pt_vertex);

  sta::EdgeId makeEdge(sta::Edge *edge, sta::VertexId pt_from, sta::VertexId pt_to);
  sta::VertexId makeVertex(sta::Vertex *vertex);
  sta::VertexId makeVirtualVertex(sta::LibertyCell *cell, sta::LibertyPort *port,
                                  bool is_driver, bool is_load, PtVertexType type);
  sta::EdgeId makeVirtualEdge(sta::VertexId pt_from, sta::VertexId pt_to,
                              sta::TimingArcSet *arc_set, bool is_wire);
  void setVirtualEdgeLms(PtEdge &pt_edge, const std::vector<LMValue> &lms);
  void initVirtualPaths(PtVertex &virtual_vertex, PtVertex &source_vertex);
  void deleteEdge(sta::EdgeId edge_id);
  void deleteVertex(sta::VertexId vertex_id);
  void reserveVertices(size_t count) { pt_vertices_.reserve(count); }
  void reserveEdges(size_t count) { pt_edges_.reserve(count); }
  // Pop trailing Sentinel vertices/edges to prevent unbounded vector
  // growth from repeated buildVirtualBuffer/removeVirtualBuffer cycles.
  void popSentinelTail();
  size_t vertexCount() const { return pt_vertices_.size(); }
  size_t edgeCount() const { return pt_edges_.size(); }
  size_t vertexCapacity() const { return pt_vertices_.capacity(); }
  size_t edgeCapacity() const { return pt_edges_.capacity(); }
  PtEdge &edge(sta::EdgeId edge_id) { return pt_edges_[edge_id]; }
  PtVertex &ptVertex(sta::VertexId vertex_id) { return pt_vertices_[vertex_id]; }
  const PtVertex &ptVertex(sta::VertexId vertex_id) const {
     return pt_vertices_[vertex_id];
  }
  PtVertex *ptVertex(const sta::Vertex *vertex) {
    auto it = vertex_map_.find(vertex);
    if (it == vertex_map_.end())
      return nullptr;
    return &pt_vertices_[it->second];
  }
  const PtVertex *ptVertex(const sta::Vertex *vertex) const {
    auto it = vertex_map_.find(vertex);
    if (it == vertex_map_.end())
      return nullptr;
    return &pt_vertices_[it->second];
  }

  void setGraphMade(bool made) { graph_made_ = made; }
  bool topoSortVertices();
  bool sorted() const { return sorted_; }
  void setSlew(PtVertex &pt_vertex, const sta::RiseFall *rf,
               sta::DcalcAPIndex ap_index, const sta::Slew &slew);
  void initLoadSlews(PtVertex &pt_vertex);
  sta::Path *makePaths(sta::VertexId vertex_id, size_t path_count);
  void deletePaths(sta::VertexId vertex_id);
  // Not sure if this initialization is necessary
  // Just copy paths from sta::Vertex to PtVertex
  void initPaths(sta::VertexId vertex_id);
  void initPaths(PtVertex &pt_vertex);

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
  sta::ArcDelay arcDelay (const PtEdge &pt_edge,
                          const sta::TimingArc *arc,
                          sta::DcalcAPIndex ap_index) const;
  float arcLm(const PtEdge &pt_edge,
                     const sta::TimingArc *timing_arc,
                     sta::DcalcAPIndex ap_index) const;
  const sta::Slew &slew(const PtVertex &pt_vertex,
                        const sta::RiseFall *rf,
                        sta::DcalcAPIndex ap_index);
  const sta::ArcDelay &wireArcDelay(const PtEdge &pt_edge,
                                    const sta::RiseFall *rf,
                                    sta::DcalcAPIndex ap_index);
  // Copy slew from PtVertex to real sta::Vertex.
  void writeSlewToGraph(const PtVertex &pt_vertex, sta::Vertex *sta_vertex);
  // Copy paths (arrival + required) from PtVertex to real sta::Vertex.
  // Skips if paths are null or tag groups don't match.
  void writePathsToGraph(const PtVertex &pt_vertex, sta::Vertex *sta_vertex);

  // Print and reset writePathsToGraph diagnostic counters.
  static void printWritePathStats();

  // Output informations of the PtGraph for debug purpose
  std::string to_string();
  void printGraph(bool dot_format = false);
  // Prune insignificant sibling arcs (LM < threshold_ratio of total)
  // to reduce cell evaluation runtime.  Sibling arcs are second-order
  // gate edges between sibling vertices on the fanin side.  Pruned
  // edges are marked sibling_skipped and excluded from delay computation
  // (findDriverDelays1), arrival/required propagation (localVisitFanin/
  // FanoutPaths), and LRS cost computation (delayLmSum).
  void pruneInsignificantSiblings(float threshold_ratio = 0.01f);

  void printGraph(const char *output_path, bool dot_format = false);
  void printDelays();
  void printSlews();
  void printCapacitances();
  void printArrivals();
  void printRequireds();
  
  void setAllArcDelaysZero();

  void delayLmSum(const sta::MinMax *minmax, float &delay_lambda_sum,
                  bool avoid_check = true);
  void delayLmSum(const sta::DcalcAnalysisPt *dcalc_ap, float &delay_lambda_sum,
                  bool avoid_check = true);
  void delayLmSum(const sta::DcalcAnalysisPt *dcalc_ap,
                  DelayLmSumResult *result, 
                  bool collect_vecs = true);
  void refgateDelayLmSum(float &delay_lambda_sum, sta::DcalcAnalysisPt *dcalc_ap);
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
  sta::LibertyCell *refGate() const { return ref_lib_cell_; }
  sta::Instance *refInstance() const { return ref_inst_; }
  void setDcalcAnalysisPt(sta::DcalcAnalysisPt *dcalc_ap) { dcalc_ap_ = dcalc_ap; }
  sta::DcalcAnalysisPt *dcalcAnalysisPt() const { return dcalc_ap_; }

  // PtGraph-local PiElmore parasitics
  PtPiElmore* findPtParasitic(VertexId drvr_id,
                               const sta::RiseFall *rf,
                               int ap_index);
  PtPiElmore& makePtParasitic(VertexId drvr_id,
                               const sta::RiseFall *rf,
                               int ap_index);
  void clearPtParasitics();
  void clearPtParasitics(VertexId drvr_id);

protected:
  void initVertexAndEdges();
  void annotateVerticesType();
  void annotateEdgesType();
  void deleteOutEdge(sta::VertexId from_id, sta::EdgeId edge_id);
  void deleteInEdge(sta::VertexId to_id, sta::EdgeId edge_id);

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
  sta::DcalcAnalysisPt *dcalc_ap_ = nullptr;

  // PtGraph-local PiElmore parasitics storage.
  // Key: driver VertexId. Value: vector indexed by rf * ap_count + ap_index.
  std::unordered_map<VertexId, std::vector<PtPiElmore>> pt_parasitics_;

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
  void initVirtual(sta::VertexId pt_from, sta::VertexId pt_to,
                   sta::TimingArcSet *arc_set, bool is_wire);

  sta::ArcDelay *arcDelays() { return arc_delays_.empty() ? nullptr : arc_delays_.data(); }
  const sta::ArcDelay *arcDelays() const { return arc_delays_.empty() ? nullptr : arc_delays_.data(); }
  size_t arcDelayCount() const { return arc_delays_.size(); }
  sta::Edge *edge() { return edge_; }
  const sta::Edge *edge() const { return edge_; }
  bool hasBase() const { return edge_ != nullptr; }
  bool isWire() const { return edge_ ? edge_->isWire() : is_wire_; }
  bool isVirtual() const {
    return type_ == PtEdgeType::VirtualGateEdge
        || type_ == PtEdgeType::VirtualWireEdge;
  }
  sta::VertexId ptFromId() const { return pt_from_; }
  sta::VertexId ptToId() const { return pt_to_; }
  const sta::TimingRole *role() const;
  PtEdgeType type() const { return type_; }
  void setType(PtEdgeType type) { type_ = type; }

  sta::EdgeId objectIdx() const { return object_idx_; }
  void setObjectIdx(sta::EdgeId idx);
  void setTimingArcSet(sta::TimingArcSet *timing_arc_set) { timing_arc_set_ = timing_arc_set; }
  sta::TimingArcSet *timingArcSet() const { return timing_arc_set_; }

  // LM access: returns base edge LMs if hasBase(), otherwise local arc_lms_
  LMValue *arcLms();
  const LMValue *arcLms() const;
  void setArcLms(const std::vector<LMValue> &lms);

  // Sibling arc skipping: insignificant sibling arcs can be excluded
  // from cost computation and delay evaluation to reduce runtime.
  bool isSiblingSkipped() const { return sibling_skipped_; }
  void setSiblingSkipped(bool v) { sibling_skipped_ = v; }

protected:
  void setArcDelays(sta::ArcDelay *arc_delay, size_t delay_count);
  void copyInfoFromEdge(size_t ap_count);

  sta::Edge *edge_{};
  std::vector<sta::ArcDelay> arc_delays_;
  std::vector<LMValue> arc_lms_;
  bool is_wire_{false};
  bool sibling_skipped_{false};
  sta::EdgeId vertex_out_next_{};
  sta::EdgeId vertex_out_prev_{};
  sta::EdgeId vertex_in_link_{};
  sta::EdgeId object_idx_{};
  sta::VertexId pt_from_{};
  sta::VertexId pt_to_{};
  sta::TimingArcSet *timing_arc_set_{nullptr};
  PtEdgeType type_{PtEdgeType::None};

private:
  friend class PtGraph;
  friend class PtVertex;
  friend class PtVertexInEdgeIterator;
  friend class PtVertexOutEdgeIterator;
  friend class LrRebuffer;
  friend class LrRebufferV2;
};

class PtVertex {
public:
  PtVertex();
  ~PtVertex();
  PtVertex(PtVertex &&other) noexcept;
  PtVertex &operator=(PtVertex &&other) noexcept;
  PtVertex(const PtVertex &) = delete;
  PtVertex &operator=(const PtVertex &) = delete;

  void init(sta::Vertex *vertex);
  void initVirtual(sta::LibertyCell *cell, sta::LibertyPort *port,
                   bool is_driver, bool is_load);
  // Proxy vertex: a real sta::Vertex used for tag_bldr init on virtual vertices.
  // Set by the caller (e.g., LrRebuffer) to the driver vertex of the net being buffered.
  void setProxyVertex(sta::Vertex *v) { proxy_vertex_ = v; }
  sta::Vertex *proxyVertex() const { return proxy_vertex_; }

  sta::VertexId objectIdx() const { return object_idx_; }
  void setObjectIdx(sta::VertexId idx);
  sta::Vertex *vertex() { return vertex_; }
  sta::Pin *pin() { return vertex_ ? vertex_->pin() : nullptr; }
  sta::Vertex *vertex() const { return vertex_; }
  bool hasBase() const { return vertex_ != nullptr; }
  sta::LibertyPort *libertyPort() const { return liberty_port_; }
  void setLibertyPort(sta::LibertyPort *port) { liberty_port_ = port; }
  sta::LibertyCell *libertyCell() const;
  float level() const { return level_; }
  void setLevel(float lvl) { level_ = lvl; }
  sta::Slew *slews() { return slews_.empty() ? nullptr : slews_.data(); }
  bool hasFanin() const;
  bool hasFanout() const;
  const sta::Slew *slews() const { return slews_.empty() ? nullptr : slews_.data(); }
  size_t slewCount() const { return slews_.size(); }
  void resizeSlews(size_t slew_count);
  bool isRoot() const;
  void copyInfoFromVertex(size_t ap_count, size_t slew_rf_count);
  void setType(PtVertexType type) { type_ = type; }
  PtVertexType type() const { return type_; }
  sta::Pin *pin() const { return vertex_ ? vertex_->pin() : nullptr; }
  void setTagGroupIndex(int index) { tag_group_index_ = index; }
  size_t tagGroupIndex() const { return tag_group_index_; }
  sta::Path *paths() const { return paths_; }
  void setPaths(sta::Path *paths);
  void setIsDriver(bool is_driver) { is_driver_ = is_driver; }
  bool isDriver() const { return is_driver_; }
  void setIsLoad(bool is_load) { is_load_ = is_load; }
  bool isLoad() const { return is_load_; }
  bool isVirtual() const {
    return type_ == PtVertexType::VirtualInput
        || type_ == PtVertexType::VirtualOutput;
  }
  // True when this driver's downstream loads include a virtual buffer,
  // meaning the original parasitic is invalid and load_cap should be recomputed.
  void setHasVirtualBuffer(bool v) { has_virtual_buffer_ = v; }
  bool hasVirtualBuffer() const { return has_virtual_buffer_; }

protected:
  sta::Vertex *vertex_{nullptr};
  sta::Vertex *proxy_vertex_{nullptr};
  sta::LibertyPort *liberty_port_{nullptr};
  sta::LibertyCell *liberty_cell_{nullptr};
  float level_{-1.0f};
  std::vector<sta::Arrival> arrivals_;
  sta::VertexId object_idx_{pt_vertex_id_null};
  sta::EdgeId out_edges_{pt_edge_id_null};
  sta::EdgeId in_edges_{pt_edge_id_null};
  std::vector<sta::Slew> slews_;
  bool is_root_{};
  PtVertexType type_{PtVertexType::None};
  int tag_group_index_ {static_cast<int>(sta::tag_group_index_max)};
  sta::Path *paths_ = nullptr;
  bool is_driver_{false};
  bool is_load_{false};
  bool has_virtual_buffer_{false};

private:
  friend class PtGraph;
  friend class PtEdge;
  friend class PtVertexInEdgeIterator;
  friend class PtVertexOutEdgeIterator;
  friend class LocalSta;
  friend class LrRebuffer;
  friend class LrRebufferV2;
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

const char *ptVertexTypeName(PtVertexType type);

} // namespace lrf

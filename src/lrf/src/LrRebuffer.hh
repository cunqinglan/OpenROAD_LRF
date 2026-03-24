#pragma once

#include <string>
#include <vector>
#include "lrf/LrfClass.hh"
#include "../../rsz/src/Rebuffer.hh"
// #include "LocalSta.hh"

namespace rsz {
class Resizer;
class BufferedNet;
using BufferedNetPtr = std::shared_ptr<BufferedNet>;
using BufferedNetSeq = std::vector<BufferedNetPtr>;
}

namespace utl {

}


namespace lrf {

class ParallelLrVisitor;
class PtGraph;
class LocalSta;
class TestLrf;

struct VirtualBufferInfo {
  std::vector<sta::VertexId> vertex_ids;
  std::vector<sta::EdgeId> edge_ids;
  std::vector<sta::EdgeId> orig_wire_edge_ids;
  bool failed = false;
};

class LrRebuffer : public rsz::Rebuffer
{
  friend class TestLrf;
  friend class CombinedVisitor;
public:
  LrRebuffer(rsz::Resizer* resizer, ParallelLrVisitor* parallel_visitor);
  // Call once in serial before creating any LrRebuffer instances in parallel.
  static void initGlobalPreamble(sta::dbSta* sta, rsz::Resizer* resizer);
  void init();
  // Compute the best buffering option and save it at best_bnet_.
  void rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);
  // void annoataLoadSlacks();
  rsz::BufferedNetPtr bufferForTiming(sta::VertexId drvr_vertex_id, const rsz::BufferedNetPtr& tree, bool allow_topology_rewrite, bool last_iteration);
  void annotateLoadLMs(PtVertex &drvr_pt_vertex, const rsz::BufferedNetPtr& tree);
  void insertBufferOptions(rsz::BufferedNetSeq& opts,
                           int level,
                           int next_segment_wl = 0);
  rsz::BufferedNetPtr addWire(const rsz::BufferedNetPtr& p,
                       odb::Point wire_end,
                       int wire_layer,
                       int level = -1);
  int applyBufferingToDb();
  const sta::Pin *drvrPin() const { return drvr_pin_; }
  const rsz::BufferedNetPtr& bestBnet() const { return best_bnet_; }
  float bestCost() const { return best_cost_; }

  // Sensitivity-based precheck: compute max S(v,e) over all buffer points
  // on the driving net. Does NOT insert any buffers.
  // Returns the maximum sensitivity score (positive = buffering is beneficial).
  float computeNetSensitivity(const sta::Pin *drvr_pin,
                              PtVertex &drvr_pt_vertex,
                              float avg_delay, float avg_leakage);

protected:
  // Cost computation: delay_LM_sum + leakage
  float computeBufferAddedCost(float buffer_delay_seconds,
                                float buffer_leakage,
                                const rsz::BufferedNetPtr& load_opt);
  void propagateLmsThroughBuffer(rsz::BufferedNetPtr& buffer_node,
                                 const rsz::BufferedNetPtr& load_opt);
  std::vector<float> mergeLmVectors(const std::vector<float>& lm1,
                                    const std::vector<float>& lm2);
  LMValue evaluateOptionCoarse(sta::VertexId pt_vertex_id, const rsz::BufferedNetPtr& option);
  LMValue evaluateOption(sta::VertexId pt_vertex_id, const rsz::BufferedNetPtr& option,
                       float original_slack);
  float cellDelayLmSum(sta::VertexId pt_vertex_id,
                       const rsz::BufferedNetPtr& load_opt,
                       sta::Slew &max_slew);
  bool hasViolation(const rsz::BufferedNetPtr& option, sta::Slew max_slew);
  VirtualBufferInfo buildVirtualBuffer(sta::VertexId drvr_vertex_id,
                                       const rsz::BufferedNetPtr& option);
  void removeVirtualBuffer(VirtualBufferInfo &info);
  void cleanupVirtualBuffer();
  float computeVirtualSlack(const VirtualBufferInfo &info);
  rsz::BufferedNetPtr attemptTopologyRewrite(const rsz::BufferedNetPtr& node,
                                             const rsz::BufferedNetPtr& left,
                                             const rsz::BufferedNetPtr& right,
                                             float best_cap);
  int bufferNum(const rsz::BufferedNetPtr& tree);
  // After exportBufferTree physically inserts buffers, write the LMs from
  // the BnetPtr tree back onto the corresponding real graph wire edges.
  void writeLmsToGraph();
  // After physical buffer insertion, estimate parasitic for each new buffer's
  // output net and sync into local_parasitic_network_map_ so that subsequent
  // PtGraphs can build PtPiElmore via recomputePtParasitics.
  void syncNewBufferParasitics(const rsz::BufferedNetPtr& tree);
  // Write timing (slew, arrival, required, arc delay) from PtGraph virtual
  // buffer vertices/edges to the corresponding real graph vertices/edges.
  void writeTimingToGraph();
  void initNewStaVertexPaths(const PtVertex &pt_vertex, sta::Vertex *sta_vertex);
  // Build PtPiElmore parasitics for virtual buffer sub-graph.
  // For original driver: adds virtual buffer input Elmore to existing PtPiElmore.
  // For virtual buffer outputs: builds synthetic PtPiElmore from BnetPtr wireRC.
  void buildVirtualParasitics(VertexId drvr_vertex_id,
                              const rsz::BufferedNetPtr& option,
                              const VirtualBufferInfo &vinfo);
  // New: build synthetic ConcreteParasiticNetwork from BnetPtr wireRC and
  // reduce to PtPiElmore.  Replaces driver's PtPiElmore with one that
  // reflects the modified topology (driver sees only buffer input cap,
  // not all original loads).  Parallel to buildVirtualParasitics for testing.
  void buildSyntheticParasitics(VertexId drvr_vertex_id,
                                const rsz::BufferedNetPtr& option,
                                const VirtualBufferInfo &vinfo);
private:
  LocalSta *local_sta_;
  ParallelLrVisitor* visitor_;
  const sta::Pin *drvr_pin_ = nullptr;
  rsz::BufferedNetPtr best_bnet_ = nullptr;
  float best_cost_ = std::numeric_limits<float>::max();
  VirtualBufferInfo best_vinfo_;
  bool verbose_ = true;
};




} // namespace lrf

// LrRebuffer: buffer insertion using EvalContext.
#pragma once

#include <string>
#include <vector>
#include "lrf/LrfClass.hh"
#include "../../rsz/src/Rebuffer.hh"

namespace rsz {
class Resizer;
class BufferedNet;
using BufferedNetPtr = std::shared_ptr<BufferedNet>;
using BufferedNetSeq = std::vector<BufferedNetPtr>;
}

namespace lrf {

struct EvalContext;
class PtGraph;
class LocalSta;

struct VirtualBufferInfo {
  std::vector<sta::VertexId> vertex_ids;
  std::vector<sta::EdgeId> edge_ids;
  std::vector<sta::EdgeId> orig_wire_edge_ids;
  bool failed = false;
};

class LrRebuffer : public rsz::Rebuffer
{
public:
  LrRebuffer(rsz::Resizer* resizer, LocalSta* local_sta, EvalContext* eval_ctx);
  static void initGlobalPreamble(sta::dbSta* sta, rsz::Resizer* resizer);
  void init();
  void rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);
  rsz::BufferedNetPtr bufferForTiming(sta::VertexId drvr_vertex_id, const rsz::BufferedNetPtr& tree, bool allow_topology_rewrite, bool last_iteration);
  void annotateLoadLMs(PtVertex &drvr_pt_vertex, const rsz::BufferedNetPtr& tree);
  void insertBufferOptions(rsz::BufferedNetSeq& opts, int level, int next_segment_wl = 0);
  rsz::BufferedNetPtr addWire(const rsz::BufferedNetPtr& p, odb::Point wire_end, int wire_layer, int level = -1);
  int applyBufferingToDb();
  void persistBufferParasitics();
  const sta::Pin *drvrPin() const { return drvr_pin_; }
  const rsz::BufferedNetPtr& bestBnet() const { return best_bnet_; }
  float bestCost() const { return best_cost_; }
  rsz::Resizer *resizer() const { return resizer_; }

  float computeNetSensitivity(const sta::Pin *drvr_pin,
                              PtVertex &drvr_pt_vertex,
                              float avg_delay, float avg_leakage);

  // ── Two-phase buffering for CombinedOperator cache reuse ──
  //
  // Phase A (cell-independent): build Steiner tree, annotate LMs, run 2 rounds
  // of coarse bufferForTiming.  The returned BnetPtr encodes the buffer option
  // list and can be reused across multiple resize candidates.
  rsz::BufferedNetPtr prepareBufferOptions(const sta::Pin *drvr_pin,
                                           PtVertex &drvr_pt_vertex);

  // Phase B (cell-dependent): given a prepared BnetPtr from prepareBufferOptions,
  // run 1 round of precise bufferForTiming on the current PtGraph state.
  // Updates best_bnet_/best_cost_ if a better option is found.
  // Must call cleanupVirtualBuffer() after each candidate.
  void evaluateBufferOnCandidate(sta::VertexId drvr_vid,
                                 const rsz::BufferedNetPtr &prepared_bnet);

  void cleanupVirtualBuffer();

  // Repair slew violations on a single driver net by inserting buffers.
  int repairSlew(const sta::Pin *drvr_pin, rsz::Resizer *resizer);

  // Rebuffer a driver pin using the same algorithm as repair_timing's
  // BufferMove (rsz::Rebuffer::rebufferPin): iterative bufferForTiming
  // followed by area recovery, then export buffer tree to DB.
  // Returns the number of inserted buffers.
  int rebufferPinRsz(const sta::Pin *drvr_pin);

  // Experiment B: generate bnet with RSZ algorithm, evaluate with LRF local timing.
  // Does NOT modify the design — only prints diagnostic info.
  void probeRszBnetWithLocalEval(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);
  // Repair cap violations on a single driver net by inserting buffers.
  // Walks the Steiner tree bottom-up; at each junction where combined
  // cap exceeds max_cap, inserts a buffer to isolate the larger branch
  // (same strategy as RepairDesign::repairNetJunc).
  static int repairCap(const sta::Pin *drvr_pin, float max_cap,
                       sta::dbSta *sta, rsz::Resizer *resizer);

  // Slew-aware buffer cell selection (mirrors RepairDesign::findBufferUnderSlew).
  // Picks the smallest buffer whose output slew stays under max_slew when
  // driving load_cap.  Falls back to the buffer with minimum achievable slew.
  static sta::LibertyCell *findBufferUnderSlew(
      rsz::Resizer *resizer, float max_slew, float load_cap);

  // Insert a repeater buffer at the given location, resize it, and update
  // load_pins / repeater_cap to reflect the buffer's input pin.
  // Returns true on success (mirrors RepairDesign::makeRepeater).
  static bool makeRepeater(rsz::Resizer *resizer,
                           const sta::Corner *corner,
                           const odb::Point &loc,
                           sta::LibertyCell *buffer_cell,
                           sta::PinSeq &load_pins,
                           float &repeater_cap);

protected:
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
  float computeVirtualSlack(const VirtualBufferInfo &info);
  rsz::BufferedNetPtr attemptTopologyRewrite(const rsz::BufferedNetPtr& node,
                                             const rsz::BufferedNetPtr& left,
                                             const rsz::BufferedNetPtr& right,
                                             float best_cap);
  int bufferNum(const rsz::BufferedNetPtr& tree);
  void writeLmsToGraph();
  void writeTimingToGraph();
  void initNewStaVertexPaths(const PtVertex &pt_vertex, sta::Vertex *sta_vertex);
  void buildSyntheticParasitics(VertexId drvr_vertex_id,
                                const rsz::BufferedNetPtr& option,
                                const VirtualBufferInfo &vinfo);

private:
  LocalSta *local_sta_;
  EvalContext *eval_ctx_;
  const sta::Pin *drvr_pin_ = nullptr;
  rsz::BufferedNetPtr best_bnet_ = nullptr;
  float best_cost_ = std::numeric_limits<float>::max();
  float last_delay_lm_sum_ = 0.0f;
  float last_slack_after_ = -1e30f;
  VirtualBufferInfo best_vinfo_;
  bool verbose_ = true;
};

} // namespace lrf

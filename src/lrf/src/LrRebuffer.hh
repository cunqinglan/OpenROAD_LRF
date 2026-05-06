// LrRebuffer: buffer insertion using EvalContext.
#pragma once

#include <string>
#include <vector>
#include "lrf/LrfClass.hh"
#include "LocalSta.hh"
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
  rsz::BufferedNetPtr bufferForTimingLrf(sta::VertexId drvr_vertex_id, const rsz::BufferedNetPtr& tree, bool allow_topology_rewrite, bool last_iteration);
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

  // Parallel-safe RSZ-style rebuffering (pure slack-DP, no recovery):
  // pre-checks, makeBufferedNet, annotateLoadSlacksSlackDp,
  // N× bufferForTimingSlackDp.  Stores result in best_bnet_/drvr_pin_.
  // No recoverArea (not parallel-safe).  Takes VertexId for thread safety.
  bool prepareRszBnet(const sta::Pin *drvr_pin, sta::VertexId drvr_vid,
                      int bft_iter = 3);

  // Slack-DP entry: parallel to prepareRszBnet but uses LRF's slack-DP
  // family (bufferForTimingSlackDp + recoverLrCost). Stores result in
  // best_bnet_/drvr_pin_. Takes VertexId (not PtVertex&) so that any
  // downstream pt_graph mutation (e.g., buildVirtualBuffer inside
  // evaluateOption) can't invalidate the caller's reference — the PtVertex
  // is looked up fresh via eval_ctx_->pt_graph->ptVertex(vid) (O(1) vector
  // index) each time it's actually needed.
  bool prepareSlackDpBnet(const sta::Pin *drvr_pin,
                           sta::VertexId drvr_vid,
                           int bft_iter = 3,
                           int recover_iter = 5);

  // Phase 2 (requires mutex): call applyBufferingToDb() to export
  // best_bnet_ to DB, persist parasitics, and write timing.

  // Experiment B: generate bnet with RSZ algorithm, evaluate with LRF local timing.
  // Does NOT modify the design — only prints diagnostic info.
  void probeRszBnetWithLocalEval(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);

  // ── Slack-based DP family (LRF-self-contained, no Rebuffer base calls) ──
  //
  // All four functions (annotate / bufferForTiming / insertBufferOptions /
  // attemptTopologyRewrite) are LRF-local copies of the Rebuffer base class
  // methods, with two key differences:
  //
  //   1. annotateLoadSlacksSlackDp: replaces Rebuffer::annotateLoadSlacks.
  //      Does NOT populate Rebuffer::arrival_paths_; reads sink slacks via
  //      sta_->vertexSlack (single memory read, multi-thread safe) and caches
  //      drvr_worst_rf_/drvr_worst_dap_ from driver's own paths instead.
  //
  //   2. bufferForTimingSlackDp / insertBufferOptionsSlackDp /
  //      attemptTopologyRewriteSlackDp: replace every bufferDelay(cell, rf, cap)
  //      with computeBufferGateDelay(cell, cap), avoiding arrival_paths_.
  //
  // CRITICAL: Rebuffer::annotateLoadSlacks (base class) is still relied on by
  // probeRszBnetWithLocalEval / probeAllOptions / other probes that need real
  // arrival_paths_. Do NOT call annotateLoadSlacksSlackDp before those probes
  // — they're the wrong API for the wrong context.
  void annotateLoadSlacksSlackDp(rsz::BufferedNetPtr& tree,
                                  sta::VertexId drvr_vid);
  rsz::BufferedNetPtr bufferForTimingSlackDp(
      sta::VertexId drvr_vertex_id,
      const rsz::BufferedNetPtr& tree,
      bool allow_topology_rewrite);
  // Slack-based wire-walk buffer insertion. With lrcost_oriented=true (used
  // by recoverLrCost), ranks options by bufferCost subject to slack ≥
  // slack_threshold; mirrors base Rebuffer::insertBufferOptions area mode
  // with bufferCost replacing area as the objective.
  void insertBufferOptionsSlackDp(rsz::BufferedNetSeq& opts, int level,
                                   int next_segment_wl,
                                   bool lrcost_oriented = false,
                                   rsz::FixedDelay slack_threshold
                                       = rsz::FixedDelay::ZERO,
                                   rsz::BufferedNet* exemplar = nullptr);
  rsz::BufferedNetPtr attemptTopologyRewriteSlackDp(
      const rsz::BufferedNetPtr& node,
      const rsz::BufferedNetPtr& left,
      const rsz::BufferedNetPtr& right,
      float best_cap);

  // Recover LR cost on a bnet produced by bufferForTimingSlackDp.
  // Full faithful port of Rebuffer::recoverArea: top-down arrival-delay
  // spread, bottom-up DP enumeration with assured-envelope guard and
  // alpha-blended slack threshold, junction full N×M cross product +
  // Pareto pruning, final selection by min bufferCost among slack-meeting
  // options. Caller must call annotateLoadLMs first.
  rsz::BufferedNetPtr recoverLrCost(sta::VertexId drvr_vertex_id,
                                    const rsz::BufferedNetPtr& root,
                                    rsz::FixedDelay slack_target,
                                    float alpha);
  void computeAndAnnotateBufferCost(const rsz::BufferedNetPtr& root);
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
  // Compute buffer gate delay = max(rise, fall) when driving load_cap,
  // using PtGraph's dcalcAnalysisPt. Standalone replacement for
  // Rebuffer::bufferDelay, which LRF cannot use because it relies on
  // arrival_paths_ populated by annotateLoadSlacks (which LRF doesn't call).
  rsz::FixedDelay computeBufferGateDelay(sta::LibertyCell *buffer_cell,
                                         float load_cap);

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
  // Real-slew ERC after virtual buffer + local STA: walks the driver's wire
  // fanout in the real sta::Graph (untouched by buildVirtualBuffer) to read
  // post-buffering slew on the orig load PtVertices, plus the driver pin's
  // own output slew. Returns Σ(slew - lib_limit × erc_slew_limit_scale)
  // over violators. Cap is left at 0 in this scope.
  LocalSta::ViolationSum computeOrigErcViolation(sta::VertexId drvr_pt_vid);
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
  // Bottom-up pass: set correct levels on virtual buffer vertices so that
  // topo sort places each VirtualOutput before its downstream loads.
  // Returns the level of the topmost (closest-to-driver) vertex in the subtree.
  float fixupVirtualLevels(const rsz::BufferedNetPtr& node,
                           float drvr_level,
                           VirtualBufferInfo &info,
                           size_t &vi);

protected:
  LocalSta *local_sta_;
  EvalContext *eval_ctx_;
  const sta::Pin *drvr_pin_ = nullptr;
  rsz::BufferedNetPtr best_bnet_ = nullptr;
  float best_cost_ = std::numeric_limits<float>::max();
  float last_delay_lm_sum_ = 0.0f;
  float last_slack_after_ = -1e30f;
  VirtualBufferInfo best_vinfo_;
  bool verbose_ = true;

  // Diagnostic flag: when true, bufferForTiming / insertBufferOptions /
  // attemptTopologyRewrite emit [DBG-PRUNE-*] trace lines on every
  // candidate decision point. Used by TestRebuffer::probeAllOptions to
  // investigate why multi-buffer topologies are not being generated.
  bool prune_debug_ = false;

  // Cached driver worst-path metadata, populated by annotateLoadSlacksFast.
  // Used as a thread-safe replacement for Rebuffer::arrival_paths_ when
  // running slack-based bufferForTiming on LRF context.
  const sta::RiseFall *drvr_worst_rf_ = nullptr;
  const sta::DcalcAnalysisPt *drvr_worst_dap_ = nullptr;

  // Saved from last bufferForTiming call (last iteration's top options)
  std::vector<rsz::BufferedNetPtr> last_top_opts_;
};

} // namespace lrf

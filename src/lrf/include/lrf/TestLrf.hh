#pragma once

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/Delay.hh"
#include "lrf/LrfClass.hh"
#include "lrf/LrConfig.hh"

#include <stdexcept>
#include <string>
#include <vector>

namespace odb
{
class dbBlock;
class dbInst;
}  // namespace odb

namespace rsz
{
class Resizer;
}  // namespace rsz

namespace sta
{
class dbSta;
class dbNetwork;
class Instance;
class DcalcAnalysisPt;
}  // namespace sta

namespace est
{
class EstimateParasitics;
}  // namespace est


namespace lrf
{
class IncreSta;
class LocalSta;
class PtGraph;
class PtVertex;
class PtEdge;

struct ErrorPoint
{
  PtVertex *local_vertex;
  PtEdge   *local_edge;
  PtVertex *open_vertex;
  PtEdge   *open_edge;
  sta::Arrival local_arrival;
  sta::Arrival open_arrival;
  sta::DcalcAnalysisPt* analysis_pt;
};

// ── IterationHelper ──────────────────────────────────────
// Tracks per-phase metrics and prints formatted deltas.
// Independent class, can be reused outside TestLrf.
class IterationHelper {
public:
  struct Metrics {
    double wns_ps = 0.0;
    double tns_ps = 0.0;
    double leakage = 0.0;   // raw watts from STA
    double runtime_s = 0.0;
  };

  IterationHelper(sta::dbSta *sta, odb::dbBlock *block,
                  LocalSta *local_sta = nullptr,
                  rsz::Resizer *resizer = nullptr);

  // Snapshot current timing + leakage
  Metrics snapshot(double runtime_s = 0.0);

  // Record a row (buffered, not printed immediately)
  void recordRow(size_t iter, const char *phase,
                 const Metrics &cur, const Metrics &before,
                 const char *decision);

  // Print all recorded rows as a clean table
  void printSummary(const Metrics &best);

  // ECO decision: compare cur vs best, manage ECO checkpoint.
  // Returns decision string. Sets should_break=true if should terminate.
  // allow_revert=false: only accept or hold (for buffer phases that shouldn't revert).
  const char *ecoDecision(const Metrics &cur, Metrics &best,
                          size_t &no_improve, size_t tolerance,
                          size_t &eco_iter, bool &should_break,
                          bool allow_revert = true);

private:
  sta::dbSta *sta_;
  odb::dbBlock *block_;
  sta::Corner *corner_;
  LocalSta *local_sta_;
  rsz::Resizer *resizer_;
  std::vector<std::string> rows_;
  Metrics last_cur_;      // cur from previous recordRow; used to compute dWNS/dTNS (delta-vs-prev-iter, not delta-vs-best)
};

class  TestLrf
{
public:
  void testLocalDelayCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalArrivalCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalRequiredCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalSlewCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testDifferenceBetweenLocalAndOpen(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testMEEAssignments(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testReportVertices(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  // Stage 1 (FF resize verification): print LM / AAT / RAT / slack on edges
  // adjacent to the named flip-flop instance. Bootstraps a RapidLrHelper,
  // runs `lm_iters` lmUpdate() cycles with RATCONS enabled, then dumps:
  //   - D-side: combinational in-edges of every load pin (D, CK, ...)
  //   - Q-side: gate edges into next combinational drivers, reached by
  //             walking each driver's wire fanout to a load and then
  //             that load's out-edges
  //   - CK->Q: regClkToQ in-edge of the Q driver vertex (informational)
  // Setup/hold check edges are skipped (no LM is maintained on them).
  void testReportFFEndpointLMs(char *inst_name, sta::dbSta* sta,
                               rsz::Resizer *resizer, odb::dbBlock *block,
                               size_t lm_iters = 3);

  // Stage 2 (FF resize verification): build a PtGraph in FF mode for the
  // named flip-flop instance and dump its vertices and edges. Confirms that
  // exactly one CK→D CheckEdge (role=setup) exists, that CK pin is included
  // without sibling FFs flooding in, and that CK→Q regClkToQ in-edge is
  // present on the Q driver.
  void testReportFFPtGraph(char *inst_name, sta::dbSta* sta,
                           rsz::Resizer *resizer, odb::dbBlock *block);

  // Stage 3 (FF resize verification): build a PtGraph in FF mode, run
  // findLocalDelays + findLocalCheckDelays, and compare the resulting
  // delays against OpenSTA references:
  //   - CK→Q (regClkToQ) gate delay  vs sta::Graph::arcDelay
  //   - setup CheckEdge delay        vs ArcDelayCalc::checkDelay called
  //                                     directly with global slews
  // Acceptance: differences < 1e-13 s for both classes of arcs.
  void testFFLocalDelay(char *inst_name, sta::dbSta* sta,
                        rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelLrResizeByArray(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            bool initialize = false,
                            float density_weight = 0.0f,
                            std::string checkpoint_dir = "",
                            float timing_margin = 0.01f,
                            bool resize_ff = false);

  void testParallelLrResizeByArrayWithBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            bool initialize = false,
                            float density_weight = 0.0f,
                            bool debug = false,
                            size_t buffering_start_iter = 5);

  // Resize iterations + RSZ-style rebuffering phases
  void testParallelLrResizeByArrayWithRszBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            bool initialize = false,
                            float density_weight = 0.0f,
                            bool debug = false);

  // Resize iterations + LRF slack-DP rebuffering phases (uses
  // BufferSdpOperator → prepareSlackDpBnet → bufferForTimingSlackDp +
  // recoverLrCost). Mirrors testParallelLrResizeByArrayWithBuffering but
  // swaps the buffering-phase operator.
  void testParallelLrResizeByArrayWithSdpBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            bool initialize = false,
                            float density_weight = 0.0f,
                            bool debug = false,
                            size_t buffering_start_iter = 5,
                            float timing_margin = 0.01f);

  // Two-phase flow:
  //   (A) init + pure resize (NO_HALVE ECO; full resize, no precheck);
  //       terminates on first non-improvement past warmup, or when total
  //       iters reach `iterations`.
  //   (B) precheck-resize (every iter) + LRF slack-DP rebuffering, runs
  //       on the remaining iter budget with HALVE_ON_CONSECUTIVE ECO.
  // `iterations` is the shared total iter cap (phase A + phase B). Both
  // phases share IncreSta/LRHelper/best snapshot; ECO transaction is
  // opened once before A and closed once after B.
  void testInitResizeThenSdpBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            bool initialize = false,
                            float density_weight = 0.0f,
                            bool debug = false,
                            size_t buffering_start_iter = 5,
                            float timing_margin = 0.01f,
                            float erc_violation_weight = -1.0f,
                            float erc_limit_scale = 0.95f);

  // Print all liberty cells information grouped by unique equiv cell groups.
  void printAllCellsInfo(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  // Single-pass resize + buffering (combined visitor)
  void testParallelLrCombinedResizeBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            bool initialize = false);

  void testBufferOnly(sta::dbSta* sta,
                      rsz::Resizer *resizer,
                      odb::dbBlock *block,
                      size_t thread_num,
                      size_t iterations = 6,
                      float PT_tradeoff = 10.0,
                      std::string lr_helper_method = "RapidLRHelper",
                      float bakoglu_k = 2.5f,
                      bool debug = false);

  // Minimal single-pass buffer operator comparison entry.
  // LM warmup → one parallelBuffering (use_sdp=false) or parallelBufferingSdp
  // (use_sdp=true) call → report WNS/TNS/leakage delta. No resize, no ECO.
  void testSingleBufferPass(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            bool use_sdp = false,
                            float PT_tradeoff = 10.0f,
                            std::string lr_helper_method = "RapidLRHelper",
                            size_t lm_warmup_rounds = 5);

  // Resize by array with precheck (ParallelVisitor + ResizePrecheckOperator)
  void testParallelLrResizeByArrayWithPrecheck(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            float top_ratio = 0.3);

  // Resize by array with precheck + buffering
  void testParallelLrResizeByArrayWithPrecheckBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper",
                            float top_ratio = 0.3);

  // Preceding resize precheck: evaluate resize benefit for all instances
  // in parallel without a conflict graph, sort by benefit, and print results.
  void testPrecedingResizeCheck(sta::dbSta* sta,
                                rsz::Resizer *resizer,
                                odb::dbBlock *block,
                                size_t thread_num,
                                float PT_tradeoff = 100.0,
                                float top_ratio = 0.3);

  // Test parallel KKT projection correctness: compare serial vs parallel results.
  void testParallelKKTProjection(sta::dbSta* sta,
                                  rsz::Resizer *resizer,
                                  odb::dbBlock *block,
                                  size_t thread_num,
                                  std::string lr_helper_method = "RapidLRHelper");

  // Test LocalSTA slew + arrival accuracy: full traversal with selective
  // resize, compare global graph against updateTiming ground truth.
  void testLocalStaAccuracy(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t max_steps);

  // Analyze slew violation pins: for each violated driver, check whether
  // downsizing fanout loads and/or upsizing the driver can fix the violation.
  void testSlewViolationFeasibility(sta::dbSta* sta,
                                    rsz::Resizer *resizer,
                                    odb::dbBlock *block);

  // Repair slew violations by buffer insertion on violated driver nets.
  void testRepairSlew(sta::dbSta* sta,
                      rsz::Resizer *resizer,
                      odb::dbBlock *block);
  // ECO experiment: lmUpdate → resize → if worse → lmUpdate → revert (no halve)
  void testEcoResizeNoHalve(sta::dbSta* sta,
                             rsz::Resizer *resizer,
                             odb::dbBlock *block,
                             size_t thread_num,
                             size_t iterations = 12,
                             float PT_tradeoff = 10.0,
                             std::string lr_helper_method = "RapidLRHelper",
                             float halve_factor = 0.5f,
                             bool use_precheck = true);

  // Sensitivity screening + rsz-style rebuffering (repair_timing algorithm).
  void testBufferingRsz(sta::dbSta* sta,
                        rsz::Resizer *resizer,
                        odb::dbBlock *block,
                        size_t thread_num,
                        float PT_tradeoff = 100.0f,
                        int top_n = 100);
  // Probe: try rebuffering one instance at a time, report WNS/TNS delta, revert.
  void probeBufferOneByOne(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num = 10,
                            bool use_rsz = true);

  // Probe: RSZ bnet generation + LRF local timing evaluation (diagnostic only).
  void probeRszBnet(sta::dbSta* sta, rsz::Resizer *resizer,
                    odb::dbBlock *block, size_t thread_num = 10);

  // Deep probe: for specific pins, compare 3 methods (RSZ, LRF-worst, LRF-sum)
  // with full local + global timing analysis.
  void probeBufferDeep(sta::dbSta* sta, rsz::Resizer *resizer,
                       odb::dbBlock *block, size_t thread_num,
                       const std::vector<std::string> &pin_names);

  // Enumerate ALL bnet options for one pin, eval local+global for each.
  void probeAllOptions(sta::dbSta* sta, rsz::Resizer *resizer,
                       odb::dbBlock *block, size_t thread_num,
                       const char *pin_name);

  // Batch variant: select top-N buffering candidates by sensitivity and run
  // probeAllOptions on each (RSZ vs SDP vs LRF 3-way comparison per pin).
  void probeAllOptionsBySensitivity(sta::dbSta* sta, rsz::Resizer *resizer,
                                    odb::dbBlock *block, size_t thread_num,
                                    int top_n);

  // ── Unified entry point ──
  // Single function that dispatches by LrConfig::mode.
  // Replaces testParallelLrResizeByArray, WithBuffering, WithPrecheck, etc.
  void runLr(sta::dbSta* sta, rsz::Resizer *resizer,
             odb::dbBlock *block, size_t thread_num,
             const LrConfig &cfg);

  // Run parallel 4-step initialization (Sharma) on an existing IncreSta.
  // Called internally by testParallelLrResize* when initialize=true.
  void runInitialization(sta::dbSta* sta, IncreSta* incre_sta,
                         rsz::Resizer *resizer, odb::dbBlock *block,
                         size_t thread_num,
                         bool minimize_leakage = true);

  // Test: level-parallel initializer (standalone, does not start LR).
  void testParallelInitializer(sta::dbSta* sta,
                               rsz::Resizer *resizer,
                               odb::dbBlock *block,
                               int thread_count,
                               bool minimize_leakage = true);

  // Debug: verify precheck predictions by applying single-gate sizing
  // one at a time, recording PtGraph timing + LM before/after each swap.
  void debugPrecheckAccuracy(sta::dbSta* sta,
                             rsz::Resizer *resizer,
                             odb::dbBlock *block,
                             size_t thread_num,
                             float PT_tradeoff = 100.0,
                             float top_ratio = 0.3,
                             std::string lr_helper_method = "RapidLRHelper");

protected:
  void printSlewComparison(char *inst_name, sta::dbSta* sta, 
                       LocalSta *local_sta, odb::dbInst *db_inst, 
                       sta::Instance *sta_inst, sta::dbNetwork *db_network);
  void printLocalDelaysAndCap(char *inst_name, sta::dbSta* sta, 
                       LocalSta *local_sta, odb::dbInst *db_inst, 
                       sta::Instance *sta_inst, sta::dbNetwork *db_network);
  bool comparePtGraphs(PtGraph *local_pt_graph, PtGraph *open_pt_graph, sta::dbSta* sta);

  void collectTimingInfoForInstancesUsingOpenSta(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, std::vector<sta::Instance*> &sta_insts,
  std::unordered_map<sta::Instance*, TimingRecord> &instance_timing_map);

  bool compareTimingRecords(const std::unordered_map<sta::Instance*, TimingRecord> &records1,
                            const std::unordered_map<sta::Instance*, TimingRecord> &records2,
                            sta::dbSta* sta);

  std::vector<ErrorPoint> error_points_;
};

void recordGraphTimingFromPtGraph(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing, bool verbose = false);

}  // namespace lrf
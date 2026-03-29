#pragma once

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "lrf/LrfClass.hh"

#include <stdexcept>
#include <string>
#include <vector>

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
  Arrival local_arrival;
  Arrival open_arrival;
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
};

class  TestLrf
{
public:
  void testLocalDelayCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalArrivalCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalRequiredCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalSlewCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testDifferenceBetweenLocalAndOpen(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelVisitor(const std::vector<odb::dbInst*>& db_insts, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  // Test single net buffer insertion
  void testBufferInsertion(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  // Test buffering on a single instance via visitor->visit() path
  void testSingleInstBuffering(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testMEEAssignments(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelResize(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testReportVertices(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelLrResizing(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, size_t thread_num, 
    size_t max_resize_num, size_t iterations, size_t num_no_improve_tolerance, bool ratcons = false,
    float PT_tradeoff = 100.0, std::string lr_helper_method = "LRHelper");

  void testParallelLrResizingBuffering(sta::dbSta* sta, 
                            rsz::Resizer *resizer, 
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method);

  void testParallelLrResizeByArray(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "LRHelper");

  void testParallelLrResizeByArrayV2(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "LRHelper");

  void testParallelLrResizeByArrayWithBufferingV2(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "LRHelper");

  void testTimingComputeAndWriteBack(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, const std::vector<odb::dbInst*> &insts);

  // Print all liberty cells information grouped by unique equiv cell groups.
  void printAllCellsInfo(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  // integrate resizing by array and parallel buffering
  void testParallelLrResizeByArrayWithBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons = false,
                            float PT_tradeoff = 100.0,
                            std::string lr_helper_method = "RapidLRHelper");

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
                            std::string lr_helper_method = "RapidLRHelper");

  // Resize by array with preceding precheck filtering
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

  // Resize by array with precheck + buffering (precheck replaces adaptive resize)
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

  // V2 resize by array with precheck (new framework: ParallelVisitor + ResizePrecheckOperator)
  void testParallelLrResizeByArrayWithPrecheckV2(sta::dbSta* sta,
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

  // V2 resize by array with precheck + buffering (new framework)
  void testParallelLrResizeByArrayWithPrecheckBufferingV2(sta::dbSta* sta,
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
                                  std::string lr_helper_method = "LRHelper");

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

  void collectTimingInfoForInstancesUsingLocalSta(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, std::vector<sta::Instance*> &sta_insts,
  std::unordered_map<sta::Instance*, TimingRecord> &instance_timing_map);

  bool compareTimingRecords(const std::unordered_map<sta::Instance*, TimingRecord> &records1,
                            const std::unordered_map<sta::Instance*, TimingRecord> &records2,
                            sta::dbSta* sta);

  std::vector<ErrorPoint> error_points_;
};

void recordGraphTimingFromPtGraph(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing, bool verbose = false);

}  // namespace lrf
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2025, The OpenROAD Authors

#pragma once

#include <string>
#include <utility>
#include <vector>

#include "sta/Clock.hh"
#include "sta/Graph.hh"
#include "sta/PatternMatch.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"

namespace lrf {
struct LrConfig;
}  // namespace lrf

namespace odb {
class dbMaster;
class dbMTerm;
class dbITerm;
class dbBTerm;
class dbInst;
class dbNet;
}  // namespace odb

namespace sta {
class dbSta;
class Scene;
class LibertyCell;
class Network;
class Sta;
class RiseFall;
class Vertex;
class Pin;
}  // namespace sta

namespace ord {

class Design;
class OpenRoad;

// ── Timing report structs (top-level for SWIG compatibility) ──
struct EndpointSlack
{
  odb::dbITerm* iterm = nullptr;
  odb::dbBTerm* bterm = nullptr;
  float slack = 0.0f;
};

struct ClockInfo
{
  std::string name;
  float period = 0.0f;
  std::vector<float> waveform;
  std::vector<odb::dbITerm*> source_iterms;
  std::vector<odb::dbBTerm*> source_bterms;
};

struct TimingArcInfo
{
  odb::dbITerm* from_iterm = nullptr;
  odb::dbBTerm* from_bterm = nullptr;
  odb::dbITerm* to_iterm = nullptr;
  odb::dbBTerm* to_bterm = nullptr;
  odb::dbMaster* master = nullptr;  // nullptr for net arcs
  float delay = 0.0f;
  float slew = 0.0f;
  float load = 0.0f;
  int fanout = 0;
  bool is_rising = false;
};

struct TimingPathInfo
{
  float slack = 0.0f;
  float path_delay = 0.0f;
  float arrival = 0.0f;
  float required = 0.0f;
  float skew = 0.0f;
  float logic_delay = 0.0f;
  int logic_depth = 0;
  int fanout = 0;
  odb::dbITerm* start_iterm = nullptr;
  odb::dbBTerm* start_bterm = nullptr;
  odb::dbITerm* end_iterm = nullptr;
  odb::dbBTerm* end_bterm = nullptr;
  std::string start_clock;
  std::string end_clock;
  std::string path_group;
  std::vector<TimingArcInfo> arcs;
};

class Timing
{
 public:
  explicit Timing(Design* design);

  enum RiseFall
  {
    Rise,
    Fall
  };
  enum MinMax
  {
    Min,
    Max
  };

  sta::ClockSeq findClocksMatching(const char* pattern,
                                   bool regexp,
                                   bool nocase);
  float getPinArrival(odb::dbITerm* db_pin, RiseFall rf, MinMax minmax = Max);
  float getPinArrival(odb::dbBTerm* db_pin, RiseFall rf, MinMax minmax = Max);
  bool isTimeInf(float time);

  float getPinSlew(odb::dbITerm* db_pin, MinMax minmax = Max);
  float getPinSlew(odb::dbBTerm* db_pin, MinMax minmax = Max);

  float getPinSlack(odb::dbITerm* db_pin, RiseFall rf, MinMax minmax = Max);
  float getPinSlack(odb::dbBTerm* db_pin, RiseFall rf, MinMax minmax = Max);

  bool isEndpoint(odb::dbITerm* db_pin);
  bool isEndpoint(odb::dbBTerm* db_pin);

  float getNetCap(odb::dbNet* net, sta::Scene* corner, MinMax minmax);
  float getPortCap(odb::dbITerm* pin, sta::Scene* corner, MinMax minmax);
  float getMaxCapLimit(odb::dbMTerm* pin);
  float getMaxSlewLimit(odb::dbMTerm* pin);
  float staticPower(odb::dbInst* inst, sta::Scene* corner);
  float dynamicPower(odb::dbInst* inst, sta::Scene* corner);

  std::vector<odb::dbMTerm*> getTimingFanoutFrom(odb::dbMTerm* input);
  std::vector<sta::Scene*> getCorners();
  sta::Scene* cmdCorner();
  sta::Scene* findCorner(const char* name);

  void makeEquivCells();
  std::vector<odb::dbMaster*> equivCells(odb::dbMaster* master);

  // ── Summary metrics ─────────────────────────────────────────
  float getWorstSlack(MinMax minmax = Max);
  float getTotalNegativeSlack(MinMax minmax = Max);
  int getEndpointCount();

  // ── Endpoint slack map (histogram data source) ──────────────
  std::vector<EndpointSlack> getEndpointSlacks(MinMax minmax = Max);

  // ── Clock domain info ───────────────────────────────────────
  std::vector<ClockInfo> getClockInfo();

  // ── Timing paths with arc detail ────────────────────────────
  std::vector<TimingPathInfo> getTimingPaths(MinMax minmax = Max,
                                             int max_paths = 100,
                                             float slack_threshold = 1e30);
  /////////////////////////////////////////////////////////////
  // ML / diagnostic feature dump
  /////////////////////////////////////////////////////////////
  // After globalRoute(save_guides=true) + estimate_parasitics, write four
  // CSVs to <prefix>_{nets,segments,sinks,congestion}.csv with per-net,
  // per-segment, per-sink, and per-GCell data. Direct C++ access to
  // grt::GlobalRouter::getRoutes(), sta::Parasitics::piModel/findElmore,
  // sta::Search::vertexSlew, and dbGCellGrid — avoids SWIG limitations.
  void dumpDiagBundle(const std::string& prefix);

  /////////////////////////////////////////////////////////////
  // Functions for LR sizing
  /////////////////////////////////////////////////////////////
  float getLmDelaySum(odb::dbInst* inst, const sta::MinMax *minmax = sta::MinMax::max());
    
  bool checkErcViolations(odb::dbInst* inst, sta::Scene* corner);

  void lmUpdate();

  float averageDelayOnCritPath();
  float getTns(MinMax minmax = Max);
  float getTns(sta::Scene* corner, MinMax minmax = Max);
  // Save/load LM vector to/from binary file for deterministic experiments.
  // The file includes design name and vertex count for validation.
  bool saveLmSnapshot(const char *path);
  bool loadLmSnapshot(const char *path);

  /////////////////////////////////////////////////////////////
  // End functions for LR sizing
  /////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////
  // Functions for testing LR sizing
  ///////////////////////////////////////////////////////////
  // Unified LR entry point — dispatches by cfg.mode.
  void runLr(const lrf::LrConfig &cfg);

  // Python-friendly: construct LrConfig from individual args.
  // mode: 0=RESIZE, 1=RESIZE_BUFFER, 2=PRECHECK, 3=PRECHECK_BUFFER, 4=COMBINED
  void runLr(int mode,
             size_t iterations = 12,
             size_t max_resize_num = 20000000,
             size_t num_no_improve_tolerance = 6,
             float PT_tradeoff = 100.0f,
             float density_weight = 0.0f,
             bool ratcons = false,
             const char *lr_helper_method = "RapidLRHelper",
             float top_ratio = 0.3f,
             const char *checkpoint_dir = "",
             bool debug = false,
             size_t buffering_start_iter = 5,
             float timing_margin = 0.01f,
             bool resize_ff = false,
             bool verbose = false);

  // Standalone Sharma 4-step parallel initializer. Mutates dbSta in place
  // (replaceCell + parasitics+timing resync); call once before runLr / any
  // test* entry so they no longer carry an `initialize` flag.
  void runInitialization(bool minimize_leakage = true);

  // Debug: verify precheck predictions one gate at a time.
  void debugPrecheckAccuracy(float PT_tradeoff = 100.0,
                             float top_ratio = 0.3);

  void testLocalDelayCompute(char *inst_name);
  void testLocalArrivalCompute(char *inst_name);
  void testLocalSlewCompute(char *inst_name);
  void testPtGraphErrors(char* inst_name);
  void testMEEAssignments();
  void testReportVertices();
  // Stage 1 (FF resize verification): dump LM/AAT/RAT/slack on edges
  // adjacent to the named flip-flop instance. Bootstraps a fresh
  // RapidLrHelper, runs `lm_iters` lmUpdate() cycles with RATCONS=true.
  void testReportFFEndpointLMs(char *inst_name, int lm_iters = 3);
  // Stage 2 (FF resize verification): build PtGraph in FF mode for the named
  // FF instance and dump vertices/edges + acceptance summary.
  void testReportFFPtGraph(char *inst_name);
  // Stage 3 (FF resize verification): compare local CK->Q and setup
  // delays against OpenSTA references on the named flip-flop.
  void testFFLocalDelay(char *inst_name);
  void testParallelResizeByArray(size_t max_resize_num,
                                 size_t iterations,
                                 size_t num_no_improve_tolerance,
                                 bool ratcons = false,
                                 float PT_tradeoff = 100.0,
                                 const char *lr_helper_method = "RapidLRHelper",
                                 float density_weight = 0.0f,
                                 bool resize_ff = false);
  void testParallelResizeByArrayWithBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper",
                                              float density_weight = 0.0f);
  void testParallelResizeByArrayWithRszBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper",
                                              float density_weight = 0.0f);
  // LRF slack-DP rebuffering variant (BufferSdpOperator →
  // prepareSlackDpBnet → bufferForTimingSlackDp + recoverLrCost).
  void testParallelResizeByArrayWithSdpBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper",
                                              float density_weight = 0.0f,
                                              float timing_margin = 0.01f);
  // Two-phase: init+resize → precheck-resize+SDP-buffering. See
  // TestLrf::testInitResizeThenSdpBuffering for details.
  // erc_violation_weight: <0 hard reject, ==0 ignore, >0 soft penalty
  //   (default -1.0 = legacy hard-reject behavior).
  // erc_limit_scale: multiplier on the lib slew/cap limits used by ERC
  //   gating (applied to both slew and cap). 0.95 = 5% margin (default,
  //   matches pre-eb01407 legalCheckAfterSwap headroom). 0.85 = 15% margin
  //   (tighter, leaves more room for post-GR RC shift).
  void testInitResizeThenSdpBuffering(size_t max_resize_num,
                                      size_t iterations,
                                      size_t num_no_improve_tolerance,
                                      bool ratcons = false,
                                      float PT_tradeoff = 100.0,
                                      const char *lr_helper_method = "RapidLRHelper",
                                      float density_weight = 0.0f,
                                      float timing_margin = 0.01f,
                                      float erc_violation_weight = -1.0f,
                                      float erc_limit_scale = 0.95f,
                                      bool resize_ff = false);
  void testCombinedResizeBuffering(size_t max_resize_num,
                                             size_t iterations,
                                             size_t num_no_improve_tolerance,
                                             bool ratcons = false,
                                             float PT_tradeoff = 100.0,
                                             const char *lr_helper_method = "RapidLRHelper");
  void testBufferOnly(size_t iterations = 6,
                      float PT_tradeoff = 10.0,
                      const char *lr_helper_method = "RapidLRHelper",
                      float bakoglu_k = 2.5,
                      bool debug = false);
  void testSingleBufferPass(bool use_sdp = false,
                            float PT_tradeoff = 10.0f,
                            const char *lr_helper_method = "RapidLRHelper",
                            size_t lm_warmup_rounds = 5);
  void testParallelResizeByArrayWithPrecheck(size_t max_resize_num,
                                             size_t iterations,
                                             size_t num_no_improve_tolerance,
                                             bool ratcons = false,
                                             float PT_tradeoff = 100.0,
                                             const char *lr_helper_method = "RapidLRHelper",
                                             float top_ratio = 0.3);
  void testParallelResizeByArrayWithPrecheckBuffering(size_t max_resize_num,
                                             size_t iterations,
                                             size_t num_no_improve_tolerance,
                                             bool ratcons = false,
                                             float PT_tradeoff = 100.0,
                                             const char *lr_helper_method = "RapidLRHelper",
                                             float top_ratio = 0.3);
  void testEcoResizeNoHalve(size_t iterations = 12,
                            float PT_tradeoff = 10.0,
                            const char *lr_helper_method = "RapidLRHelper",
                            float halve_factor = 0.5f,
                            bool use_precheck = true);
  void testPrecedingResizeCheck(float PT_tradeoff = 100.0,
                                float top_ratio = 0.3);
  void testParallelKKTProjection(const char *lr_helper_method = "RapidLRHelper");
  void testLocalStaAccuracy(size_t max_steps = 50);
  void testSlewViolationFeasibility();
  void testRepairSlew();
  void testBufferingRsz(float PT_tradeoff = 100.0f, int top_n = 100);
  void probeBufferOneByOne(bool use_rsz = true);
  void probeRszBnet();
  void probeBufferDeep(const char *pin_names_csv);
  void probeAllOptions(const char *pin_name);
  void probeAllOptionsBySensitivity(int top_n);
  /////////////////////////////////////////////////////////////
  // End functions for testing LR sizing
  /////////////////////////////////////////////////////////////

 private:
  sta::dbSta* getSta();
  const sta::MinMax* getMinMax(MinMax type);
  sta::LibertyCell* getLibertyCell(odb::dbMaster* master);
  std::array<sta::Vertex*, 2> vertices(const sta::Pin* pin);
  bool isEndpoint(sta::Pin* sta_pin);
  float getPinSlew(sta::Pin* sta_pin, MinMax minmax);
  float getPinArrival(sta::Pin* sta_pin, RiseFall rf, MinMax minmax);
  float getPinSlack(sta::Pin* sta_pin, RiseFall rf, MinMax minmax);
  float slewAllCorners(sta::Vertex* vertex, const sta::MinMax* minmax);
  std::vector<float> arrivalsClk(const sta::RiseFall* rf,
                                 sta::Clock* clk,
                                 const sta::RiseFall* clk_rf,
                                 sta::Vertex* vertex);
  float getPinArrivalTime(sta::Clock* clk,
                          const sta::RiseFall* clk_rf,
                          sta::Vertex* vertex,
                          const sta::RiseFall* arrive_hold);
  sta::Graph* cmdGraph();
  sta::Network* cmdLinkedNetwork();
  std::pair<odb::dbITerm*, odb::dbBTerm*> staToDBPin(const sta::Pin* pin);
  Design* design_;
};

}  // namespace ord

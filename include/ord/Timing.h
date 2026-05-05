// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2025, The OpenROAD Authors

#pragma once

#include <string>
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
class Corner;
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

  float getNetCap(odb::dbNet* net, sta::Corner* corner, MinMax minmax);
  float getPortCap(odb::dbITerm* pin, sta::Corner* corner, MinMax minmax);
  float getMaxCapLimit(odb::dbMTerm* pin);
  float getMaxSlewLimit(odb::dbMTerm* pin);
  float staticPower(odb::dbInst* inst, sta::Corner* corner);
  float dynamicPower(odb::dbInst* inst, sta::Corner* corner);

  std::vector<odb::dbMTerm*> getTimingFanoutFrom(odb::dbMTerm* input);
  std::vector<sta::Corner*> getCorners();
  sta::Corner* cmdCorner();
  sta::Corner* findCorner(const char* name);

  void makeEquivCells();
  std::vector<odb::dbMaster*> equivCells(odb::dbMaster* master);

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
    
  bool checkErcViolations(odb::dbInst* inst, sta::Corner* corner);

  void lmUpdate();

  float averageDelayOnCritPath();
  float getWorstSlack(MinMax minmax = Max);
  float getTns(MinMax minmax = Max);
  float getTns(sta::Corner* corner, MinMax minmax = Max);
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
             bool initialize = false,
             const char *checkpoint_dir = "",
             bool debug = false,
             size_t buffering_start_iter = 5,
             float timing_margin = 0.01f,
             bool resize_ff = false);

  // Test: level-parallel initializer (standalone, does not start LR).
  void testParallelInitializer(bool minimize_leakage = true);

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
                                 bool initialize = false,
                                 float density_weight = 0.0f,
                                 bool resize_ff = false);
  void testParallelResizeByArrayWithBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper",
                                              bool initialize = false,
                                              float density_weight = 0.0f);
  void testParallelResizeByArrayWithRszBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper",
                                              bool initialize = false,
                                              float density_weight = 0.0f);
  // LRF slack-DP rebuffering variant (BufferSdpOperator →
  // prepareSlackDpBnet → bufferForTimingSlackDp + recoverLrCost).
  void testParallelResizeByArrayWithSdpBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper",
                                              bool initialize = false,
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
                                      bool initialize = false,
                                      float density_weight = 0.0f,
                                      float timing_margin = 0.01f,
                                      float erc_violation_weight = -1.0f,
                                      float erc_limit_scale = 0.95f);
  void testCombinedResizeBuffering(size_t max_resize_num,
                                             size_t iterations,
                                             size_t num_no_improve_tolerance,
                                             bool ratcons = false,
                                             float PT_tradeoff = 100.0,
                                             const char *lr_helper_method = "RapidLRHelper",
                                             bool initialize = false);
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

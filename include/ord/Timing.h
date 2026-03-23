// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2025, The OpenROAD Authors

#pragma once

#include <vector>

#include "sta/Clock.hh"
#include "sta/Graph.hh"
#include "sta/PatternMatch.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"

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
  // Functions for LR sizing
  /////////////////////////////////////////////////////////////
  float getLmDelaySum(odb::dbInst* inst, const sta::MinMax *minmax = sta::MinMax::max());
    
  bool checkErcViolations(odb::dbInst* inst, sta::Corner* corner);

  void lmUpdate();

  float averageDelayOnCritPath();
  float getWorstSlack(MinMax minmax = Max);
  float getTns(MinMax minmax = Max);
  float getTns(sta::Corner* corner, MinMax minmax = Max);
  /////////////////////////////////////////////////////////////
  // End functions for LR sizing
  /////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////
  // Functions for testing LR sizing
  ///////////////////////////////////////////////////////////
  void testLocalDelayCompute(char *inst_name);
  void testLocalArrivalCompute(char *inst_name);
  void testLocalSlewCompute(char *inst_name);
  void testPtGraphErrors(char* inst_name);
  void testParallelVisitor(const std::vector<odb::dbInst*> &inst_names);
  void testMEEAssignments();
  void testParallelResize();
  void testParallelLrResizing(size_t max_resize_num,
                             size_t iterations,
                             size_t num_no_improve_tolerance,
                             bool ratcons = false,
                             float PT_tradeoff = 100.0,
                             const char *lr_helper_method = "LRHelper");
  void testTimingComputeAndWriteBack(const std::vector<odb::dbInst*> &insts);
  void testReportVertices();
  void testBufferInsertion(char *inst_name);
  void testParallelResizingBuffering(size_t max_resize_num,
                                     size_t iterations,
                                     size_t num_no_improve_tolerance,
                                     bool ratcons = false,
                                     float PT_tradeoff = 100.0,
                                     const char *lr_helper_method = "LRHelper");
  void testParallelResizeByArray(size_t max_resize_num,
                                 size_t iterations,
                                 size_t num_no_improve_tolerance,
                                 bool ratcons = false,
                                 float PT_tradeoff = 100.0,
                                 const char *lr_helper_method = "LRHelper");
  void testParallelResizeByArrayWithBuffering(size_t max_resize_num,
                                              size_t iterations,
                                              size_t num_no_improve_tolerance,
                                              bool ratcons = false,
                                              float PT_tradeoff = 100.0,
                                              const char *lr_helper_method = "RapidLRHelper");
  void testCombinedResizeBuffering(size_t max_resize_num,
                                             size_t iterations,
                                             size_t num_no_improve_tolerance,
                                             bool ratcons = false,
                                             float PT_tradeoff = 100.0,
                                             const char *lr_helper_method = "RapidLRHelper");
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
  void testPrecedingResizeCheck(float PT_tradeoff = 100.0,
                                float top_ratio = 0.3);
  void testSingleInstBuffering(char *inst_name);
  void testParallelKKTProjection(const char *lr_helper_method = "RapidLRHelper");
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

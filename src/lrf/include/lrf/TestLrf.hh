#pragma once

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "lrf/LrfClass.hh"

#include <stdexcept>

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

class  TestLrf
{
public:
  void testLocalDelayCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalArrivalCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalRequiredCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testLocalSlewCompute(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testDifferenceBetweenLocalAndOpen(char *inst_name, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelVisitor(const std::vector<odb::dbInst*>& db_insts, sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testMEEAssignments(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelResize(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testReportVertices(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block);

  void testParallelLrResizing(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, size_t thread_num, 
    size_t max_resize_num, size_t iterations, size_t num_no_improve_tolerance, bool ratcons = false,
    float PT_tradeoff = 100.0, std::string lr_helper_method = "LRHelper");

  void testTimingComputeAndWriteBack(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, const std::vector<odb::dbInst*> &insts);

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
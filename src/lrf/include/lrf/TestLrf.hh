#pragma once

#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"

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

protected:
  void printSlewComparison(char *inst_name, sta::dbSta* sta, 
                       LocalSta *local_sta, odb::dbInst *db_inst, 
                       sta::Instance *sta_inst, sta::dbNetwork *db_network);
  void printLocalDelaysAndCap(char *inst_name, sta::dbSta* sta, 
                       LocalSta *local_sta, odb::dbInst *db_inst, 
                       sta::Instance *sta_inst, sta::dbNetwork *db_network);
  void comparePtGraphs(PtGraph *local_pt_graph, PtGraph *open_pt_graph, sta::dbSta* sta);

  std::vector<ErrorPoint> error_points_;
};

}  // namespace lrf
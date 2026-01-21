#pragma once

#include "sta/VertexVisitor.hh"
#include "sta/NetworkClass.hh"
#include "sta/GraphClass.hh"
#include "sta/Delay.hh"
#include "LibertyClass.hh"
#include "lrf/LrfClass.hh"

#include <unordered_map>

namespace sta {
  class ArcDelayCalc;
  class dbSta;
}

namespace rsz {
  class Resizer;
}

namespace lrf {

class LocalSta;
class PtGraph;

// We don't want to copy this visitor, each swap examination should
// have its own instance.
class ParallelLrVisitor
{
public:
  ParallelLrVisitor(sta::dbSta *db_sta, LocalSta *local_sta);
  virtual ~ParallelLrVisitor();
  virtual bool visit(sta::Instance *inst);
  bool visit(sta::Instance *inst, TimingRecord &timing_record);
  // Apply cell type changes to OpenROAD and OpenSTA, and 
  // update timing information from PtGraph to sta::Graph.
  virtual void applyChangesToDb(rsz::Resizer *resizer);
  virtual void updateTimingFromPtGraph();
  void updateVertexInfo(sta::VertexId vertex_id);
  void updateEdgeInfo(sta::EdgeId edge_id);

  virtual ParallelLrVisitor *copy() const;
  bool checkVisitorStatus() const;
  void operator()(sta::Instance *inst) { visit(inst); }
  void printVisitedInstNames() const;
  PtGraph *ptGraph() const { return pt_graph_; }
  sta::Instance *refInst() const { return ref_inst_; }
  sta::LibertyCell *bestCell() const { return best_cell_; }
  void init(float averge_delay, float average_power, float wns,
    std::unordered_map<sta::LibertyCell*, sta::LibertyCellSeq*> *cache,
    std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map);
  void setAverageDelay(float avg_delay) { average_delay_ = avg_delay; }
  void setAverageLeakage(float avg_leakage) { average_leakage_ = avg_leakage; }
  void setSwappableCellsCache(std::unordered_map<sta::LibertyCell*, sta::LibertyCellSeq*> *cache)
  {
    swappable_cells_cache_ = cache;
  }
  void setInstInfoMap(std::unordered_map<sta::Instance*, LocalCellInfo*> *map)
  {
    inst_info_map_ = map;
  }

protected:  
  void recordGraphTimingFromPtGraphPara(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing);
  float swapCost(float delay_lm_sum, float power);

  sta::dbSta *db_sta_;
  sta::Instance *ref_inst_;
  LocalSta *local_sta_;
  PtGraph *pt_graph_;
  sta::ArcDelayCalc *arc_delay_calc_;
  sta::Slack slack_before_swap_;
  std::vector<std::string> visited_instances_;
  sta::LibertyCell *best_cell_ = nullptr;
  float average_delay_ = 1.0;
  float average_leakage_ = 1.0;
  float slack_margin_= 0.0;
  std::unordered_map<sta::LibertyCell*, sta::LibertyCellSeq*> *swappable_cells_cache_ = nullptr;
  std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map_;
};





}  // namespace lrf


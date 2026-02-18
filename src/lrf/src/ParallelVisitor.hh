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
  class BufferedNet;
}

namespace lrf {

class LocalSta;
class PtGraph;
class ParallelLibData;
class LrRebuffer;

enum class MoveType {
  Resizing,
  BufferInsertion
};

// We don't want to copy this visitor, each swap examination should
// have its own instance.
class ParallelLrVisitor
{
public:
  ParallelLrVisitor(sta::dbSta *db_sta, LocalSta *local_sta, rsz::Resizer *resizer);
  virtual ~ParallelLrVisitor();
  virtual bool visit(sta::Instance *inst, MoveType move_type = MoveType::Resizing);
  bool visit(sta::Instance *inst, TimingRecord &timing_record);
  bool singleGateSizing(sta::Instance *inst);
  // Functions for buffer insertion
  bool tryBuffering(sta::Instance *inst);
  // Apply cell type changes to OpenROAD and OpenSTA, and 
  // update timing information from PtGraph to sta::Graph.
  virtual void applyChangesToDb(rsz::Resizer *resizer, MoveType move_type = MoveType::Resizing);
  void applyResizeChangesToDb(rsz::Resizer *resizer);
  void applyBufferingChangesToDb(rsz::Resizer *resizer);
  virtual void updateTimingFromPtGraph();
  void updateVertexInfo(sta::VertexId vertex_id);
  void updateEdgeInfo(sta::EdgeId edge_id);

  virtual ParallelLrVisitor *copy() const;
  bool checkVisitorStatus() const;
  void operator()(sta::Instance *inst) { visit(inst); }
  void printVisitedInstNames() const;
  PtGraph *ptGraph() const { return pt_graph_; }
  void setPtGraph(PtGraph *pt_graph) { pt_graph_ = pt_graph; }
  sta::Instance *refInst() const { return ref_inst_; }
  sta::LibertyCell *bestCell() const { return best_cell_; }
  void init(float averge_delay, float average_power, float wns, 
    float PT_tradeoff,
    std::unordered_map<sta::LibertyCell*, sta::LibertyCellSeq*> *cache,
    std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map);
  void init(float averge_delay, float average_power, float wns, 
    float PT_tradeoff, ParallelLibData *parallel_lib_data);

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
  void setPTTradeoff(float PT_tradeoff) { PT_tradeoff_ = PT_tradeoff; }
  void setSlackMargin(float slack_margin) { slack_margin_ = slack_margin; }
  bool equivVtCells(sta::LibertyCell *cell1, sta::LibertyCell *cell2);
  void setClockPeriod(float clock_period) { clock_period_ = clock_period; }
  void setParallelLibData(ParallelLibData *parallel_lib_data) { parallel_lib_data_ = parallel_lib_data; }

  // Functions for runtime profiling
  void printRuntimeProfile() const;

protected:  
  // Function for testing purpose
  void recordGraphTimingFromPtGraphPara(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing, bool verbose = false);

  // Function of paralllel gate sizing
  float swapCost(float delay_lm_sum, float power);
  
  bool trySwap(sta::Instance *inst);
  bool trySwapV1(sta::Instance *inst);
  std::vector<std::pair<sta::LibertyCell*, std::pair<size_t, size_t>>> getLegalEquivCells(
                                  std::vector<sta::LibertyCellSeq> *equiv_cells_vec,
                                  sta::LibertyCell *ori_cell);
  LocalSta *localSta() const { return local_sta_; }
  sta::ArcDelayCalc *arcDelayCalc() const { return arc_delay_calc_; }

  sta::dbSta *db_sta_;
  sta::Instance *ref_inst_;
  LocalSta *local_sta_;
  rsz::Resizer *resizer_;
  PtGraph *pt_graph_;
  sta::ArcDelayCalc *arc_delay_calc_;
  sta::Slack slack_before_swap_;
  std::vector<std::string> visited_instances_;
  sta::LibertyCell *best_cell_ = nullptr;
  float average_delay_ = 1.0;
  float average_leakage_ = 1.0;
  float slack_margin_= 0.0;
  float PT_tradeoff_ = 100.0;
  std::unordered_map<sta::LibertyCell*, sta::LibertyCellSeq*> *swappable_cells_cache_ = nullptr;
  std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map_;
  ParallelLibData *parallel_lib_data_ = nullptr;
  float clock_period_ = 0.0;
  std::unique_ptr<LrRebuffer> rebuffer_ = nullptr;

  std::map<std::string, double> runtime_map_ = {
    {"visit", 0.0},
    {"pt_graph_construction", 0.0},
    {"equiv_cell_check", 0.0},
    {"equiv_cell_count", 0.0},
    {"swap", 0.0},
    {"writeTimingToDb", 0.0},
    {"applyDb", 0.0},
    {"single_gate_sizing", 0.0},
    {"buffer_insertion", 0.0},
    {"buffer_count", 0.0},
  };
private:
  friend class LrRebuffer;
};





}  // namespace lrf


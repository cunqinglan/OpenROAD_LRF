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

// We don't want to copy this visitor, each swap examination should
// have its own instance.
class ParallelLrVisitor
{
public:
  ParallelLrVisitor(sta::dbSta *db_sta, LocalSta *local_sta, rsz::Resizer *resizer);
  virtual ~ParallelLrVisitor();
  virtual bool visit(sta::Instance *inst, sta::VertexId vid);
  bool visit(sta::Instance *inst, TimingRecord &timing_record);
  bool singleGateSizing(sta::Instance *inst);
  // Lightweight pass: build PtGraph, run delay/arrival/required, write back
  // slew+paths without cell evaluation. Used for non-selected instances
  // in precheck mode to keep timing propagation fresh.
  void visitSlewOnly(sta::Instance *inst);
  void setMoveType(MoveType move_type);
  
  // Apply cell type changes to OpenROAD and OpenSTA, and 
  // update timing information from PtGraph to sta::Graph.
  virtual void applyChangesToDb(rsz::Resizer *resizer);
  void applyResizeChangesToDb(rsz::Resizer *resizer);
  void applyBufferingChangesToDb(rsz::Resizer *resizer);
  virtual void updateTimingFromPtGraph();
  void updateVertexInfo(sta::VertexId vertex_id);
  void updateEdgeInfo(sta::EdgeId edge_id);

  virtual ParallelLrVisitor *copy() const;
  bool checkVisitorStatus() const;
  void operator()(sta::Instance *inst) { visit(inst, sta::object_id_null); }
  void printVisitedInstNames() const;
  PtGraph *ptGraph() const { return pt_graph_; }
  LrRebuffer *rebuffer() const { return rebuffer_; }
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
  float slackMargin() const { return slack_margin_; }
  bool equivVtCells(sta::LibertyCell *cell1, sta::LibertyCell *cell2);
  void setClockPeriod(float clock_period) { clock_period_ = clock_period; }
  void setParallelLibData(ParallelLibData *parallel_lib_data) { parallel_lib_data_ = parallel_lib_data; }
  void setEquivCellArray(LibertyCellArray *array, PosMap *pos_map) {
    equiv_cell_array_ = array;
    equiv_cell_pos_map_ = pos_map;
  }

  // Functions for runtime profiling
  void printRuntimeProfile() const;

  // Lightweight precheck: evaluate small neighborhood, return cost change without applying.
  float trySwapPrecheck(sta::Instance *inst, int col_padding = 1, int row_padding = 1);

protected:
  // Function for testing purpose
  void recordGraphTimingFromPtGraphPara(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing, bool verbose = false);

  // Function of paralllel gate sizing
  float swapCost(float delay_lm_sum, float power);

  // Replace the given instance with equivalent cells to evaluate and improve timing/power; returns true on success.
  bool trySwap(sta::Instance *inst);
  // Alternate implementation of trySwap (version 1) using a different strategy; returns true on success.
  bool trySwapV1(sta::Instance *inst);
  // Neighborhood search in equiv cell array for resizing; returns true on success.
  bool trySwapByArray(sta::Instance *inst, int col_padding = 3, int row_padding = 1);
  // Insert buffering for the given instance to improve timing; returns true on success.
  bool tryBuffering(sta::Instance *inst);
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
  std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map_ = nullptr;
  ParallelLibData *parallel_lib_data_ = nullptr;
  LibertyCellArray *equiv_cell_array_ = nullptr;
  PosMap *equiv_cell_pos_map_ = nullptr;
  float clock_period_ = 0.0;
  LrRebuffer *rebuffer_ = nullptr;
  MoveType move_type_ = MoveType::Resizing;

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
    {"precheck", 0.0},
    {"rebuffer_total", 0.0},
    {"rebuffer_setup", 0.0},
    {"rebuffer_coarse", 0.0},
    {"rebuffer_precise", 0.0},
    {"rebuffer_pin_count", 0.0}
  };
private:
  friend class LrRebuffer;
  friend class TestLrf;
};

// Visitor for embarrassingly-parallel precheck: evaluates resize benefit
// per instance without modifying the database. Stores results in an
// externally-owned vector indexed by TaskArranger vertex ID.
class PrecheckVisitor : public ParallelLrVisitor
{
public:
  PrecheckVisitor(sta::dbSta *db_sta, LocalSta *local_sta, rsz::Resizer *resizer,
                  std::vector<ResizeBenefit> *results);

  bool visit(sta::Instance *inst, sta::VertexId vid) override;
  void applyChangesToDb(rsz::Resizer *resizer) override {}
  ParallelLrVisitor *copy() const override;

private:
  std::vector<ResizeBenefit> *results_;
};

// Visitor for embarrassingly-parallel buffer sensitivity precheck:
// evaluates buffer insertion benefit per instance without modifying the database.
class BufferSensitivityVisitor : public ParallelLrVisitor
{
public:
  BufferSensitivityVisitor(sta::dbSta *db_sta, LocalSta *local_sta, rsz::Resizer *resizer,
                           std::vector<ResizeBenefit> *results);

  bool visit(sta::Instance *inst, sta::VertexId vid) override;
  void applyChangesToDb(rsz::Resizer *resizer) override {}
  ParallelLrVisitor *copy() const override;

private:
  std::vector<ResizeBenefit> *results_;
};

}  // namespace lrf


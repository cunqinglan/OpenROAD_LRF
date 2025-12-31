#pragma once

#include "sta/VertexVisitor.hh"
#include "sta/NetworkClass.hh"
#include "sta/GraphClass.hh"
#include "sta/Delay.hh"
#include "LibertyClass.hh"

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
  virtual void visit(sta::Instance *inst);
  // Apply cell type changes to OpenROAD and OpenSTA, and 
  // update timing information from PtGraph to sta::Graph.
  virtual void applyChangesToDb(rsz::Resizer *resizer);
  virtual void updateTimingFromPtGraph();
  void updateVertexInfo(sta::VertexId vertex_id);
  void updateEdgeInfo(sta::EdgeId edge_id);

  virtual ParallelLrVisitor *copy() const;
  void operator()(sta::Instance *inst) { visit(inst); }
  void printVisitedInstNames() const;
  PtGraph *ptGraph() const { return pt_graph_; }
  sta::Instance *refInst() const { return ref_inst_; }
  sta::LibertyCell *bestCell() const { return best_cell_; }

protected:  
  sta::dbSta *db_sta_;
  sta::Instance *ref_inst_;
  LocalSta *local_sta_;
  PtGraph *pt_graph_;
  sta::ArcDelayCalc *arc_delay_calc_;
  sta::Slack slack_before_swap_;
  std::vector<std::string> visited_instances_;
  sta::LibertyCell *best_cell_ = nullptr;
};





}  // namespace lrf


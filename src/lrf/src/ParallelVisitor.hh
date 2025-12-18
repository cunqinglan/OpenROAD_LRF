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
  ParallelLrVisitor(sta::dbSta *db_sta, LocalSta *local_sta,
                   rsz::Resizer *resizer);
  virtual ~ParallelLrVisitor();
  virtual void visit(sta::Instance *inst);
  virtual ParallelLrVisitor *copy() const;
  void operator()(sta::Instance *inst) { visit(inst); }

protected:  
  sta::dbSta *db_sta_;
  sta::Instance *ref_inst_;
  LocalSta *local_sta_;
  PtGraph *pt_graph_;
  sta::ArcDelayCalc *arc_delay_calc_;
  rsz::Resizer *resizer_;
  sta::Slack slack_before_swap_;
};





}  // namespace lrf


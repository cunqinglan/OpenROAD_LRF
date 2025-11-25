#pragma once

#include "sta/VertexVisitor.hh"

namespace sta {
  class ArcDelayCalc;
  class dbSta;
}

namespace rsz {
  class Resizer;
}

namespace lrf {

using namespace sta;

class LocalSta;
class PtGraph;

class ParallelLrVisitor : public VertexVisitor
{
public:
  ParallelLrVisitor(dbSta *db_sta, LocalSta *local_sta);
  virtual ~ParallelLrVisitor();
  virtual void visit(Instance *inst);
  virtual VertexVisitor *copy() const;

protected:  
  dbSta *db_sta_;
  LocalSta *local_sta_;
  ArcDelayCalc *arc_delay_calc_;
  rsz::Resizer *resizer_;
};





}  // namespace lrf


#pragma once


#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/FuncExpr.hh"
#include "sta/MinMax.hh"
#include "sta/StaState.hh"
#include "utl/Logger.h"

namespace sta {
class Vertex;
class Instance;
}

namespace lrf {

class LRHelper;
class ParallelVisitor;

class LrSizer: public sta::dbStaState
{
public:
  LrSizer(sta::dbSta* sta, LRHelper* lr_helper,
    ParallelVisitor *visitor);
  ~LrSizer();

  void criticalPathSizing();
  bool repairCriticalPath(sta::Vertex* end);
  bool sizeCriticalPathGates(sta::Path* path);
  bool singleGateSizing(sta::Instance* inst, ParallelVisitor *visitor);
protected:
  sta::InstanceSeq topo_sorted_instances_;

private:
  const sta::MinMax* min_ = sta::MinMax::min();
  const sta::MinMax* max_ = sta::MinMax::max();
  float para_tsh_discount_ = 0.1;
  LRHelper *lr_helper_;
  ParallelVisitor *visitor_ = nullptr;
};

} // namespace lrf

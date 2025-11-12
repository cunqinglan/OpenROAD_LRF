#pragma once


#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/FuncExpr.hh"
#include "sta/MinMax.hh"
#include "sta/StaState.hh"
#include "utl/Logger.h"
#include "lrf/LrfMgr.hh"



namespace lrf {


class LrSizer: public std::dbStaState
{
public:
  LrSizer();
  ~LrSizer();

  void TopoSort();
  void collectInstancesInWindow(Instance* inst);
  void collectInstancesInWindow(dbInst* db_inst);

  float delayLambdaSum(EdgeSet& edges);

protected:
  InstanceSeq topo_sorted_instances_;

private:
  const MinMax* min_ = MinMax::min();
  const MinMax* max_ = MinMax::max();

  LrfMgr* lrf_mgr_ = nullptr;
};

} // namespace lrf

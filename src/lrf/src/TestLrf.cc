#pragma once

#include "db_sta/dbSta.hh"
#include "sta/ArcDelayCalc.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "LocalSta.hh"
#include "LRHelper.hh"
// #include 

namespace lrf
{

void 
testLocalDelayCompute(rsz::Resizer *resizer);
{
  sta::dbSta* sta = getSta();
  IncreSta *sta = new IncreSta(sta);
  LocalSta *local_sta = sta->localSta();
  LRHelper *lrf_helper = sta->lrfHelper();
  Network *network = sta->network();
  resizer->makeEquivCells();

  LeafInstanceIterator *inst_iter = network->leafInstanceIterator();
  while (inst_iter->hasNext()) {
    Instance *inst = inst_iter->next();
    if (network->name(inst) == "g1") {
      printf("Collecting local graph for instance %s\n", network->name(inst));
      PtGraph *pt_graph = local_sta->makePtGraph(inst, true);
      printf("Local graph for instance %s created with %s \n", 
             network->name(inst), 
             pt_graph->to_string().c_str());
      sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
      pt_graph->printDelays();
      local_sta->findLocalDelays(pt_graph, arc_delay_calc);
      pt_graph->printDelays();
      
      auto equiv_cells = resizer->getSwappableCells();
      for (auto *lib_cell : equiv_cells) {
        printf("Swapping to equiv cell: %s\n", lib_cell->name());
        LibertyCell *new_cell = lib_cell;
        local_sta->virtualSwapCell(pt_graph, inst, new_cell);
        local_sta->findLocalDelays(pt_graph, arc_delay_calc);
        pt_graph->printDelays();
      }
      delete arc_delay_calc;
    }
  }
  delete inst_iter;
  delete sta;
}

}  namespace lrf
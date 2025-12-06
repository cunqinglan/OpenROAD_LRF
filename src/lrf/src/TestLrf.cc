
#include "db_sta/dbSta.hh"
#include "sta/ArcDelayCalc.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "LocalSta.hh"
#include "LrHelper.hh"
#include "lrf/IncreSta.hh"
#include "lrf/TestLrf.hh"
#include "odb/db.h"
#include "sta/Liberty.hh"
// #include 

namespace lrf
{

void 
TestLrf::testLocalDelayCompute(char *inst_name, sta::dbSta* sta, 
                               rsz::Resizer *resizer, odb::dbBlock *block)
{
  // Test local delay computation and cell swapping of given instance.
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  LRHelper *lrf_helper = incre_sta->lrHelper();
  sta::dbNetwork *db_network = sta->getDbNetwork();
  resizer->makeEquivCells();

  // LeafInstanceIterator *inst_iter = db_network->leafInstanceIterator();
  // while (inst_iter->hasNext()) {
  //   Instance *inst = inst_iter->next();
  //   if (strcmp(db_network->name(inst), inst_name) == 0) {
  //     printf("Collecting local graph for instance %s\n", db_network->name(inst));
  //     PtGraph *pt_graph = local_sta->makePtGraph(inst, true);
  //     printf("Local graph for instance %s created with %s \n", 
  //            db_network->name(inst), 
  //            pt_graph->to_string().c_str());
  //     sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  //     pt_graph->printDelays();
  //     local_sta->findLocalDelays(pt_graph, arc_delay_calc);
  //     pt_graph->printDelays();
      
  //     LibertyCell *orig_cell = sta->network()->libertyCell(inst);
  //     LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
  //     for (auto *lib_cell : *equiv_cells) {
  //       printf("Swapping to equiv cell: %s\n", lib_cell->name());
  //       LibertyCell *new_cell = lib_cell;
  //       local_sta->virtualSwapCell(pt_graph, inst, new_cell);
  //       local_sta->findLocalDelays(pt_graph, arc_delay_calc);
  //       pt_graph->printDelays();
  //     }
  //     delete arc_delay_calc;
  //   }
  // }
  // delete inst_iter;
  // delete incre_sta;

  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("Instance %s not found in the block.\n", inst_name);
    return;
  }

  sta::Instance *sta_inst = db_network->dbToSta(db_inst);

  {
    // In this domain, we test the functionality of Local delay computation
    printf("----- Testing Local Delay Computation for instance %s -----\n", inst_name);
    printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
    PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, true);
    printf("Local graph for instance %s created with %s \n", 
            db_network->name(sta_inst), 
            pt_graph->to_string().c_str());
    sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
    printf("Printing delays computed by LocalSta for instance %s\n", 
            db_network->name(sta_inst));
    pt_graph->printDelays();
    // local_sta->findLocalDelays(pt_graph, arc_delay_calc);
    // pt_graph->printDelays();
    
    sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
    sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
    int swap_count = 0;
    for (auto *lib_cell : *equiv_cells) {
      if (swap_count++ > 1) break; // Limit number of swaps for testing
      printf("LocalSTA: Swapping to equiv cell: %s\n", lib_cell->name());
      sta::LibertyCell *new_cell = lib_cell;
      local_sta->virtualSwapCell(pt_graph, sta_inst, new_cell);
      local_sta->findLocalDelays(pt_graph, arc_delay_calc);
      pt_graph->printDelays();

      // Also test dbMaster swap
      printf("OpenSTA: Swapping to equiv cell: %s\n", lib_cell->name());
      odb::dbMaster *master = db_network->staToDb(lib_cell);
      db_inst->swapMaster(master);
      sta->updateTiming(false);
      PtGraph *pt_graph_sta = local_sta->makePtGraph(sta_inst, false);
      pt_graph_sta->printDelays();
    }
    delete arc_delay_calc;
  }

  {
    // printf("----- Testing OpenSTA Delay Computation for instance %s -----\n", inst_name);
    // printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
    // fflush(stdout);
    // // Printf original local graph delay
    // PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, false);
    // printf("Printing delays computed by OpenSTA for instance %s\n", 
    //         db_network->name(sta_inst));
    // pt_graph->printDelays();

    // sta::LibertyCell *orig_cell = db_network->libertyCell(db_inst);
    // sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
    // int swap_count = 0;
    // for (auto *lib_cell : *equiv_cells) {
    //   if (swap_count++ > 2) break; // Limit number of swaps for testing
    //   printf("Swapping to equiv cell: %s\n", lib_cell->name());
      
    // }
  }
}

}  // namespace lrf
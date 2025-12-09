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
// #include "lrf/PtGraph.hh"
// #include 

namespace lrf
{

void
TestLrf::printLocalDelaysAndCap(char *inst_name, sta::dbSta* sta, 
                       LocalSta *local_sta, odb::dbInst *db_inst, 
                       sta::Instance *sta_inst, sta::dbNetwork *db_network)
{
  // In this domain, we test the functionality of Local delay computation
  printf("----- Testing Local Delay Computation for instance %s -----\n", inst_name);
  printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
  PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, true);
  printf("Local graph for instance %s created with %s \n", 
          db_network->name(sta_inst), 
          "");
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  printf("Printing delays computed by LocalSta for instance %s\n", 
          db_network->name(sta_inst));
  // pt_graph->printDelays();
  // local_sta->findLocalDelays(pt_graph, arc_delay_calc);
  // pt_graph->printDelays();
  
  sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
  sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
  int swap_count = 0;
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for testing
    printf("LocalSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    sta::LibertyCell *new_cell = lib_cell;
    local_sta->virtualSwapCell(pt_graph, sta_inst, new_cell);
    local_sta->findLocalDelays(pt_graph, arc_delay_calc);
    pt_graph->printDelays();
    local_sta->printLocalParasitics(pt_graph);
  }
  delete arc_delay_calc;
  
  swap_count = 0;
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for 
    // Also test dbMaster swap
    printf("OpenSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    odb::dbMaster *master = db_network->staToDb(lib_cell);
    db_inst->swapMaster(master);
    sta->updateTiming(false);
    PtGraph *pt_graph_sta = local_sta->makePtGraph(sta_inst, true);
    pt_graph_sta->printDelays();
    local_sta->printParasitics(pt_graph_sta);
  }
}

void 
TestLrf::testLocalDelayCompute(char *inst_name, sta::dbSta* sta, 
                               rsz::Resizer *resizer, odb::dbBlock *block)
{
  // Test local delay computation and cell swapping of given instance.
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  // LRHelper *lrf_helper = incre_sta->lrHelper();
  sta::dbNetwork *db_network = sta->getDbNetwork();
  resizer->makeEquivCells();

  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("Instance %s not found in the block.\n", inst_name);
    return;
  }

  sta::Instance *sta_inst = db_network->dbToSta(db_inst);

  printLocalDelaysAndCap(inst_name, sta, local_sta, db_inst, sta_inst, db_network);

}

void
TestLrf::testLocalArrivalCompute(char *inst_name, sta::dbSta* sta, 
                               rsz::Resizer *resizer, odb::dbBlock *block)
{
  // Test local arrival computation of given instance.
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  // LRHelper *lrf_helper = incre_sta->lrHelper();
  sta::dbNetwork *db_network = sta->getDbNetwork();
  resizer->makeEquivCells();

  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("Instance %s not found in the block.\n", inst_name);
    return;
  }

  sta::Instance *sta_inst = db_network->dbToSta(db_inst);

  printf("----- Testing Local Arrival Computation for instance %s -----\n", inst_name);
  printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
  PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, true);
  printf("Local graph for instance %s created with %s \n", 
          db_network->name(sta_inst), 
          pt_graph->to_string().c_str());
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  printf("Printing arrivals computed by LocalSta for instance %s\n", 
          db_network->name(sta_inst));
  // pt_graph->printDelays();
  local_sta->findLocalDelays(pt_graph, arc_delay_calc);
  // pt_graph->printDelays();
  printf("Original Local Arrivals:\n");
  local_sta->printLocalArrivals(pt_graph);
  local_sta->findLocalArrivals(pt_graph);
  local_sta->printLocalArrivals(pt_graph);
}

void
TestLrf::printSlewComparison(char *inst_name, sta::dbSta* sta, 
                       LocalSta *local_sta, odb::dbInst *db_inst, 
                       sta::Instance *sta_inst, sta::dbNetwork *db_network)
{
  // In this domain, we test the functionality of Local delay computation
  printf("----- Testing Local Slew Computation for instance %s -----\n", inst_name);
  printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
  PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, true);
  printf("Local graph for instance %s created with %s \n", 
          db_network->name(sta_inst), 
          "");
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  printf("Printing delays computed by LocalSta for instance %s\n", 
          db_network->name(sta_inst));
  // pt_graph->printDelays();
  // local_sta->findLocalDelays(pt_graph, arc_delay_calc);
  // pt_graph->printDelays();
  local_sta->printLocalSlews(pt_graph);
  
  sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
  sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
  int swap_count = 0;
  local_sta->debug_info_.clear();
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for testing
    printf("LocalSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    sta::LibertyCell *new_cell = lib_cell;
    local_sta->virtualSwapCell(pt_graph, sta_inst, new_cell);
    local_sta->findLocalDelays(pt_graph, arc_delay_calc);
    local_sta->printLocalSlews(pt_graph);
    for (auto &info : local_sta->debug_info_) {
      printf("%s", info.c_str());
      fflush(stdout);
    }
    local_sta->debug_info_.clear();
  }
  delete arc_delay_calc;
  
  swap_count = 0;
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for 
    // Also test dbMaster swap
    printf("OpenSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    odb::dbMaster *master = db_network->staToDb(lib_cell);
    // Print global debug info from GraphDelayCalc
    sta::GraphDelayCalc *global_dcalc = sta->graphDelayCalc();
    global_dcalc->debug_info_.clear();
    db_inst->swapMaster(master);
    sta->updateTiming(false);

    for (auto &info : global_dcalc->debug_info_) {
      printf("%s", info.c_str());
      fflush(stdout);
    }
    global_dcalc->debug_info_.clear();

    PtGraph *pt_graph_sta = local_sta->makePtGraph(sta_inst, true);
    
    local_sta->printLocalSlews(pt_graph_sta);
  }
}

void 
TestLrf::testLocalSlewCompute(char *inst_name, sta::dbSta* sta, 
                               rsz::Resizer *resizer, odb::dbBlock *block)
{
  // Test local delay computation and cell swapping of given instance.
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  // LRHelper *lrf_helper = incre_sta->lrHelper();
  sta::dbNetwork *db_network = sta->getDbNetwork();
  resizer->makeEquivCells();

  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("Instance %s not found in the block.\n", inst_name);
    return;
  }

  sta::Instance *sta_inst = db_network->dbToSta(db_inst);

  printSlewComparison(inst_name, sta, local_sta, db_inst, sta_inst, db_network);
}

}  // namespace lrf
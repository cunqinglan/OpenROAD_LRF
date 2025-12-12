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
#include "sta/Corner.hh"
#include "LocalSearch.hh"
#include "PtGraph.hh"

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
    local_sta->virtualReplaceCell(pt_graph, new_cell);
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
  sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
  sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
  int swap_count = 0;
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for testing
    printf("LocalSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    local_sta->virtualReplaceCell(pt_graph, lib_cell);
    local_sta->findLocalDelays(pt_graph, arc_delay_calc);
    local_sta->findLocalArrivals(pt_graph);
    local_sta->printLocalArrivals(pt_graph);
  }
  delete arc_delay_calc;

  swap_count = 0;
  local_sta->setDebugLabel("OpenSTA");
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for 
    // Also test dbMaster swap
    printf("OpenSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    odb::dbMaster *master = db_network->staToDb(lib_cell);
    db_inst->swapMaster(master);
    sta->updateTiming(false);
    PtGraph *pt_graph_sta = local_sta->makePtGraph(sta_inst, true);
    local_sta->printLocalArrivals(pt_graph_sta);
  }
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
    local_sta->virtualReplaceCell(pt_graph, new_cell);
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

void 
TestLrf::testDifferenceBetweenLocalAndOpen(char *inst_name, sta::dbSta* sta, 
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

  printf("----- Testing Difference Between Local and OpenSTA for instance %s -----\n", inst_name);
  printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
  
  sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
  sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
  sta::LibertyCell *swap_to_cell = (*equiv_cells)[0];
  sta::LibertyCell *swap_to_cell1 = (*equiv_cells)[1];
  printf("Swapping to equiv cell: %s from %s\n", swap_to_cell->name(), orig_cell->name());
  // Vitually replace cell in LocalSta and compute delays and arrivals
  PtGraph *pt_graph_local = local_sta->makePtGraph(sta_inst, true);
  local_sta->virtualReplaceCell(pt_graph_local, swap_to_cell);
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  local_sta->findLocalDelays(pt_graph_local, arc_delay_calc);
  local_sta->setDebugLabel("LocalSTA");
  local_sta->findLocalArrivals(pt_graph_local);

  // Now swap in OpenSTA and compute PtGraph
  odb::dbMaster *to_master = db_network->staToDb(swap_to_cell);
  db_inst->swapMaster(to_master);
  sta->updateTiming(false);
  PtGraph *pt_graph_open = local_sta->makePtGraph(sta_inst, true);
  local_sta->setDebugLabel("OpenSTA");
  local_sta->findLocalArrivals(pt_graph_open);

  odb::dbMaster *from_master = db_network->staToDb(orig_cell);
  db_inst->swapMaster(from_master);
  sta->updateTiming(false);
  

  comparePtGraphs(pt_graph_local, pt_graph_open, sta);

  // local_sta->virtualReplaceCell(pt_graph_local, swap_to_cell1);
  // local_sta->findLocalDelays(pt_graph_local, arc_delay_calc);
  // local_sta->findLocalArrivals(pt_graph_local);

  // odb::dbMaster *to_master1 = db_network->staToDb(swap_to_cell1);
  // db_inst->swapMaster(to_master1);
  // sta->updateTiming(false);
  // PtGraph *pt_graph_orig = local_sta->makePtGraph(sta_inst, true);
  // local_sta->findLocalArrivals(pt_graph_orig);

  // comparePtGraphs(pt_graph_orig, pt_graph_local, sta);
}

void 
TestLrf::comparePtGraphs(PtGraph *local_pt_graph, PtGraph *open_pt_graph, sta::dbSta* sta)
{
  // Compare delays and arrivals between two PtGraphs
  printf("Comparing PtGraphs between LocalSta and OpenSTA\n");
  // Compare delays
  size_t local_edge_count = local_pt_graph->ptEdges().size();
  size_t open_edge_count = open_pt_graph->ptEdges().size();
  if (local_edge_count != open_edge_count) {
    printf("Edge count mismatch: LocalSta has %zu edges, OpenSTA has %zu edges\n", 
            local_edge_count, open_edge_count);
            fflush(stdout);
    return;
  }
  size_t local_vertex_count = local_pt_graph->ptVertices().size();
  size_t open_vertex_count = open_pt_graph->ptVertices().size();
  if (local_vertex_count != open_vertex_count) {
    printf("Vertex count mismatch: LocalSta has %zu vertices, OpenSTA has %zu vertices\n", 
            local_vertex_count, open_vertex_count);
            fflush(stdout);
    return;
  }
  // Compare delays on edges
  for (size_t i = 1; i < local_edge_count; ++i) {
    const PtEdge &local_edge = local_pt_graph->ptEdges()[i];
    const PtEdge &open_edge = open_pt_graph->ptEdges()[i];
    const sta::Edge *local_edge_obj = local_edge.edge();
    const sta::Edge *open_edge_obj = open_edge.edge();
    if (local_edge_obj != open_edge_obj) {
      printf("Edge mismatch at index %zu\n", i);
      fflush(stdout);
      continue;
    }
    sta::TimingArcSet *arc_set = local_edge_obj->timingArcSet();
    for (auto *arc : arc_set->arcs()) {
      for (sta::DcalcAnalysisPt *dcalc_ptr : sta->corners()->dcalcAnalysisPts()) {
        sta::ArcDelay local_delay = local_pt_graph->arcDelay(local_edge, arc, dcalc_ptr->index()) * 1e12;
        sta::ArcDelay open_delay = open_pt_graph->arcDelay(open_edge, arc, dcalc_ptr->index()) * 1e12;
        double delay_diff = std::abs(local_delay - open_delay);
        if (delay_diff > 1e-5) {
          printf("Delay mismatch for arc %s in dcalc_pt %u\n", 
                  arc->to_string().c_str(), dcalc_ptr->index());
          fflush(stdout);
        }
      }
    }
  }

  // Compare arrivals on vertices
  for (size_t i = 1; i < local_vertex_count; ++i) {
    const PtVertex &local_vertex = local_pt_graph->ptVertices()[i];
    const PtVertex &open_vertex = open_pt_graph->ptVertices()[i];
    const sta::Vertex *local_vertex_obj = local_vertex.vertex();
    const sta::Vertex *open_vertex_obj = open_vertex.vertex();
    if (local_vertex_obj != open_vertex_obj) {
      printf("Vertex mismatch at index %zu\n", i);
      fflush(stdout);
      continue;
    }
    // We first check arrivals for all dcalc pts
    int cnt = 0;
    PtVertexPathIterator local_path_iter(const_cast<PtVertex&>(local_vertex), sta);
    PtVertexPathIterator open_path_iter(const_cast<PtVertex&>(open_vertex), sta);
    while (local_path_iter.hasNext() && open_path_iter.hasNext()) {
      sta::Path *local_path = local_path_iter.next();
      sta::Path *open_path = open_path_iter.next();
      if (local_path->dcalcAnalysisPt(sta) != open_path->dcalcAnalysisPt(sta)) {
        printf("DcalcApIndex mismatch at vertex index %zu\n", i);
        fflush(stdout);
        continue;
      }
      sta::Arrival local_arrival = local_path->arrival() * 1e12;
      sta::Arrival open_arrival = open_path->arrival() * 1e12;
      double arrival_diff = std::abs(local_arrival - open_arrival);
      if (arrival_diff > 1e-9) {
        printf("Arrival mismatch at vertex %s for dcalc_pt %u, pathIdx = %u, Local arrival %f, Open arrival %f, arrival difference = %f\n", 
                local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPt(sta)->index(), cnt, local_arrival, open_arrival, arrival_diff);
        fflush(stdout);

      } else {
        // printf("Arrival match at vertex %s for dcalc_pt %u, Local arrival %f, Open arrival %f, arrival difference = %f\n", 
        //         local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPt(sta)->index(), local_arrival, open_arrival, arrival_diff);
        // fflush(stdout);
      }
      sta::Required local_required = local_path->required() * 1e12;
      sta::Required open_required = open_path->required() * 1e12;
      double required_diff = std::abs(local_required - open_required);
      if (required_diff > 1e-9) {
        printf("Required mismatch at vertex %s for dcalc_pt %u, pathIdx = %u, Local required %f, Open required %f, required difference = %f\n", 
                local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPt(sta)->index(), cnt, local_required, open_required, required_diff);
        fflush(stdout);
      }
      cnt++;
    }
  }
}


}  // namespace lrf
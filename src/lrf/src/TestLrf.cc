#include <map>
#include <tuple>
#include "db_sta/dbSta.hh"
#include "sta/ArcDelayCalc.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "LocalSta.hh"
#include "LrHelper.hh"
#include "LrRebuffer.hh"
#include "lrf/IncreSta.hh"
#include "lrf/TestLrf.hh"
#include "odb/db.h"
#include "sta/Liberty.hh"
#include "sta/Corner.hh"
#include "sta/FuncExpr.hh"
#include "LocalSearch.hh"
#include "PtGraph.hh"
#include "ParallelVisitor.hh"
#include "sta/DispatchQueue.hh"
#include "TaskArranger.hh"
#include "sta/TimingRole.hh"
#include "sta/PowerClass.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/Delay.hh"
#include "est/EstimateParasitics.h"
#include "sta/EquivCells.hh"
#include "sta/Path.hh"
#include "search/TagGroup.hh"
#include "odb/db.h"
#include "PortDirection.hh"
  
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>
#include <chrono>

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
    sta->updateTiming(true);
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
    sta->updateTiming(true);
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
  // local_sta->debug_info_.clear();
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for testing
    printf("LocalSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    sta::LibertyCell *new_cell = lib_cell;
    local_sta->virtualReplaceCell(pt_graph, new_cell);
    local_sta->findLocalDelays(pt_graph, arc_delay_calc);
    local_sta->printLocalSlews(pt_graph);
    // for (auto &info : local_sta->debug_info_) {
    //   printf("%s", info.c_str());
    //   fflush(stdout);
    // }
    // local_sta->debug_info_.clear();
  }
  delete arc_delay_calc;
  
  swap_count = 0;
  for (auto *lib_cell : *equiv_cells) {
    if (swap_count++ >= 1) break; // Limit number of swaps for 
    // Also test dbMaster swap
    printf("OpenSTA: Swapping to equiv cell: %s from %s\n", lib_cell->name(), orig_cell->name());
    odb::dbMaster *master = db_network->staToDb(lib_cell);
    // Print global debug info from GraphDelayCalc
    // sta::GraphDelayCalc *global_dcalc = sta->graphDelayCalc();
    // global_dcalc->debug_info_.clear();
    db_inst->swapMaster(master);
    sta->updateTiming(true);

    // for (auto &info : global_dcalc->debug_info_) {
    //   printf("%s", info.c_str());
    //   fflush(stdout);
    // }
    // global_dcalc->debug_info_.clear();

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

  sta->findRequireds();

  printf("----- Testing Difference Between Local and OpenSTA for instance %s -----\n", inst_name);
  printf("Collecting local graph for instance %s\n", db_network->name(sta_inst));
  
  sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
  sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
  sta::LibertyCell *swap_to_cell = (*equiv_cells)[0];
  sta::LibertyCell *swap_to_cell1 = (*equiv_cells)[1];
  (void)swap_to_cell1;
  printf("Swapping to equiv cell: %s from %s\n", swap_to_cell->name(), orig_cell->name());
  // Vitually replace cell in LocalSta and compute delays and arrivals
  PtGraph *pt_graph_local = local_sta->makePtGraph(sta_inst, true);
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  local_sta->setDebugLabel("LocalSTA");
  local_sta->increAndGetLocalTimingCost(pt_graph_local, arc_delay_calc, swap_to_cell);

  // Now swap in OpenSTA and compute PtGraph
  odb::dbMaster *to_master = db_network->staToDb(swap_to_cell);
  db_inst->swapMaster(to_master);
  sta->updateTiming(true);
  sta->findRequireds();
  PtGraph *pt_graph_open = local_sta->makePtGraph(sta_inst, true);
  local_sta->setDebugLabel("OpenSTA");

  odb::dbMaster *from_master = db_network->staToDb(orig_cell);
  db_inst->swapMaster(from_master);
  sta->updateTiming(true);
  sta->findRequireds();
  
  comparePtGraphs(pt_graph_local, pt_graph_open, sta);

  // local_sta->virtualReplaceCell(pt_graph_local, swap_to_cell1);
  // local_sta->findLocalDelays(pt_graph_local, arc_delay_calc);
  // local_sta->findLocalArrivals(pt_graph_local);
  // local_sta->findLocalRequireds(pt_graph_local);

  // printf("Swapping to equiv cell: %s from %s\n", swap_to_cell1->name(), orig_cell->name());
  // odb::dbMaster *to_master1 = db_network->staToDb(swap_to_cell1);
  // db_inst->swapMaster(to_master1);
  // sta->updateTiming(true);
  // sta->findRequireds();
  // PtGraph *pt_graph_orig = local_sta->makePtGraph(sta_inst, true);

  // comparePtGraphs(pt_graph_orig, pt_graph_local, sta);

  pt_graph_local->printGraph("dotfile", true);
}

bool
TestLrf::comparePtGraphs(PtGraph *local_pt_graph, PtGraph *open_pt_graph, sta::dbSta* sta)
{
  bool same = true;
  // Compare delays and arrivals between two PtGraphs
  printf("Comparing PtGraphs between LocalSta and OpenSTA\n");
  // Compare delays
  size_t local_edge_count = local_pt_graph->ptEdges().size();
  size_t open_edge_count = open_pt_graph->ptEdges().size();
  if (local_edge_count != open_edge_count) {
    printf("Edge count mismatch: LocalSta has %zu edges, OpenSTA has %zu edges\n", 
            local_edge_count, open_edge_count);
            fflush(stdout);
    return false;
  }
  size_t local_vertex_count = local_pt_graph->ptVertices().size();
  size_t open_vertex_count = open_pt_graph->ptVertices().size();
  if (local_vertex_count != open_vertex_count) {
    printf("Vertex count mismatch: LocalSta has %zu vertices, OpenSTA has %zu vertices\n", 
            local_vertex_count, open_vertex_count);
    fflush(stdout);
    return false;
  }
  // Compare delays on edges
  for (size_t i = 0; i < local_edge_count; ++i) {
    const PtEdge &local_edge = local_pt_graph->ptEdges()[i];
    const PtEdge &open_edge = open_pt_graph->ptEdges()[i];
    const sta::Edge *local_edge_obj = local_edge.edge();
    const sta::Edge *open_edge_obj = open_edge.edge();
    if (!local_edge_obj && !open_edge_obj) continue;
    if (local_edge_obj != open_edge_obj) {
      printf("Edge mismatch at index %zu: Local=%s, Open=%s\n", 
             i,
             (local_edge_obj ? local_edge_obj->to_string(sta->graph()).c_str() : "null"),
             (open_edge_obj ? open_edge_obj->to_string(sta->graph()).c_str() : "null"));
      fflush(stdout);
      same = false;
      continue;
    }
    sta::TimingArcSet *arc_set = local_edge_obj->timingArcSet();
    for (auto *arc : arc_set->arcs()) {
      for (sta::DcalcAnalysisPt *dcalc_ptr : sta->corners()->dcalcAnalysisPts()) {
        sta::ArcDelay local_delay = local_pt_graph->arcDelay(local_edge, arc, dcalc_ptr->index()) * 1e12;
        sta::ArcDelay open_delay = open_pt_graph->arcDelay(open_edge, arc, dcalc_ptr->index()) * 1e12;
        double delay_diff = std::abs(local_delay - open_delay);
        if (delay_diff > 1e-5) {
          printf("Delay mismatch for arc %s of edge %s in dcalc_pt %u: local=%f, open=%f, diff=%f\n", 
                  arc->to_string().c_str(), local_edge_obj->to_string(sta->graph()).c_str(), dcalc_ptr->index(), local_delay, open_delay, delay_diff);
          
          // Debugging input slew and output load
          const sta::RiseFall *in_rf = arc->fromEdge()->asRiseFall();
          const sta::RiseFall *out_rf = arc->toEdge()->asRiseFall();
          
          // Get Slews
          // Note: PtGraph stores slews on vertices.
          // Input slew is at the "from" vertex of the edge.
          const PtVertex &from_vertex = local_pt_graph->ptVertex(local_edge.ptFromId());
          // Output Load is harder to get directly from PtGraph result, 
          const PtVertex &to_vertex = local_pt_graph->ptVertex(local_edge.ptToId());
          
          sta::Slew local_in_slew = local_pt_graph->slew(from_vertex, in_rf, dcalc_ptr->index()); 
          // OpenSTA global graph slew
          sta::Slew open_in_slew = sta->graph()->slew(open_edge.edge()->from(sta->graph()), in_rf, dcalc_ptr->index());

          sta::Slew local_out_slew = local_pt_graph->slew(to_vertex, out_rf, dcalc_ptr->index());
          sta::Slew open_out_slew = sta->graph()->slew(open_edge.edge()->to(sta->graph()), out_rf, dcalc_ptr->index());
          
          printf("\tInput Slew (%s): local=%e, open=%e, diff=%e | output Slew (%s): local=%e, open=%e, diff=%e\n", 
                 in_rf->name(), local_in_slew, open_in_slew, std::abs(local_in_slew - open_in_slew),
                 out_rf->name(), local_out_slew, open_out_slew, std::abs(local_out_slew - open_out_slew));
                 
          // Load Capacitance Check (Approximation via Parasitics)
          // This requires accessing the Parasitic Network which might be different between Local and Global
          // Let's print the pointer to the parasitics to see if they are using the same one.
          
          fflush(stdout);
          same = false;
        }
      }
    }
  }

  // Compare arrivals on vertices
  for (size_t i = 0; i < local_vertex_count; ++i) {
    const PtVertex &local_vertex = local_pt_graph->ptVertices()[i];
    const PtVertex &open_vertex = open_pt_graph->ptVertices()[i];
    const sta::Vertex *local_vertex_obj = local_vertex.vertex();
    const sta::Vertex *open_vertex_obj = open_vertex.vertex();
    if (!local_vertex_obj && !open_vertex_obj) continue;
    if (local_vertex_obj != open_vertex_obj) {
      printf("Vertex mismatch at index %zu: Local=%s, Open=%s\n", 
             i, 
             (local_vertex_obj ? local_vertex_obj->name(sta->network()) : "null"),
             (open_vertex_obj ? open_vertex_obj->name(sta->network()) : "null"));
      fflush(stdout);
      same = false;
      continue;
    }

    // Compare slews
    if (local_vertex.slewCount() != open_vertex.slewCount()) {
      printf("Slew count mismatch at vertex %s: Local=%zu, Open=%zu\n", 
             local_vertex_obj->name(sta->network()), local_vertex.slewCount(), open_vertex.slewCount());
      fflush(stdout);
      same = false;
    } else if (local_vertex.slewCount() > 0) {
      const sta::Slew *local_slews = local_vertex.slews();
      const sta::Slew *open_slews = open_vertex.slews();
      for (int k = 0; k < local_vertex.slewCount(); ++k) {
        if (std::abs(local_slews[k] - open_slews[k]) > 1e-14) {
           printf("Slew mismatch at vertex %s index %d: Local=%e, Open=%e, diff=%e\n", 
                  local_vertex_obj->name(sta->network()), k, local_slews[k], open_slews[k], std::abs(local_slews[k] - open_slews[k]));
           fflush(stdout);
           same = false;
        }
      }
    }

    // We first check arrivals for all dcalc pts
    int cnt = 0;
    PtVertexPathIterator local_path_iter(const_cast<PtVertex&>(local_vertex), sta);
    PtVertexPathIterator open_path_iter(const_cast<PtVertex&>(open_vertex), sta);
    while (local_path_iter.hasNext() && open_path_iter.hasNext()) {
      sta::Path *local_path = local_path_iter.next();
      sta::Path *open_path = open_path_iter.next();
      if (local_path->dcalcAnalysisPt(sta) != open_path->dcalcAnalysisPt(sta)) {
        printf("DcalcApIndex mismatch at vertex index %zu: Local=%u, Open=%u\n", 
               i,
               local_path->dcalcAnalysisPt(sta)->index(),
               open_path->dcalcAnalysisPt(sta)->index());
        fflush(stdout);
        same = false;
        continue;
      }
      sta::Arrival local_arrival = local_path->arrival() * 1e12;
      sta::Arrival open_arrival = open_path->arrival() * 1e12;
      double arrival_diff = std::abs(local_arrival - open_arrival);
      if (arrival_diff > 1e-9) {
        printf("Arrival mismatch at vertex %s for dcalc_pt %u, pathIdx = %u, Local arrival %f, Open arrival %f, arrival difference = %f\n", 
                local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPt(sta)->index(), cnt, local_arrival, open_arrival, arrival_diff);
        fflush(stdout);
        same = false;
      } 

      sta::Required local_required = local_path->required() * 1e12;
      sta::Required open_required = open_path->required() * 1e12;
      double required_diff = std::abs(local_required - open_required);
      if (required_diff > 1e-9) {
        printf("Required mismatch at vertex %s for dcalc_pt %u, pathIdx = %u, Local required %f, Open required %f, required difference = %f\n", 
                local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPt(sta)->index(), cnt, local_required, open_required, required_diff);
        fflush(stdout);
        same = false;
      }
      cnt++;
    }
  }
  return same;
}

bool
TestLrf::compareTimingRecords(const std::unordered_map<sta::Instance*, TimingRecord> &records1,
                              const std::unordered_map<sta::Instance*, TimingRecord> &records2,
                              sta::dbSta* sta)
{
  printf("Comparing TimingRecords between two runs\n");
  fflush(stdout);
  bool same = true;
  if (records1.size() != records2.size()) {
    printf("TimingRecord size mismatch: %zu vs %zu\n", records1.size(), records2.size());
    return false;
  }

  for (const auto &[inst, record1] : records1) {
    if (records2.find(inst) == records2.end()) {
      printf("Instance %s not found in second record map\n", sta->network()->name(inst));
      same = false;
      continue;
    }
    const TimingRecord &record2 = records2.at(inst);
    if (record1.orig_cell != record2.orig_cell) {
      printf("Original cell mismatch for instance %s: %s vs %s\n", 
             sta->network()->name(inst), 
             record1.orig_cell ? record1.orig_cell->name() : "nullptr",
             record2.orig_cell ? record2.orig_cell->name() : "nullptr");
      same = false;
    }

    // Compare liberty_timing_map
    if (record1.liberty_timing_map.size() != record2.liberty_timing_map.size()) {
       printf("Liberty timing map size mismatch for instance %s: %zu vs %zu\n", 
              sta->network()->name(inst), 
              record1.liberty_timing_map.size(), 
              record2.liberty_timing_map.size());
       same = false;
       continue;
    }

    for (const auto &[lib_name, cell_timing1] : record1.liberty_timing_map) {
      if (record2.liberty_timing_map.find(lib_name) == record2.liberty_timing_map.end()) {
        printf("Liberty cell %s not found in second record for instance %s\n", 
               lib_name.c_str(), sta->network()->name(inst));
        same = false;
        continue;
      }
      const GraphTiming &cell_timing2 = record2.liberty_timing_map.at(lib_name);
      
      // Compare GraphTiming details (Vertex Timing)
      for (const auto &[v_name, v_info1] : cell_timing1.vertex_timing_map) {
        if (cell_timing2.vertex_timing_map.find(v_name) == cell_timing2.vertex_timing_map.end()) {
          printf("Vertex %s not found in second record for instance %s, lib %s\n",
                v_name.c_str(), sta->network()->name(inst), lib_name.c_str());
          same = false;
          continue;
        }
        const TimingInfo &v_info2 = cell_timing2.vertex_timing_map.at(v_name);
        
        // Compare paths (arrivals/requireds)
        if (v_info1.paths.size() != v_info2.paths.size()) {
          printf("Path count mismatch for vertex %s (lib %s): %zu vs %zu\n", v_name.c_str(), lib_name.c_str(), v_info1.paths.size(), v_info2.paths.size());
          same = false;
        } else if (v_info1.tag_group_index != v_info2.tag_group_index) {
          printf("TagGroupIndex mismatch for vertex %s (lib %s): %d vs %d\n", v_name.c_str(), lib_name.c_str(), v_info1.tag_group_index, v_info2.tag_group_index);
          same = false;
        } else {
          for (size_t i = 0; i < v_info1.paths.size(); ++i) {
            const sta::Path &p1 = v_info1.paths[i];
            const sta::Path &p2 = v_info2.paths[i];
            
            // Check DcalcAnalysisPt
            if (p1.dcalcAnalysisPt(sta) != p2.dcalcAnalysisPt(sta)) {
              printf("DcalcAnalysisPt mismatch for vertex %s (lib %s) path %zu\n", v_name.c_str(), lib_name.c_str(), i);
              same = false;
            }

            // Check Arrival
            double arr1 = p1.arrival() * 1e12;
            double arr2 = p2.arrival() * 1e12;
            if (std::abs(arr1 - arr2) > 1e-5) {
              printf("Arrival mismatch for vertex %s (lib %s) path %zu: %f (tag:%d) vs %f (tag:%d)\n", v_name.c_str(), lib_name.c_str(), i, arr1, p1.tagIndex(sta), arr2, p2.tagIndex(sta));
              same = false;
            }

            // Check Required
           double req1 = p1.required() * 1e12;
           double req2 = p2.required() * 1e12;
           if (std::abs(req1 - req2) > 1e-5) {
              printf("Required mismatch for vertex %s (lib %s) path %zu: %f vs %f\n", v_name.c_str(), lib_name.c_str(), i, req1, req2);
              same = false;
           }
          }
        }

        // Compare slews
        if (v_info1.slews.size() != v_info2.slews.size()) {
          printf("Slew count mismatch for vertex %s (lib %s)\n", v_name.c_str(), lib_name.c_str());
          same = false;
        } else {
          for (size_t i = 0; i < v_info1.slews.size(); ++i) {
              if (std::abs(v_info1.slews[i] - v_info2.slews[i]) > 1e-14) {
                printf("Slew mismatch for vertex %s (lib %s) index %zu: %f vs %f diff=%f\n", v_name.c_str(), lib_name.c_str(), i, v_info1.slews[i] * 1e12, v_info2.slews[i] * 1e12, std::abs(v_info1.slews[i] - v_info2.slews[i]) * 1e12);
                same = false;
              }
          }
        }
      }

      // Compare GraphTiming details (Edge Timing)
      for (const auto &[e_name, e_info1] : cell_timing1.edge_timing_map) {
         if (cell_timing2.edge_timing_map.find(e_name) == cell_timing2.edge_timing_map.end()) {
           printf("Edge %s not found in second record for instance %s, lib %s\n",
                  e_name.c_str(), sta->network()->name(inst), lib_name.c_str());
           same = false;
           continue;
         }
         const TimingInfo &e_info2 = cell_timing2.edge_timing_map.at(e_name);
         
         if (e_info1.delays.empty() || e_info2.delays.empty()) {
           printf("No delays recorded for edge %s (lib %s)\n", e_name.c_str(), lib_name.c_str());
           same = false;
           continue;
         }
         if (e_info1.delays.size() != e_info2.delays.size()) {
            printf("Delay count mismatch for edge %s (lib %s)\n", e_name.c_str(), lib_name.c_str());
            same = false;
         } else {
            for (size_t i = 0; i < e_info1.delays.size(); ++i) {
               if (std::abs(e_info1.delays[i] * 1e12 - e_info2.delays[i] * 1e12) > 1e-5) {
                 printf("Delay mismatch for edge %s (lib %s) index %zu: %e vs %e, diff=%e\n", e_name.c_str(), lib_name.c_str(), i, e_info1.delays[i], e_info2.delays[i], std::abs(e_info1.delays[i] * 1e12 - e_info2.delays[i] * 1e12));
                 same = false;
               }
            }
         }
      }
    }
  }
  fflush(stdout);
  return same;
}

void
TestLrf::testParallelVisitor(const std::vector<odb::dbInst*>& db_insts, sta::dbSta* sta, 
                               rsz::Resizer *resizer, odb::dbBlock *block)
{
  for (auto *db_inst : db_insts) {
    if (!db_inst) {
      printf("Instance not found in the block.\n");
      return;
    }
  }
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  sta::dbNetwork *db_network = sta->getDbNetwork();
  resizer->makeEquivCells();

  // 只测 2~3 个线程
  sta::DispatchQueue dq(/*thread_count=*/3);
  dq.setThreadCount(3);

  // 为每个实例准备一个 ParallelLrVisitor，并投递到队列
  size_t n = db_insts.size();
  std::vector<std::unique_ptr<ParallelLrVisitor>> visitors;
  visitors.reserve(n);

  std::vector<sta::Instance*> insts;
  for (size_t i = 0; i < n; ++i) {
    odb::dbInst *db_inst = db_insts[i];
    if (!db_inst) {
      printf("Instance not found in the block.\n");
      continue;
    }
    sta::Instance *sta_inst = db_network->dbToSta(db_inst);
    sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
    resizer->getSwappableCells(orig_cell);
    insts.push_back(sta_inst);
  }

  for (size_t i = 0; i < 3; ++i) {
    visitors.emplace_back(std::make_unique<ParallelLrVisitor>(sta, local_sta, resizer));
  }

  std::vector<ParallelLrVisitor*> visitor_ptrs;
  for(auto& v : visitors) visitor_ptrs.push_back(v.get());

  for (size_t i = 0; i < n; ++i) {    
    // 投递一个任务，执行 visit
    sta::Instance *sta_inst = insts[i];
    dq.dispatch([visitor_ptrs, sta_inst](int id) {
      if (id >= 0 && id < visitor_ptrs.size()) {
        visitor_ptrs[id]->visit(sta_inst, sta::object_id_null);
      }
    });
  }

  // 等待所有任务完成
  dq.finishTasks();
}

void 
TestLrf::testMEEAssignments(sta::dbSta* sta, 
                            rsz::Resizer * /*resizer*/, 
                            odb::dbBlock * /*block*/)
{
  // Test MEE assignments
  printf("----- Testing MEE Assignments -----\n");
  sta->searchPreamble();
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  TaskArranger *arranger = local_sta->taskArranger();
  arranger->init();
  arranger->printGraph();
}

void
TestLrf::testParallelResize(sta::dbSta* sta, 
                            rsz::Resizer *resizer, 
                            odb::dbBlock *block)
{
  // Test parallel resize
  printf("----- Testing Parallel Resize -----\n");
  sta->searchPreamble();
  IncreSta *incre_sta = new IncreSta(sta);
  incre_sta->parallelResize(resizer, 1, 1);
  // incre_sta->localSta()->taskArranger()->printGraph();
  incre_sta->localSta()->taskArranger()->printFailed();
}

void
TestLrf::testParallelLrResizing(sta::dbSta* sta, 
                            rsz::Resizer *resizer, 
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  // Test parallel LR resizing
  printf("----- Testing Parallel LR Resizing -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  // Set configuration for LRHelper
  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  int thread_count = local_sta->threadCount();
  incre_sta->setMaxResizeNum(max_resize_num); // Limit max resize number per iteration
  printf("Thread count has set to %d\n", thread_count);
  // Conduct iterative resizing
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0; // Count of no improvement iterations of each ECO record.
  size_t eco_iter = 0; // Termination flag, when eco cannot improve PPA
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns;
  sta::Slack wns;
  printf("Initial Worst Negative Slack: %f\n", best_wns * 1e12);
  printf("Initial Total Negative Slack: %f\n", best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  printf("Initial Average Delay on Critical Path: %f\n", avg_delay * 1e12);
  printf("Initial Average Leakage: %f\n", avg_leakage * 1e10);
  // Initialize conflict graph in task arranger
  local_sta->initParallel();
  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR Resizing Iteration %zu -----\n", i+1);
    // incre_sta->parallelResizeV1(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeAdaptive(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    printf("Parallel resize took %f seconds\n", elapsed.count());
    // Measure parasitics update time (perform update and time it)
    auto par_start = std::chrono::high_resolution_clock::now();
    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    auto par_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_parasitics = par_end - par_start;
    printf("Parasitics update took %f seconds\n", elapsed_parasitics.count());

    // Measure evaluation time (timing update + slack/leakage computation)
    auto eval_start = std::chrono::high_resolution_clock::now();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());
    float leakage = 0;
    odb::dbSet<odb::dbInst> insts = block->getInsts();
    for (odb::dbInst *inst : insts) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }
    auto eval_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_eval = eval_end - eval_start;
    printf("Evaluation took %f seconds\n", elapsed_eval.count());
    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);

    // Measure LM update time
    auto lm_start = std::chrono::high_resolution_clock::now();
    incre_sta->lmUpdate();
    auto lm_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_lm = lm_end - lm_start;
    printf("LM update took %f seconds\n", elapsed_lm.count());

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting new design.\n");
      no_improve_count_ = 0;
    }
    else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS is positive and improvement in WNS or leakage, accepting new design.\n");
      no_improve_count_ = 0;
    }
    else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
      printf("No improvement in WNS, but within tolerance, accepting new design.\n");
      continue;
    }
    else if (eco_iter > 2) {
      printf("No improvement in WNS for %zu ECO iterations, terminating resizing.\n", eco_iter);
      break;
    }
    else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
}

void
TestLrf::testParallelLrResizingBuffering(sta::dbSta* sta, 
                            rsz::Resizer *resizer, 
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  // Test parallel LR resizing
  printf("----- Testing Parallel LR Resizing -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  // Set configuration for LRHelper
  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  int thread_count = local_sta->threadCount();
  incre_sta->setMaxResizeNum(max_resize_num); // Limit max resize number per iteration
  printf("Thread count has set to %d\n", thread_count);
  // Conduct iterative resizing
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0; // Count of no improvement iterations of each ECO record.
  size_t eco_iter = 0; // Termination flag, when eco cannot improve PPA
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns;
  sta::Slack wns;
  printf("Initial Worst Negative Slack: %f\n", best_wns * 1e12);
  printf("Initial Total Negative Slack: %f\n", best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  printf("Initial Average Delay on Critical Path: %f\n", avg_delay * 1e12);
  printf("Initial Average Leakage: %f\n", avg_leakage * 1e10);
  // Initialize conflict graph in task arranger
  local_sta->initParallel();
  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR Resizing Iteration %zu -----\n", i+1);
    // incre_sta->parallelResizeV1(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeAdaptive(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    printf("Parallel resize took %f seconds\n", elapsed.count());
    // Measure parasitics update time (perform update and time it)
    auto par_start = std::chrono::high_resolution_clock::now();
    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    auto par_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_parasitics = par_end - par_start;
    printf("Parasitics update took %f seconds\n", elapsed_parasitics.count());

    // Measure evaluation time (timing update + slack/leakage computation)
    auto eval_start = std::chrono::high_resolution_clock::now();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());
    float leakage = 0;
    odb::dbSet<odb::dbInst> insts = block->getInsts();
    for (odb::dbInst *inst : insts) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }
    auto eval_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_eval = eval_end - eval_start;
    printf("Evaluation took %f seconds\n", elapsed_eval.count());
    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);

    // Measure LM update time
    auto lm_start = std::chrono::high_resolution_clock::now();
    incre_sta->lmUpdate();
    auto lm_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_lm = lm_end - lm_start;
    printf("LM update took %f seconds\n", elapsed_lm.count());

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting new design.\n");
      no_improve_count_ = 0;
    }
    else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS is positive and improvement in WNS or leakage, accepting new design.\n");
      no_improve_count_ = 0;
    }
    else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
      printf("No improvement in WNS, but within tolerance, accepting new design.\n");
      continue;
    }
    else if (eco_iter > 2) {
      printf("No improvement in WNS for %zu ECO iterations, terminating resizing.\n", eco_iter);
      break;
    }
    else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  if (wns < 0) {
    sta->findRequireds();
    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelBuffering(resizer, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    printf("Parallel buffering took %f seconds\n", elapsed.count());

    // Measure evaluation time (timing update + slack/leakage computation)
    auto eval_start = std::chrono::high_resolution_clock::now();
    sta->delaysInvalid();
    
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());
    float leakage = 0;
    odb::dbSet<odb::dbInst> insts = block->getInsts();
    for (odb::dbInst *inst : insts) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }
    auto eval_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed_eval = eval_end - eval_start;
    printf("Evaluation took %f seconds\n", elapsed_eval.count());
    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
  }
}

void
TestLrf::testParallelLrResizeByArray(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  printf("----- Testing Parallel LR Resize By Array -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR ResizeByArray Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    printf("parallelResizeByArray took %f seconds\n",
           std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Evaluation took %f seconds\n",
           std::chrono::duration<double>(end - start).count());
    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
      continue;
    } else if (eco_iter > 2) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  delete incre_sta;
}

////////////////////////////////////////////////////////////////
// IterationHelper
////////////////////////////////////////////////////////////////

IterationHelper::IterationHelper(sta::dbSta *sta, odb::dbBlock *block)
  : sta_(sta), block_(block)
{
  corner_ = sta->corners()->findCorner("default");
}

IterationHelper::Metrics
IterationHelper::snapshot(double runtime_s)
{
  Metrics m;
  m.wns_ps = sta_->worstSlack(sta::MinMax::max()) * 1e12;
  m.tns_ps = sta_->totalNegativeSlack(sta::MinMax::max()) * 1e12;
  m.runtime_s = runtime_s;
  if (corner_) {
    for (odb::dbInst *inst : block_->getInsts()) {
      sta::Instance *si = sta_->getDbNetwork()->dbToSta(inst);
      if (!si) continue;
      m.leakage += sta_->power(si, corner_).leakage();
    }
  }
  return m;
}

void
IterationHelper::recordRow(size_t iter, const char *phase,
                           const Metrics &cur, const Metrics &before,
                           const char *decision)
{
  char buf[256];
  snprintf(buf, sizeof(buf),
           " %4zu | %-7s | %9.3f | %+7.3f | %12.3f | %+9.3f | %11.3f | %s (%.1fs)",
           iter, phase,
           cur.wns_ps, cur.wns_ps - before.wns_ps,
           cur.tns_ps, cur.tns_ps - before.tns_ps,
           cur.leakage * 1e10,
           decision, cur.runtime_s);
  rows_.push_back(buf);
  // Print immediately so progress is visible before run completes
  printf("[ITER]%s\n", buf);
  fflush(stdout);
}

void
IterationHelper::printSummary(const Metrics &best)
{
  printf("\n");
  printf("==============================================================================================\n");
  printf(" Iter | Phase   | WNS(ps)   |   dWNS  | TNS(ps)      |    dTNS   | Leakage(uW) | Decision\n");
  printf("----------------------------------------------------------------------------------------------\n");
  for (const auto &row : rows_)
    printf("%s\n", row.c_str());
  printf("----------------------------------------------------------------------------------------------\n");
  printf(" Best checkpoint: WNS %.3f ps, TNS %.3f ps, Leakage %.3f uW\n",
         best.wns_ps, best.tns_ps, best.leakage * 1e10);
  printf("==============================================================================================\n");
  fflush(stdout);
}

const char *
IterationHelper::ecoDecision(const Metrics &cur, Metrics &best,
                             size_t &no_improve, size_t tolerance,
                             size_t &eco_iter, bool &should_break,
                             bool allow_revert)
{
  should_break = false;
  double cur_wns = cur.wns_ps / 1e12;  // back to seconds for comparison
  double best_wns = best.wns_ps / 1e12;

  // Accept if WNS improved (negative slack region)
  if (cur_wns > best_wns && cur_wns < 0) {
    best = cur;
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::beginEco(block_);
    no_improve = 0;
    return "accept(wns)";
  }
  // Accept if timing met and (WNS or leakage improved)
  if (cur_wns >= 0.0 && (cur_wns > best_wns || cur.leakage < best.leakage)) {
    best = cur;
    odb::dbDatabase::endEco(block_);
    odb::dbDatabase::beginEco(block_);
    no_improve = 0;
    return "accept(met)";
  }

  // No improvement — buffer phases only hold, never revert or terminate
  if (!allow_revert)
    return "hold";

  // Resize: tolerate, then revert, then terminate
  if (no_improve < tolerance) {
    no_improve++;
    return "tolerate";
  }
  if (eco_iter > 2) {
    should_break = true;
    return "terminate";
  }
  odb::dbDatabase::endEco(block_);
  odb::dbDatabase::undoEco(block_);
  odb::dbDatabase::beginEco(block_);
  eco_iter++;
  return "revert";
}

void
TestLrf::testParallelLrResizeByArrayV2(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  printf("----- Testing Parallel LR Resize By Array V2 (New Framework) -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR ResizeByArrayV2 Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayV2(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    printf("parallelResizeByArrayV2 took %f seconds\n",
           std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
      continue;
    } else if (eco_iter > 2) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }

  // Post-resize buffering pass (V2 framework)
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns < 0) {
    printf("----- V2 Buffering Pass -----\n");
    sta->findRequireds();
    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    incre_sta->parallelBufferingV2(resizer, PT_tradeoff);
    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());
    printf("After V2 buffering, WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  }

  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  printf("----- Testing Parallel LR Resize By Array With Buffering -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    // Perform parallel resize by array
    sta->findRequireds();
    printf("----- LR ResizeByArray Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    printf("parallelResizeByArray took %f seconds\n",
          std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Worst Negative Slack after RSZ: %f\n", wns * 1e12);
    printf("Total Negative Slack after RSZ: %f\n", tns * 1e12);
    printf("Total Leakage Power after RSZ: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
    } else if (eco_iter > 2) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }

    // Perform parallel buffering if WNS is negative after resize
    if (wns < 0 && i > 3) {
      printf("----- WNS negative, performing parallel buffering -----\n");
      double wns_before_buf = wns * 1e12;
      double tns_before_buf = tns * 1e12;
      sta->findRequireds();
      est_parasitics->updateWireParasiticsNoDeleteNetwork();
      incre_sta->parallelBuffering(resizer, PT_tradeoff);
      sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();
      tns = sta->totalNegativeSlack(sta::MinMax::max());
      wns = sta->worstSlack(sta::MinMax::max());
      printf("Buffering diff: WNS %.3f -> %.3f ps (delta=%.3f), TNS %.3f -> %.3f ps (delta=%.3f)\n",
             wns_before_buf, wns * 1e12, wns * 1e12 - wns_before_buf,
             tns_before_buf, tns * 1e12, tns * 1e12 - tns_before_buf);
      fflush(stdout);
      float buf_leakage = 0;
      for (odb::dbInst *inst : block->getInsts()) {
        sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
        if (!sta_inst) continue;
        sta::PowerResult power_result = sta->power(sta_inst, corner);
        buf_leakage += power_result.leakage();
      }
      printf("Worst Negative Slack after buffering: %f\n", wns * 1e12);
      printf("Total Negative Slack after buffering: %f\n", tns * 1e12);
      printf("Total Leakage Power after buffering: %f\n", buf_leakage * 1e10);
      fflush(stdout);
      incre_sta->lmUpdate();

      if (wns > best_wns && wns < 0) {
        best_wns = wns;
        best_tns = tns;
        best_leakage = buf_leakage;
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::beginEco(block);
        printf("Buffering improved WNS, accepting.\n");
        no_improve_count_ = 0;
      } else if (wns >= 0.0 && (wns > best_wns || buf_leakage < best_leakage)) {
        best_wns = wns;
        best_tns = tns;
        best_leakage = buf_leakage;
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::beginEco(block);
        printf("Buffering: WNS positive, accepting.\n");
        no_improve_count_ = 0;
      } else if (no_improve_count_ < num_no_improve_tolerance) {
        no_improve_count_++;
        printf("Buffering did not improve (tolerance %zu/%zu), keeping.\n",
               no_improve_count_, num_no_improve_tolerance);
      } else {
        printf("Buffering did not improve, reverting.\n");
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::undoEco(block);
        odb::dbDatabase::beginEco(block);
      }
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithBufferingV2(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  printf("----- Testing V2 Resize By Array With Buffering -----\n");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  incre_sta->lrHelper()->setRatcons(ratcons);
  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block);
  IterationHelper::Metrics best = helper.snapshot();
  printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  size_t no_improve = 0;
  size_t eco_iter = 0;
  // Rows are buffered and printed as a clean table at the end

  for (size_t i = 0; i < iterations; ++i) {
    // ---- Snapshot before resize ----
    sta->findRequireds();
    auto before_rsz = helper.snapshot();

    // ---- Resize phase ----
    auto t0 = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayV2(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto t1 = std::chrono::high_resolution_clock::now();
    double rsz_sec = std::chrono::duration<double>(t1 - t0).count();

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    incre_sta->lmUpdate();

    auto after_rsz = helper.snapshot(rsz_sec);
    bool should_break = false;
    const char *decision = helper.ecoDecision(after_rsz, best,
                                              no_improve, num_no_improve_tolerance,
                                              eco_iter, should_break);
    helper.recordRow(i+1, "resize", after_rsz, before_rsz, decision);
    if (should_break) break;

    // ---- Buffering phase (after iteration 3, if WNS < 0) ----
    if (after_rsz.wns_ps < 0 && i > 3) {
      sta->findRequireds();
      auto before_buf = helper.snapshot();

      est_parasitics->updateWireParasiticsNoDeleteNetwork();
      auto tb0 = std::chrono::high_resolution_clock::now();
      incre_sta->parallelBufferingV2(resizer, PT_tradeoff);
      auto tb1 = std::chrono::high_resolution_clock::now();
      double buf_sec = std::chrono::duration<double>(tb1 - tb0).count();

      sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();
      incre_sta->lmUpdate();

      auto after_buf = helper.snapshot(buf_sec);
      const char *buf_decision = helper.ecoDecision(after_buf, best,
                                                    no_improve, num_no_improve_tolerance,
                                                    eco_iter, should_break,
                                                    /*allow_revert=*/false);
      helper.recordRow(i+1, "buffer", after_buf, before_buf, buf_decision);
    }
  }

  helper.printSummary(best);

  // Final accept/revert
  double final_wns = sta->worstSlack(sta::MinMax::max()) * 1e12;
  double best_wns_ps = best.wns_ps;
  if (final_wns > best_wns_ps) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %.3f, TNS: %.3f\n",
           final_wns, sta->totalNegativeSlack(sta::MinMax::max()) * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %.3f, TNS: %.3f\n",
           best.wns_ps, best.tns_ps);
  }
  delete incre_sta;
}

void
TestLrf::testParallelLrCombinedResizeBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method)
{
  printf("----- Testing Combined Resize + Buffering -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- Combined Resize+Buffering Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeAndBuffering(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    printf("parallelResizeAndBuffering took %f seconds\n",
          std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
    } else if (eco_iter > 2) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithPrecheck(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float top_ratio)
{
  printf("----- Testing Parallel LR Resize By Array With Precheck -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  bool was_converged = false;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR ResizeByArrayWithPrecheck Iteration %zu (top_ratio=%.4f) -----\n",
           i+1, top_ratio);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayWithPrecheck(resizer, avg_delay, avg_leakage,
                                                  PT_tradeoff, top_ratio);
    auto end = std::chrono::high_resolution_clock::now();
    printf("Iteration %zu took %f seconds\n", i+1,
           std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    // Post-convergence regression: immediate rollback + halve, no tolerance.
    if (was_converged && wns < 0.0) {
      printf("Post-convergence regression (WNS %e), immediate rollback.\n", wns);
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      top_ratio *= 0.5f;
      printf("Halved top_ratio to %.4f.\n", top_ratio);
      fflush(stdout);
      eco_iter++;
      if (eco_iter > 6) {
        printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
        break;
      }
      continue;
    }

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      was_converged = true;
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
      continue;
    } else if (eco_iter > 6) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      top_ratio *= 0.5f;
      printf("Halved top_ratio to %.4f after rollback.\n", top_ratio);
      fflush(stdout);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithPrecheckV2(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float top_ratio)
{
  printf("----- Testing Parallel LR Resize By Array With Precheck V2 (New Framework) -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  bool was_converged = false;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR ResizeByArrayWithPrecheckV2 Iteration %zu (top_ratio=%.4f) -----\n",
           i+1, top_ratio);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayWithPrecheckV2(resizer, avg_delay, avg_leakage,
                                                    PT_tradeoff, top_ratio);
    auto end = std::chrono::high_resolution_clock::now();
    printf("Iteration %zu took %f seconds\n", i+1,
           std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    // Post-convergence regression: immediate rollback + halve, no tolerance.
    if (was_converged && wns < 0.0) {
      printf("Post-convergence regression (WNS %e), immediate rollback.\n", wns);
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      top_ratio *= 0.5f;
      printf("Halved top_ratio to %.4f.\n", top_ratio);
      fflush(stdout);
      eco_iter++;
      if (eco_iter > 6) {
        printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
        break;
      }
      continue;
    }

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      was_converged = true;
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
      continue;
    } else if (eco_iter > 6) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      top_ratio *= 0.5f;
      printf("Halved top_ratio to %.4f after rollback.\n", top_ratio);
      fflush(stdout);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithPrecheckBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float top_ratio)
{
  printf("----- Testing Parallel LR Resize By Array With Precheck + Buffering -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR ResizeByArrayWithPrecheckBuffering Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayWithPrecheck(resizer, avg_delay, avg_leakage,
                                                  PT_tradeoff, top_ratio);
    auto end = std::chrono::high_resolution_clock::now();
    printf("Iteration %zu took %f seconds\n", i+1,
           std::chrono::duration<double>(end - start).count());

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());

    float leakage = 0;
    for (odb::dbInst *inst : block->getInsts()) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }

    printf("Worst Negative Slack after RSZ: %f\n", wns * 1e12);
    printf("Total Negative Slack after RSZ: %f\n", tns * 1e12);
    printf("Total Leakage Power after RSZ: %f\n", leakage * 1e10);
    fflush(stdout);
    incre_sta->lmUpdate();

    if (wns > best_wns && wns < 0) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting.\n");
      no_improve_count_ = 0;
    } else if (wns >= 0.0 && (wns > best_wns || leakage < best_leakage)) {
      best_wns = wns;
      best_tns = tns;
      best_leakage = leakage;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("WNS positive, WNS or leakage improved, accepting.\n");
      no_improve_count_ = 0;
    } else if (no_improve_count_ < num_no_improve_tolerance) {
      no_improve_count_++;
    } else if (eco_iter > 2) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
    }

    // Perform parallel buffering if WNS is negative after resize
    if (wns < 0 && i > 3) {
      printf("----- WNS negative, performing parallel buffering -----\n");
      double wns_before_buf = wns * 1e12;
      double tns_before_buf = tns * 1e12;
      sta->findRequireds();
      est_parasitics->updateWireParasiticsNoDeleteNetwork();
      incre_sta->parallelBuffering(resizer, PT_tradeoff);
      sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();
      tns = sta->totalNegativeSlack(sta::MinMax::max());
      wns = sta->worstSlack(sta::MinMax::max());
      printf("Buffering diff: WNS %.3f -> %.3f ps (delta=%.3f), TNS %.3f -> %.3f ps (delta=%.3f)\n",
             wns_before_buf, wns * 1e12, wns * 1e12 - wns_before_buf,
             tns_before_buf, tns * 1e12, tns * 1e12 - tns_before_buf);
      fflush(stdout);
      float buf_leakage = 0;
      for (odb::dbInst *inst : block->getInsts()) {
        sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
        if (!sta_inst) continue;
        sta::PowerResult power_result = sta->power(sta_inst, corner);
        buf_leakage += power_result.leakage();
      }
      printf("Worst Negative Slack after buffering: %f\n", wns * 1e12);
      printf("Total Negative Slack after buffering: %f\n", tns * 1e12);
      printf("Total Leakage Power after buffering: %f\n", buf_leakage * 1e10);
      fflush(stdout);
      incre_sta->lmUpdate();

      if (wns > best_wns && wns < 0) {
        best_wns = wns;
        best_tns = tns;
        best_leakage = buf_leakage;
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::beginEco(block);
        printf("Buffering improved WNS, accepting.\n");
        no_improve_count_ = 0;
      } else if (wns >= 0.0 && (wns > best_wns || buf_leakage < best_leakage)) {
        best_wns = wns;
        best_tns = tns;
        best_leakage = buf_leakage;
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::beginEco(block);
        printf("Buffering: WNS positive, accepting.\n");
        no_improve_count_ = 0;
      } else if (no_improve_count_ < num_no_improve_tolerance) {
        no_improve_count_++;
        printf("Buffering did not improve (tolerance %zu/%zu), keeping.\n",
               no_improve_count_, num_no_improve_tolerance);
      } else {
        printf("Buffering did not improve, reverting.\n");
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::undoEco(block);
        odb::dbDatabase::beginEco(block);
      }
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  delete incre_sta;
}

void
TestLrf::testPrecedingResizeCheck(sta::dbSta* sta,
                                  rsz::Resizer *resizer,
                                  odb::dbBlock *block,
                                  size_t thread_num,
                                  float PT_tradeoff,
                                  float top_ratio)
{
  printf("----- Testing Preceding Resize Check -----\n");
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  printf("Average Delay on Critical Path: %f ps\n", avg_delay * 1e12);
  printf("Average Leakage: %f\n", avg_leakage * 1e10);

  auto results = incre_sta->precedingResizeCheck(resizer, avg_delay, avg_leakage,
                                                 PT_tradeoff, top_ratio);

  sta::Slack wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack tns = sta->totalNegativeSlack(sta::MinMax::max());
  printf("WNS: %f ps, TNS: %f ps\n", wns * 1e12, tns * 1e12);
  printf("precedingResizeCheck returned %zu instances with positive benefit\n",
         results.size());
  fflush(stdout);
  delete incre_sta;
}

void
TestLrf::testReportVertices(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block)
{
  // Test report vertices
  printf("----- Testing Report Vertices -----\n");
  lrf::IncreSta *incre_sta = new IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  auto &ordered_vertices = lr_helper->ensureSorted(sta);
  for (auto *vertex : ordered_vertices) {
    printf("Vertex: %s, level = %d\n", vertex->to_string(sta->network()).c_str(), vertex->level());
    fflush(stdout);
  }
}

void
TestLrf::testTimingComputeAndWriteBack(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block, const std::vector<odb::dbInst*> &db_insts)
{
  // Test timing compute and write back
  printf("----- Testing Timing Compute and Write Back -----\n");
  printf("Received %zu db_insts to process\n", db_insts.size());
  fflush(stdout);
  
  lrf::IncreSta *incre_sta = new IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();
  // incre_sta->lmUpdate();
  resizer->makeEquivCells();

  std::vector<sta::Instance*> sta_insts;
  for (size_t i = 0; i < db_insts.size(); ++i) {
    auto *db_inst = db_insts[i];
    if (db_inst == nullptr) {
      printf("Warning: nullptr db_inst found at index %zu in db_insts vector\n", i);
      continue;
    }
    sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(db_inst);
    if (sta_inst == nullptr) {
      printf("Warning: dbToSta returned nullptr for db_inst %s at index %zu\n", 
             db_inst->getName().c_str(), i);
      continue;
    }
    sta_insts.push_back(sta_inst);
  }
  
  printf("Successfully converted %zu db_insts to %zu sta_insts\n", 
         db_insts.size(), sta_insts.size());
  fflush(stdout);

  local_sta->initParallel();
  float average_delay = incre_sta->averageDelayOnCritPath();
  float average_leakage = incre_sta->averageLeakage();
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta, local_sta, resizer);
  visitor->setAverageDelay(average_delay);
  visitor->setAverageLeakage(average_leakage);

  std::unordered_map<sta::Instance*, TimingRecord> timing_record_map_opensta;
  collectTimingInfoForInstancesUsingOpenSta(sta, resizer, block, sta_insts, timing_record_map_opensta);
  std::unordered_map<sta::Instance*, TimingRecord> timing_record_map_localsta;
  collectTimingInfoForInstancesUsingLocalSta(sta, resizer, block, sta_insts, timing_record_map_localsta);

  compareTimingRecords(timing_record_map_opensta, timing_record_map_localsta, sta);
}

void 
TestLrf::collectTimingInfoForInstancesUsingOpenSta(sta::dbSta* sta, 
          rsz::Resizer *resizer, 
          odb::dbBlock *block,
          std::vector<sta::Instance*> &sta_insts,
          std::unordered_map<sta::Instance*, TimingRecord> &instance_timing_map)
{
  sta->updateTiming(true);
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new lrf::IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  for (auto *sta_inst : sta_insts) {
    // Validate instance pointer first
    if (sta_inst == nullptr) {
      printf("Warning: nullptr sta_inst found in sta_insts, skipping\n");
      continue;
    }
    
    /////////
    TimingRecord inst_timing_record;
    inst_timing_record.inst = sta_inst;
    sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
    inst_timing_record.orig_cell = orig_cell;
    /////////
    
    if (!orig_cell) {
      printf("Original cell not found for instance %s\n", sta->getDbNetwork()->name(sta_inst));
      continue;
    }
    sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
    sta::LibertyCellSeq legal_equiv_cells;
    for (sta::LibertyCell *equiv_cell : *equiv_cells) {
      if (sta::equivCellsArcs(orig_cell, equiv_cell)) {
        legal_equiv_cells.push_back(equiv_cell);
      }
    }
    if (legal_equiv_cells.size() < 2) {
      printf("ParallelLrVisitor::visit no legal equiv cells for %s\n",
             orig_cell->name());
      fflush(stdout);
      continue;
    }

    odb::dbMaster *orig_master = sta->getDbNetwork()->staToDb(orig_cell);

    lrf::PtGraph *pt_graph = nullptr;
    for (auto *equiv_cell : legal_equiv_cells) {
      GraphTiming cell_graph_timing;
      cell_graph_timing.cell = equiv_cell;
      pt_graph = local_sta->makePtGraph(sta_inst, true);
      float slack_before = local_sta->localSlackAroundRef(pt_graph);
      printf("OpenSTA::Collecting timing info for instance %s with equiv cell %s, original cell %s\n", 
              sta->getDbNetwork()->name(sta_inst), equiv_cell->name(), orig_cell->name());
      fflush(stdout);

      odb::dbMaster *master = sta->getDbNetwork()->staToDb(equiv_cell);
      odb::dbInst *db_inst = sta->getDbNetwork()->staToDb(sta_inst);
      db_inst->swapMaster(master);
      // sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();
      pt_graph = local_sta->makePtGraph(sta_inst, true);
      float slack = local_sta->localSlackAroundRef(pt_graph);
      printf("Instance %s libcell %s Local Slack around Ref: %f ps, original slack %f ps\n", 
              sta->getDbNetwork()->name(sta_inst), sta->network()->libertyCell(sta_inst)->name(), slack * 1e12, slack_before * 1e12);
      fflush(stdout);

      // We can further collect slacks here
      printf("OpenSTA: recordGraphTimingFromPtGraph \n");
      recordGraphTimingFromPtGraph(sta, pt_graph, cell_graph_timing, true);
      inst_timing_record.liberty_timing_map[std::string(equiv_cell->name())] = cell_graph_timing;
      pt_graph->printGraph("dotfile", true);
      break; // Only test the first legal equiv cell for now
    }
    
    // Restore original master
    odb::dbInst *db_inst = sta->getDbNetwork()->staToDb(sta_inst);
    if (db_inst->getMaster() != orig_master) {
      db_inst->swapMaster(orig_master);
      // sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();
    }

    instance_timing_map[sta_inst] = inst_timing_record;
  }
  delete incre_sta;
}

void 
TestLrf::collectTimingInfoForInstancesUsingLocalSta(sta::dbSta* sta, 
  rsz::Resizer *resizer, 
  odb::dbBlock *block, 
  std::vector<sta::Instance*> &sta_insts, 
  std::unordered_map<sta::Instance*, TimingRecord> &instance_timing_map)
{
  sta->updateTiming(true);
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new lrf::IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  lrf::ParallelLrVisitor *visitor = new lrf::ParallelLrVisitor(sta, local_sta, resizer);
  visitor->setAverageDelay(incre_sta->averageDelayOnCritPath());
  visitor->setAverageLeakage(incre_sta->averageLeakage());
  for (auto *sta_inst : sta_insts) {
    printf("Visiting instance %s using ParallelLrVisitor\n", 
            sta->getDbNetwork()->name(sta_inst));
    fflush(stdout);
    TimingRecord inst_timing_record;
    inst_timing_record.inst = sta_inst;
    inst_timing_record.orig_cell = sta->network()->libertyCell(sta_inst);
    visitor->visit(sta_inst, inst_timing_record);
    visitor->applyChangesToDb(resizer);

    instance_timing_map[sta_inst] = inst_timing_record;

    // PtGraph *pt_graph_visitor = visitor->ptGraph();
    // PtGraph *pt_graph_temp = local_sta->makePtGraph(sta_inst, false);
    // if (!comparePtGraphs(pt_graph_visitor, pt_graph_temp, sta)) {
    //   throw std::runtime_error("PtGraph from ParallelLrVisitor does not match that from LocalSta");
    // }
  }
  delete visitor;
  delete incre_sta;
}

void 
recordGraphTimingFromPtGraph(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing, bool verbose)
{
  printf("Recording Graph Timing from PtGraph for cell %s\n", 
          graph_timing.cell ? graph_timing.cell->name() : "nullptr");
  fflush(stdout);
  // First copy slews and paths from pt_graph's vertex to graph_timing
  for (PtVertex &pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    // if (!pt_vertex.vertex() || (pt_vertex.type() != PtVertexType::RefInput
    //  && pt_vertex.type() != PtVertexType::RefOutput)) 
    //   continue;
    // First copy slews from pt_vertex to graph_timing
    std::string vertex_name = pt_vertex.vertex()->name(sta->network());
    TimingInfo vertex_timing_info;
    vertex_timing_info.type = TimingType::VERTEX;
    const sta::Slew *slews = pt_vertex.slews();
    vertex_timing_info.slews.clear();
    for (int i = 0; i < pt_vertex.slewCount(); ++i) {
      vertex_timing_info.slews.push_back(slews[i]);
    }
    // Then copy paths (including arrivals and requireds)
    sta::Path *pt_paths = pt_vertex.paths();
    vertex_timing_info.paths.clear();
    int path_count = pt_graph->tagGroup(pt_vertex)->pathCount();
    for (int i = 0; i < path_count; ++i) {
      sta::Path path = pt_paths[i];
      vertex_timing_info.paths.push_back(path);
      printf(" OpenSta: Recorded path for vertex %s: dcalc_pt %u, rf %d, arrival %f, required %f, tagIndex %d\n",
              vertex_name.c_str(),
              path.dcalcAnalysisPt(sta) ? path.dcalcAnalysisPt(sta)->index() : 0,
              path.rfIndex(sta),
              path.arrival() * 1e12,
              path.required() * 1e12,
              path.tagIndex(sta));
      fflush(stdout);
    }
    vertex_timing_info.tag_group_index = pt_vertex.tagGroupIndex();
    graph_timing.vertex_timing_map[vertex_name] = vertex_timing_info;
  }

  // Second copy delays from pt_graph's edges to graph_timing
  std::unordered_map<std::string, int> edge_name_counts;
  
  // Sort edges to ensure deterministic order if ptEdges order is not guaranteed
  // However, PtGraph vector order should be deterministic enough for regression testing
  for (const PtEdge &pt_edge : pt_graph->ptEdges()) {
    PtVertex &pt_to_vertex = pt_graph->ptVertex(pt_edge.ptToId());
    if (!pt_edge.edge() || pt_to_vertex.type() != PtVertexType::RefOutput
     || pt_to_vertex.type() != PtVertexType::RefDriver) {
      continue;
    }
    std::string base_edge_name = pt_edge.edge()->to_string(sta->network());
    
    // Handle multiple edges with same name (e.g. multiple arcs sets between same pins)
    std::string unique_edge_name = base_edge_name;
    if (edge_name_counts.find(base_edge_name) == edge_name_counts.end()) {
      edge_name_counts[base_edge_name] = 0;
    } else {
      edge_name_counts[base_edge_name]++;
      unique_edge_name += "#" + std::to_string(edge_name_counts[base_edge_name]);
    }

    TimingInfo edge_timing_info;
    edge_timing_info.type = TimingType::EDGE;
    const sta::ArcDelay *delays = pt_edge.arcDelays();
    for (int i = 0; i < pt_edge.arcDelayCount(); ++i) {
      edge_timing_info.delays.push_back(delays[i]);
    }
    // Store using the unique name
    graph_timing.edge_timing_map[unique_edge_name] = edge_timing_info;
  }
}

void
TestLrf::testBufferInsertion(char *inst_name, sta::dbSta* sta, 
                             rsz::Resizer *resizer, odb::dbBlock *block)
{
  printf("----- Testing Buffer Insertion (starting from instance %s) -----\n", inst_name);
  fflush(stdout);
  
  // Validate input parameters
  if (!sta) {
    printf("Error: sta is nullptr\n");
    return;
  }
  if (!resizer) {
    printf("Error: resizer is nullptr\n");
    return;
  }
  if (!block) {
    printf("Error: block is nullptr\n");
    return;
  }
  
  printf("Input validation passed\n");
  fflush(stdout);
  
  // Initialize IncreSta and LocalSta
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  sta::dbNetwork *db_network = sta->getDbNetwork();
  sta->findRequireds();
  for (int i = 0; i < 5; ++i) {
    incre_sta->lmUpdate();
  }
  
  if (!db_network) {
    printf("Error: db_network is nullptr\n");
    delete incre_sta;
    return;
  }
  
  printf("IncreSta and LocalSta initialized\n");
  fflush(stdout);
  
  // Initialize resizer
  printf("Initializing resizer...\n");
  fflush(stdout);
  
  resizer->resizePreamble();
  resizer->makeEquivCells();
  
  // Get initial WNS and TNS
  sta::Slack initial_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack initial_tns = sta->totalNegativeSlack(sta::MinMax::max());
  printf("Initial WNS: %.3f ps\n", initial_wns * 1e12);
  printf("Initial TNS: %.3f ps\n", initial_tns * 1e12);
  fflush(stdout);
  
  // Get all instances in the block
  odb::dbSet<odb::dbInst> all_insts = block->getInsts();
  printf("Total instances in block: %u\n", all_insts.size());
  fflush(stdout);
  
  // Find starting instance
  odb::dbInst *start_inst = block->findInst(inst_name);
  bool start_processing = (start_inst == nullptr); // If not found, start from beginning
  
  if (!start_processing) {
    printf("Will start processing from instance: %s\n", inst_name);
  } else {
    printf("Instance %s not found, will process all instances from beginning\n", inst_name);
  }
  fflush(stdout);
  
  // Iterate through all instances
  int instances_processed = 0;
  int total_buffers_inserted = 0;
  
  for (odb::dbInst *db_inst : all_insts) {
    // Skip until we reach the starting instance
    if (!start_processing) {
      if (db_inst == start_inst) {
        start_processing = true;
      } else {
        continue;
      }
    }
    
    sta::Instance *sta_inst = db_network->dbToSta(db_inst);
    if (!sta_inst) {
      continue;
    }
    
    instances_processed++;
    printf("\n===== Processing instance %d: %s =====\n", instances_processed, db_network->name(sta_inst));
    fflush(stdout);
    
    // Get output pins (driver pins) of the instance
    sta::InstancePinIterator *pin_iter = db_network->pinIterator(sta_inst);
    std::vector<const sta::Pin*> driver_pins;
    
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      if (db_network->isDriver(pin)) {
        driver_pins.push_back(pin);
      }
    }
    delete pin_iter;
    
    if (driver_pins.empty()) {
      printf("  No driver pins found, skipping...\n");
      fflush(stdout);
      continue;
    }
    
    printf("  Found %zu driver pins\n", driver_pins.size());
    fflush(stdout);
    
    // Build PtGraph for the instance
    PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, true);
    if (!pt_graph) {
      printf("  Failed to create PtGraph, skipping...\n");
      fflush(stdout);
      continue;
    }
    
    printf("  PtGraph created with %zu vertices and %zu edges\n", 
           pt_graph->ptVertices().size(), 
           pt_graph->ptEdges().size());
    fflush(stdout);
    
    // Create visitor and LrRebuffer for this instance
    ParallelLrVisitor *visitor = new ParallelLrVisitor(sta, local_sta, resizer);
    visitor->setPtGraph(pt_graph);
    
    LrRebuffer *lr_rebuffer = new LrRebuffer(resizer, visitor);
    lr_rebuffer->init();
    
    printf("  LrRebuffer initialized\n");
    fflush(stdout);
    
    // Process each driver pin
    int inst_buffers_inserted = 0;
    for (const sta::Pin *drvr_pin : driver_pins) {
      // Find the corresponding PtVertex for this driver pin
      sta::Vertex *drvr_vertex = db_network->graph()->pinDrvrVertex(drvr_pin);
      if (!drvr_vertex) {
        continue;
      }
      
      PtVertex *drvr_pt_vertex = pt_graph->ptVertex(drvr_vertex);
      if (!drvr_pt_vertex) {
        continue;
      }
      
      // Call rebufferPin
      lr_rebuffer->rebufferPin(drvr_pin, *drvr_pt_vertex);
      int inserted_count = lr_rebuffer->applyBufferingToDb();
      inst_buffers_inserted += inserted_count;
      
      if (inserted_count > 0) {
        printf("    Pin %s: inserted %d buffers\n", db_network->pathName(drvr_pin), inserted_count);
        fflush(stdout);
      }
    }
    
    printf("  Instance total: %d buffers inserted\n", inst_buffers_inserted);
    fflush(stdout);
    
    // Cleanup for this instance
    delete lr_rebuffer;
    delete visitor;
    
    total_buffers_inserted += inst_buffers_inserted;
    
    // If we inserted buffers, update timing and check WNS/TNS
    if (inst_buffers_inserted > 0) {
      printf("  Updating timing after buffer insertion...\n");
      fflush(stdout);
      
      sta->updateTiming(true);
      
      sta::Slack current_wns = sta->worstSlack(sta::MinMax::max());
      sta::Slack current_tns = sta->totalNegativeSlack(sta::MinMax::max());
      
      printf("  Current WNS: %.3f ps (initial: %.3f ps, delta: %.3f ps)\n", 
             current_wns * 1e12, initial_wns * 1e12, (current_wns - initial_wns) * 1e12);
      printf("  Current TNS: %.3f ps (initial: %.3f ps, delta: %.3f ps)\n", 
             current_tns * 1e12, initial_tns * 1e12, (current_tns - initial_tns) * 1e12);
      fflush(stdout);
      
      // Check if WNS or TNS changed
      double wns_delta = std::abs((current_wns - initial_wns) * 1e12);
      double tns_delta = std::abs((current_tns - initial_tns) * 1e12);
      
      if (wns_delta > 1e-6 || tns_delta > 1e-6) {
        printf("\n===== SUCCESS: WNS or TNS changed! =====\n");
        printf("Instance: %s\n", db_network->name(sta_inst));
        printf("Buffers inserted: %d\n", inst_buffers_inserted);
        printf("WNS delta: %.3f ps\n", wns_delta);
        printf("TNS delta: %.3f ps\n", tns_delta);
        printf("Instances processed: %d\n", instances_processed);
        printf("========================================\n");
        fflush(stdout);
        break;
      } else {
        printf("  WNS and TNS did not change significantly, continuing...\n");
        fflush(stdout);
      }
    }
  }
  
  // Final summary
  sta::Slack final_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack final_tns = sta->totalNegativeSlack(sta::MinMax::max());
  
  printf("\n----- Buffer Insertion Summary -----\n");
  printf("Instances processed: %d\n", instances_processed);
  printf("Total buffers inserted: %d\n", total_buffers_inserted);
  printf("Initial WNS: %.3f ps\n", initial_wns * 1e12);
  printf("Final WNS:   %.3f ps (delta: %.3f ps)\n", final_wns * 1e12, (final_wns - initial_wns) * 1e12);
  printf("Initial TNS: %.3f ps\n", initial_tns * 1e12);
  printf("Final TNS:   %.3f ps (delta: %.3f ps)\n", final_tns * 1e12, (final_tns - initial_tns) * 1e12);
  printf("------------------------------------\n");
  fflush(stdout);
  
  if (total_buffers_inserted == 0) {
    printf("No buffers were inserted\n");
    fflush(stdout);
  }
  
  // Cleanup
  delete incre_sta;
}

void
TestLrf::printAllCellsInfo(sta::dbSta* sta, rsz::Resizer *resizer, odb::dbBlock *block)
{
  // Build/ensure equivalence information so we enumerate meaningful cells.
  // Ensure equivalence info is available
  resizer->makeEquivCells();
  const sta::LibertyCellSeq &unique_equiv_cells = sta->equivCellsRecorder()->uniqueEquivCells();

  printf("----- All Liberty Cells Info (by unique equiv cell groups) -----\n");
  fflush(stdout);

  // For each unique representative, treat its equiv_cells as one type/group
  for (const sta::LibertyCell *rep_cell : unique_equiv_cells) {
    if (!rep_cell) continue;
    sta::LibertyCellSeq *equiv_cells = sta->equivCells(const_cast<sta::LibertyCell*>(rep_cell));
    if (!equiv_cells) continue;

    printf("Type Representative: %s, group_size=%zu\n", rep_cell->name(), equiv_cells->size());
    for (sta::LibertyCell *cell : *equiv_cells) {
      if (!cell) continue;

      float area = cell->area();

      // Sum input capacitance across input ports
      float input_cap_sum = 0.0f;
      sta::LibertyCellPortIterator port_iter(cell);
      while (port_iter.hasNext()) {
        sta::LibertyPort *port = port_iter.next();
        if (!port) continue;
        sta::PortDirection *dir = port->direction();
        if (!dir) continue;
        if (dir->isAnyInput()) {
          input_cap_sum += port->capacitance();
        }
      }

      // Compute representative intrinsic delay: take max intrinsicDelay over input ports
      double worst_intrinsic = 0.0;
      sta::LibertyCellPortIterator port_iter2(cell);
      while (port_iter2.hasNext()) {
        sta::LibertyPort *port = port_iter2.next();
        if (!port) continue;
        sta::PortDirection *dir = port->direction();
        if (!dir) continue;
        if (dir->isAnyInput()) {
          sta::ArcDelay d = port->intrinsicDelay(sta);
          if (d > worst_intrinsic) worst_intrinsic = d;
        }
      }

      printf("  %s : area=%g, input_cap_sum=%g F, intrinsic_delay(default)=%g ps\n",
             cell->name(), area, input_cap_sum, worst_intrinsic * 1e12);
    }
    fflush(stdout);
  }

  printf("----- End All Liberty Cells Info -----\n");
  fflush(stdout);
}


void
TestLrf::testSingleInstBuffering(char *inst_name, sta::dbSta* sta,
                                  rsz::Resizer *resizer, odb::dbBlock *block)
{
  printf("----- Test Single Instance Buffering: %s -----\n", inst_name);
  fflush(stdout);

  sta::dbNetwork *db_network = sta->getDbNetwork();
  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("Error: instance '%s' not found\n", inst_name);
    return;
  }

  sta::Instance *inst = db_network->dbToSta(db_inst);
  if (!inst) {
    printf("Error: failed to convert dbInst to sta::Instance\n");
    return;
  }

  // Setup: IncreSta for LM values, resizer preamble
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  sta->findRequireds();
  for (int i = 0; i < 5; i++) {
    incre_sta->lmUpdate();
  }
  resizer->resizePreamble();
  resizer->makeEquivCells();
  LrRebuffer::initGlobalPreamble(sta, resizer);

  // Report initial timing
  sta::Slack wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack tns = sta->totalNegativeSlack(sta::MinMax::max());
  printf("Initial WNS: %.3f ps, TNS: %.3f ps\n", wns * 1e12, tns * 1e12);
  fflush(stdout);

  // [Layer 3] Run visit() twice WITHOUT applying changes; results must be identical
  printf("===== [Layer 3] Stability: calling visit() twice =====\n");
  fflush(stdout);

  for (int round = 1; round <= 2; round++) {
    printf("----- Round %d -----\n", round);
    fflush(stdout);

    ParallelLrVisitor *visitor = new ParallelLrVisitor(sta, local_sta, resizer);
    visitor->init(0, 0, wns, 100.0, nullptr, nullptr);
    visitor->setMoveType(MoveType::BufferInsertion);

    bool success = visitor->visit(inst, sta::object_id_null);
    printf("[Round %d] visit() returned: %s\n", round, success ? "true" : "false");
    fflush(stdout);

    // Do NOT apply changes — discard visitor to restore state
    delete visitor;
  }

  printf("===== [Layer 3] If Round 1 and Round 2 outputs match, graph restore is correct =====\n");
  fflush(stdout);

  // Now apply the real buffering
  printf("===== Applying buffering =====\n");
  ParallelLrVisitor *visitor = new ParallelLrVisitor(sta, local_sta, resizer);
  visitor->init(0, 0, wns, 100.0, nullptr, nullptr);
  visitor->setMoveType(MoveType::BufferInsertion);
  bool success = visitor->visit(inst, sta::object_id_null);

  printf("visit() returned: %s\n", success ? "true" : "false");
  fflush(stdout);

  if (success) {
    // ===== BEFORE apply: report OpenSTA timing on driver output net =====
    const sta::Pin *drvr_pin = visitor->rebuffer()->drvrPin();
    sta::Graph *graph = sta->graph();
    sta::dbNetwork *network = sta->getDbNetwork();

    printf("===== [TIMING CMP] Before Apply: OpenSTA per-pin timing =====\n");
    {
      sta::Vertex *drvr_vertex = graph->pinDrvrVertex(drvr_pin);
      if (drvr_vertex) {
        sta::Slack drvr_slack = sta->vertexSlack(drvr_vertex, sta::MinMax::max());
        sta::Arrival drvr_arr = sta->vertexArrival(drvr_vertex, sta::MinMax::max());
        printf("  [BEFORE] driver %s  arr=%.3f ps  slack=%.3f ps\n",
               drvr_vertex->to_string(sta).c_str(),
               drvr_arr * 1e12, drvr_slack * 1e12);
      }
      // Report load pins
      sta::Net *net = network->net(drvr_pin);
      if (net) {
        sta::NetPinIterator *pin_iter = network->pinIterator(net);
        while (pin_iter->hasNext()) {
          const sta::Pin *pin = pin_iter->next();
          if (pin == drvr_pin) continue;
          sta::Vertex *load_vertex = graph->pinLoadVertex(pin);
          if (load_vertex) {
            sta::Slack load_slack = sta->vertexSlack(load_vertex, sta::MinMax::max());
            sta::Arrival load_arr = sta->vertexArrival(load_vertex, sta::MinMax::max());
            printf("  [BEFORE] load   %s  arr=%.3f ps  slack=%.3f ps\n",
                   load_vertex->to_string(sta).c_str(),
                   load_arr * 1e12, load_slack * 1e12);
          }
        }
        delete pin_iter;
      }
    }
    fflush(stdout);

    // ===== Apply buffering =====
    visitor->applyChangesToDb(resizer);
    printf("Buffering changes applied to DB\n");

    sta->updateTiming(true);
    sta->findRequireds();
    sta::Slack wns_after = sta->worstSlack(sta::MinMax::max());
    sta::Slack tns_after = sta->totalNegativeSlack(sta::MinMax::max());
    printf("After buffering WNS: %.3f ps, TNS: %.3f ps\n",
           wns_after * 1e12, tns_after * 1e12);
    printf("WNS delta: %.3f ps, TNS delta: %.3f ps\n",
           (wns_after - wns) * 1e12, (tns_after - tns) * 1e12);

    // ===== AFTER apply: Compare LOCAL vs OPENSTA arc delays =====
    // Rebuild virtual buffer on PtGraph to collect LOCAL timing, then compare with OPENSTA.
    printf("===== [ARC DELAY COMPARE] LOCAL vs OPENSTA =====\n");
    {
      struct ArcDelayRecord {
        std::string from_port, to_port, edge_label, cond;
        bool is_wire;
        uintptr_t arc_set_id;
        float delay_ps;
      };

      // Helper: get "when" condition string from TimingArcSet
      auto arcSetCond = [](sta::TimingArcSet *aset) -> std::string {
        if (!aset) return "";
        auto *cond = aset->cond();
        return cond ? cond->to_string() : "";
      };

      // Helper: get "inst/port" for PtVertex (real pin -> pathName, virtual -> cell/port)
      auto ptQualifiedName = [&](PtVertex &v) -> std::string {
        if (v.pin()) return network->pathName(v.pin());
        sta::LibertyPort *lp = v.libertyPort();
        if (lp) {
          sta::LibertyCell *cell = lp->libertyCell();
          return std::string(cell ? cell->name() : "?") + "/" + lp->name();
        }
        return "?";
      };

      sta::DcalcAPIndex ap = sta->corners()->dcalcAnalysisPts()[0]->index();
      LrRebuffer *rebuffer = visitor->rebuffer();
      PtGraph *pt_graph = visitor->ptGraph();
      LocalSta *local_sta = rebuffer->local_sta_;
      sta::ArcDelayCalc *arc_delay_calc = visitor->arcDelayCalc();

      // --- Collect LOCAL arc delays by rebuilding virtual buffer ---
      std::vector<ArcDelayRecord> local_arcs;

      // Find driver PtVertex id from driver pin
      sta::Vertex *drvr_sta_vertex = graph->pinDrvrVertex(drvr_pin);
      PtVertex *drvr_ptv = pt_graph->ptVertex(drvr_sta_vertex);
      sta::VertexId drvr_vertex_id = drvr_ptv ? drvr_ptv->objectIdx() : sta::VertexId(0);
      const rsz::BufferedNetPtr &best_bnet = rebuffer->bestBnet();

      if (drvr_ptv && best_bnet) {
        VirtualBufferInfo vinfo = rebuffer->buildVirtualBuffer(drvr_vertex_id, best_bnet);
        if (!vinfo.failed) {
          pt_graph->topoSortVertices();
          local_sta->updateLocalTiming(pt_graph, arc_delay_calc);

          // === Report LMs on all edges ===
          printf("\n  === Edge LM Report ===\n");
          printf("  %-8s %-40s %-6s %12s %12s\n",
                 "type", "from->to", "edge", "delay(ps)", "LM");
          printf("  %-8s %-40s %-6s %12s %12s\n",
                 "----", "--------", "----", "--------", "--------");
          for (const PtEdge &e : pt_graph->ptEdges()) {
            if (e.type() == PtEdgeType::Sentinel) continue;
            if (!e.hasBase() && !e.isVirtual()) continue;
            PtVertex &from_v = pt_graph->ptVertex(e.ptFromId());
            PtVertex &to_v = pt_graph->ptVertex(e.ptToId());
            std::string from_name = ptQualifiedName(from_v);
            std::string to_name = ptQualifiedName(to_v);
            std::string from_to = from_name + "->" + to_name;
            const LMValue *lms = e.arcLms();
            sta::TimingArcSet *arc_set = e.timingArcSet();
            if (!arc_set) continue;
            const char *etype = e.isVirtual()
                ? (e.isWire() ? "v_wire" : "v_gate")
                : (e.isWire() ? "wire" : "gate");
            if (e.isWire()) {
              for (const sta::RiseFall *rf : sta::RiseFall::range()) {
                sta::ArcDelay d = pt_graph->wireArcDelay(e, rf, ap);
                int lm_index = rf->index() * sta->graph()->apCount() + ap;
                float lm_val = (lms != nullptr) ? lms[lm_index] : 0.0f;
                printf("  %-8s %-40s %-6s %12.3f %12.6f%s\n",
                       etype, from_to.c_str(), rf->shortName(),
                       (float)(sta::delayAsFloat(d) * 1e12), lm_val,
                       lms ? "" : " (NULL)");
              }
            } else {
              for (sta::TimingArc *arc : arc_set->arcs()) {
                sta::ArcDelay d = pt_graph->arcDelay(e, arc, ap);
                size_t lm_index = lmIndex(arc, ap, sta->graph()->apCount());
                float lm_val = (lms != nullptr) ? lms[lm_index] : 0.0f;
                std::string label = std::string(arc->fromEdge()->to_string())
                                    + "->" + arc->toEdge()->to_string();
                printf("  %-8s %-40s %-6s %12.3f %12.6f%s\n",
                       etype, from_to.c_str(), label.c_str(),
                       (float)(sta::delayAsFloat(d) * 1e12), lm_val,
                       lms ? "" : " (NULL)");
              }
            }
          }
          printf("  === End Edge LM Report ===\n\n");
          fflush(stdout);

          // 1. Driver instance gate arcs (RefInstEdge)
          PtVertexInEdgeIterator in_iter(drvr_vertex_id, pt_graph);
          while (in_iter.hasNext()) {
            PtEdge &e = in_iter.next();
            if (e.type() != PtEdgeType::RefInstEdge) continue;
            PtVertex &from_v = pt_graph->ptVertex(e.ptFromId());
            PtVertex &to_v = pt_graph->ptVertex(drvr_vertex_id);
            std::string from_name = ptQualifiedName(from_v);
            std::string to_name = ptQualifiedName(to_v);
            sta::TimingArcSet *arc_set = e.timingArcSet();
            if (arc_set) {
              std::string cond = arcSetCond(arc_set);
              for (sta::TimingArc *arc : arc_set->arcs()) {
                sta::ArcDelay d = pt_graph->arcDelay(e, arc, ap);
                std::string label = std::string(arc->fromEdge()->to_string())
                                    + "->" + arc->toEdge()->to_string();
                local_arcs.push_back({from_name, to_name, label, cond, false,
                                      (uintptr_t)arc_set,
                                      (float)(sta::delayAsFloat(d) * 1e12)});
              }
            }
          }

          // 2. Virtual buffer tree arcs (wire + gate)
          for (EdgeId eid : vinfo.edge_ids) {
            PtEdge &e = pt_graph->edge(eid);
            PtVertex &from_v = pt_graph->ptVertex(e.ptFromId());
            PtVertex &to_v = pt_graph->ptVertex(e.ptToId());
            std::string from_name = ptQualifiedName(from_v);
            std::string to_name = ptQualifiedName(to_v);
            if (e.isWire()) {
              for (const sta::RiseFall *rf : sta::RiseFall::range()) {
                sta::ArcDelay d = pt_graph->wireArcDelay(e, rf, ap);
                local_arcs.push_back({from_name, to_name,
                                      rf->shortName(), "", true, 0,
                                      (float)(sta::delayAsFloat(d) * 1e12)});
              }
            } else {
              sta::TimingArcSet *arc_set = e.timingArcSet();
              if (arc_set) {
                std::string cond = arcSetCond(arc_set);
                for (sta::TimingArc *arc : arc_set->arcs()) {
                  sta::ArcDelay d = pt_graph->arcDelay(e, arc, ap);
                  std::string label = std::string(arc->fromEdge()->to_string())
                                      + "->" + arc->toEdge()->to_string();
                  local_arcs.push_back({from_name, to_name, label, cond, false,
                                        (uintptr_t)arc_set,
                                        (float)(sta::delayAsFloat(d) * 1e12)});
                }
              }
            }
          }

          // 3. Collect LOCAL worst arrival/slack per vertex
          // Only collect driver instance + virtual buffer + direct load vertices
          struct VtRecord { std::string name; float arr_ps; float slk_ps; };
          std::vector<VtRecord> local_vt;
          // Driver instance pins
          for (size_t vid = 0; vid < pt_graph->vertexCount(); vid++) {
            PtVertex &ptv = pt_graph->ptVertex(sta::VertexId(vid));
            if (ptv.type() == PtVertexType::Sentinel) continue;
            // Include: driver instance pins (RefInput/RefOutput) and virtual vertices
            if (ptv.type() != PtVertexType::RefInput
                && ptv.type() != PtVertexType::RefOutput
                && ptv.type() != PtVertexType::VirtualInput
                && ptv.type() != PtVertexType::VirtualOutput) continue;
            sta::Path *worst = local_sta->ptVertexWorstSlackPath(ptv, sta::MinMax::max());
            if (worst) {
              local_vt.push_back({ptQualifiedName(ptv),
                                  (float)(worst->arrival() * 1e12),
                                  (float)(worst->slack(local_sta) * 1e12)});
            }
          }
          // Also include direct load pins (RefDriver that are wire targets)
          for (EdgeId eid : vinfo.edge_ids) {
            PtEdge &e = pt_graph->edge(eid);
            if (!e.isWire()) continue;
            PtVertex &to_v = pt_graph->ptVertex(e.ptToId());
            if (to_v.type() == PtVertexType::RefDriver
                || to_v.type() == PtVertexType::RefInput) {
              sta::Path *worst = local_sta->ptVertexWorstSlackPath(to_v, sta::MinMax::max());
              if (worst) {
                local_vt.push_back({ptQualifiedName(to_v),
                                    (float)(worst->arrival() * 1e12),
                                    (float)(worst->slack(local_sta) * 1e12)});
              }
            }
          }

          rebuffer->removeVirtualBuffer(vinfo);

          // --- Collect OPENSTA arc delays ---
          std::vector<ArcDelayRecord> opensta_arcs;
          sta::Vertex *drvr_vertex = graph->pinDrvrVertex(drvr_pin);

          // OPENSTA driver gate arcs
          if (drvr_vertex) {
            sta::VertexInEdgeIterator oin_iter(drvr_vertex, graph);
            while (oin_iter.hasNext()) {
              sta::Edge *edge = oin_iter.next();
              if (edge->role()->isWire()) continue;
              sta::Vertex *from_v = edge->from(graph);
              const sta::Pin *from_pin = from_v->pin();
              std::string from_port = from_pin ? network->pathName(from_pin) : "?";
              const sta::Pin *to_pin_v = drvr_vertex->pin();
              std::string to_port = to_pin_v ? network->pathName(to_pin_v) : "?";
              sta::TimingArcSet *aset = edge->timingArcSet();
              std::string cond = arcSetCond(aset);
              for (sta::TimingArc *arc : aset->arcs()) {
                sta::ArcDelay d = graph->arcDelay(edge, arc, ap);
                std::string label = std::string(arc->fromEdge()->to_string())
                                    + "->" + arc->toEdge()->to_string();
                opensta_arcs.push_back({from_port, to_port, label, cond, false,
                                        (uintptr_t)aset, (float)(d * 1e12)});
              }
            }
          }

          // OPENSTA BFS: buffer tree wire + gate arcs
          // Helper: get qualified name for OPENSTA pins (buffer pins -> cell/port)
          auto openStaQualName = [&](const sta::Pin *pin) -> std::string {
            if (!pin) return "?";
            sta::Instance *inst = network->instance(pin);
            sta::LibertyCell *cell = network->libertyCell(inst);
            if (cell && cell->isBuffer())
              return std::string(cell->name()) + "/" + network->portName(pin);
            return network->pathName(pin);
          };

          std::vector<sta::Vertex*> queue;
          if (drvr_vertex) queue.push_back(drvr_vertex);
          while (!queue.empty()) {
            sta::Vertex *cur = queue.back();
            queue.pop_back();
            std::string cur_port = openStaQualName(cur->pin());
            sta::VertexOutEdgeIterator out_iter(cur, graph);
            while (out_iter.hasNext()) {
              sta::Edge *edge = out_iter.next();
              sta::Vertex *to_v = edge->to(graph);
              if (!edge->role()->isWire()) continue;
              const sta::Pin *to_pin = to_v->pin();
              std::string to_full = openStaQualName(to_pin);
              for (const sta::RiseFall *rf : sta::RiseFall::range()) {
                sta::ArcDelay wd = graph->wireArcDelay(edge, rf, ap);
                opensta_arcs.push_back({cur_port, to_full, rf->shortName(), "", true,
                                        0, (float)(wd * 1e12)});
              }
              // BFS through buffers
              if (to_pin && network->isLeaf(to_pin)) {
                sta::Instance *to_inst = network->instance(to_pin);
                sta::LibertyCell *to_cell = network->libertyCell(to_inst);
                if (to_cell && to_cell->isBuffer()) {
                  sta::InstancePinIterator *ipin_iter = network->pinIterator(to_inst);
                  while (ipin_iter->hasNext()) {
                    const sta::Pin *ipin = ipin_iter->next();
                    if (network->direction(ipin)->isOutput()) {
                      sta::Vertex *out_v = graph->pinDrvrVertex(ipin);
                      if (out_v) {
                        // Buffer gate arcs
                        sta::VertexInEdgeIterator buf_in_iter(out_v, graph);
                        while (buf_in_iter.hasNext()) {
                          sta::Edge *gate_edge = buf_in_iter.next();
                          if (gate_edge->role()->isWire()) continue;
                          std::string gf_port = openStaQualName(gate_edge->from(graph)->pin());
                          std::string gt_port = openStaQualName(out_v->pin());
                          sta::TimingArcSet *gaset = gate_edge->timingArcSet();
                          std::string gcond = arcSetCond(gaset);
                          for (sta::TimingArc *arc : gaset->arcs()) {
                            sta::ArcDelay gd = graph->arcDelay(gate_edge, arc, ap);
                            std::string label = std::string(arc->fromEdge()->to_string())
                                                + "->" + arc->toEdge()->to_string();
                            opensta_arcs.push_back({gf_port, gt_port, label, gcond, false,
                                                    (uintptr_t)gaset, (float)(gd * 1e12)});
                          }
                        }
                        queue.push_back(out_v);
                      }
                    }
                  }
                  delete ipin_iter;
                }
              }
            }
          }

          // --- Key-based matching ---
          using ArcKey = std::tuple<std::string, std::string, std::string, bool, uintptr_t>;
          struct ArcVal { float delay_ps; std::string cond; };
          std::map<ArcKey, ArcVal> opensta_map;
          for (const auto &o : opensta_arcs) {
            opensta_map[{o.from_port, o.to_port, o.edge_label, o.is_wire, o.arc_set_id}]
                = {o.delay_ps, o.cond};
          }

          float max_err = 0, sum_err = 0;
          size_t matched = 0, unmatched_local = 0;
          printf("  %-8s %-30s %-6s %-20s %10s %10s %10s\n",
                 "type", "from->to", "edge", "when", "LOCAL", "OPENSTA", "err");
          printf("  %-8s %-30s %-6s %-20s %10s %10s %10s\n",
                 "----", "--------", "----", "----", "-----", "-------", "---");

          for (const auto &l : local_arcs) {
            ArcKey key = {l.from_port, l.to_port, l.edge_label, l.is_wire, l.arc_set_id};
            std::string from_to = l.from_port + "->" + l.to_port;
            std::string cond_disp = l.cond.empty() ? "-" : l.cond;
            auto it = opensta_map.find(key);
            if (it != opensta_map.end()) {
              float err = l.delay_ps - it->second.delay_ps;
              if (std::abs(err) > max_err) max_err = std::abs(err);
              sum_err += std::abs(err);
              matched++;
              printf("  %-8s %-30s %-6s %-20s %10.3f %10.3f %10.3f\n",
                     l.is_wire ? "wire" : "gate", from_to.c_str(),
                     l.edge_label.c_str(), cond_disp.c_str(),
                     l.delay_ps, it->second.delay_ps, err);
              opensta_map.erase(it);
            } else {
              unmatched_local++;
              printf("  %-8s %-30s %-6s %-20s %10.3f %10s %10s\n",
                     l.is_wire ? "wire" : "gate", from_to.c_str(),
                     l.edge_label.c_str(), cond_disp.c_str(),
                     l.delay_ps, "N/A", "---");
            }
          }
          for (const auto &[key, val] : opensta_map) {
            const auto &[fp, tp, el, iw, asid] = key;
            std::string cd = val.cond.empty() ? "-" : val.cond;
            printf("  %-8s %-30s %-6s %-20s %10s %10.3f %10s\n",
                   iw ? "wire" : "gate", (fp + "->" + tp).c_str(),
                   el.c_str(), cd.c_str(), "N/A", val.delay_ps, "---");
          }
          printf("  --- Arc Delay Summary: %zu matched, %zu LOCAL-only, %zu OPENSTA-only, "
                 "max_err=%.3f ps, avg_err=%.3f ps ---\n",
                 matched, unmatched_local, opensta_map.size(),
                 max_err, matched > 0 ? sum_err / matched : 0.0f);

          // --- Worst arrival/slack comparison ---
          printf("\n  === Worst Arrival / Slack Comparison (max) ===\n");
          printf("  %-30s %12s %12s %12s %12s %12s %12s\n",
                 "pin", "L_arr(ps)", "O_arr(ps)", "arr_err", "L_slk(ps)", "O_slk(ps)", "slk_err");
          printf("  %-30s %12s %12s %12s %12s %12s %12s\n",
                 "---", "--------", "--------", "-------", "--------", "--------", "-------");

          // Collect OPENSTA worst arrival/slack
          auto getOpenStaWorst = [&](sta::Vertex *v) -> std::pair<float, float> {
            float worst_arr = -1e30, worst_slk = 1e30;
            bool found = false;
            sta::VertexPathIterator path_iter(v, sta);
            while (path_iter.hasNext()) {
              sta::Path *path = path_iter.next();
              if (path->pathAnalysisPt(sta)->pathMinMax() != sta::MinMax::max()) continue;
              float arr = path->arrival() * 1e12;
              float slk = path->slack(sta) * 1e12;
              if (!found || slk < worst_slk) { worst_arr = arr; worst_slk = slk; found = true; }
            }
            return {worst_arr, worst_slk};
          };

          std::map<std::string, std::pair<float, float>> opensta_vt;
          // Driver instance pins
          sta::Instance *drv_inst = network->instance(drvr_pin);
          sta::InstancePinIterator *dpin_iter = network->pinIterator(drv_inst);
          while (dpin_iter->hasNext()) {
            const sta::Pin *pin = dpin_iter->next();
            sta::Vertex *v = graph->pinDrvrVertex(pin);
            if (!v) v = graph->pinLoadVertex(pin);
            if (v) opensta_vt[network->pathName(pin)] = getOpenStaWorst(v);
          }
          delete dpin_iter;
          // Buffer tree + load pins via BFS
          std::vector<sta::Vertex*> vtq;
          if (drvr_vertex) vtq.push_back(drvr_vertex);
          while (!vtq.empty()) {
            sta::Vertex *cur = vtq.back(); vtq.pop_back();
            sta::VertexOutEdgeIterator oe(cur, graph);
            while (oe.hasNext()) {
              sta::Edge *edge = oe.next();
              sta::Vertex *to_v = edge->to(graph);
              const sta::Pin *tp = to_v->pin();
              if (!edge->role()->isWire() || !tp || !network->isLeaf(tp)) continue;
              sta::Instance *ti = network->instance(tp);
              sta::LibertyCell *tc = network->libertyCell(ti);
              if (tc && tc->isBuffer()) {
                opensta_vt[openStaQualName(tp)] = getOpenStaWorst(to_v);
                sta::InstancePinIterator *bpi = network->pinIterator(ti);
                while (bpi->hasNext()) {
                  const sta::Pin *bp = bpi->next();
                  if (network->direction(bp)->isOutput()) {
                    sta::Vertex *bov = graph->pinDrvrVertex(bp);
                    if (bov) {
                      opensta_vt[openStaQualName(bp)] = getOpenStaWorst(bov);
                      vtq.push_back(bov);
                    }
                  }
                }
                delete bpi;
              } else {
                opensta_vt[network->pathName(tp)] = getOpenStaWorst(to_v);
              }
            }
          }

          for (const auto &lv : local_vt) {
            auto it = opensta_vt.find(lv.name);
            if (it != opensta_vt.end()) {
              printf("  %-30s %12.3f %12.3f %12.3f %12.3f %12.3f %12.3f\n",
                     lv.name.c_str(),
                     lv.arr_ps, it->second.first, lv.arr_ps - it->second.first,
                     lv.slk_ps, it->second.second, lv.slk_ps - it->second.second);
            } else {
              printf("  %-30s %12.3f %12s %12s %12.3f %12s %12s\n",
                     lv.name.c_str(), lv.arr_ps, "N/A", "---", lv.slk_ps, "N/A", "---");
            }
          }
          fflush(stdout);
        } else {
          printf("  buildVirtualBuffer failed for best option\n");
          rebuffer->removeVirtualBuffer(vinfo);
        }
      } else {
        printf("  No driver PtVertex or best option found\n");
      }
    }
    printf("===== [ARC DELAY COMPARE] End =====\n");
  } else {
    printf("No buffering applied (visit returned false)\n");
  }
  fflush(stdout);

  delete visitor;
  delete incre_sta;

  printf("----- End Test Single Instance Buffering -----\n");
  fflush(stdout);
}

void
TestLrf::testParallelKKTProjection(sta::dbSta* sta,
                                    rsz::Resizer *resizer,
                                    odb::dbBlock *block,
                                    size_t thread_num,
                                    std::string lr_helper_method)
{
  printf("----- Testing Parallel KKT Projection (serial vs %zu threads) -----\n",
         thread_num);
  fflush(stdout);

  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->findRequireds();

  // Create IncreSta with thread_num threads
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();

  // Initialize LM values via one round of updateAllEdgeLms
  lr_helper->ensureSorted(sta);
  lr_helper->updateAllEdgeLms(sta);

  // Record LM state before serial KKT
  int pre_serial_frame = lr_helper->recordLM();

  // --- Serial KKT ---
  auto serial_start = std::chrono::high_resolution_clock::now();
  bool serial_result = lr_helper->KKTProjection(sta);
  auto serial_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> serial_elapsed = serial_end - serial_start;
  printf("Serial KKT: %s, took %f seconds\n",
         serial_result ? "satisfied" : "NOT satisfied", serial_elapsed.count());

  // Record LM state after serial KKT
  int post_serial_frame = lr_helper->recordLM();

  // Restore to pre-serial state for parallel run
  lr_helper->restoreLM(pre_serial_frame);

  // --- Parallel KKT ---
  auto parallel_start = std::chrono::high_resolution_clock::now();
  bool parallel_result = lr_helper->parallelKKTProjection(sta);
  auto parallel_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> parallel_elapsed = parallel_end - parallel_start;
  printf("Parallel KKT (%zu threads): %s, took %f seconds\n",
         thread_num,
         parallel_result ? "satisfied" : "NOT satisfied",
         parallel_elapsed.count());
  printf("Speedup: %.2fx\n",
         serial_elapsed.count() / std::max(parallel_elapsed.count(), 1e-9));

  // Record LM state after parallel KKT
  int post_parallel_frame = lr_helper->recordLM();

  // --- Compare arc LMs between serial and parallel results ---
  // Restore serial result, snapshot it, then restore parallel and compare
  lr_helper->restoreLM(post_serial_frame);
  // Collect serial arc LMs
  sta::Graph *graph = sta->graph();
  size_t total_arcs = 0;
  size_t mismatch_count = 0;
  double max_rel_error = 0.0;

  // First pass: count arcs
  sta::VertexIterator vert_iter(graph);
  while (vert_iter.hasNext()) {
    sta::Vertex *v = vert_iter.next();
    sta::VertexOutEdgeIterator edge_iter(v, graph);
    while (edge_iter.hasNext()) {
      sta::Edge *edge = edge_iter.next();
      for (sta::TimingArc *arc : edge->timingArcSet()->arcs()) {
        for (size_t ap = 0; ap < graph->apCount(); ap++) {
          total_arcs++;
        }
      }
    }
  }

  // Snapshot serial LMs
  std::vector<LMValue> serial_lms(total_arcs);
  size_t idx = 0;
  sta::VertexIterator vert_iter2(graph);
  while (vert_iter2.hasNext()) {
    sta::Vertex *v = vert_iter2.next();
    sta::VertexOutEdgeIterator edge_iter(v, graph);
    while (edge_iter.hasNext()) {
      sta::Edge *edge = edge_iter.next();
      LMValue const *lms = edge->arcLms();
      for (sta::TimingArc *arc : edge->timingArcSet()->arcs()) {
        for (size_t ap = 0; ap < graph->apCount(); ap++) {
          size_t lm_idx = arc->index() * graph->apCount() + ap;
          serial_lms[idx++] = lms[lm_idx];
        }
      }
    }
  }

  // Restore parallel result and compare
  lr_helper->restoreLM(post_parallel_frame);
  idx = 0;
  sta::VertexIterator vert_iter3(graph);
  while (vert_iter3.hasNext()) {
    sta::Vertex *v = vert_iter3.next();
    sta::VertexOutEdgeIterator edge_iter(v, graph);
    while (edge_iter.hasNext()) {
      sta::Edge *edge = edge_iter.next();
      LMValue const *lms = edge->arcLms();
      for (sta::TimingArc *arc : edge->timingArcSet()->arcs()) {
        for (size_t ap = 0; ap < graph->apCount(); ap++) {
          size_t lm_idx = arc->index() * graph->apCount() + ap;
          LMValue parallel_val = lms[lm_idx];
          LMValue serial_val = serial_lms[idx++];
          if (serial_val != 0.0) {
            double rel_error = std::abs(parallel_val - serial_val) / std::abs(serial_val);
            if (rel_error > 1e-6) {
              mismatch_count++;
              if (rel_error > max_rel_error)
                max_rel_error = rel_error;
            }
          } else if (parallel_val != 0.0) {
            mismatch_count++;
          }
        }
      }
    }
  }

  printf("Arc LM comparison: %zu total arcs, %zu mismatches, max relative error: %e\n",
         total_arcs, mismatch_count, max_rel_error);
  if (mismatch_count == 0) {
    printf("PASS: Serial and parallel KKT produce identical results.\n");
  } else {
    printf("FAIL: Serial and parallel KKT produce different results!\n");
  }
  fflush(stdout);

  delete incre_sta;
  printf("----- End Test Parallel KKT Projection -----\n");
  fflush(stdout);
}

}  // namespace lrf
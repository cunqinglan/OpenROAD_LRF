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
#include "ParallelVisitor.hh"
#include "sta/DispatchQueue.hh"
#include "TaskArranger.hh"
#include "sta/PowerClass.hh"
#include "sta/Delay.hh"
#include "est/EstimateParasitics.h"
#include "sta/EquivCells.hh"
#include "sta/Path.hh"
#include "search/TagGroup.hh"
#include "odb/db.h"
  
#include <cmath>
#include <unordered_map>
#include <vector>

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
  printf("Swapping to equiv cell: %s from %s\n", swap_to_cell->name(), orig_cell->name());
  // Vitually replace cell in LocalSta and compute delays and arrivals
  PtGraph *pt_graph_local = local_sta->makePtGraph(sta_inst, true);
  local_sta->virtualReplaceCell(pt_graph_local, swap_to_cell);
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  local_sta->findLocalDelays(pt_graph_local, arc_delay_calc);
  local_sta->setDebugLabel("LocalSTA");
  local_sta->findLocalArrivals(pt_graph_local);
  local_sta->findLocalRequireds(pt_graph_local);

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

  local_sta->virtualReplaceCell(pt_graph_local, swap_to_cell1);
  local_sta->findLocalDelays(pt_graph_local, arc_delay_calc);
  local_sta->findLocalArrivals(pt_graph_local);
  local_sta->findLocalRequireds(pt_graph_local);

  printf("Swapping to equiv cell: %s from %s\n", swap_to_cell1->name(), orig_cell->name());
  odb::dbMaster *to_master1 = db_network->staToDb(swap_to_cell1);
  db_inst->swapMaster(to_master1);
  sta->updateTiming(true);
  sta->findRequireds();
  PtGraph *pt_graph_orig = local_sta->makePtGraph(sta_inst, true);

  comparePtGraphs(pt_graph_orig, pt_graph_local, sta);

  pt_graph_orig->printGraph("dotfile", true);
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
    visitors.emplace_back(std::make_unique<ParallelLrVisitor>(sta, local_sta));
  }

  std::vector<ParallelLrVisitor*> visitor_ptrs;
  for(auto& v : visitors) visitor_ptrs.push_back(v.get());

  for (size_t i = 0; i < n; ++i) {    
    // 投递一个任务，执行 visit
    sta::Instance *sta_inst = insts[i];
    dq.dispatch([visitor_ptrs, sta_inst](int id) {
      if (id >= 0 && id < visitor_ptrs.size()) {
        visitor_ptrs[id]->visit(sta_inst);
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
                            float PT_tradeoff)
{
  // Test parallel LR resizing
  printf("----- Testing Parallel LR Resizing -----\n");
  sta::Corner *corner = sta->corners()->findCorner("default");
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  sta->searchPreamble();
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  int thread_count = local_sta->threadCount();
  incre_sta->setMaxResizeNum(max_resize_num); // Limit max resize number per iteration
  printf("Thread count has set to %d\n", thread_count);
  // Conduct iterative resizing
  incre_sta->lmUpdate();

  odb::dbDatabase::beginEco(block);
  float best_leakage = 0;
  size_t no_improve_count_ = 0;
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
  for (size_t i = 0; i < iterations; ++i) {
    sta->findRequireds();
    printf("----- LR Resizing Iteration %zu -----\n", i+1);
    incre_sta->parallelResize(resizer, avg_delay, avg_leakage, PT_tradeoff);
    incre_sta->lmUpdate();

    est_parasitics->updateWireParasiticsNoDeleteNetwork();
    // After resizing, evaluate timing and power
    sta->delaysInvalid();
    sta->updateTiming(true);
    tns = sta->totalNegativeSlack(sta::MinMax::max());
    wns = sta->worstSlack(sta::MinMax::max());
    float leakage = 0;
    odb::dbSet<dbInst> insts = block->getInsts();
    for (odb::dbInst *inst : insts) {
      sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(inst);
      if (!sta_inst) continue;
      sta::PowerResult power_result = sta->power(sta_inst, corner);
      leakage += power_result.leakage();
    }
    
    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f\n", leakage * 1e10);
    fflush(stdout);
    if ( wns > best_wns ) {
      best_wns = wns;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      printf("Improvement in WNS, accepting new design.\n");
      no_improve_count_ = 0;
    } 
    else if (no_improve_count_ < num_no_improve_tolerance) {
      printf("No improvement in WNS, but within tolerance, accepting new design.\n");
      continue;
    } 
    else {
      no_improve_count_++;
      printf("No improvement in WNS for 3 iterations, reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      odb::dbDatabase::beginEco(block);
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    printf("Final design accepted with WNS: %f\n", wns * 1e12);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    printf("Reverted to best design with WNS: %f\n", best_wns * 1e12);
  }
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
  lrf::IncreSta *incre_sta = new IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();
  // incre_sta->lmUpdate();
  resizer->makeEquivCells();

  std::vector<sta::Instance*> sta_insts;
  for (auto *db_inst : db_insts) {
    sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(db_inst);
    sta_insts.push_back(sta_inst);
  }

  local_sta->initParallel();
  float average_delay = incre_sta->averageDelayOnCritPath();
  float average_leakage = incre_sta->averageLeakage();
  lrf::ParallelLrVisitor *visitor = new lrf::ParallelLrVisitor(sta, local_sta);
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
  lrf::IncreSta *incre_sta = new lrf::IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  for (auto *sta_inst : sta_insts) {
    /////////
    TimingRecord inst_timing_record;
    inst_timing_record.inst = sta_inst;
    inst_timing_record.orig_cell = sta->network()->libertyCell(sta_inst);
    /////////
    sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
    
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

    int cnt = 1;
    for (auto *equiv_cell : legal_equiv_cells) {
      cnt++;
      if (cnt > 2) break; // Only test first 2 equiv cells
      GraphTiming cell_graph_timing;
      cell_graph_timing.cell = equiv_cell;
      printf("Collecting timing info for instance %s with equiv cell %s\n", 
              sta->getDbNetwork()->name(sta_inst), equiv_cell->name());
      fflush(stdout);

      odb::dbMaster *master = sta->getDbNetwork()->staToDb(equiv_cell);
      odb::dbInst *db_inst = sta->getDbNetwork()->staToDb(sta_inst);
      db_inst->swapMaster(master);
      // sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();
      lrf::PtGraph *pt_graph = local_sta->makePtGraph(sta_inst, true);

      // We can further collect slacks here
      recordGraphTimingFromPtGraph(sta, pt_graph, cell_graph_timing);
      inst_timing_record.liberty_timing_map[std::string(equiv_cell->name())] = cell_graph_timing;
      pt_graph->printGraph("dotfile", true);
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
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new lrf::IncreSta(sta, 1);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  lrf::ParallelLrVisitor *visitor = new lrf::ParallelLrVisitor(sta, local_sta);
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
recordGraphTimingFromPtGraph(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing)
{
  printf("OpenSta::Recording Graph Timing from PtGraph for cell %s\n", 
          graph_timing.cell ? graph_timing.cell->name() : "nullptr");
  fflush(stdout);
  // First copy slews and paths from pt_graph's vertex to graph_timing
  for (PtVertex &pt_vertex : pt_graph->ptVertices()) {
    if (!pt_vertex.vertex() || pt_vertex.type() != PtVertexType::RefInput
  || pt_vertex.type() != PtVertexType::RefOutput) 
      continue;
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


}  // namespace lrf
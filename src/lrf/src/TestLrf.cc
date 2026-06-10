#include <map>
#include <tuple>
#include <tcl.h>
#include "db_sta/dbSta.hh"
#include "sta/ArcDelayCalc.hh"
#include "sta/Sdc.hh"
#include "sta/Clock.hh"
#include "rsz/Resizer.hh"
#include "sta/Liberty.hh"
#include "LocalSta.hh"
#include "LrHelper.hh"
#include "lrf/IncreSta.hh"
#include "lrf/TestLrf.hh"
#include "LrfUtil.hh"
#include "PlacementDensityMap.hh"
#include "Initializer.hh"
#include "ParallelInitializer.hh"
#include "odb/db.h"
#include "sta/Liberty.hh"
#include "sta/Scene.hh"
#include "sta/FuncExpr.hh"
#include "LocalSearch.hh"
#include "PtGraph.hh"
#include "NetlistTransformation.hh"
#include "LrRebuffer.hh"
#include "TestRebuffer.hh"
#include "sta/DispatchQueue.hh"
#include "TaskArranger.hh"
#include "sta/TimingRole.hh"
#include "sta/PowerClass.hh"
#include "sta/Delay.hh"
#include "est/EstimateParasitics.h"
#include "sta/EquivCells.hh"
#include "sta/Path.hh"
#include "EcoController.hh"
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

// ── Slew violation check helper ─────────────────────────────────
// Returns total slew violation (ns) and count.
static void
checkSlewViolations(sta::dbSta *sta, odb::dbBlock *block,
                    LocalSta *local_sta, const char *label,
                    double &total_ns, size_t &count)
{
  total_ns = 0.0;
  count = 0;
  sta::dbNetwork *db_net = sta->getDbNetwork();
  for (odb::dbITerm *iterm : block->getITerms()) {
    odb::dbNet *net = iterm->getNet();
    if (!net) continue;
    auto sig = net->getSigType();
    if (sig == odb::dbSigType::POWER || sig == odb::dbSigType::GROUND
        || sig == odb::dbSigType::CLOCK)
      continue;
    odb::dbMTerm *mterm = iterm->getMTerm();
    if (!mterm) continue;
    sta::LibertyPort *lib_port = sta->network()->libertyPort(
        db_net->dbToSta(mterm));
    if (!lib_port) continue;
    float limit = local_sta->getPortMaxSlewLimit(lib_port);
    if (limit <= 0 || limit >= 1.0) continue;
    sta::Pin *sta_pin = db_net->dbToSta(iterm);
    if (!sta_pin) continue;
    sta::Vertex *vtx = sta->graph()->pinLoadVertex(sta_pin);
    if (!vtx) vtx = sta->graph()->pinDrvrVertex(sta_pin);
    if (!vtx) continue;
    float slew = 0.0;
    for (const sta::RiseFall *rf : sta::RiseFall::range())
      for (sta::DcalcAPIndex dap = 0; dap < sta->graph()->apCount(); dap++)
        slew = std::max(slew, (float)delayAsFloat(
            sta->graph()->slew(vtx, rf, dap)));
    if (slew > limit) {
      total_ns += (slew - limit) * 1e9;
      count++;
    }
  }
  if (lrfVerbose()) {
    if (count > 0)
      printf("[VIOL] %s: %zu slew violations, total=%.4f ns\n", label, count, total_ns);
    else
      printf("[VIOL] %s: No slew violation\n", label);
  }
}

// ── Checkpoint save helper ──────────────────────────────────────
// Called at first regression point to save ODB + LM.
// ODB contains tech + netlist + placement (complete, lossless).
// LM is saved separately (not part of ODB).
static void
saveCheckpoint(const std::string &checkpoint_dir,
               sta::dbSta *sta, odb::dbBlock *block,
               IncreSta *incre_sta)
{
  if (checkpoint_dir.empty())
    return;

  // Ensure directory exists
  std::string mkdir_cmd = "mkdir -p " + checkpoint_dir;
  system(mkdir_cmd.c_str());

  std::string design_name = block->getName();
  std::string lm_path  = checkpoint_dir + "/checkpoint.lm";
  std::string odb_path = checkpoint_dir + "/checkpoint.odb";

  // Save LM snapshot
  bool lm_ok = incre_sta->saveLmToFile(lm_path, design_name);
  printf("[CHECKPOINT] LM snapshot: %s (%s)\n",
         lm_path.c_str(), lm_ok ? "ok" : "FAILED");

  // Save ODB (complete: tech + netlist + placement)
  Tcl_Interp *interp = sta->tclInterp();
  std::string odb_cmd = "write_db " + odb_path;
  Tcl_Eval(interp, odb_cmd.c_str());
  printf("[CHECKPOINT] ODB: %s\n", odb_path.c_str());

  printf("[CHECKPOINT] Saved to %s (design=%s)\n",
         checkpoint_dir.c_str(), design_name.c_str());
  fflush(stdout);
}

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
      for (sta::DcalcAPIndex dcalc_ptr = 0; dcalc_ptr < sta->graph()->apCount(); dcalc_ptr++) {
        sta::ArcDelay local_delay = local_pt_graph->arcDelay(local_edge, arc, dcalc_ptr) * 1e12;
        sta::ArcDelay open_delay = open_pt_graph->arcDelay(open_edge, arc, dcalc_ptr) * 1e12;
        double delay_diff = std::abs(local_delay - open_delay);
        if (delay_diff > 1e-5) {
          printf("Delay mismatch for arc %s of edge %s in dcalc_pt %u: local=%f, open=%f, diff=%f\n", 
                  arc->to_string().c_str(), local_edge_obj->to_string(sta->graph()).c_str(), dcalc_ptr, local_delay, open_delay, delay_diff);
          
          // Debugging input slew and output load
          const sta::RiseFall *in_rf = arc->fromEdge()->asRiseFall();
          const sta::RiseFall *out_rf = arc->toEdge()->asRiseFall();
          
          // Get Slews
          // Note: PtGraph stores slews on vertices.
          // Input slew is at the "from" vertex of the edge.
          const PtVertex &from_vertex = local_pt_graph->ptVertex(local_edge.ptFromId());
          // Output Load is harder to get directly from PtGraph result, 
          const PtVertex &to_vertex = local_pt_graph->ptVertex(local_edge.ptToId());
          
          sta::Slew local_in_slew = local_pt_graph->slew(from_vertex, in_rf, dcalc_ptr); 
          // OpenSTA global graph slew
          sta::Slew open_in_slew = sta->graph()->slew(open_edge.edge()->from(sta->graph()), in_rf, dcalc_ptr);

          sta::Slew local_out_slew = local_pt_graph->slew(to_vertex, out_rf, dcalc_ptr);
          sta::Slew open_out_slew = sta->graph()->slew(open_edge.edge()->to(sta->graph()), out_rf, dcalc_ptr);
          
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
      if (local_path->dcalcAnalysisPtIndex(sta) != open_path->dcalcAnalysisPtIndex(sta)) {
        printf("DcalcApIndex mismatch at vertex index %zu: Local=%u, Open=%u\n", 
               i,
               local_path->dcalcAnalysisPtIndex(sta),
               open_path->dcalcAnalysisPtIndex(sta));
        fflush(stdout);
        same = false;
        continue;
      }
      sta::Arrival local_arrival = local_path->arrival() * 1e12;
      sta::Arrival open_arrival = open_path->arrival() * 1e12;
      double arrival_diff = std::abs(local_arrival - open_arrival);
      if (arrival_diff > 1e-9) {
        printf("Arrival mismatch at vertex %s for dcalc_pt %u, pathIdx = %u, Local arrival %f, Open arrival %f, arrival difference = %f\n", 
                local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPtIndex(sta), cnt, local_arrival, open_arrival, arrival_diff);
        fflush(stdout);
        same = false;
      } 

      sta::Required local_required = local_path->required() * 1e12;
      sta::Required open_required = open_path->required() * 1e12;
      double required_diff = std::abs(local_required - open_required);
      if (required_diff > 1e-9) {
        printf("Required mismatch at vertex %s for dcalc_pt %u, pathIdx = %u, Local required %f, Open required %f, required difference = %f\n", 
                local_vertex_obj->name(sta->network()), local_path->dcalcAnalysisPtIndex(sta), cnt, local_required, open_required, required_diff);
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
            if (p1.dcalcAnalysisPtIndex(sta) != p2.dcalcAnalysisPtIndex(sta)) {
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

////////////////////////////////////////////////////////////////
// Stage 1: FF endpoint LM dump
////////////////////////////////////////////////////////////////

namespace {

// Print one (edge, arc) record with LM, AAT, RAT, slack and clamp/floor flags.
// `side` is a short tag like "D-in", "Q-fan", or "CK-Q" for the human reader.
void
printFFArcRecord(sta::dbSta *sta, sta::Edge *edge, sta::TimingArc *arc,
                 sta::DcalcAPIndex dcalc_ap, size_t ap_index,
                 size_t ap_count, const sta::MinMax *minmax,
                 const char *side)
{
  const sta::RiseFall *from_rf = arc->fromEdge()->asRiseFall();
  const sta::RiseFall *to_rf   = arc->toEdge()->asRiseFall();
  if (!from_rf || !to_rf) return;

  sta::Graph *graph = sta->graph();
  sta::Vertex *from_v = edge->from(graph);
  sta::Vertex *to_v   = edge->to(graph);
  std::string from_pin = sta->network()->pathName(from_v->pin());
  std::string to_pin   = sta->network()->pathName(to_v->pin());

  sta::Arrival from_aat = sta->arrival(from_v, from_rf->asRiseFallBoth(), sta->scenes(), minmax);
  sta::Required to_rat  = sta->required(to_v, to_rf->asRiseFallBoth(), sta->scenes(), minmax);
  sta::Delay delay      = sta->arcDelay(edge, arc, dcalc_ap);
  sta::Slack arc_slack  = to_rat - (from_aat + delay);

  sta::LMValue *lms = edge->arcLms();
  size_t lm_idx = arc->index() * ap_count + ap_index;
  float lm = lms ? lms[lm_idx] : -1.0f;

  bool clamp_aat = (from_aat < 0.0f);
  bool clamp_rat = (to_rat < 0.0f);
  // RapidLrHelper uses LM_FLOOR = 1e-16; treat LM <= 1.1e-16 as floored.
  bool is_floor  = (lm > 0.0f && lm <= 1.1e-16f);
  bool is_inf    = (from_aat == sta::INF || from_aat == -sta::INF
                    || to_rat == sta::INF || to_rat == -sta::INF);

  printf("  [%-7s] %-50s %s -> %-50s %s | role=%-15s | "
         "lm=%.3e | aat=%+10.3f rat=%+10.3f delay=%+8.3f slack=%+10.3f ps "
         "| clamp_aat=%d clamp_rat=%d is_floor=%d is_inf=%d\n",
         side, from_pin.c_str(), from_rf->name(), to_pin.c_str(), to_rf->name(),
         edge->role()->to_string().c_str(),
         lm,
         from_aat * 1e12, to_rat * 1e12, delay * 1e12, arc_slack * 1e12,
         clamp_aat, clamp_rat, is_floor, is_inf);
}

} // anonymous namespace

void
TestLrf::testReportFFEndpointLMs(char *inst_name, sta::dbSta* sta,
                                 rsz::Resizer * /*resizer*/,
                                 odb::dbBlock *block, size_t lm_iters)
{
  printf("===== testReportFFEndpointLMs: inst=%s, lm_iters=%zu =====\n",
         inst_name, lm_iters);

  sta::dbNetwork *db_network = sta->getDbNetwork();
  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("  ERROR: instance %s not found in block.\n", inst_name);
    return;
  }
  sta::Instance *ff_inst = db_network->dbToSta(db_inst);
  sta::LibertyCell *ff_cell = sta->network()->libertyCell(ff_inst);
  if (!ff_cell) {
    printf("  ERROR: instance %s has no liberty cell.\n", inst_name);
    return;
  }
  if (!ff_cell->hasSequentials()) {
    printf("  WARN: instance %s (cell=%s) is not a flip-flop "
           "(hasSequentials=false). Continuing anyway.\n",
           inst_name, ff_cell->name());
  }

  // Bootstrap LR: same sequence as testParallelLrResizeByArray.
  sta->searchPreamble();
  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta);
  incre_sta->makeLRHelper("rapidlrhelper");
  LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(true);          // enable endpoint LM rescaling
  lr_helper->setTimingMargin(0.01f);    // match RapidLrHelper default
  // updateAllEdgeLms iterates sorted_lm_vertices_; populate it once.
  lr_helper->ensureSorted(sta);

  // Run a few lmUpdate cycles so LMs settle to non-trivial values.
  for (size_t i = 0; i < lm_iters; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    sta::Slack wns = sta->worstSlack(sta::MinMax::max());
    sta::Slack tns = sta->totalNegativeSlack(sta::MinMax::max());
    printf("  [iter %zu] wns=%.3f ps  tns=%.3f ps  (lmUpdate done)\n",
           i + 1, wns * 1e12, tns * 1e12);
    fflush(stdout);
  }

  // Set up AP for indexing arcLms.
  sta::Scene *corner = sta->cmdScene();
  if (!corner) corner = sta->findScene("default");
  sta::DcalcAPIndex dcalc_ap =
      corner->dcalcAnalysisPtIndex(sta::MinMax::max());
  const size_t ap_index = dcalc_ap;
  const size_t ap_count = sta->graph()->apCount();
  const sta::MinMax *minmax = sta::MinMax::max();
  printf("  AP setup: corner=%s, ap_index=%zu, ap_count=%zu\n",
         corner->name(), ap_index, ap_count);

  sta::Graph *graph = sta->graph();
  sta::InstancePinIterator *pin_iter = sta->network()->pinIterator(ff_inst);
  while (pin_iter->hasNext()) {
    sta::Pin *pin = pin_iter->next();
    std::string pin_name = sta->network()->pathName(pin);
    bool is_load = sta->network()->isLoad(pin);
    bool is_drvr = sta->network()->isDriver(pin);
    printf("\n-- pin=%s  isLoad=%d  isDrvr=%d --\n", pin_name.c_str(), is_load, is_drvr);

    if (is_load) {
      // D-side: combinational in-edges to this load pin.
      // Distinguish data pins (D, SE, SI, ...) from clock pins (CK):
      // clock pins carry RAT < 0 / arrival = INF artefacts that aren't part
      // of our Δsetup × LM cost — tag them "CK-in" for visibility but they
      // are not part of the acceptance check.
      // Detect FF clock input via the liberty port's `clock : true` flag —
      // findLeafPinClocks only matches top-level SDC clock declarations.
      sta::LibertyPort *lib_port = sta->network()->libertyPort(pin);
      bool is_clk_pin = (lib_port && lib_port->isClock());
      const char *side_tag = is_clk_pin ? "CK-in" : "D-in";

      sta::Vertex *load_v = graph->pinLoadVertex(pin);
      if (!load_v) continue;
      sta::VertexInEdgeIterator iter(load_v, graph);
      while (iter.hasNext()) {
        sta::Edge *e = iter.next();
        if (e->role()->isTimingCheck()) continue;
        sta::TimingArcSet *aset = e->timingArcSet();
        if (!aset) continue;
        for (sta::TimingArc *arc : aset->arcs()) {
          printFFArcRecord(sta, e, arc, dcalc_ap, ap_index, ap_count,
                           minmax, side_tag);
        }
      }
    }
    if (is_drvr) {
      // Q-side: walk wire out-edges to fanout loads, then the gate edges
      // out of those loads (= delay arcs into next-stage drivers).
      sta::Vertex *drv_v = graph->pinDrvrVertex(pin);
      if (!drv_v) continue;

      // Optional: print the FF's own CK->Q (regClkToQ) in-edge.
      sta::VertexInEdgeIterator drv_in_iter(drv_v, graph);
      while (drv_in_iter.hasNext()) {
        sta::Edge *e = drv_in_iter.next();
        if (e->role()->isTimingCheck()) continue;
        sta::TimingArcSet *aset = e->timingArcSet();
        if (!aset) continue;
        for (sta::TimingArc *arc : aset->arcs()) {
          printFFArcRecord(sta, e, arc, dcalc_ap, ap_index, ap_count,
                           minmax, "CK-Q");
        }
      }

      sta::VertexOutEdgeIterator wire_iter(drv_v, graph);
      while (wire_iter.hasNext()) {
        sta::Edge *wire_e = wire_iter.next();
        if (!wire_e->isWire()) continue;
        sta::Vertex *fanout_load = wire_e->to(graph);
        sta::VertexOutEdgeIterator gate_iter(fanout_load, graph);
        while (gate_iter.hasNext()) {
          sta::Edge *gate_e = gate_iter.next();
          if (gate_e->role()->isTimingCheck()) continue;
          sta::TimingArcSet *aset = gate_e->timingArcSet();
          if (!aset) continue;
          for (sta::TimingArc *arc : aset->arcs()) {
            printFFArcRecord(sta, gate_e, arc, dcalc_ap, ap_index, ap_count,
                             minmax, "Q-fan");
          }
        }
      }
    }
  }
  delete pin_iter;

  delete incre_sta;
  printf("\n===== end testReportFFEndpointLMs =====\n");
  fflush(stdout);
}

void
TestLrf::testReportFFPtGraph(char *inst_name, sta::dbSta* sta,
                             rsz::Resizer * /*resizer*/, odb::dbBlock *block)
{
  printf("===== testReportFFPtGraph: inst=%s =====\n", inst_name);

  sta::dbNetwork *db_network = sta->getDbNetwork();
  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("  ERROR: instance %s not found in block.\n", inst_name);
    return;
  }
  sta::Instance *ff_inst = db_network->dbToSta(db_inst);

  // Need a fresh LocalSta so PtGraph construction has the right StaState.
  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();

  PtGraph *pt_graph = new PtGraph(sta);
  local_sta->makePtGraphFF(pt_graph, ff_inst);

  printf("\nFF cell: %s\n",
         sta->network()->libertyCell(ff_inst)->name());
  printf("PtGraph: %zu vertices, %zu edges\n",
         pt_graph->ptVertices().size(), pt_graph->ptEdges().size());

  // ----- Vertices -----
  printf("\n-- PtVertices --\n");
  printf("  %-4s %-15s %-50s %-25s %s/%s\n",
         "id", "type", "pin", "lib_port", "drvr", "load");
  for (size_t vid = 0; vid < pt_graph->ptVertices().size(); ++vid) {
    PtVertex &pv = pt_graph->ptVertex(vid);
    if (pv.type() == PtVertexType::Sentinel) continue;
    sta::Vertex *v = pv.vertex();
    if (!v) continue;
    sta::Pin *pin = v->pin();
    sta::LibertyPort *lib_port = sta->network()->libertyPort(pin);
    printf("  %-4zu %-15s %-50s %-25s %d/%d\n",
           vid, ptVertexTypeName(pv.type()),
           sta->network()->pathName(pin),
           lib_port ? lib_port->name() : "(none)",
           pv.isDriver(), pv.isLoad());
  }

  // ----- Edges -----
  printf("\n-- PtEdges --\n");
  printf("  %-4s %-16s %-15s %-30s -> %-30s\n",
         "id", "type", "role", "from", "to");
  size_t check_edge_count = 0;
  size_t reg_clk_q_count = 0;
  for (size_t eid = 0; eid < pt_graph->ptEdges().size(); ++eid) {
    const PtEdge &pe = pt_graph->ptEdges()[eid];
    if (pe.type() == PtEdgeType::Sentinel) continue;
    sta::TimingArcSet *aset = pe.timingArcSet();
    const char *role_name = aset ? aset->role()->to_string().c_str() : "(none)";
    PtVertex &from_pv = pt_graph->ptVertex(pe.ptFromId());
    PtVertex &to_pv   = pt_graph->ptVertex(pe.ptToId());
    std::string from_name = (from_pv.vertex())
        ? sta->network()->pathName(from_pv.vertex()->pin()) : std::string("(virtual)");
    std::string to_name = (to_pv.vertex())
        ? sta->network()->pathName(to_pv.vertex()->pin()) : std::string("(virtual)");
    printf("  %-4zu %-16s %-15s %-30s -> %-30s\n",
           eid, ptEdgeTypeName(pe.type()), role_name,
           from_name.c_str(), to_name.c_str());
    if (pe.type() == PtEdgeType::CheckEdge) check_edge_count++;
    if (aset && aset->role() == sta::TimingRole::regClkToQ()) reg_clk_q_count++;
  }

  // ----- Sibling FF leak check -----
  // Count instances of OTHER sequential cells in vertex_map_ (would indicate
  // CK net's sibling FFs leaked in).
  size_t sibling_ff_count = 0;
  for (auto &pv : pt_graph->ptVertices()) {
    if (pv.type() == PtVertexType::Sentinel) continue;
    sta::Vertex *v = pv.vertex();
    if (!v) continue;
    sta::Instance *inst = sta->network()->instance(v->pin());
    if (!inst || inst == ff_inst) continue;
    sta::LibertyCell *lc = sta->network()->libertyCell(inst);
    if (lc && lc->hasSequentials()) sibling_ff_count++;
  }

  printf("\n-- Acceptance summary --\n");
  printf("  CheckEdge count       : %zu  (expect: >=1, ideally =1 setup)\n",
         check_edge_count);
  printf("  regClkToQ edge count  : %zu  (expect: >=1)\n", reg_clk_q_count);
  printf("  sibling FF vertices   : %zu  (expect: 0 — clock-pin expansion bounded)\n",
         sibling_ff_count);
  printf("  total vertices        : %zu  (expect small for local_graph: ~10-25)\n",
         pt_graph->ptVertices().size());

  delete pt_graph;
  delete incre_sta;
  printf("\n===== end testReportFFPtGraph =====\n");
  fflush(stdout);
}

void
TestLrf::testFFLocalDelay(char *inst_name, sta::dbSta* sta,
                          rsz::Resizer * /*resizer*/, odb::dbBlock *block)
{
  printf("===== testFFLocalDelay: inst=%s =====\n", inst_name);

  sta::dbNetwork *db_network = sta->getDbNetwork();
  odb::dbInst *db_inst = block->findInst(inst_name);
  if (!db_inst) {
    printf("  ERROR: instance %s not found in block.\n", inst_name);
    return;
  }
  sta::Instance *ff_inst = db_network->dbToSta(db_inst);

  // Make timing global is up-to-date so we have a valid baseline.
  sta->searchPreamble();
  sta->updateTiming(true);

  IncreSta *incre_sta = new IncreSta(sta);
  LocalSta *local_sta = incre_sta->localSta();
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();

  PtGraph *pt_graph = new PtGraph(sta);
  local_sta->makePtGraphFF(pt_graph, ff_inst);
  local_sta->findLocalDelays(pt_graph, arc_delay_calc);
  local_sta->findLocalCheckDelays(pt_graph, arc_delay_calc);

  sta::DcalcAPIndex dcalc_ap = pt_graph->apIndex();
  const size_t ap_index = dcalc_ap;

  printf("\n-- CK->Q (regClkToQ) gate delays --\n");
  printf("  %-30s -> %-30s %4s %4s | %12s %12s %12s\n",
         "from", "to", "f_rf", "t_rf", "local(ps)", "global(ps)", "diff(ps)");

  size_t ckq_mismatch = 0;
  size_t setup_mismatch = 0;
  const float TOL_PS = 0.0001f;  // 1e-13 s tolerance

  // CK->Q regClkToQ edges: type=RefInstEdge, role=regClkToQ.
  for (size_t eid = 0; eid < pt_graph->edgeCount(); ++eid) {
    PtEdge &pe = pt_graph->edge(eid);
    if (pe.type() == PtEdgeType::Sentinel) continue;
    sta::TimingArcSet *aset = pe.timingArcSet();
    if (!aset) continue;
    if (aset->role() != sta::TimingRole::regClkToQ()) continue;

    sta::Edge *sta_edge = pe.edge();
    const PtVertex &from_pv = pt_graph->ptVertex(pe.ptFromId());
    const PtVertex &to_pv   = pt_graph->ptVertex(pe.ptToId());
    std::string from_name = from_pv.vertex()
        ? sta->network()->pathName(from_pv.vertex()->pin()) : std::string("(virtual)");
    std::string to_name = to_pv.vertex()
        ? sta->network()->pathName(to_pv.vertex()->pin()) : std::string("(virtual)");

    for (sta::TimingArc *arc : aset->arcs()) {
      const sta::RiseFall *from_rf = arc->fromEdge()->asRiseFall();
      const sta::RiseFall *to_rf   = arc->toEdge()->asRiseFall();
      if (!from_rf || !to_rf) continue;

      sta::ArcDelay local_d = pt_graph->arcDelay(pe, arc, ap_index);
      sta::ArcDelay global_d = sta->arcDelay(sta_edge, arc, dcalc_ap);
      float diff_ps = (local_d - global_d) * 1e12f;
      const char *flag = (std::fabs(diff_ps) > TOL_PS) ? "  MISMATCH" : "";
      if (std::fabs(diff_ps) > TOL_PS) ckq_mismatch++;
      printf("  %-30s -> %-30s %4s %4s | %+12.4f %+12.4f %+12.4f%s\n",
             from_name.c_str(), to_name.c_str(), from_rf->name(), to_rf->name(),
             local_d * 1e12, global_d * 1e12, diff_ps, flag);
    }
  }

  printf("\n-- Setup CheckEdge delays --\n");
  printf("  %-30s -> %-30s %4s %4s | %12s %12s %12s\n",
         "from(CK)", "to(D)", "f_rf", "t_rf", "local(ps)", "ref(ps)", "diff(ps)");

  for (size_t eid = 0; eid < pt_graph->edgeCount(); ++eid) {
    const PtEdge &pe = pt_graph->edge(eid);
    if (pe.type() != PtEdgeType::CheckEdge) continue;
    sta::TimingArcSet *aset = pe.timingArcSet();
    if (!aset) continue;

    const PtVertex &from_pv = pt_graph->ptVertex(pe.ptFromId());
    const PtVertex &to_pv   = pt_graph->ptVertex(pe.ptToId());
    sta::Vertex *from_v = from_pv.vertex();
    sta::Vertex *to_v   = to_pv.vertex();
    if (!from_v || !to_v) continue;
    std::string from_name = sta->network()->pathName(from_v->pin());
    std::string to_name   = sta->network()->pathName(to_v->pin());

    for (sta::TimingArc *arc : aset->arcs()) {
      const sta::RiseFall *from_rf = arc->fromEdge()->asRiseFall();
      const sta::RiseFall *to_rf   = arc->toEdge()->asRiseFall();
      if (!from_rf || !to_rf) continue;

      sta::ArcDelay local_d = pt_graph->arcDelay(pe, arc, ap_index);
      // Reference: call checkDelay directly with global slews on both ends.
      sta::Slew ck_slew = sta->graph()->slew(from_v, from_rf, ap_index);
      sta::Slew d_slew  = sta->graph()->slew(to_v, to_rf, ap_index);
      sta::ArcDelay ref_d = arc_delay_calc->checkDelay(
          to_v->pin(), arc, ck_slew, d_slew, 0.0f, sta->cmdScene(), sta::MinMax::max());
      float diff_ps = (local_d - ref_d) * 1e12f;
      const char *flag = (std::fabs(diff_ps) > TOL_PS) ? "  MISMATCH" : "";
      if (std::fabs(diff_ps) > TOL_PS) setup_mismatch++;
      printf("  %-30s -> %-30s %4s %4s | %+12.4f %+12.4f %+12.4f%s\n",
             from_name.c_str(), to_name.c_str(), from_rf->name(), to_rf->name(),
             local_d * 1e12, ref_d * 1e12, diff_ps, flag);
    }
  }

  printf("\n-- Acceptance summary --\n");
  printf("  CK->Q  mismatches  : %zu  (expect 0; tol=%g ps)\n",
         ckq_mismatch, TOL_PS);
  printf("  setup  mismatches  : %zu  (expect 0; tol=%g ps)\n",
         setup_mismatch, TOL_PS);

  delete arc_delay_calc;
  delete pt_graph;
  delete incre_sta;
  printf("\n===== end testFFLocalDelay =====\n");
  fflush(stdout);
}

////////////////////////////////////////////////////////////////
// Density map helper
////////////////////////////////////////////////////////////////

static void
setupDensityMap(PlacementDensityMap &density_map,
                sta::dbSta *sta, odb::dbBlock *block,
                IncreSta *incre_sta,
                float density_weight)
{
  density_map.build(block);
  double total_area = 0.0;
  int count = 0;
  for (odb::dbInst* inst : block->getInsts()) {
    if (!inst->isPlaced()) continue;
    sta::Instance* si = sta->getDbNetwork()->dbToSta(inst);
    if (!si) continue;
    sta::LibertyCell* lc = sta->network()->libertyCell(si);
    if (lc) { total_area += lc->area(); count++; }
  }
  float avg_area = (count > 0) ? static_cast<float>(total_area / count) : 1.0f;
  incre_sta->setDensityMap(&density_map, density_weight, avg_area);
  printf("DensityMap: %dx%d bins, avg_area=%.4f, weight=%.2f\n",
         density_map.binCntX(), density_map.binCntY(), avg_area, density_weight);
  fflush(stdout);
}

////////////////////////////////////////////////////////////////
// IterationHelper
////////////////////////////////////////////////////////////////

IterationHelper::IterationHelper(sta::dbSta *sta, odb::dbBlock *block,
                                 LocalSta *local_sta, rsz::Resizer *resizer)
  : sta_(sta), block_(block), local_sta_(local_sta), resizer_(resizer)
{
  corner_ = sta->findScene("default");
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
                           const Metrics &cur, const Metrics &best,
                           const char *decision)
{
  // dWNS/dTNS = delta vs the state THIS operator started from, i.e. the
  // previous row's post-execute best (after each row the netlist is left at
  // that row's committed `best`, which is exactly where the next operator
  // begins). This makes the delta reflect what this operator actually did:
  // a reverted operator shows the (negative) regression it was reverted for,
  // not a phantom jump off a discarded snapshot. For the first row there is
  // no previous row, so fall back to `best` (delta 0).
  const Metrics &prev = have_last_best_ ? last_best_ : best;

  char buf[512];
  snprintf(buf, sizeof(buf),
           " %4zu | %-7s | %9.3f/%-9.3f | %+7.3f | %12.3f/%-12.3f | %+9.3f | %11.3f/%-11.3f | %s (%.1fs)",
           iter, phase,
           cur.wns_ps, best.wns_ps, cur.wns_ps - prev.wns_ps,
           cur.tns_ps, best.tns_ps, cur.tns_ps - prev.tns_ps,
           cur.leakage * 1e6, best.leakage * 1e6,
           decision, cur.runtime_s);
  rows_.push_back(buf);
  // Print header + data every row so the log is always self-describing,
  // no matter where you tail it from.
  printf("[ITER]  Iter | Phase   | WNS(ps) cur/best    |  dWNS   | TNS(ps) cur/best          |   dTNS    | Leakage(uW) cur/best    | Decision\n");
  printf("[ITER]%s\n", buf);
  fflush(stdout);

  last_best_ = best;
  have_last_best_ = true;
}

void
IterationHelper::printSummary(const Metrics &best)
{
  printf("\n");
  printf("==========================================================================================================================\n");
  printf(" Iter | Phase   | WNS(ps) cur/best    |  dWNS   | TNS(ps) cur/best          |   dTNS    | Leakage(uW) cur/best    | Decision\n");
  printf("--------------------------------------------------------------------------------------------------------------------------\n");
  for (const auto &row : rows_)
    printf("%s\n", row.c_str());
  printf("--------------------------------------------------------------------------------------------------------------------------\n");
  printf(" Best checkpoint: WNS %.3f ps, TNS %.3f ps, Leakage %.3f uW\n",
         best.wns_ps, best.tns_ps, best.leakage * 1e6);
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
  local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
  sta_->delaysInvalid();
  sta_->updateTiming(true);
  odb::dbDatabase::beginEco(block_);
  eco_iter++;
  return "revert";
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
                            std::string lr_helper_method,
                            float density_weight,
                            std::string checkpoint_dir,
                            float timing_margin,
                            bool resize_ff)
{
  if (lrfVerbose()) {
    printf("----- Testing Parallel LR Resize By Array (New Framework, timing_margin=%.4f) -----\n",
           timing_margin);
  }

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);
  lr_helper->setTimingMargin(timing_margin);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f ps, TNS: %.3f ps\n", best.wns_ps, best.tns_ps);
  }
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  // Build placement density map for density-aware swap cost.
  PlacementDensityMap density_map;
  setupDensityMap(density_map, sta, block, incre_sta, density_weight);

  float top_ratio = 0.3f;

  // ECO controller — uses best experimentally-verified strategy.
  EcoConfig eco_cfg = EcoConfig::make(EcoStrategy::HALVE_ON_CONSECUTIVE);
  eco_cfg.max_eco_reverts = num_no_improve_tolerance;
  eco_cfg.warmup_iters = 0;  // EXPERIMENT: match develop_lr (no warmup) to test TNS regression hypothesis
  EcoController eco(eco_cfg, incre_sta, sta, block, resizer);

  // Enable incremental parasitic tracking via ODB callbacks.
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  est_parasitics->setIncrementalParasiticsEnabled(true);
  est_parasitics->setDbCbkOwner(block);

  EcoDecision decision = EcoDecision::ACCEPT;
  for (size_t i = 0; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    auto start = std::chrono::high_resolution_clock::now();

    if (eco.usePrecheck()) {
      if (lrfVerbose()) {
        printf("----- ECO Iteration %zu (precheck, ratio=%.4f) -----\n",
               i+1, incre_sta->adaptiveTopRatio());
      }
      incre_sta->parallelResizeByArrayWithPrecheck(resizer, avg_delay, avg_leakage,
                                                      PT_tradeoff, top_ratio);
    } else if (resize_ff) {
      if (lrfVerbose()) {
        printf("----- LR ResizeByArrayWithFF Iteration %zu -----\n", i+1);
      }
      incre_sta->parallelResizeByArrayWithFF(resizer, avg_delay, avg_leakage,
                                             PT_tradeoff);
    } else {
      if (lrfVerbose()) {
        printf("----- LR ResizeByArray Iteration %zu -----\n", i+1);
      }
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    }

    auto end = std::chrono::high_resolution_clock::now();
    if (lrfVerbose()) {
      printf("Iteration %zu took %f seconds\n", i+1,
             std::chrono::duration<double>(end - start).count());
    }

    auto t_sync0 = std::chrono::high_resolution_clock::now();
    est_parasitics->updateWireParasiticsNoDeleteNetworkIncremental();
    local_sta->syncParasiticMapFromGlobal();
    auto t_sync1 = std::chrono::high_resolution_clock::now();
    sta->delaysInvalid();
    sta->updateTiming(true);
    auto t_sync2 = std::chrono::high_resolution_clock::now();
    sta::Slack tns = sta->totalNegativeSlack(sta::MinMax::max());
    sta::Slack wns = sta->worstSlack(sta::MinMax::max());

    float leakage = incre_sta->totalLeakageFast();
    auto t_sync3 = std::chrono::high_resolution_clock::now();

    if (lrfVerbose()) {
      printf("Worst Negative Slack: %f\n", wns * 1e12);
    }
    if (lrfVerbose()) {
      printf("Total Negative Slack: %f\n", tns * 1e12);
    }
    if (lrfVerbose()) {
      printf("Total Leakage Power: %f uW\n", leakage * 1e6);
    }

    {
      char label[32];
      snprintf(label, sizeof(label), "Iter %zu", i+1);
      double viol_ns; size_t viol_cnt;
      checkSlewViolations(sta, block, local_sta, label, viol_ns, viol_cnt);
    }

    fflush(stdout);
    if (lrfVerbose()) {
      printf("[ITER_OVERHEAD] parasitic_sync=%.3f  global_sta=%.3f  leakage_calc=%.3f\n",
             std::chrono::duration<double>(t_sync1 - t_sync0).count(),
             std::chrono::duration<double>(t_sync2 - t_sync1).count(),
             std::chrono::duration<double>(t_sync3 - t_sync2).count());
    }
    fflush(stdout);

    // ── ECO decision via EcoController ──
    IterationHelper::Metrics cur;
    cur.wns_ps = wns * 1e12;
    cur.tns_ps = tns * 1e12;
    cur.leakage = leakage;  // raw watts (same scale as snapshot()/best)
    cur.runtime_s = std::chrono::duration<double>(end - start).count();

    decision = eco.decide(i, cur, best);
    float new_ratio = eco.updateRatio(decision);
    incre_sta->setAdaptiveTopRatio(new_ratio);
    eco.execute(decision, best, cur);

    helper.recordRow(i+1, eco.inEco() ? "eco" : "phase1", cur, best,
                     eco.decisionStr(decision));
    if (lrfVerbose()) {
      printf("Decision: %s\n", eco.decisionStr(decision));
    }
    fflush(stdout);

    if (decision == EcoDecision::TERMINATE)
      break;
  }
  // Disable incremental parasitic tracking.
  est_parasitics->removeDbCbkOwner();
  est_parasitics->setIncrementalParasiticsEnabled(false);

  // Final accept/revert (skipped on TERMINATE: executeTerminate already
  // rolled back to `best` and closed the ECO frame).
  if (decision != EcoDecision::TERMINATE) {
    IterationHelper::Metrics final_m = helper.snapshot();
    if (final_m.wns_ps > best.wns_ps) {
      odb::dbDatabase::endEco(block);
      if (lrfVerbose()) {
        printf("Final design accepted with WNS: %.3f ps\n", final_m.wns_ps);
      }
    } else {
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      local_sta->taskArranger()->markDirty();
      if (lrfVerbose()) {
        printf("Reverted to best design with WNS: %.3f ps\n", best.wns_ps);
      }
    }
  }

  if (lrfVerbose()) {
    printf("==============================\n");
  }
  eco.printSummary();
  helper.printSummary(best);
  delete incre_sta;
}

// ═══════════════════════════════════════════════════════════
// testParallelLrResizeByArrayBestEco — pure-resize loop with the most
// primitive ECO possible: keep only the best-WNS solution.
//
// Like the plain no-ECO loop, every parallelResizeByArray pass runs straight
// through to `iterations` and each iteration builds on the previous one — the
// netlist is never reverted mid-run, so the search trajectory (regressions
// included) is exactly the free-running one. The ONLY ECO behaviour is:
//   • whenever an iteration reaches a new best WNS, commit everything so far
//     with endEco()+beginEco() — that state becomes the locked-in baseline;
//   • non-improving iterations just keep accumulating in the open ECO frame;
//   • after the whole loop, undoEco() rolls the open frame back to the last
//     committed best, so the design that survives is the best-WNS solution
//     seen across all iterations.
// ODB's ECO journal is linear/LIFO, so committing at each new best is what
// lets the final undoEco land on the best (rather than the initial state).
// ═══════════════════════════════════════════════════════════
void
TestLrf::testParallelLrResizeByArrayBestEco(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float density_weight,
                            float timing_margin,
                            bool resize_ff)
{
  printf("----- Testing Parallel LR Resize By Array (BEST ECO, timing_margin=%.4f) -----\n",
         timing_margin);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);
  lr_helper->setTimingMargin(timing_margin);

  incre_sta->setMaxResizeNum(max_resize_num);

  // Open the first ECO frame. The committed baseline always equals the best
  // WNS seen so far; the open frame holds the changes made since that best.
  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  printf("Initial WNS: %.3f ps, TNS: %.3f ps\n", best.wns_ps, best.tns_ps);
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  // Build placement density map for density-aware swap cost.
  PlacementDensityMap density_map;
  setupDensityMap(density_map, sta, block, incre_sta, density_weight);

  // Enable incremental parasitic tracking via ODB callbacks.
  est::EstimateParasitics *est_parasitics = resizer->getEstimateParasitics();
  est_parasitics->setIncrementalParasiticsEnabled(true);
  est_parasitics->setDbCbkOwner(block);

  bool best_uncommitted = false;  // true once at least one new best was committed
  for (size_t i = 0; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    auto start = std::chrono::high_resolution_clock::now();

    if (resize_ff) {
      printf("----- LR ResizeByArrayWithFF (BestEco) Iteration %zu -----\n", i+1);
      incre_sta->parallelResizeByArrayWithFF(resizer, avg_delay, avg_leakage,
                                             PT_tradeoff);
    } else {
      printf("----- LR ResizeByArray (BestEco) Iteration %zu -----\n", i+1);
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    }

    auto end = std::chrono::high_resolution_clock::now();
    printf("Iteration %zu took %f seconds\n", i+1,
           std::chrono::duration<double>(end - start).count());

    // Sync parasitics + timing so the WNS read below is current.
    est_parasitics->updateWireParasiticsNoDeleteNetworkIncremental();
    local_sta->syncParasiticMapFromGlobal();
    sta->delaysInvalid();
    sta->updateTiming(true);
    sta::Slack tns = sta->totalNegativeSlack(sta::MinMax::max());
    sta::Slack wns = sta->worstSlack(sta::MinMax::max());
    float leakage = incre_sta->totalLeakageFast();

    printf("Worst Negative Slack: %f\n", wns * 1e12);
    printf("Total Negative Slack: %f\n", tns * 1e12);
    printf("Total Leakage Power: %f uW\n", leakage * 1e6);

    {
      char label[32];
      snprintf(label, sizeof(label), "Iter %zu", i+1);
      double viol_ns; size_t viol_cnt;
      checkSlewViolations(sta, block, local_sta, label, viol_ns, viol_cnt);
    }

    IterationHelper::Metrics cur;
    cur.wns_ps = wns * 1e12;
    cur.tns_ps = tns * 1e12;
    cur.leakage = leakage;
    cur.runtime_s = std::chrono::duration<double>(end - start).count();
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);

    // Primitive ECO: a new best WNS → commit everything so far as the new
    // locked-in baseline. Iterations never revert here, so the next pass keeps
    // building on this state regardless. Non-improving iterations leave the
    // frame open to accumulate.
    const char *action = "keep";
    if (cur.wns_ps > best.wns_ps) {
      best = cur;
      odb::dbDatabase::endEco(block);   // commit current netlist (new best)
      odb::dbDatabase::beginEco(block); // reopen frame for subsequent changes
      best_uncommitted = false;
      action = "commit(best)";
    } else {
      best_uncommitted = true;
    }
    helper.recordRow(i+1, "besteco", cur, best, action);
    fflush(stdout);
  }

  // Disable incremental parasitic tracking before the final rollback.
  est_parasitics->removeDbCbkOwner();
  est_parasitics->setIncrementalParasiticsEnabled(false);

  // Restore the best-WNS solution: close the currently-open frame and undo it,
  // which rolls the netlist back to the last committed best. (If the final
  // iteration WAS the best, the open frame is empty and this is a no-op undo.)
  odb::dbDatabase::endEco(block);
  if (best_uncommitted) {
    odb::dbDatabase::undoEco(block);
    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);
    local_sta->taskArranger()->markDirty();
  }

  IterationHelper::Metrics final_m = helper.snapshot();
  printf("Restored best solution (BEST ECO) with WNS: %.3f ps, TNS: %.3f ps\n",
         final_m.wns_ps, final_m.tns_ps);

  printf("==============================\n");
  helper.printSummary(best);
  delete incre_sta;
}

// ═══════════════════════════════════════════════════════════
// testEcoResizeNoHalve — ECO experiment:
//   Phase1: full resize, accept until first regression
//   Phase2 (ECO): precheck with adaptive ratio
//     - accept → keep ratio, reset consecutive_revert count
//     - revert → only halve on consecutive reverts (not after accept)
//     - lmUpdate before revert to let LM learn from the bad state
// ═══════════════════════════════════════════════════════════
void
TestLrf::testEcoResizeNoHalve(sta::dbSta* sta,
                               rsz::Resizer *resizer,
                               odb::dbBlock *block,
                               size_t thread_num,
                               size_t iterations,
                               float PT_tradeoff,
                               std::string lr_helper_method,
                               float halve_factor,
                               bool use_precheck)
{
  printf("----- ECO Resize (halve_factor=%.2f, precheck=%s) -----\n",
         halve_factor, use_precheck ? "yes" : "no");

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(true);
  incre_sta->setMaxResizeNum(20000000);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f ps, TNS: %.3f ps\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  size_t accept_count = 0;
  size_t total_revert_count = 0;
  size_t consecutive_reverts = 0;  // resets on accept
  bool in_eco = false;
  float top_ratio = 0.3f;

  for (size_t i = 0; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();

    auto start = std::chrono::high_resolution_clock::now();

    if (!in_eco) {
      // Phase1: full resize
      printf("----- Phase1 Iteration %zu -----\n", i+1);
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    } else if (use_precheck) {
      // Phase2: precheck with adaptive ratio
      float ratio = incre_sta->adaptiveTopRatio();
      if (lrfVerbose()) {
        printf("----- ECO Iteration %zu (precheck, ratio=%.4f) -----\n", i+1, ratio);
      }
      incre_sta->parallelResizeByArrayWithPrecheck(resizer, avg_delay, avg_leakage,
                                                    PT_tradeoff, top_ratio);
    } else {
      // Phase2: full resize (no precheck)
      if (lrfVerbose()) {
        printf("----- ECO Iteration %zu (full resize) -----\n", i+1);
      }
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    }

    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);

    double cur_wns = cur.wns_ps / 1e12;
    double best_wns_s = best.wns_ps / 1e12;

    // ── Accept ──
    if ((cur_wns > best_wns_s && cur_wns < 0)
        || (cur_wns >= 0.0 && (cur_wns > best_wns_s || cur.leakage < best.leakage))) {
      best = cur;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      accept_count++;
      consecutive_reverts = 0;  // reset — don't halve on next revert
      helper.recordRow(i+1, in_eco ? "eco" : "phase1", cur, best, "accept");
      if (lrfVerbose()) {
        printf("Decision: accept (consecutive_reverts reset to 0)\n");
      }

    // ── Warmup accept (first 3 iterations) ──
    } else if (i < 3) {
      best = cur;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      helper.recordRow(i+1, "phase1", cur, best, "accept(warmup)");
      if (lrfVerbose()) {
        printf("Decision: accept(warmup, iter %zu < 3)\n", i+1);
      }

    // ── Revert ──
    } else {
      if (!in_eco) {
        // First regression after warmup: enter ECO mode, set initial ratio
        in_eco = true;
        int change_count = incre_sta->lastChangeCount();
        TaskArranger *ta = local_sta->taskArranger();
        int total = static_cast<int>(ta->vertexCount());
        float init_ratio = (total > 0)
            ? static_cast<float>(change_count) * 1.5f / total : 0.3f;
        incre_sta->setAdaptiveTopRatio(init_ratio);
        printf("First regression → ECO mode, init ratio=%.4f\n", init_ratio);
      } else if (consecutive_reverts > 0) {
        // Consecutive revert (previous was also revert) → halve
        float new_ratio = incre_sta->adaptiveTopRatio() * halve_factor;
        incre_sta->setAdaptiveTopRatio(new_ratio);
        printf("Consecutive revert → halve ratio to %.4f\n", new_ratio);
      } else {
        // First revert after accept → don't halve, keep ratio
        printf("First revert after accept → keep ratio %.4f\n",
               incre_sta->adaptiveTopRatio());
      }

      // lmUpdate on the worse state before reverting
      incre_sta->lmUpdate();

      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      odb::dbDatabase::beginEco(block);

      consecutive_reverts++;
      total_revert_count++;
      char decision[64];
      snprintf(decision, sizeof(decision), "revert(consec=%zu)", consecutive_reverts);
      helper.recordRow(i+1, "eco", cur, best, decision);
      if (lrfVerbose()) {
        printf("Decision: revert (consecutive=%zu, total=%zu)\n",
               consecutive_reverts, total_revert_count);
      }

      if (consecutive_reverts > 5) {
        printf("Too many consecutive reverts (%zu), terminating.\n", consecutive_reverts);
        break;
      }
    }
    fflush(stdout);
  }

  // Final check
  IterationHelper::Metrics final_m = helper.snapshot();
  double final_wns = final_m.wns_ps / 1e12;
  double best_wns_s = best.wns_ps / 1e12;
  if (final_wns > best_wns_s) {
    odb::dbDatabase::endEco(block);
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);
  }

  if (lrfVerbose()) {
    printf("==============================\n");
  }
  printf("ECO summary: %zu accepts, %zu total reverts\n",
         accept_count, total_revert_count);
  helper.printSummary(best);
  delete incre_sta;
}

// ═══════════════════════════════════════════════════════════
// runLr — unified entry point, dispatches by LrConfig::mode
// ═══════════════════════════════════════════════════════════
void
TestLrf::runLr(sta::dbSta* sta, rsz::Resizer *resizer,
               odb::dbBlock *block, size_t thread_num,
               const LrConfig &cfg)
{
  printf("----- runLr: mode=%d, PT=%.1f, density_w=%.2f, iter=%zu -----\n",
         static_cast<int>(cfg.mode), cfg.PT_tradeoff,
         cfg.density_weight, cfg.iterations);
  fflush(stdout);

  // verbose is controlled by the LRF_VERBOSE env var via lrf::lrfVerbose().

  switch (cfg.mode) {
    case LrMode::RESIZE:
      testParallelLrResizeByArray(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.num_no_improve_tolerance, cfg.ratcons,
          cfg.PT_tradeoff, cfg.lr_helper_method,
          cfg.density_weight,
          cfg.checkpoint_dir, cfg.timing_margin,
          cfg.resize_ff);
      break;
    case LrMode::RESIZE_BUFFER:
      testParallelLrResizeByArrayWithBuffering(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.num_no_improve_tolerance, cfg.ratcons,
          cfg.PT_tradeoff, cfg.lr_helper_method,
          cfg.density_weight, cfg.debug,
          cfg.buffering_start_iter);
      break;
    case LrMode::PRECHECK:
      testParallelLrResizeByArrayWithPrecheck(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.num_no_improve_tolerance, cfg.ratcons,
          cfg.PT_tradeoff, cfg.lr_helper_method,
          cfg.top_ratio);
      break;
    case LrMode::PRECHECK_BUFFER:
      testParallelLrResizeByArrayWithPrecheckBuffering(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.num_no_improve_tolerance, cfg.ratcons,
          cfg.PT_tradeoff, cfg.lr_helper_method,
          cfg.top_ratio);
      break;
    case LrMode::COMBINED:
      testParallelLrCombinedResizeBuffering(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.num_no_improve_tolerance, cfg.ratcons,
          cfg.PT_tradeoff, cfg.lr_helper_method);
      break;
    case LrMode::RESIZE_RSZ_BUFFER:
      testParallelLrResizeByArrayWithRszBuffering(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.num_no_improve_tolerance, cfg.ratcons,
          cfg.PT_tradeoff, cfg.lr_helper_method,
          cfg.density_weight, cfg.debug);
      break;
    case LrMode::RESIZE_BEST_ECO:
      testParallelLrResizeByArrayBestEco(
          sta, resizer, block, thread_num,
          cfg.max_resize_num, cfg.iterations,
          cfg.ratcons, cfg.PT_tradeoff, cfg.lr_helper_method,
          cfg.density_weight, cfg.timing_margin,
          cfg.resize_ff);
      break;
  }
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
                            std::string lr_helper_method,
                            float density_weight,
                            bool debug,
                            size_t buffering_start_iter)
{
  printf("----- Testing Parallel LR Resize+Buffering (revert-halve ECO, buffering_start_iter=%zu) -----\n",
         buffering_start_iter);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->setDebug(debug);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  // Build placement density map for density-aware swap cost.
  PlacementDensityMap density_map;
  setupDensityMap(density_map, sta, block, incre_sta, density_weight);

  // Resize ECO controller
  EcoConfig eco_cfg = EcoConfig::make(EcoStrategy::HALVE_ON_CONSECUTIVE);
  eco_cfg.max_eco_reverts = num_no_improve_tolerance;
  EcoController eco(eco_cfg, incre_sta, sta, block, resizer);

  // Buffering ECO: accept/revert only, never TERMINATEs the outer loop,
  // no leakage-plateau check (buffering optimizes timing, not leakage).
  BufferEcoController buffer_eco(incre_sta, sta, block, resizer);

  EcoDecision decision = EcoDecision::ACCEPT;
  for (size_t i = 0; i < iterations; ++i) {
    // ── Resize phase ──
    incre_sta->lmUpdate();
    sta->findRequireds();
    auto start = std::chrono::high_resolution_clock::now();
    if (eco.usePrecheck()) {
      float ratio = incre_sta->adaptiveTopRatio();
      printf("----- LR ResizeByArrayWithPrecheck Iteration %zu (ratio=%.4f) -----\n",
             i+1, ratio);
      incre_sta->parallelResizeByArrayWithPrecheck(
          resizer, avg_delay, avg_leakage, PT_tradeoff, ratio);
    } else {
      if (lrfVerbose()) {
        printf("----- LR ResizeByArray Iteration %zu -----\n", i+1);
      }
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("After resize: WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);
    fflush(stdout);

    // ── Resize ECO decision ──
    decision = eco.decide(i, cur, best);
    float new_ratio = eco.updateRatio(decision);
    incre_sta->setAdaptiveTopRatio(new_ratio);
    eco.execute(decision, best, cur);

    helper.recordRow(i+1, eco.inEco() ? "eco" : "resize", cur, best,
                     eco.decisionStr(decision));
    if (lrfVerbose()) {
      printf("Decision: %s\n", eco.decisionStr(decision));
    }
    fflush(stdout);

    if (decision == EcoDecision::TERMINATE)
      break;

    // ── Buffering phase (only when WNS < 0 and iter >= buffering_start_iter) ──
    sta::Slack wns_after_resize = sta->worstSlack(sta::MinMax::max());
    if (wns_after_resize < 0 && (i + 1) >= buffering_start_iter) {
      incre_sta->lmUpdate();
      printf("----- Buffering pass (iter %zu) -----\n", i+1);
      double wns_before = wns_after_resize * 1e12;
      double tns_before = sta->totalNegativeSlack(sta::MinMax::max()) * 1e12;
      sta->findRequireds();
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

      auto buf_start = std::chrono::high_resolution_clock::now();
      incre_sta->parallelBuffering(resizer, PT_tradeoff);
      auto buf_end = std::chrono::high_resolution_clock::now();
      double buf_runtime = std::chrono::duration<double>(buf_end - buf_start).count();

      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);

      IterationHelper::Metrics buf_cur = helper.snapshot(buf_runtime);
      double wns_delta = buf_cur.wns_ps - wns_before;
      double tns_delta = buf_cur.tns_ps - tns_before;
      printf("Buffering diff: WNS %.3f -> %.3f ps (delta=%+.3f), "
             "TNS %.3f -> %.3f ps (delta=%+.3f), Leakage %.3f uW (%.1fs)\n",
             wns_before, buf_cur.wns_ps, wns_delta,
             tns_before, buf_cur.tns_ps, tns_delta,
             buf_cur.leakage * 1e6, buf_runtime);
      fflush(stdout);

      // Buffering ECO: accept/revert only (no halve, no lmUpdate-before-revert).
      EcoDecision buf_decision = buffer_eco.decide(i, buf_cur, best);
      buffer_eco.execute(buf_decision, best, buf_cur);
      helper.recordRow(i+1, "buffer", buf_cur, best,
                       buffer_eco.decisionStr(buf_decision));
      printf("Buffering: %s\n", buffer_eco.decisionStr(buf_decision));
      fflush(stdout);
    }
  }

  // Final accept/revert (skipped on TERMINATE: executeTerminate already
  // rolled back to `best` and closed the ECO frame).
  if (decision != EcoDecision::TERMINATE) {
    IterationHelper::Metrics final_m = helper.snapshot();
    if (final_m.wns_ps > best.wns_ps) {
      odb::dbDatabase::endEco(block);
      if (lrfVerbose()) {
        printf("Final design accepted with WNS: %.3f ps\n", final_m.wns_ps);
      }
    } else {
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      local_sta->taskArranger()->markDirty();
      if (lrfVerbose()) {
        printf("Reverted to best design with WNS: %.3f ps\n", best.wns_ps);
      }
    }
  }

  helper.printSummary(best);
  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithSdpBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float density_weight,
                            bool debug,
                            size_t buffering_start_iter,
                            float timing_margin)
{
  printf("----- Testing Parallel LR Resize + SDP Buffering (revert-halve ECO, "
         "buffering_start_iter=%zu, timing_margin=%.4f, minimize_leakage=false, "
         "max_runtime=2h) -----\n",
         buffering_start_iter, timing_margin);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->setDebug(debug);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);
  lr_helper->setTimingMargin(timing_margin);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  PlacementDensityMap density_map;
  setupDensityMap(density_map, sta, block, incre_sta, density_weight);

  // Resize ECO controller
  EcoConfig eco_cfg = EcoConfig::make(EcoStrategy::HALVE_ON_CONSECUTIVE);
  eco_cfg.max_eco_reverts = num_no_improve_tolerance;
  eco_cfg.max_runtime_seconds = 7200.0;   // hard 2h wall-clock cap
  EcoController eco(eco_cfg, incre_sta, sta, block, resizer);

  // Buffering ECO: accept/revert only, never TERMINATEs the outer loop,
  // no leakage-plateau check (buffering optimizes timing, not leakage).
  BufferEcoController buffer_eco(incre_sta, sta, block, resizer);

  // Eagerly populate sta::Edge.delay_diffs_ so the very first precheck iter
  // (once ECO turns on use_precheck) doesn't pay the full-graph re-eval inline.
  incre_sta->initDelayDiff();

  EcoDecision decision = EcoDecision::ACCEPT;
  for (size_t i = 0; i < iterations; ++i) {
    // ── Resize phase ──
    incre_sta->lmUpdate();
    sta->findRequireds();
    auto start = std::chrono::high_resolution_clock::now();
    if (eco.usePrecheck()) {
      float ratio = incre_sta->adaptiveTopRatio();
      printf("----- LR ResizeByArrayWithPrecheck Iteration %zu (ratio=%.4f) -----\n",
             i+1, ratio);
      incre_sta->parallelResizeByArrayWithPrecheck(
          resizer, avg_delay, avg_leakage, PT_tradeoff, ratio);
    } else {
      if (lrfVerbose()) {
        printf("----- LR ResizeByArray Iteration %zu -----\n", i+1);
      }
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("After resize: WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);
    fflush(stdout);

    // ── Resize ECO decision ──
    decision = eco.decide(i, cur, best);
    float new_ratio = eco.updateRatio(decision);
    incre_sta->setAdaptiveTopRatio(new_ratio);
    eco.execute(decision, best, cur);

    helper.recordRow(i+1, eco.inEco() ? "eco" : "resize", cur, best,
                     eco.decisionStr(decision));
    if (lrfVerbose()) {
      printf("Decision: %s\n", eco.decisionStr(decision));
    }
    fflush(stdout);

    if (decision == EcoDecision::TERMINATE)
      break;

    // ── SDP Buffering phase ──
    sta::Slack wns_after_resize = sta->worstSlack(sta::MinMax::max());
    if (wns_after_resize < 0 && (i + 1) >= buffering_start_iter) {
      incre_sta->lmUpdate();
      printf("----- SDP Buffering pass (iter %zu) -----\n", i+1);
      double wns_before = wns_after_resize * 1e12;
      double tns_before = sta->totalNegativeSlack(sta::MinMax::max()) * 1e12;
      sta->findRequireds();
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

      auto buf_start = std::chrono::high_resolution_clock::now();
      // ↓ Only line that differs from WithBuffering: SDP variant.
      incre_sta->parallelBufferingSdp(resizer, PT_tradeoff);
      auto buf_end = std::chrono::high_resolution_clock::now();
      double buf_runtime = std::chrono::duration<double>(buf_end - buf_start).count();

      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);

      IterationHelper::Metrics buf_cur = helper.snapshot(buf_runtime);
      double wns_delta = buf_cur.wns_ps - wns_before;
      double tns_delta = buf_cur.tns_ps - tns_before;
      printf("SDP Buffering diff: WNS %.3f -> %.3f ps (delta=%+.3f), "
             "TNS %.3f -> %.3f ps (delta=%+.3f), Leakage %.3f uW (%.1fs)\n",
             wns_before, buf_cur.wns_ps, wns_delta,
             tns_before, buf_cur.tns_ps, tns_delta,
             buf_cur.leakage * 1e6, buf_runtime);
      fflush(stdout);

      // Buffering ECO: accept/revert only (no halve, no lmUpdate-before-revert).
      EcoDecision buf_decision = buffer_eco.decide(i, buf_cur, best);
      buffer_eco.execute(buf_decision, best, buf_cur);
      helper.recordRow(i+1, "buffer", buf_cur, best,
                       buffer_eco.decisionStr(buf_decision));
      printf("SDP Buffering: %s\n", buffer_eco.decisionStr(buf_decision));
      fflush(stdout);
    }
  }

  // Final accept/revert (skipped on TERMINATE: executeTerminate already
  // rolled back to `best` and closed the ECO frame).
  if (decision != EcoDecision::TERMINATE) {
    IterationHelper::Metrics final_m = helper.snapshot();
    if (final_m.wns_ps > best.wns_ps) {
      odb::dbDatabase::endEco(block);
      if (lrfVerbose()) {
        printf("Final design accepted with WNS: %.3f ps\n", final_m.wns_ps);
      }
    } else {
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      local_sta->taskArranger()->markDirty();
      if (lrfVerbose()) {
        printf("Reverted to best design with WNS: %.3f ps\n", best.wns_ps);
      }
    }
  }

  helper.printSummary(best);
  delete incre_sta;
}

// Two-phase flow: (A) init + pure resize until ECO TERMINATE,
// (B) precheck-resize + LRF slack-DP rebuffering on top of phase-A's best state.
// Both phases share IncreSta/LRHelper/best; ECO transaction spans both.
void
TestLrf::testInitResizeThenSdpBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float density_weight,
                            bool debug,
                            size_t buffering_start_iter,
                            float timing_margin,
                            float erc_violation_weight,
                            float erc_limit_scale,
                            bool resize_ff)
{
  printf("----- Testing Init Resize -> SDP Buffering "
         "(iterations=%zu, tol=%zu, "
         "buffering_start_iter=%zu, timing_margin=%.4f, "
         "erc_violation_weight=%.3g, erc_limit_scale=%.3f, "
         "resize_ff=%d) -----\n",
         iterations, num_no_improve_tolerance,
         buffering_start_iter, timing_margin, erc_violation_weight,
         erc_limit_scale, resize_ff);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->setDebug(debug);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);
  lr_helper->setTimingMargin(timing_margin);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  PlacementDensityMap density_map;
  setupDensityMap(density_map, sta, block, incre_sta, density_weight);

  // ═══════════════════════════════════════════════════════════
  // Phase A: pure resize until ECO TERMINATE.
  //   Strategy = NO_HALVE (full resize every iter, ratio untouched,
  //                         use_precheck=false); terminate after
  //                         num_no_improve_tolerance consecutive reverts.
  // ═══════════════════════════════════════════════════════════
  // Single ECO controller spans both phases. HALVE_ON_CONSECUTIVE so that the
  // first regression in phase A computes a sensible init_ratio (used by
  // phase B's forced precheck path); warmup_iters lets phase A absorb early
  // double-regress as REVERT_WARMUP. Phase A breaks on the first REVERT;
  // phase B continues with the same controller (in_eco_, ratio, counters).
  printf("===== Phase A: pure resize (cap=%zu) =====\n", iterations);
  EcoConfig eco_cfg = EcoConfig::make(EcoStrategy::ADAPTIVE_FROM_CHANGE);
  // EcoController checks `consecutive_reverts_ > max_eco_reverts`. Phase A's
  // break-trigger REVERT brings consecutive_reverts_ to 1, so phase B inherits
  // N reverts of headroom: the (N+1)-th total revert TERMINATEs. (N=0 is the
  // strictest setting — phase A's first REVERT already TERMINATEs.)
  eco_cfg.max_eco_reverts = num_no_improve_tolerance;
  eco_cfg.warmup_iters = 3;
  EcoController eco(eco_cfg, incre_sta, sta, block, resizer);

  size_t i = 0;
  EcoDecision decision = EcoDecision::ACCEPT;
  for (; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    auto start = std::chrono::high_resolution_clock::now();
    if (resize_ff) {
      printf("----- Phase A: LR ResizeByArrayWithFF Iteration %zu -----\n", i+1);
      incre_sta->parallelResizeByArrayWithFF(
          resizer, avg_delay, avg_leakage, PT_tradeoff,
          erc_violation_weight, erc_limit_scale);
    } else {
      if (lrfVerbose()) {
        printf("----- Phase A: LR ResizeByArray Iteration %zu -----\n", i+1);
      }
      incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff,
                                       erc_violation_weight, erc_limit_scale);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("Phase A after resize: WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);
    fflush(stdout);

    decision = eco.decide(i, cur, best);
    float new_ratio = eco.updateRatio(decision);
    incre_sta->setAdaptiveTopRatio(new_ratio);
    eco.execute(decision, best, cur);

    helper.recordRow(i+1, eco.inEco() ? "A-eco" : "A-resize", cur, best,
                     eco.decisionStr(decision));
    if (lrfVerbose()) {
      printf("Phase A Decision: %s\n", eco.decisionStr(decision));
    }
    fflush(stdout);

    // Phase A "until ECO triggers" semantic: any REVERT (or TERMINATE)
    // ends phase A. ACCEPT and REVERT_WARMUP keep phase A alive.
    if (decision == EcoDecision::REVERT || decision == EcoDecision::TERMINATE)
      break;
  }
  printf("===== Phase A done. Best: WNS=%.3f ps, TNS=%.3f ps =====\n",
         best.wns_ps, best.tns_ps);

  // TERMINATE means executeTerminate already left the netlist at `best` with
  // the ECO frame closed; init_ratio is also unset, so the forced-precheck
  // path in phase B would crash with top_ratio = -1.0. Skip phase B entirely.
  if (decision == EcoDecision::TERMINATE) {
    printf("Phase A terminated; skipping Phase B.\n");
    eco.printSummary();
    helper.printSummary(best);
    delete incre_sta;
    return;
  }

  // ═══════════════════════════════════════════════════════════
  // Phase B: precheck-resize (every iter) + SDP buffering.
  // Continues with the same `eco` (in_eco_=true, ratio set to init_ratio
  // by phase A's break-triggering REVERT, counters carried over).
  // ═══════════════════════════════════════════════════════════
  printf("===== Phase B: precheck resize + SDP buffering =====\n");

  // Buffering ECO: accept/revert only, never TERMINATEs the outer loop,
  // no leakage-plateau check (buffering optimizes timing, not leakage).
  BufferEcoController buffer_eco(incre_sta, sta, block, resizer);

  // Step past phase A's terminating iter so phase B doesn't reuse the
  // index (entering phase B's body implies phase A broke at i=k; the
  // body for i=k already ran in phase A, including its execute(REVERT)).
  // No-op when phase A exited naturally (i is already == iterations).
  for (++i; i < iterations; ++i) {
    // ── Resize phase (always precheck) ──
    incre_sta->lmUpdate();
    sta->findRequireds();
    auto start = std::chrono::high_resolution_clock::now();
    float ratio = incre_sta->adaptiveTopRatio();
    printf("----- Phase B: LR ResizeByArrayWithPrecheck Iteration %zu (ratio=%.4f) -----\n",
           i+1, ratio);
    incre_sta->parallelResizeByArrayWithPrecheck(
        resizer, avg_delay, avg_leakage, PT_tradeoff, ratio,
        erc_violation_weight, erc_limit_scale, resize_ff);
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("Phase B after resize: WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);
    fflush(stdout);

    decision = eco.decide(i, cur, best);
    float new_ratio = eco.updateRatio(decision);
    incre_sta->setAdaptiveTopRatio(new_ratio);
    eco.execute(decision, best, cur);

    helper.recordRow(i+1, eco.inEco() ? "B-eco" : "B-resize", cur, best,
                     eco.decisionStr(decision));
    if (lrfVerbose()) {
      printf("Phase B Decision: %s\n", eco.decisionStr(decision));
    }
    fflush(stdout);

    if (decision == EcoDecision::TERMINATE)
      break;

    // ── SDP Buffering phase ──
    sta::Slack wns_after_resize = sta->worstSlack(sta::MinMax::max());
    if (wns_after_resize < 0 && (i + 1) >= buffering_start_iter) {
      incre_sta->lmUpdate();
      printf("----- SDP Buffering pass (iter %zu) -----\n", i+1);
      double wns_before = wns_after_resize * 1e12;
      double tns_before = sta->totalNegativeSlack(sta::MinMax::max()) * 1e12;
      sta->findRequireds();
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

      auto buf_start = std::chrono::high_resolution_clock::now();
      incre_sta->parallelBufferingSdp(resizer, PT_tradeoff, /*top_ratio=*/0.01f,
                                      erc_violation_weight, erc_limit_scale);
      auto buf_end = std::chrono::high_resolution_clock::now();
      double buf_runtime = std::chrono::duration<double>(buf_end - buf_start).count();

      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);

      IterationHelper::Metrics buf_cur = helper.snapshot(buf_runtime);
      double wns_delta = buf_cur.wns_ps - wns_before;
      double tns_delta = buf_cur.tns_ps - tns_before;
      printf("SDP Buffering diff: WNS %.3f -> %.3f ps (delta=%+.3f), "
             "TNS %.3f -> %.3f ps (delta=%+.3f), Leakage %.3f uW (%.1fs)\n",
             wns_before, buf_cur.wns_ps, wns_delta,
             tns_before, buf_cur.tns_ps, tns_delta,
             buf_cur.leakage * 1e6, buf_runtime);
      fflush(stdout);

      EcoDecision buf_decision = buffer_eco.decide(i, buf_cur, best);
      buffer_eco.execute(buf_decision, best, buf_cur);
      helper.recordRow(i+1, "buffer", buf_cur, best,
                       buffer_eco.decisionStr(buf_decision));
      printf("SDP Buffering: %s\n", buffer_eco.decisionStr(buf_decision));
      fflush(stdout);
    }
  }
  eco.printSummary();

  // ── Final accept/revert (skipped on TERMINATE: executeTerminate already
  //    rolled back to `best` and closed the ECO frame). ──
  if (decision != EcoDecision::TERMINATE) {
    IterationHelper::Metrics final_m = helper.snapshot();
    if (final_m.wns_ps > best.wns_ps) {
      odb::dbDatabase::endEco(block);
      if (lrfVerbose()) {
        printf("Final design accepted with WNS: %.3f ps\n", final_m.wns_ps);
      }
    } else {
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      local_sta->taskArranger()->markDirty();
      if (lrfVerbose()) {
        printf("Reverted to best design with WNS: %.3f ps\n", best.wns_ps);
      }
    }
  }

  helper.printSummary(best);
  delete incre_sta;
}

void
TestLrf::testParallelLrResizeByArrayWithRszBuffering(sta::dbSta* sta,
                            rsz::Resizer *resizer,
                            odb::dbBlock *block,
                            size_t thread_num,
                            size_t max_resize_num,
                            size_t iterations,
                            size_t num_no_improve_tolerance,
                            bool ratcons,
                            float PT_tradeoff,
                            std::string lr_helper_method,
                            float density_weight,
                            bool debug)
{
  printf("----- Testing Parallel LR Resize + RSZ Buffering (revert-halve ECO) -----\n");

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->setDebug(debug);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  // Build placement density map for density-aware swap cost.
  PlacementDensityMap density_map;
  setupDensityMap(density_map, sta, block, incre_sta, density_weight);

  // ECO controller
  EcoConfig eco_cfg = EcoConfig::make(EcoStrategy::HALVE_ON_CONSECUTIVE);
  eco_cfg.max_eco_reverts = num_no_improve_tolerance;
  EcoController eco(eco_cfg, incre_sta, sta, block, resizer);

  EcoDecision decision = EcoDecision::ACCEPT;
  for (size_t i = 0; i < iterations; ++i) {
    // ── Resize phase ──
    incre_sta->lmUpdate();
    sta->findRequireds();
    if (lrfVerbose()) {
      printf("----- LR ResizeByArray Iteration %zu -----\n", i+1);
    }
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArray(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("After resize: WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);
    fflush(stdout);

    // ── ECO decision ──
    decision = eco.decide(i, cur, best);
    float new_ratio = eco.updateRatio(decision);
    incre_sta->setAdaptiveTopRatio(new_ratio);
    eco.execute(decision, best, cur);

    helper.recordRow(i+1, eco.inEco() ? "eco" : "resize", cur, best,
                     eco.decisionStr(decision));
    if (lrfVerbose()) {
      printf("Decision: %s\n", eco.decisionStr(decision));
    }
    fflush(stdout);

    if (decision == EcoDecision::TERMINATE)
      break;

    // ── RSZ Buffering phase (after iter 3, only when WNS < 0) ──
    sta::Slack wns_after_resize = sta->worstSlack(sta::MinMax::max());
    if (wns_after_resize < 0 && i > 3) {
      printf("----- RSZ Buffering pass (iter %zu) -----\n", i+1);
      double wns_before = wns_after_resize * 1e12;
      double tns_before = sta->totalNegativeSlack(sta::MinMax::max()) * 1e12;
      sta->findRequireds();
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

      auto buf_start = std::chrono::high_resolution_clock::now();
      incre_sta->parallelBufferingRsz(resizer, PT_tradeoff);
      auto buf_end = std::chrono::high_resolution_clock::now();
      double buf_runtime = std::chrono::duration<double>(buf_end - buf_start).count();

      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);

      IterationHelper::Metrics buf_cur = helper.snapshot(buf_runtime);
      double wns_delta = buf_cur.wns_ps - wns_before;
      double tns_delta = buf_cur.tns_ps - tns_before;
      printf("RSZ Buffering diff: WNS %.3f -> %.3f ps (delta=%+.3f), "
             "TNS %.3f -> %.3f ps (delta=%+.3f), Leakage %.3f uW (%.1fs)\n",
             wns_before, buf_cur.wns_ps, wns_delta,
             tns_before, buf_cur.tns_ps, tns_delta,
             buf_cur.leakage * 1e6, buf_runtime);
      fflush(stdout);

      // Accept if WNS improved or within 1.1x slack margin
      double buf_wns = buf_cur.wns_ps / 1e12;
      double best_wns_s = best.wns_ps / 1e12;
      double wns_thresh = best_wns_s < 0 ? best_wns_s * 1.1 : 0;
      if ((buf_wns > best_wns_s && buf_wns < 0)
          || (buf_wns >= 0.0 && (buf_wns > best_wns_s || buf_cur.leakage < best.leakage))) {
        best = buf_cur;
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::beginEco(block);
        helper.recordRow(i+1, "rsz_buffer", buf_cur, best, "accept");
        printf("RSZ Buffering: accept (WNS improved)\n");
      } else if (buf_wns >= wns_thresh && buf_cur.leakage < best.leakage) {
        best = buf_cur;
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::beginEco(block);
        helper.recordRow(i+1, "rsz_buffer", buf_cur, best, "accept(margin)");
        printf("RSZ Buffering: accept (WNS within 1.1x margin, leakage improved)\n");
      } else {
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::undoEco(block);
        local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
        sta->delaysInvalid();
        sta->updateTiming(true);
        odb::dbDatabase::beginEco(block);
        helper.recordRow(i+1, "rsz_buffer", buf_cur, best, "revert");
        printf("RSZ Buffering: revert (WNS=%.3f < thresh=%.3f or no leakage gain)\n",
               buf_wns * 1e12, wns_thresh * 1e12);
      }
      fflush(stdout);
    }
  }

  // Final accept/revert (skipped on TERMINATE: executeTerminate already
  // rolled back to `best` and closed the ECO frame).
  if (decision != EcoDecision::TERMINATE) {
    IterationHelper::Metrics final_m = helper.snapshot();
    if (final_m.wns_ps > best.wns_ps) {
      odb::dbDatabase::endEco(block);
      if (lrfVerbose()) {
        printf("Final design accepted with WNS: %.3f ps\n", final_m.wns_ps);
      }
    } else {
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      local_sta->taskArranger()->markDirty();
      if (lrfVerbose()) {
        printf("Reverted to best design with WNS: %.3f ps\n", best.wns_ps);
      }
    }
  }

  helper.printSummary(best);
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
  printf("----- Testing Combined Resize + Buffering (ECO) -----\n");

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  size_t eco_iter = 0;
  bool in_eco = false;  // true after first regression triggers revert-halve

  const size_t replace_interval = 3;  // re-placement every K iterations

  for (size_t i = 0; i < iterations; ++i) {
    // Periodic re-placement for accurate parasitics
    if (i > 0 && i % replace_interval == 0) {
      printf("[RE-PLACEMENT] Iteration %zu: running global_placement + estimate_parasitics\n", i+1);
      fflush(stdout);
      auto rp_start = std::chrono::high_resolution_clock::now();
      Tcl_Eval(sta->tclInterp(), "global_placement -routability_driven -init_density_penalty 0.05 -initial_place_max_iter 10");
      Tcl_Eval(sta->tclInterp(), "estimate_parasitics -placement");
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      local_sta->taskArranger()->markDirty();
      auto rp_end = std::chrono::high_resolution_clock::now();
      printf("[RE-PLACEMENT] took %.1f seconds\n",
             std::chrono::duration<double>(rp_end - rp_start).count());
      fflush(stdout);
    }

    incre_sta->lmUpdate();
    sta->findRequireds();
    printf("----- Combined Resize+Buffering Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeAndBuffering(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();
    printf("Iteration %zu took %.1f seconds\n", i+1, runtime);

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6);
    fflush(stdout);

    double cur_wns = cur.wns_ps / 1e12;
    double best_wns_s = best.wns_ps / 1e12;

    // Accept if WNS improved (negative slack) or timing met with better leakage
    if ((cur_wns > best_wns_s && cur_wns < 0)
        || (cur_wns >= 0.0 && (cur_wns > best_wns_s || cur.leakage < best.leakage))) {
      best = cur;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter = 0;
      in_eco = false;
      helper.recordRow(i+1, "combined", cur, best, "accept");
      if (lrfVerbose()) {
        printf("Decision: accept\n");
      }
    } else if (i < 3) {
      // First few iterations: always accept (early iterations often regress before converging)
      best = cur;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      helper.recordRow(i+1, "combined", cur, best, "accept(warmup)");
      if (lrfVerbose()) {
        printf("Decision: accept(warmup, iter %zu < 3)\n", i+1);
      }
    } else {
      // Regression or no improvement after warmup — revert + halve
      if (!in_eco) {
        // First regression: compute initial ratio from last change count
        int change_count = incre_sta->lastChangeCount();
        TaskArranger *ta = local_sta->taskArranger();
        int total = static_cast<int>(ta->vertexCount());
        float init_ratio = (total > 0)
            ? static_cast<float>(change_count) * 1.5f / total
            : 0.3f;
        incre_sta->setAdaptiveTopRatio(init_ratio);
        in_eco = true;
        printf("First regression, init eco ratio=%.4f (change=%d, total=%d)\n",
               init_ratio, change_count, total);
      } else {
        // Subsequent regression: halve ratio
        float new_ratio = incre_sta->adaptiveTopRatio() * 0.25f;
        incre_sta->setAdaptiveTopRatio(new_ratio);
        printf("ECO halve ratio to %.4f\n", new_ratio);
      }

      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
      helper.recordRow(i+1, "combined", cur, best, "revert+halve");
      if (lrfVerbose()) {
        printf("Decision: revert+halve (eco_iter=%zu)\n", eco_iter);
      }

      if (eco_iter > 6) {
        printf("Too many ECO iterations (%zu), terminating.\n", eco_iter);
        break;
      }
    }
    fflush(stdout);
  }

  // Final check
  IterationHelper::Metrics final_m = helper.snapshot();
  if (final_m.wns_ps > best.wns_ps) {
    odb::dbDatabase::endEco(block);
    if (lrfVerbose()) {
      printf("Final design accepted with WNS: %.3f ps\n", final_m.wns_ps);
    }
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);
    local_sta->taskArranger()->markDirty();
    if (lrfVerbose()) {
      printf("Reverted to best design with WNS: %.3f ps\n", best.wns_ps);
    }
  }

  helper.printSummary(best);
  delete incre_sta;
}

void
TestLrf::testBufferOnly(sta::dbSta* sta,
                        rsz::Resizer *resizer,
                        odb::dbBlock *block,
                        size_t thread_num,
                        size_t iterations,
                        float PT_tradeoff,
                        std::string lr_helper_method,
                        float bakoglu_k,
                        bool debug)
{
  printf("----- Testing Buffer-Only Mode (bakoglu_k=%.2f, debug=%d) -----\n",
         bakoglu_k, debug);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->setDebug(debug);
  incre_sta->setBufferOnlyMode(true);
  incre_sta->setBakogluK(bakoglu_k);

  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(true);

  incre_sta->setMaxResizeNum(20000000);

  // ── LM warm-up: run several rounds of lmUpdate without any
  //    resize/buffer so that Lagrange multipliers converge to
  //    values consistent with the current netlist. ──
  const size_t lm_warmup_rounds = 5;
  printf("LM warm-up: %zu rounds\n", lm_warmup_rounds);
  for (size_t w = 0; w < lm_warmup_rounds; ++w) {
    incre_sta->lmUpdate();
    sta->findRequireds();
  }
  printf("LM warm-up done\n");

  odb::dbDatabase::beginEco(block);
  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics best = helper.snapshot();
  incre_sta->recordMetrics(best.wns_ps, best.tns_ps, best.leakage);
  if (lrfVerbose()) {
    printf("Initial WNS: %.3f, TNS: %.3f\n", best.wns_ps, best.tns_ps);
  }

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  size_t eco_iter = 0;

  for (size_t i = 0; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    printf("----- Buffer-Only Iteration %zu -----\n", i+1);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeAndBuffering(resizer, avg_delay, avg_leakage, PT_tradeoff);
    auto end = std::chrono::high_resolution_clock::now();
    double runtime = std::chrono::duration<double>(end - start).count();
    printf("Iteration %zu took %.1f seconds\n", i+1, runtime);

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);

    IterationHelper::Metrics cur = helper.snapshot(runtime);
    incre_sta->recordMetrics(cur.wns_ps, cur.tns_ps, cur.leakage);
    printf("WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW\n",
           cur.wns_ps, cur.tns_ps, cur.leakage * 1e6);
    fflush(stdout);

    double cur_wns = cur.wns_ps / 1e12;
    double best_wns_s = best.wns_ps / 1e12;

    if ((cur_wns > best_wns_s && cur_wns < 0)
        || (cur_wns >= 0.0 && (cur_wns > best_wns_s || cur.leakage < best.leakage))) {
      best = cur;
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::beginEco(block);
      eco_iter = 0;
      helper.recordRow(i+1, "buffer_only", cur, best, "accept");
      if (lrfVerbose()) {
        printf("Decision: accept\n");
      }
    } else {
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      odb::dbDatabase::beginEco(block);
      eco_iter++;
      helper.recordRow(i+1, "buffer_only", cur, best, "revert");
      if (lrfVerbose()) {
        printf("Decision: revert (eco_iter=%zu)\n", eco_iter);
      }

      if (eco_iter > 3) {
        printf("Too many consecutive rejections (%zu), terminating.\n", eco_iter);
        break;
      }
    }
    fflush(stdout);
  }

  IterationHelper::Metrics final_m = helper.snapshot();
  odb::dbDatabase::endEco(block);
  printf("Final buffer-only WNS: %.3f, TNS: %.3f\n",
         final_m.wns_ps, final_m.tns_ps);
  helper.printSummary(best);
  delete incre_sta;
}

void
TestLrf::testSingleBufferPass(sta::dbSta* sta,
                              rsz::Resizer *resizer,
                              odb::dbBlock *block,
                              size_t thread_num,
                              bool use_sdp,
                              float PT_tradeoff,
                              std::string lr_helper_method,
                              size_t lm_warmup_rounds)
{
  printf("----- Single Buffer Pass: %s (warmup=%zu, PT=%.1f) -----\n",
         use_sdp ? "SDP" : "LRF", lm_warmup_rounds, PT_tradeoff);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();
  incre_sta->makeLRHelper(lr_helper_method);
  incre_sta->lrHelper()->setRatcons(true);
  incre_sta->setMaxResizeNum(20000000);

  for (size_t w = 0; w < lm_warmup_rounds; ++w) {
    incre_sta->lmUpdate();
    sta->findRequireds();
  }

  IterationHelper helper(sta, block, local_sta, resizer);
  IterationHelper::Metrics before = helper.snapshot();
  printf("Before: WNS=%.3f ps  TNS=%.3f ps  Leakage=%.3f uW\n",
         before.wns_ps, before.tns_ps, before.leakage * 1e6);

  local_sta->initParallel();
  incre_sta->lmUpdate();
  sta->findRequireds();
  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

  auto t0 = std::chrono::high_resolution_clock::now();
  if (use_sdp)
    incre_sta->parallelBufferingSdp(resizer, PT_tradeoff);
  else
    incre_sta->parallelBuffering(resizer, PT_tradeoff);
  auto t1 = std::chrono::high_resolution_clock::now();
  double runtime = std::chrono::duration<double>(t1 - t0).count();

  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
  sta->delaysInvalid();
  sta->updateTiming(true);

  IterationHelper::Metrics after = helper.snapshot(runtime);
  printf("After:  WNS=%.3f ps  TNS=%.3f ps  Leakage=%.3f uW (%.1fs)\n",
         after.wns_ps, after.tns_ps, after.leakage * 1e6, runtime);
  printf("Delta:  dWNS=%+.3f ps  dTNS=%+.3f ps  dLeakage=%+.3f uW  runtime=%.1fs\n",
         after.wns_ps - before.wns_ps,
         after.tns_ps - before.tns_ps,
         (after.leakage - before.leakage) * 1e6,
         runtime);
  fflush(stdout);

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
  printf("----- Testing Parallel LR Resize By Array With Precheck (New Framework) -----\n");
  sta::Scene *corner = sta->findScene("default");

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  bool was_converged = false;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  if (lrfVerbose()) {
    printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    printf("----- LR ResizeByArrayWithPrecheck Iteration %zu (top_ratio=%.4f, adaptive=%.4f) -----\n",
           i+1, top_ratio, incre_sta->adaptiveTopRatio());
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayWithPrecheck(resizer, avg_delay, avg_leakage,
                                                    PT_tradeoff, top_ratio);
    auto end = std::chrono::high_resolution_clock::now();
    if (lrfVerbose()) {
      printf("Iteration %zu took %f seconds\n", i+1,
             std::chrono::duration<double>(end - start).count());
    }

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
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

    if (lrfVerbose()) {
      printf("Worst Negative Slack: %f\n", wns * 1e12);
    }
    if (lrfVerbose()) {
      printf("Total Negative Slack: %f\n", tns * 1e12);
    }
    if (lrfVerbose()) {
      printf("Total Leakage Power: %f uW\n", leakage * 1e6);
    }
    fflush(stdout);

    // Adaptive instance filter: after iter > 3, always use adaptive topN.
    //   Normal: N = last_change_count * 1.5, ratio = N / total (monotonically decreasing)
    //   Revert: ratio *= 0.25 (aggressive 1/4 discount)
    if (i >= 3) {
      bool regression = was_converged ? (wns < 0.0) : (wns < best_wns);
      if (regression) {
        // Revert + 1/4 discount
        float cur_ratio = incre_sta->adaptiveTopRatio();
        float new_ratio = (cur_ratio > 0.0f)
            ? cur_ratio * 0.25f          // 1/4 discount
            : top_ratio * 0.25f;         // first regression
        incre_sta->setAdaptiveTopRatio(new_ratio);
        printf("Timing regression (WNS %e, best %e), revert + 1/4 discount ratio to %.4f.\n",
               wns, best_wns, new_ratio);
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::undoEco(block);
        local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
        sta->delaysInvalid();
        sta->updateTiming(true);
        odb::dbDatabase::beginEco(block);
        eco_iter++;
        if (eco_iter > 6) {
          printf("Too many ECO iterations (%zu), terminating.\n", eco_iter);
          break;
        }
        continue;
      }
      // Improving: update ratio from change_count * 1.5, but never increase
      int change_count = incre_sta->lastChangeCount();
      TaskArranger *ta = incre_sta->localSta()->taskArranger();
      int total = static_cast<int>(ta->vertexCount());
      if (total > 0) {
        float candidate = static_cast<float>(change_count) * 1.5f / total;
        float cur_ratio = incre_sta->adaptiveTopRatio();
        float cap = (cur_ratio > 0.0f) ? cur_ratio : top_ratio;
        float new_ratio = std::min(candidate, cap);  // only goes down
        incre_sta->setAdaptiveTopRatio(new_ratio);
      }
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
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      odb::dbDatabase::beginEco(block);
      top_ratio *= 0.5f;
      incre_sta->setAdaptiveTopRatio(top_ratio);
      printf("Halved top_ratio to %.4f after rollback (synced adaptive).\n", top_ratio);
      fflush(stdout);
      eco_iter++;
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    if (lrfVerbose()) {
      printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
    }
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    if (lrfVerbose()) {
      printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
    }
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
  sta::Scene *corner = sta->findScene("default");

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();
  lr_helper->setRatcons(ratcons);

  incre_sta->setMaxResizeNum(max_resize_num);

  odb::dbDatabase::beginEco(block);
  float best_leakage = std::numeric_limits<float>::max();
  size_t no_improve_count_ = 0;
  size_t eco_iter = 0;
  bool was_converged = false;
  sta::Slack best_wns = sta->worstSlack(sta::MinMax::max());
  sta::Slack best_tns = sta->totalNegativeSlack(sta::MinMax::max());
  sta::Slack tns, wns;
  if (lrfVerbose()) {
    printf("Initial WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
  }
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  for (size_t i = 0; i < iterations; ++i) {
    incre_sta->lmUpdate();
    sta->findRequireds();
    printf("----- LR ResizeByArrayWithPrecheckBuffering Iteration %zu (top_ratio=%.4f) -----\n",
           i+1, top_ratio);
    auto start = std::chrono::high_resolution_clock::now();
    incre_sta->parallelResizeByArrayWithPrecheck(resizer, avg_delay, avg_leakage,
                                                    PT_tradeoff, top_ratio);
    auto end = std::chrono::high_resolution_clock::now();
    if (lrfVerbose()) {
      printf("Iteration %zu took %f seconds\n", i+1,
             std::chrono::duration<double>(end - start).count());
    }

    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
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

    if (lrfVerbose()) {
      printf("Worst Negative Slack after RSZ: %f\n", wns * 1e12);
    }
    if (lrfVerbose()) {
      printf("Total Negative Slack after RSZ: %f\n", tns * 1e12);
    }
    if (lrfVerbose()) {
      printf("Total Leakage Power after RSZ: %f uW\n", leakage * 1e6);
    }
    fflush(stdout);

    // Post-convergence regression: immediate rollback + halve
    if (was_converged && wns < 0.0) {
      printf("Post-convergence regression (WNS %e), immediate rollback.\n", wns);
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
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
    } else if (eco_iter > 6) {
      printf("No improvement for %zu ECO iterations, terminating.\n", eco_iter);
      break;
    } else {
      printf("Reverting to previous design.\n");
      odb::dbDatabase::endEco(block);
      odb::dbDatabase::undoEco(block);
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      odb::dbDatabase::beginEco(block);
      top_ratio *= 0.5f;
      printf("Halved top_ratio to %.4f after rollback.\n", top_ratio);
      fflush(stdout);
      eco_iter++;
    }

    // Perform parallel buffering if WNS is negative after resize
    if (wns < 0 && i > 3) {
      printf("----- WNS negative, performing parallel buffering -----\n");
      double wns_before_buf = wns * 1e12;
      double tns_before_buf = tns * 1e12;
      sta->findRequireds();
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
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
    }
  }
  tns = sta->totalNegativeSlack(sta::MinMax::max());
  wns = sta->worstSlack(sta::MinMax::max());
  if (wns > best_wns) {
    odb::dbDatabase::endEco(block);
    if (lrfVerbose()) {
      printf("Final design accepted with WNS: %f, TNS: %f\n", wns * 1e12, tns * 1e12);
    }
  } else {
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    if (lrfVerbose()) {
      printf("Reverted to best design with WNS: %f, TNS: %f\n", best_wns * 1e12, best_tns * 1e12);
    }
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
  printf("Average Leakage: %f uW\n", avg_leakage * 1e6);

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
      printf("No legal equiv cells for %s\n",
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
recordGraphTimingFromPtGraph(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing, bool verbose)
{
  printf("Recording Graph Timing from PtGraph for cell %s\n", 
          graph_timing.cell ? graph_timing.cell->name() : "nullptr");
  fflush(stdout);
  // First copy slews and paths from pt_graph's vertex to graph_timing,
  // and write back to the global graph using the same dispatch as
  // ParallelVisitor::updateVertexInfo.
  for (PtVertex &pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    // Record slews from pt_vertex to graph_timing
    std::string vertex_name = pt_vertex.vertex()->name(sta->network());
    TimingInfo vertex_timing_info;
    vertex_timing_info.type = TimingType::VERTEX;
    const sta::Slew *slews = pt_vertex.slews();
    vertex_timing_info.slews.clear();
    for (int i = 0; i < pt_vertex.slewCount(); ++i) {
      vertex_timing_info.slews.push_back(slews[i]);
    }
    // Record paths (including arrivals and requireds)
    sta::Path *pt_paths = pt_vertex.paths();
    vertex_timing_info.paths.clear();
    int path_count = pt_graph->tagGroup(pt_vertex)->pathCount();
    for (int i = 0; i < path_count; ++i) {
      sta::Path path = pt_paths[i];
      vertex_timing_info.paths.push_back(path);
      printf(" OpenSta: Recorded path for vertex %s: dcalc_pt %u, rf %d, arrival %f, required %f, tagIndex %d\n",
              vertex_name.c_str(),
              path.dcalcAnalysisPtIndex(sta) ? path.dcalcAnalysisPtIndex(sta) : 0,
              path.rfIndex(sta),
              path.arrival() * 1e12,
              path.required() * 1e12,
              path.tagIndex(sta));
      fflush(stdout);
    }
    vertex_timing_info.tag_group_index = pt_vertex.tagGroupIndex();
    graph_timing.vertex_timing_map[vertex_name] = vertex_timing_info;

    // Write back to global graph (consistent with ParallelVisitor::updateVertexInfo)
    sta::Vertex *sta_vertex = pt_vertex.vertex();
    PtVertexType type = pt_vertex.type();
    if (type == PtVertexType::RefInput
     || type == PtVertexType::RefOutput
     || type == PtVertexType::SiblingLoad) {
      pt_graph->writeSlewToGraph(pt_vertex, sta_vertex);
      pt_graph->writePathsToGraph(pt_vertex, sta_vertex);
    }
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
TestLrf::testParallelKKTProjection(sta::dbSta* sta,
                                    rsz::Resizer *resizer,
                                    odb::dbBlock *block,
                                    size_t thread_num,
                                    std::string lr_helper_method)
{
  printf("----- Testing Parallel KKT Projection (serial vs %zu threads) -----\n",
         thread_num);
  fflush(stdout);


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

// ============================================================
//  testLocalStaAccuracy — Full traversal with selective resize.
//  Compares slew and arrival from LocalSTA write-back against
//  updateTiming ground truth.
// ============================================================
void
TestLrf::testLocalStaAccuracy(sta::dbSta* sta, rsz::Resizer *resizer,
                               odb::dbBlock *block, size_t max_steps)
{
  printf("\n========================================\n");
  printf(" LocalSTA Accuracy Test (slew + arrival)\n");
  printf("========================================\n\n");
  fflush(stdout);

  sta->updateTiming(true);
  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, 1);
  LocalSta *local_sta = incre_sta->localSta();
  resizer->makeEquivCells();
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();

  // ---- Generate resize sequence ----
  struct ResizeStep {
    sta::Instance *inst;
    sta::LibertyCell *orig_cell;
    sta::LibertyCell *target_cell;
  };
  std::vector<ResizeStep> sequence;
  for (odb::dbInst *db_inst : block->getInsts()) {
    if (sequence.size() >= max_steps) break;
    sta::Instance *sta_inst = sta->getDbNetwork()->dbToSta(db_inst);
    if (!sta_inst) continue;
    sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
    if (!orig_cell) continue;
    sta::LibertyCellSeq *equiv_cells = sta->equivCells(orig_cell);
    if (!equiv_cells || equiv_cells->size() < 2) continue;
    sta::LibertyCell *target = nullptr;
    for (sta::LibertyCell *ec : *equiv_cells) {
      if (ec != orig_cell && sta::equivCellsArcs(orig_cell, ec)) {
        target = ec; break;
      }
    }
    if (!target) continue;
    sequence.push_back({sta_inst, orig_cell, target});
  }
  printf("Generated %zu resize steps\n\n", sequence.size());
  fflush(stdout);

  // ---- Watchpoint: track g16/A ----
  sta::Vertex *watch_vtx = nullptr;
  {
    odb::dbInst *wi = block->findInst("g16");
    if (wi) {
      sta::Instance *wsi = sta->getDbNetwork()->dbToSta(wi);
      sta::InstancePinIterator *pit = sta->network()->pinIterator(wsi);
      while (pit->hasNext()) {
        sta::Pin *p = pit->next();
        if (std::string(sta->network()->portName(p)) == "A") {
          watch_vtx = sta->graph()->pinLoadVertex(p);
          break;
        }
      }
      delete pit;
    }
  }

  // ---- Write-back helper with watchpoint ----
  auto writeBackTiming = [&](PtGraph *pt_graph, sta::Instance *ref_inst) {
    for (PtVertex &pv : pt_graph->ptVertices()) {
      if (pv.type() == PtVertexType::Sentinel || !pv.vertex()) continue;
      PtVertexType type = pv.type();
      if (type == PtVertexType::RefDriver
       || type == PtVertexType::RefInput
       || type == PtVertexType::RefOutput
       || type == PtVertexType::SiblingLoad
       || type == PtVertexType::SiblingDrvr) {
        if (pv.vertex() == watch_vtx) {
          // Read current global + local arrival for tag 333 (rise, max)
          float local_arr = 0, global_arr = 0;
          sta::Path *pp = pv.paths();
          sta::TagGroup *ptg = pt_graph->tagGroup(pv);
          if (pp && ptg) {
            for (size_t i = 0; i < ptg->pathCount(); i++)
              if (pp[i].rfIndex(sta) == 0)
                local_arr = std::max(local_arr, (float)(pp[i].arrival() * 1e12));
          }
          sta::Path *sp = watch_vtx->paths();
          sta::TagGroup *stg = sta->search()->tagGroup(watch_vtx);
          if (sp && stg) {
            for (size_t i = 0; i < stg->pathCount(); i++)
              if (sp[i].rfIndex(sta) == 0)
                global_arr = std::max(global_arr, (float)(sp[i].arrival() * 1e12));
          }
          if (std::abs(local_arr - global_arr) > 0.001)
            printf("[W-g16/A] ref=%s type=%s glb=%.3f → writing=%.3f (diff=%.3f)\n",
                   sta->network()->pathName(ref_inst),
                   ptVertexTypeName(type), global_arr, local_arr,
                   local_arr - global_arr);
        }
        pt_graph->writeSlewToGraph(pv, pv.vertex());
        pt_graph->writePathsToGraph(pv, pv.vertex());
      }
    }
  };

  // ---- Build resize lookup ----
  std::map<sta::Instance*, sta::LibertyCell*> resize_map;
  for (auto &step : sequence)
    resize_map[step.inst] = step.target_cell;

  // ---- Pass 0: full traversal with resize ----
  size_t inst_count = 0, resize_count = 0;
  for (odb::dbInst *db_inst : block->getInsts()) {
    sta::Instance *inst = sta->getDbNetwork()->dbToSta(db_inst);
    if (!inst || !sta->network()->libertyCell(inst)) continue;
    if (sta->network()->libertyCell(inst)->hasSequentials()) continue;
    PtGraph *pg = local_sta->makePtGraph(inst, false);
    // Debug: watch g16's input pin A
    bool is_g16 = (std::string(sta->network()->pathName(inst)) == "g16");
    if (is_g16) {
      sta::Scene *dc = sta->findScene("default");
      sta::DcalcAPIndex ddap = dc->dcalcAnalysisPtIndex(sta::MinMax::max());
      printf("[G16] All PtGraph vertices (%zu total):\n", pg->ptVertices().size());
      for (size_t vi = 0; vi < pg->ptVertices().size(); vi++) {
        PtVertex &pv = pg->ptVertices()[vi];
        if (!pv.vertex()) { printf("  [%zu] sentinel\n", vi); continue; }
        const sta::Pin *pin = pv.vertex()->pin();
        bool is_port = pin ? sta->network()->isTopLevelPort(pin) : false;
        bool is_drvr = pin ? sta->network()->isDriver(pin) : false;
        bool is_load = pin ? sta->network()->isLoad(pin) : false;
        printf("  [%zu] %s type=%s hasFanin=%d isPort=%d isDrvr=%d isLoad=%d",
               vi, pv.vertex()->name(sta->network()),
               ptVertexTypeName(pv.type()), pv.hasFanin(),
               is_port, is_drvr, is_load);
        // Print in-edge count and out-edge count
        int in_cnt = 0, out_cnt = 0;
        PtVertexInEdgeIterator in_iter(vi, pg);
        while (in_iter.hasNext()) { in_iter.next(); in_cnt++; }
        PtVertexOutEdgeIterator out_iter(vi, pg);
        while (out_iter.hasNext()) { out_iter.next(); out_cnt++; }
        printf(" inEdges=%d outEdges=%d", in_cnt, out_cnt);
        // Print arrivals
        sta::Path *pp = pv.paths();
        sta::TagGroup *ptg = pg->tagGroup(pv);
        if (pp && ptg) {
          printf(" arrivals:");
          for (size_t i = 0; i < ptg->pathCount(); i++)
            printf("[rf=%d arr=%.3f] ", pp[i].rfIndex(sta), pp[i].arrival()*1e12);
        } else printf(" NO_PATHS");
        printf("\n");
      }
      fflush(stdout);
    }
    auto it = resize_map.find(inst);
    if (it != resize_map.end()) {
      local_sta->increAndGetLocalTimingCost(pg, arc_delay_calc, it->second);
      sta->replaceCell(inst, it->second);
      resize_count++;
    } else {
      local_sta->updateLocalTiming(pg, arc_delay_calc);
    }
    if (is_g16) {
      printf("[G16] After updateLocalTiming:\n");
      for (PtVertex &pv : pg->ptVertices()) {
        if (!pv.vertex()) continue;
        sta::Path *pp = pv.paths();
        sta::TagGroup *ptg = pg->tagGroup(pv);
        printf("  %s (type=%s): ", pv.vertex()->name(sta->network()),
               ptVertexTypeName(pv.type()));
        if (pp && ptg) {
          for (size_t i = 0; i < ptg->pathCount(); i++)
            printf("[tag=%d rf=%d arr=%.3f] ", pp[i].tagIndex(sta), pp[i].rfIndex(sta), pp[i].arrival()*1e12);
        } else printf("NO PATHS");
        printf("\n");
      }
      fflush(stdout);
    }
    writeBackTiming(pg, inst);
    inst_count++;
  }
  printf("Pass 0: %zu instances (%zu resized)\n", inst_count, resize_count);

  // ---- Pass 1: second traversal (no resize) ----
  size_t pass1_count = 0;
  for (odb::dbInst *db_inst : block->getInsts()) {
    sta::Instance *inst = sta->getDbNetwork()->dbToSta(db_inst);
    if (!inst || !sta->network()->libertyCell(inst)) continue;
    if (sta->network()->libertyCell(inst)->hasSequentials()) continue;
    PtGraph *pg = local_sta->makePtGraph(inst, false);
    local_sta->updateLocalTiming(pg, arc_delay_calc);
    writeBackTiming(pg, inst);
    pass1_count++;
  }
  printf("Pass 1: %zu instances (no resize)\n", pass1_count);
  printf("\n");
  fflush(stdout);

  // ---- Debug: fanin cone arrival after write-back, before updateTiming ----
  {
    sta::Scene *dc = sta->findScene("default");
    sta::DcalcAPIndex ddap = dc->dcalcAnalysisPtIndex(sta::MinMax::max());
    const char *pins[] = {
      "g158293/A", "g163646/Y", "g163646/A",
      "g184873/Y", "g184873/A", "g184873/B", "g184873/C",
      "g16/Y", "g16/A", nullptr
    };
    printf("--- Fanin cone arrival (after writeBack, before updateTiming) ---\n");
    for (int k = 0; pins[k]; k++) {
      std::string pname(pins[k]);
      std::string inst_name = pname.substr(0, pname.rfind('/'));
      std::string pin_short = pname.substr(pname.rfind('/') + 1);
      odb::dbInst *di = block->findInst(inst_name.c_str());
      if (!di) { printf("  %s: inst not found\n", pins[k]); continue; }
      sta::Instance *si = sta->getDbNetwork()->dbToSta(di);
      sta::InstancePinIterator *pit = sta->network()->pinIterator(si);
      while (pit->hasNext()) {
        sta::Pin *p = pit->next();
        if (std::string(sta->network()->portName(p)) == pin_short) {
          sta::Vertex *v = sta->network()->isDriver(p)
              ? sta->graph()->pinDrvrVertex(p) : sta->graph()->pinLoadVertex(p);
          if (!v) break;
          // Read all paths for this vertex
          sta::Path *paths = v->paths();
          sta::TagGroup *tg = sta->search()->tagGroup(v);
          printf("  %s:", pins[k]);
          if (paths && tg) {
            for (size_t i = 0; i < tg->pathCount(); i++) {
              printf(" [tag=%d rf=%d arr=%.3f]",
                     paths[i].tagIndex(sta), paths[i].rfIndex(sta),
                     paths[i].arrival() * 1e12);
            }
          } else {
            printf(" NO PATHS");
          }
          printf("\n");
          break;
        }
      }
      delete pit;
    }
    fflush(stdout);
  }

  // ---- Snapshot: read slew + arrival from global graph ----
  sta::Scene *corner = sta->findScene("default");
  sta::DcalcAPIndex dap = corner->dcalcAnalysisPtIndex(sta::MinMax::max());

  struct VtxRecord {
    sta::Vertex *vtx;
    std::string name;
    bool is_driver;
    float wb_slew_r, wb_slew_f;
    float open_slew_r, open_slew_f;
    float wb_arr_r, wb_arr_f;
    float open_arr_r, open_arr_f;
  };
  std::vector<VtxRecord> records;

  sta::VertexIterator vtx_iter(sta->graph());
  while (vtx_iter.hasNext()) {
    sta::Vertex *vtx = vtx_iter.next();
    VtxRecord rec;
    rec.vtx = vtx;
    rec.name = vtx->name(sta->network());
    rec.is_driver = sta->network()->isDriver(vtx->pin());
    rec.wb_slew_r = sta->graph()->slew(vtx, sta::RiseFall::rise(), dap) * 1e12;
    rec.wb_slew_f = sta->graph()->slew(vtx, sta::RiseFall::fall(), dap) * 1e12;
    // Read arrival from paths
    sta::Path *paths = vtx->paths();
    sta::TagGroup *tg = sta->search()->tagGroup(vtx);
    rec.wb_arr_r = rec.wb_arr_f = 0;
    if (paths && tg) {
      for (size_t i = 0; i < tg->pathCount(); i++) {
        float arr = paths[i].arrival() * 1e12;
        int rf_idx = paths[i].rfIndex(sta);
        if (rf_idx == 0) rec.wb_arr_r = std::max(rec.wb_arr_r, arr);
        else             rec.wb_arr_f = std::max(rec.wb_arr_f, arr);
      }
    }
    rec.open_slew_r = rec.open_slew_f = 0;
    rec.open_arr_r = rec.open_arr_f = 0;
    records.push_back(rec);
  }

  // ---- updateTiming for ground truth ----
  sta->updateTiming(true);
  sta->findRequireds();

  // ---- Read ground truth ----
  for (auto &rec : records) {
    rec.open_slew_r = sta->graph()->slew(rec.vtx, sta::RiseFall::rise(), dap) * 1e12;
    rec.open_slew_f = sta->graph()->slew(rec.vtx, sta::RiseFall::fall(), dap) * 1e12;
    sta::Path *paths = rec.vtx->paths();
    sta::TagGroup *tg = sta->search()->tagGroup(rec.vtx);
    if (paths && tg) {
      for (size_t i = 0; i < tg->pathCount(); i++) {
        float arr = paths[i].arrival() * 1e12;
        int rf_idx = paths[i].rfIndex(sta);
        if (rf_idx == 0) rec.open_arr_r = std::max(rec.open_arr_r, arr);
        else             rec.open_arr_f = std::max(rec.open_arr_f, arr);
      }
    }
  }

  // ---- Statistics: separate load vs driver ----
  struct ErrAccum { double sum=0, mx=0; size_t cnt=0, nz=0;
    void add(double e) { sum+=std::abs(e); mx=std::max(mx,std::abs(e)); cnt++; if(std::abs(e)>0.001) nz++; }
    double mean() const { return cnt>0 ? sum/cnt : 0; }
  };
  ErrAccum slew_drv, arr_load_comb, arr_load_seq, arr_drv, arr_all;

  for (auto &rec : records) {
    double aerr = std::max(std::abs(rec.wb_arr_r - rec.open_arr_r),
                           std::abs(rec.wb_arr_f - rec.open_arr_f));
    arr_all.add(aerr);
    if (rec.is_driver) {
      double serr = std::max(std::abs(rec.wb_slew_r - rec.open_slew_r),
                             std::abs(rec.wb_slew_f - rec.open_slew_f));
      slew_drv.add(serr);
      arr_drv.add(aerr);
    } else {
      // Check if this load pin belongs to a sequential cell
      sta::Instance *inst = sta->network()->instance(rec.vtx->pin());
      bool is_seq = inst && sta->network()->libertyCell(inst)
                    && sta->network()->libertyCell(inst)->hasSequentials();
      if (is_seq) arr_load_seq.add(aerr);
      else        arr_load_comb.add(aerr);
    }
  }

  printf("=== Slew (driver vertices) ===\n");
  printf("  Count: %zu  non-zero: %zu  mean: %.6f ps  max: %.6f ps\n\n",
         slew_drv.cnt, slew_drv.nz, slew_drv.mean(), slew_drv.mx);

  printf("=== Arrival (combinational load vertices) ===\n");
  printf("  Count: %zu  non-zero: %zu  mean: %.6f ps  max: %.6f ps\n",
         arr_load_comb.cnt, arr_load_comb.nz, arr_load_comb.mean(), arr_load_comb.mx);
  // Histogram of comb load arrival errors
  {
    size_t h[7] = {}; // 0: <1ps, 1: 1-5, 2: 5-10, 3: 10-20, 4: 20-50, 5: 50-100, 6: >=100
    size_t comb_port = 0; // top-level ports (no liberty cell)
    for (auto &rec : records) {
      if (rec.is_driver) continue;
      sta::Instance *inst = sta->network()->instance(rec.vtx->pin());
      if (inst && sta->network()->libertyCell(inst)
          && sta->network()->libertyCell(inst)->hasSequentials()) continue;
      bool is_port = !inst || !sta->network()->libertyCell(inst);
      double aerr = std::max(std::abs(rec.wb_arr_r - rec.open_arr_r),
                             std::abs(rec.wb_arr_f - rec.open_arr_f));
      if (is_port && aerr > 1.0) { comb_port++; continue; }
      if (aerr < 1.0)        h[0]++;
      else if (aerr < 5.0)   h[1]++;
      else if (aerr < 10.0)  h[2]++;
      else if (aerr < 20.0)  h[3]++;
      else if (aerr < 50.0)  h[4]++;
      else if (aerr < 100.0) h[5]++;
      else                    h[6]++;
    }
    printf("  Histogram (excl ports): <1ps=%zu  1-5=%zu  5-10=%zu  10-20=%zu  20-50=%zu  50-100=%zu  >=100=%zu\n",
           h[0], h[1], h[2], h[3], h[4], h[5], h[6]);
    printf("  Top-level ports with >1ps err: %zu\n\n", comb_port);
  }

  printf("=== Arrival (sequential load vertices — CLK/D/etc) ===\n");
  printf("  Count: %zu  non-zero: %zu  mean: %.6f ps  max: %.6f ps\n\n",
         arr_load_seq.cnt, arr_load_seq.nz, arr_load_seq.mean(), arr_load_seq.mx);

  printf("=== Arrival (driver vertices) ===\n");
  printf("  Count: %zu  non-zero: %zu  mean: %.6f ps  max: %.6f ps\n\n",
         arr_drv.cnt, arr_drv.nz, arr_drv.mean(), arr_drv.mx);

  printf("=== Arrival (all vertices) ===\n");
  printf("  Count: %zu  non-zero: %zu  mean: %.6f ps  max: %.6f ps\n\n",
         arr_all.cnt, arr_all.nz, arr_all.mean(), arr_all.mx);

  // Top 10 worst by slew error
  std::sort(records.begin(), records.end(),
            [](const VtxRecord &a, const VtxRecord &b) {
              return std::max(std::abs(a.wb_slew_r-a.open_slew_r), std::abs(a.wb_slew_f-a.open_slew_f))
                   > std::max(std::abs(b.wb_slew_r-b.open_slew_r), std::abs(b.wb_slew_f-b.open_slew_f));
            });
  printf("Top 10 worst SLEW:\n");
  printf("  %-40s %10s %10s %10s\n", "Vertex", "WB(ps)", "Open(ps)", "Err(ps)");
  for (size_t j = 0; j < std::min(records.size(), (size_t)10); j++) {
    auto &r = records[j];
    double e_r = r.wb_slew_r - r.open_slew_r;
    double e_f = r.wb_slew_f - r.open_slew_f;
    double e = std::abs(e_r) > std::abs(e_f) ? e_r : e_f;
    float wb = std::abs(e_r) > std::abs(e_f) ? r.wb_slew_r : r.wb_slew_f;
    float op = std::abs(e_r) > std::abs(e_f) ? r.open_slew_r : r.open_slew_f;
    printf("  %-40.40s %10.3f %10.3f %10.3f\n", r.name.c_str(), wb, op, e);
  }

  // Top 10 worst arrival — LOAD vertices only
  std::sort(records.begin(), records.end(),
            [](const VtxRecord &a, const VtxRecord &b) {
              double ea = a.is_driver ? 0 : std::max(std::abs(a.wb_arr_r-a.open_arr_r), std::abs(a.wb_arr_f-a.open_arr_f));
              double eb = b.is_driver ? 0 : std::max(std::abs(b.wb_arr_r-b.open_arr_r), std::abs(b.wb_arr_f-b.open_arr_f));
              return ea > eb;
            });
  printf("\nTop 10 worst ARRIVAL (load vertices only):\n");
  printf("  %-40s %10s %10s %10s\n", "Vertex", "WB(ps)", "Open(ps)", "Err(ps)");
  for (size_t j = 0, shown = 0; j < records.size() && shown < 10; j++) {
    auto &r = records[j];
    if (r.is_driver) continue;
    double e_r = r.wb_arr_r - r.open_arr_r;
    double e_f = r.wb_arr_f - r.open_arr_f;
    double e = std::abs(e_r) > std::abs(e_f) ? e_r : e_f;
    float wb = std::abs(e_r) > std::abs(e_f) ? r.wb_arr_r : r.wb_arr_f;
    float op = std::abs(e_r) > std::abs(e_f) ? r.open_arr_r : r.open_arr_f;
    printf("  %-40.40s %10.3f %10.3f %10.3f\n", r.name.c_str(), wb, op, e);
    shown++;
  }

  // Top 10 worst arrival — comb load only, with fanin driver info
  printf("\nTop 10 worst ARRIVAL (comb load, excl seq cells):\n");
  printf("  %-40s %-20s %10s %10s %10s %s\n", "Vertex", "Cell", "WB(ps)", "Open(ps)", "Err(ps)", "FaninInfo");
  for (size_t j = 0, shown = 0; j < records.size() && shown < 10; j++) {
    auto &r = records[j];
    if (r.is_driver) continue;
    sta::Instance *inst = sta->network()->instance(r.vtx->pin());
    if (inst && sta->network()->libertyCell(inst)
        && sta->network()->libertyCell(inst)->hasSequentials()) continue;
    double e_r = r.wb_arr_r - r.open_arr_r;
    double e_f = r.wb_arr_f - r.open_arr_f;
    double e = std::abs(e_r) > std::abs(e_f) ? e_r : e_f;
    if (std::abs(e) < 0.001) continue;
    float wb = std::abs(e_r) > std::abs(e_f) ? r.wb_arr_r : r.wb_arr_f;
    float op = std::abs(e_r) > std::abs(e_f) ? r.open_arr_r : r.open_arr_f;
    std::string cell = (inst && sta->network()->libertyCell(inst))
        ? sta->network()->libertyCell(inst)->name() : std::string("?");
    std::string fanin_info;
    sta::VertexInEdgeIterator in_iter(r.vtx, sta->graph());
    while (in_iter.hasNext()) {
      sta::Edge *edge = in_iter.next();
      if (edge->isWire()) {
        sta::Vertex *drvr = edge->from(sta->graph());
        sta::Instance *di = sta->network()->instance(drvr->pin());
        bool drvr_seq = di && sta->network()->libertyCell(di)
                        && sta->network()->libertyCell(di)->hasSequentials();
        fanin_info = drvr_seq ? "fanin=SEQ" : "fanin=COMB";
        break;
      }
    }
    printf("  %-40.40s %-20.20s %10.3f %10.3f %10.3f %s\n",
           r.name.c_str(), cell.c_str(), wb, op, e, fanin_info.c_str());
    shown++;
  }

  // Comb load error decomposition: for loads with >5ps error,
  // check if the error comes from fanin driver arrival.
  {
    // Build a lookup: vertex -> record index
    std::map<sta::Vertex*, size_t> vtx_idx;
    for (size_t i = 0; i < records.size(); i++)
      vtx_idx[records[i].vtx] = i;

    printf("\nComb load error decomposition (err > 5ps, excl ports/seq):\n");
    printf("  %-30s %8s | %-30s %8s %8s | %s\n",
           "Load", "LoadErr", "FaninDrvr", "DrvrErr", "DrvrSlew", "Same?");
    size_t decomp_shown = 0;
    // Sort by error first
    std::vector<size_t> sorted_idx;
    for (size_t i = 0; i < records.size(); i++) sorted_idx.push_back(i);
    std::sort(sorted_idx.begin(), sorted_idx.end(),
              [&](size_t a, size_t b) {
                auto ea = std::max(std::abs(records[a].wb_arr_r-records[a].open_arr_r),
                                   std::abs(records[a].wb_arr_f-records[a].open_arr_f));
                auto eb = std::max(std::abs(records[b].wb_arr_r-records[b].open_arr_r),
                                   std::abs(records[b].wb_arr_f-records[b].open_arr_f));
                return ea > eb;
              });
    for (size_t idx : sorted_idx) {
      if (decomp_shown >= 20) break;
      auto &r = records[idx];
      if (r.is_driver) continue;
      sta::Instance *inst = sta->network()->instance(r.vtx->pin());
      if (!inst || !sta->network()->libertyCell(inst)) continue;
      if (sta->network()->libertyCell(inst)->hasSequentials()) continue;
      double load_err = std::max(std::abs(r.wb_arr_r - r.open_arr_r),
                                 std::abs(r.wb_arr_f - r.open_arr_f));
      if (load_err < 5.0) continue;
      // Find fanin driver
      sta::VertexInEdgeIterator in_iter(r.vtx, sta->graph());
      while (in_iter.hasNext()) {
        sta::Edge *edge = in_iter.next();
        if (!edge->isWire()) continue;
        sta::Vertex *drvr = edge->from(sta->graph());
        auto dit = vtx_idx.find(drvr);
        if (dit != vtx_idx.end()) {
          auto &dr = records[dit->second];
          double drvr_err = std::max(std::abs(dr.wb_arr_r - dr.open_arr_r),
                                     std::abs(dr.wb_arr_f - dr.open_arr_f));
          double slew_err = std::max(std::abs(dr.wb_slew_r - dr.open_slew_r),
                                     std::abs(dr.wb_slew_f - dr.open_slew_f));
          bool same_sign = (r.wb_arr_r - r.open_arr_r) * (dr.wb_arr_r - dr.open_arr_r) > 0;
          printf("  %-30.30s %8.2f | %-30.30s %8.2f %8.2f | %s\n",
                 r.name.c_str(), load_err,
                 dr.name.c_str(), drvr_err, slew_err,
                 same_sign ? "YES" : "NO");
        } else {
          printf("  %-30.30s %8.2f | (driver not in records)\n",
                 r.name.c_str(), load_err);
        }
        break;
      }
      decomp_shown++;
    }
    fflush(stdout);
  }

  // Top 10 worst arrival — DRIVER vertices only
  std::sort(records.begin(), records.end(),
            [](const VtxRecord &a, const VtxRecord &b) {
              double ea = a.is_driver ? std::max(std::abs(a.wb_arr_r-a.open_arr_r), std::abs(a.wb_arr_f-a.open_arr_f)) : 0;
              double eb = b.is_driver ? std::max(std::abs(b.wb_arr_r-b.open_arr_r), std::abs(b.wb_arr_f-b.open_arr_f)) : 0;
              return ea > eb;
            });
  printf("\nTop 10 worst ARRIVAL (driver vertices only):\n");
  printf("  %-40s %10s %10s %10s\n", "Vertex", "WB(ps)", "Open(ps)", "Err(ps)");
  for (size_t j = 0, shown = 0; j < records.size() && shown < 10; j++) {
    auto &r = records[j];
    if (!r.is_driver) continue;
    double e_r = r.wb_arr_r - r.open_arr_r;
    double e_f = r.wb_arr_f - r.open_arr_f;
    double e = std::abs(e_r) > std::abs(e_f) ? e_r : e_f;
    float wb = std::abs(e_r) > std::abs(e_f) ? r.wb_arr_r : r.wb_arr_f;
    float op = std::abs(e_r) > std::abs(e_f) ? r.open_arr_r : r.open_arr_f;
    printf("  %-40.40s %10.3f %10.3f %10.3f\n", r.name.c_str(), wb, op, e);
    shown++;
  }

  // ================================================================
  //  Slew Violation Analysis: compare eval-flow violation check
  //  before vs after updateTiming.
  // ================================================================
  {
    printf("\n========================================\n");
    printf(" Slew Violation Analysis\n");
    printf("========================================\n\n");
    fflush(stdout);

    sta::Scene *corner = sta->findScene("default");

    // Helper: compute slew violations like test_lrf.py get_score()
    // Uses Timing.h-style API: getPinSlew vs getMaxSlewLimit per ITerm
    auto computeSlewViolations = [&](const char *label) {
      double slew_total = 0.0;
      size_t violation_count = 0;
      size_t pin_count = 0;
      struct ViolPin { std::string name; float slew; float limit; float diff; };
      std::vector<ViolPin> worst;

      for (odb::dbITerm *iterm : block->getITerms()) {
        odb::dbNet *net = iterm->getNet();
        if (!net) continue;
        auto sig = net->getSigType();
        if (sig == odb::dbSigType::POWER || sig == odb::dbSigType::GROUND
            || sig == odb::dbSigType::CLOCK)
          continue;

        odb::dbMTerm *mterm = iterm->getMTerm();
        if (!mterm) continue;

        // Get slew limit
        float limit = local_sta->getPortMaxSlewLimit(
            sta->network()->libertyPort(sta->getDbNetwork()->dbToSta(mterm)));
        if (limit <= 0 || limit >= 1.0) continue;  // no valid limit

        // Get actual slew from global graph
        sta::Pin *sta_pin = sta->getDbNetwork()->dbToSta(iterm);
        if (!sta_pin) continue;
        sta::Vertex *vtx = sta->graph()->pinLoadVertex(sta_pin);
        if (!vtx) vtx = sta->graph()->pinDrvrVertex(sta_pin);
        if (!vtx) continue;

        float slew = 0.0;
        for (const sta::RiseFall *rf : sta::RiseFall::range()) {
          for (sta::DcalcAPIndex dcalc_ap = 0; dcalc_ap < sta->graph()->apCount(); dcalc_ap++) {
            float s = sta->graph()->slew(vtx, rf, dcalc_ap);
            slew = std::max(slew, (float)delayAsFloat(s));
          }
        }

        pin_count++;
        if (slew > limit) {
          float diff = (slew - limit) * 1e9;  // convert to ns
          slew_total += diff;
          violation_count++;
          if (worst.size() < 10 || diff > worst.back().diff)
            worst.push_back({sta->network()->pathName(sta_pin),
                             slew * 1e12f, limit * 1e12f, diff});
        }
      }

      // Sort worst and keep top 10
      std::sort(worst.begin(), worst.end(),
                [](const ViolPin &a, const ViolPin &b) { return a.diff > b.diff; });
      if (worst.size() > 10) worst.resize(10);

      printf("[%s] Pins checked: %zu  Violations: %zu  Total: %.4f ns\n",
             label, pin_count, violation_count, slew_total);
      if (!worst.empty()) {
        printf("  Top violations:\n");
        printf("  %-40s %10s %10s %10s\n", "Pin", "Slew(ps)", "Limit(ps)", "Diff(ns)");
        for (auto &v : worst)
          printf("  %-40.40s %10.3f %10.3f %10.4f\n",
                 v.name.c_str(), v.slew, v.limit, v.diff);
      }
      fflush(stdout);
    };

    // Check violations AFTER updateTiming (ground truth)
    sta->updateTiming(true);
    sta->findRequireds();
    computeSlewViolations("After updateTiming");
  }

  printf("\n========================================\n");
  printf(" End LocalSTA Accuracy Test\n");
  printf("========================================\n");
  fflush(stdout);

  delete arc_delay_calc;
  delete incre_sta;
}

// ============================================================
//  testSlewViolationFeasibility — Analyze each slew violation:
//  can it be fixed by resize (downsize loads + upsize driver)?
// ============================================================
void
TestLrf::testSlewViolationFeasibility(sta::dbSta* sta,
                                       rsz::Resizer *resizer,
                                       odb::dbBlock *block)
{
  printf("\n========================================\n");
  printf(" Slew Violation Resize Feasibility\n");
  printf("========================================\n\n");
  fflush(stdout);

  sta->updateTiming(true);
  sta->findRequireds();
  resizer->makeEquivCells();

  IncreSta *incre_sta = new IncreSta(sta, 1);
  LocalSta *local_sta = incre_sta->localSta();
  sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
  sta::dbNetwork *db_net = sta->getDbNetwork();
  sta::Scene *corner = sta->findScene("default");
  sta::DcalcAPIndex dcalc_ap = corner->dcalcAnalysisPtIndex(sta::MinMax::max());

  // Collect all violation driver pins (output pins with slew > limit)
  struct ViolDriver {
    sta::Pin *drvr_pin;
    sta::Vertex *drvr_vtx;
    float slew;
    float limit;
    std::string name;
  };
  std::vector<ViolDriver> viol_drivers;

  for (odb::dbITerm *iterm : block->getITerms()) {
    odb::dbNet *net = iterm->getNet();
    if (!net) continue;
    auto sig = net->getSigType();
    if (sig == odb::dbSigType::POWER || sig == odb::dbSigType::GROUND
        || sig == odb::dbSigType::CLOCK)
      continue;
    odb::dbMTerm *mterm = iterm->getMTerm();
    if (!mterm) continue;
    sta::LibertyPort *lib_port = sta->network()->libertyPort(
        db_net->dbToSta(mterm));
    if (!lib_port) continue;
    float limit = local_sta->getPortMaxSlewLimit(lib_port);
    if (limit <= 0 || limit >= 1.0) continue;

    sta::Pin *sta_pin = db_net->dbToSta(iterm);
    if (!sta_pin) continue;
    if (!sta->network()->isDriver(sta_pin)) continue;
    sta::Vertex *vtx = sta->graph()->pinDrvrVertex(sta_pin);
    if (!vtx) continue;

    float slew = 0.0;
    for (const sta::RiseFall *rf : sta::RiseFall::range()) {
      float s = delayAsFloat(sta->graph()->slew(vtx, rf, dcalc_ap));
      slew = std::max(slew, s);
    }
    if (slew > limit) {
      viol_drivers.push_back({sta_pin, vtx, slew, limit,
                              sta->network()->pathName(sta_pin)});
    }
  }

  // Sort by excess (worst first)
  std::sort(viol_drivers.begin(), viol_drivers.end(),
            [](const ViolDriver &a, const ViolDriver &b) {
              return (a.slew - a.limit) > (b.slew - b.limit);
            });

  printf("Found %zu driver pins with slew violation\n\n", viol_drivers.size());

  // Helper: estimate max output slew for a given port driving load_cap
  auto estimateMaxSlew = [&](sta::LibertyPort *port, float load_cap) -> float {
    if (!port) return 0;
    sta::LibertyCell *cell = port->libertyCell();
    float max_slew = 0;
    for (sta::TimingArcSet *arc_set : cell->timingArcSets()) {
      if (arc_set->role()->isTimingCheck()) continue;
      for (sta::TimingArc *arc : arc_set->arcs()) {
        if (arc->to() != port) continue;
        sta::GateTimingModel *model
            = dynamic_cast<sta::GateTimingModel*>(arc->model());
        if (!model) continue;
        float in_slew = 50e-12;  // 50ps typical input slew
        float arc_delay;
        float arc_slew;
        model->gateDelay(sta->cmdSdc()->operatingConditions(sta::MinMax::max()),
                         in_slew, load_cap, arc_delay, arc_slew);
        max_slew = std::max(max_slew, arc_slew);
      }
    }
    return max_slew;
  };

  for (auto &vd : viol_drivers) {
    printf("=== Driver: %s ===\n", vd.name.c_str());
    printf("  Slew: %.3f ps  Limit: %.3f ps  Excess: %.3f ps\n",
           vd.slew * 1e12, vd.limit * 1e12, (vd.slew - vd.limit) * 1e12);

    sta::Instance *drvr_inst = sta->network()->instance(vd.drvr_pin);
    sta::LibertyCell *drvr_cell = sta->network()->libertyCell(drvr_inst);
    sta::LibertyPort *drvr_port = sta->network()->libertyPort(vd.drvr_pin);
    float drvr_res = drvr_port ? drvr_port->driveResistance() : 0;
    printf("  Driver cell: %s  inst: %s  R_drvr: %.2f ohm\n",
           drvr_cell ? drvr_cell->name() : "?",
           sta->network()->pathName(drvr_inst), drvr_res);

    // Build PtGraph and compute LM cost for this instance
    {
      PtGraph *pg = local_sta->makePtGraph(drvr_inst, false);
      DelayLmSumResult cost = local_sta->increAndGetLocalTimingCost(
          pg, arc_delay_calc, drvr_cell);
      printf("  delay_lm_sum: %.6f\n", cost.delay_lm_sum);

      // Check vertex slack from global graph
      printf("  Vertex timing:\n");
      for (PtVertex &pv : pg->ptVertices()) {
        if (pv.type() == PtVertexType::Sentinel || !pv.vertex()) continue;
        sta::Vertex *sv = pv.vertex();
        sta::Path *wp = sta->vertexWorstSlackPath(sv, sta::MinMax::max());
        float slk = wp ? delayAsFloat(wp->slack(sta)) * 1e12 : 0;
        float arr = wp ? delayAsFloat(wp->arrival()) * 1e12 : 0;
        float req = wp ? delayAsFloat(wp->required()) * 1e12 : 0;
        if (pv.type() == PtVertexType::RefInput
         || pv.type() == PtVertexType::RefOutput
         || pv.type() == PtVertexType::RefDriver) {
          printf("    %-12s %-30.30s arr=%.1f req=%.1f slack=%.1f ps%s\n",
                 ptVertexTypeName(pv.type()),
                 sv->name(sta->network()),
                 arr, req, slk,
                 wp ? "" : " (no path)");
        }
      }
    }

    sta::Net *net = sta->network()->net(vd.drvr_pin);
    if (!net) { printf("  No net!\n\n"); continue; }

    // Enumerate fanout loads
    struct FanoutLoad {
      const sta::Pin *pin;
      sta::LibertyCell *cell;
      sta::LibertyPort *port;
      float cap;
      float min_cap;
      std::string pin_name;
      std::string cell_name;
      std::string min_cell_name;
    };
    std::vector<FanoutLoad> loads;
    float total_cap = 0.0, total_min_cap = 0.0;

    sta::NetConnectedPinIterator *pin_iter
        = sta->network()->connectedPinIterator(net);
    while (pin_iter->hasNext()) {
      const sta::Pin *pin = pin_iter->next();
      if (pin == vd.drvr_pin) continue;
      if (!sta->network()->isLoad(pin)) continue;
      sta::LibertyPort *load_port = sta->network()->libertyPort(pin);
      if (!load_port) continue;
      sta::Instance *load_inst = sta->network()->instance(pin);
      sta::LibertyCell *load_cell = sta->network()->libertyCell(load_inst);

      float cap = load_port->capacitance();
      float min_cap = cap;
      std::string min_cell_name = load_cell ? load_cell->name() : "?";

      if (load_cell) {
        sta::LibertyCellSeq *equivs = sta->equivCells(load_cell);
        if (equivs) {
          for (sta::LibertyCell *ec : *equivs) {
            sta::LibertyPort *ep = ec->findLibertyPort(load_port->name());
            if (ep && ep->capacitance() < min_cap) {
              min_cap = ep->capacitance();
              min_cell_name = ec->name();
            }
          }
        }
      }

      total_cap += cap;
      total_min_cap += min_cap;
      loads.push_back({pin, load_cell, load_port, cap, min_cap,
                       sta->network()->pathName(pin),
                       load_cell ? load_cell->name() : "?",
                       min_cell_name});
    }
    delete pin_iter;

    // Wire cap
    float total_load_cap = sta->graphDelayCalc()->loadCap(vd.drvr_pin, sta->cmdScene(), sta::MinMax::max());
    float wire_cap = total_load_cap - total_cap;
    if (wire_cap < 0) wire_cap = 0;

    printf("  Fanout: %zu loads\n", loads.size());
    printf("  Load cap: %.4f fF (pin: %.4f + wire: %.4f)\n",
           total_load_cap * 1e15, total_cap * 1e15, wire_cap * 1e15);
    printf("  Min cap (all downsize): %.4f fF (pin: %.4f + wire: %.4f)\n",
           (total_min_cap + wire_cap) * 1e15, total_min_cap * 1e15, wire_cap * 1e15);

    // Case 1: downsize all loads, keep current driver
    float min_total_load = total_min_cap + wire_cap;
    float est_min_slew = estimateMaxSlew(drvr_port, min_total_load);
    printf("  [Case 1] Downsize all loads: slew %.3f ps -> %s\n",
           est_min_slew * 1e12,
           est_min_slew <= vd.limit ? "FIXABLE" : "NOT FIXABLE");

    // Case 2: downsize all loads + upsize driver to strongest equiv
    float best_slew = est_min_slew;
    std::string best_drvr_name = drvr_cell ? drvr_cell->name() : "?";
    if (drvr_cell) {
      sta::LibertyCellSeq *drvr_equivs = sta->equivCells(drvr_cell);
      if (drvr_equivs) {
        for (sta::LibertyCell *ec : *drvr_equivs) {
          sta::LibertyPort *ep = ec->findLibertyPort(drvr_port->name());
          if (ep) {
            float s = estimateMaxSlew(ep, min_total_load);
            if (s < best_slew) {
              best_slew = s;
              best_drvr_name = ec->name();
            }
          }
        }
      }
    }
    printf("  [Case 2] + upsize driver to %s: slew %.3f ps -> %s\n",
           best_drvr_name.c_str(), best_slew * 1e12,
           best_slew <= vd.limit ? "FIXABLE" : "NOT FIXABLE");

    // Case 3: keep current loads, only upsize driver
    float best_drvr_only_slew = vd.slew;
    std::string best_drvr_only_name = drvr_cell ? drvr_cell->name() : "?";
    if (drvr_cell) {
      sta::LibertyCellSeq *drvr_equivs = sta->equivCells(drvr_cell);
      if (drvr_equivs) {
        for (sta::LibertyCell *ec : *drvr_equivs) {
          sta::LibertyPort *ep = ec->findLibertyPort(drvr_port->name());
          if (ep) {
            float s = estimateMaxSlew(ep, total_load_cap);
            if (s < best_drvr_only_slew) {
              best_drvr_only_slew = s;
              best_drvr_only_name = ec->name();
            }
          }
        }
      }
    }
    printf("  [Case 3] Only upsize driver to %s: slew %.3f ps -> %s\n",
           best_drvr_only_name.c_str(), best_drvr_only_slew * 1e12,
           best_drvr_only_slew <= vd.limit ? "FIXABLE" : "NOT FIXABLE");

    // Per-load details (sorted by cap, top 15)
    std::sort(loads.begin(), loads.end(),
              [](const FanoutLoad &a, const FanoutLoad &b) { return a.cap > b.cap; });
    printf("\n  %-40s %-25s %10s %10s %-25s\n",
           "Load Pin", "Cell", "Cap(fF)", "MinCap(fF)", "MinCell");
    size_t show = std::min(loads.size(), (size_t)15);
    for (size_t i = 0; i < show; i++) {
      auto &ld = loads[i];
      printf("  %-40.40s %-25.25s %10.4f %10.4f %-25.25s\n",
             ld.pin_name.c_str(), ld.cell_name.c_str(),
             ld.cap * 1e15, ld.min_cap * 1e15, ld.min_cell_name.c_str());
    }
    if (loads.size() > show)
      printf("  ... and %zu more loads\n", loads.size() - show);
    printf("\n");
    fflush(stdout);
  }

  printf("========================================\n");
  printf(" End Slew Violation Feasibility\n");
  printf("========================================\n");
  fflush(stdout);

  delete arc_delay_calc;
  delete incre_sta;
}

// ============================================================
//  testRepairSlew — Demo: repair slew violations by buffer insertion
// ============================================================
void
TestLrf::testRepairSlew(sta::dbSta* sta,
                        rsz::Resizer *resizer,
                        odb::dbBlock *block)
{
  printf("\n========================================\n");
  printf(" Repair Slew Violations by Buffer Insertion\n");
  printf("========================================\n");

  sta::dbNetwork *db_network = sta->getDbNetwork();

  // Step 1: Find all slew violations (same logic as MLCAD evaluation)
  sta->ensureGraph();
  sta->searchPreamble();
  sta->ensureClkArrivals();
  sta->findDelays();

  sta::LibertyLibrary *default_lib = db_network->defaultLibertyLibrary();
  float default_max_slew = sta::INF;
  if (default_lib) {
    bool exists = false;
    default_lib->defaultMaxSlew(default_max_slew, exists);
    if (!exists) default_max_slew = sta::INF;
  }
  printf("Default max slew limit: %.3f ps\n", default_max_slew * 1e12);

  // Collect violated driver pins
  struct ViolatedDriver {
    const sta::Pin *drvr_pin;
    float worst_slew;
    float limit;
  };
  std::vector<ViolatedDriver> violations;

  sta::Graph *graph = sta->graph();
  sta::VertexIterator viter(graph);
  while (viter.hasNext()) {
    sta::Vertex *vertex = viter.next();
    if (!vertex->isDriver(db_network))
      continue;
    const sta::Pin *pin = vertex->pin();
    if (db_network->isTopLevelPort(pin))
      continue;
    sta::LibertyPort *port = db_network->libertyPort(pin);
    if (!port)
      continue;

    // Get slew limit for this port
    float limit = 0.0f;
    bool exists = false;
    port->slewLimit(sta::MinMax::max(), limit, exists);
    if (!exists)
      limit = default_max_slew;

    // Check actual slew
    sta::DcalcAPIndex dcalc_ap
        = sta->cmdScene()->dcalcAnalysisPtIndex(sta::MinMax::max());
    float worst = 0.0f;
    for (auto rf : sta::RiseFall::range()) {
      float s = graph->slew(vertex, rf, dcalc_ap);
      worst = std::max(worst, s);
    }

    if (worst > limit) {
      violations.push_back({pin, worst, limit});
    }
  }

  printf("Found %zu slew violations before repair\n", violations.size());
  // Sort by severity (worst first)
  std::sort(violations.begin(), violations.end(),
            [](const ViolatedDriver &a, const ViolatedDriver &b) {
              return (a.worst_slew - a.limit) > (b.worst_slew - b.limit);
            });

  for (size_t i = 0; i < std::min(violations.size(), size_t(20)); i++) {
    auto &v = violations[i];
    printf("  [%zu] %s: slew=%.3fps limit=%.3fps excess=%.3fps\n",
           i, db_network->pathName(v.drvr_pin),
           v.worst_slew * 1e12, v.limit * 1e12,
           (v.worst_slew - v.limit) * 1e12);
  }

  if (violations.empty()) {
    printf("No slew violations to repair.\n");
    printf("========================================\n");
    return;
  }

  // Step 2: For each violated driver, upsize to smallest equiv cell
  // that fixes the slew violation.  Iterate until no more progress
  // (handles cascaded buffer chains).
  resizer->makeEquivCells();
  sta::DcalcAPIndex dcalc_ap
      = sta->cmdScene()->dcalcAnalysisPtIndex(sta::MinMax::max());
  int total_upsized = 0;

  // Helper: estimate max output slew for a port driving load_cap
  auto estimateMaxSlew = [&](sta::LibertyPort *port, float load_cap) -> float {
    if (!port) return sta::INF;
    sta::LibertyCell *cell = port->libertyCell();
    float max_slew = 0;
    for (sta::TimingArcSet *arc_set : cell->timingArcSets()) {
      if (arc_set->role()->isTimingCheck()) continue;
      for (sta::TimingArc *arc : arc_set->arcs()) {
        if (arc->to() != port) continue;
        sta::GateTimingModel *model
            = dynamic_cast<sta::GateTimingModel*>(arc->model());
        if (!model) continue;
        float in_slew = 50e-12;
        float arc_delay;
        float arc_slew;
        model->gateDelay(sta->cmdSdc()->operatingConditions(sta::MinMax::max()),
                         in_slew, load_cap, arc_delay, arc_slew);
        max_slew = std::max(max_slew, arc_slew);
      }
    }
    return max_slew;
  };

  // Multiple passes: upsizing one buffer may improve upstream slew,
  // enabling further fixes in the chain.
  for (int pass = 0; pass < 5; pass++) {
    int upsized_this_pass = 0;

    // Re-collect violations after each pass
    sta->ensureGraph();
    sta->findDelays();

    for (auto &v : violations) {
      sta::Vertex *vertex = graph->pinDrvrVertex(v.drvr_pin);
      if (!vertex) continue;

      // Re-check current slew
      float worst = 0.0f;
      for (auto rf : sta::RiseFall::range()) {
        float s = graph->slew(vertex, rf, dcalc_ap);
        worst = std::max(worst, s);
      }
      if (worst <= v.limit) continue;  // already fixed

      sta::Instance *inst = sta->network()->instance(v.drvr_pin);
      sta::LibertyCell *cur_cell = sta->network()->libertyCell(inst);
      sta::LibertyPort *drvr_port = sta->network()->libertyPort(v.drvr_pin);
      if (!cur_cell || !drvr_port) continue;

      // Get current load cap
      float load_cap = sta->graphDelayCalc()->loadCap(v.drvr_pin, sta->cmdScene(), sta::MinMax::max());

      // Find smallest equiv cell (by area) that fixes the violation
      sta::LibertyCellSeq *equivs = sta->equivCells(cur_cell);
      if (!equivs) continue;

      // Sort by area
      std::vector<sta::LibertyCell*> sorted_equivs(equivs->begin(), equivs->end());
      std::sort(sorted_equivs.begin(), sorted_equivs.end(),
                [](sta::LibertyCell *a, sta::LibertyCell *b) {
                  return a->area() < b->area();
                });

      sta::LibertyCell *best = nullptr;
      for (sta::LibertyCell *ec : sorted_equivs) {
        if (ec == cur_cell) continue;
        if (!sta::equivCellsArcs(cur_cell, ec)) continue;
        sta::LibertyPort *ep = ec->findLibertyPort(drvr_port->name());
        if (!ep) continue;
        float est_slew = estimateMaxSlew(ep, load_cap);
        if (est_slew <= v.limit) {
          best = ec;
          break;  // smallest area that fixes
        }
      }

      if (best) {
        printf("  [pass %d] %s: %s -> %s (load=%.2ffF, est_slew=%.1fps)\n",
               pass, db_network->pathName(v.drvr_pin),
               cur_cell->name(), best->name(),
               load_cap * 1e15,
               estimateMaxSlew(best->findLibertyPort(drvr_port->name()), load_cap) * 1e12);
        sta->replaceCell(inst, best);
        upsized_this_pass++;
        total_upsized++;
      }
    }

    printf("  Pass %d: upsized %d cells\n", pass, upsized_this_pass);
    fflush(stdout);
    if (upsized_this_pass == 0) break;

    // Update parasitics + timing for next pass
    sta->delaysInvalid();
  }

  // Step 3: Final check
  sta->findDelays();
  int remaining = 0;
  printf("\n--- Final violation check ---\n");
  for (auto &v : violations) {
    sta::Vertex *vertex = graph->pinDrvrVertex(v.drvr_pin);
    if (!vertex) continue;
    float worst = 0.0f;
    for (auto rf : sta::RiseFall::range()) {
      float s = graph->slew(vertex, rf, dcalc_ap);
      worst = std::max(worst, s);
    }
    if (worst > v.limit) {
      remaining++;
      printf("  Still violated: %s slew=%.3fps limit=%.3fps\n",
             db_network->pathName(v.drvr_pin),
             worst * 1e12, v.limit * 1e12);
    } else {
      printf("  Fixed: %s slew=%.3fps limit=%.3fps\n",
             db_network->pathName(v.drvr_pin),
             worst * 1e12, v.limit * 1e12);
    }
  }

  printf("\n========================================\n");
  printf(" Repair Summary\n");
  printf("  Violations before: %zu\n", violations.size());
  printf("  Cells upsized:     %d\n", total_upsized);
  printf("  Violations after:  %d\n", remaining);
  printf("========================================\n");
  fflush(stdout);
}

void
TestLrf::runInitialization(sta::dbSta* sta, IncreSta* incre_sta,
                           rsz::Resizer *resizer, odb::dbBlock *block,
                           size_t thread_num, bool minimize_leakage)
{
  resizer->makeEquivCells();
  ParallelInitializer initializer(sta, incre_sta, resizer, block,
                                  thread_num, minimize_leakage);
  initializer.run();
}

// ============================================================
//  testBufferingRsz — Sensitivity screening + rsz rebuffering
// ============================================================
void
TestLrf::testBufferingRsz(sta::dbSta* sta,
                           rsz::Resizer *resizer,
                           odb::dbBlock *block,
                           size_t thread_num,
                           float PT_tradeoff,
                           int top_n)
{
  printf("\n========================================\n");
  printf(" Sensitivity Screening + RSZ Rebuffering\n");
  printf("========================================\n");

  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, thread_num);
  LocalSta *local_sta = incre_sta->localSta();
  local_sta->setParasiticsEst(resizer->getEstimateParasitics());

  sta->updateTiming(true);
  sta->findRequireds();
  double wns_before = sta->worstSlack(sta::MinMax::max()) * 1e12;
  double tns_before = sta->totalNegativeSlack(sta::MinMax::max()) * 1e12;
  printf("Before: WNS=%.3f ps, TNS=%.3f ps\n", wns_before, tns_before);

  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

  incre_sta->parallelBufferingRsz(resizer, PT_tradeoff, top_n);

  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
  sta->delaysInvalid();
  sta->updateTiming(true);
  sta->findRequireds();

  double wns_after = sta->worstSlack(sta::MinMax::max()) * 1e12;
  double tns_after = sta->totalNegativeSlack(sta::MinMax::max()) * 1e12;
  printf("After:  WNS=%.3f ps, TNS=%.3f ps\n", wns_after, tns_after);
  printf("Delta:  WNS=%+.3f ps, TNS=%+.3f ps\n",
         wns_after - wns_before, tns_after - tns_before);

  delete incre_sta;
}

void
TestLrf::probeRszBnet(sta::dbSta* sta, rsz::Resizer *resizer,
                      odb::dbBlock *block, size_t thread_num)
{
  printf("----- Probe: RSZ bnet + LRF local eval -----\n");
  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, thread_num);
  incre_sta->probeRszBnet(resizer, 10.0f, 100);
  delete incre_sta;
}

void
TestLrf::probeBufferOneByOne(sta::dbSta* sta,
                              rsz::Resizer *resizer,
                              odb::dbBlock *block,
                              size_t thread_num,
                              bool use_rsz)
{
  printf("\n========================================================\n");
  printf(" True Probe: buffer one pin at a time, RSZ vs LRF\n");
  printf("========================================================\n");

  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, thread_num);
  LocalSta *local_sta = incre_sta->localSta();
  local_sta->setParasiticsEst(resizer->getEstimateParasitics());
  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

  // Baseline timing
  sta->updateTiming(true);
  sta->findRequireds();
  double base_wns = sta->worstSlack(sta::MinMax::max());
  double base_tns = sta->totalNegativeSlack(sta::MinMax::max());
  printf("Baseline: WNS=%.3f ps, TNS=%.3f ps\n",
         base_wns * 1e12, base_tns * 1e12);

  // Use sensitivity screening to find candidates (same as parallelBuffering)
  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  std::vector<size_t> selected = incre_sta->bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, 100);
  if (selected.empty()) {
    printf("No buffering candidates. Done.\n");
    delete incre_sta;
    return;
  }

  TaskArranger *task_arranger = local_sta->taskArranger();
  sta::Network *network = sta->network();
  sta::Graph *graph = sta->graph();

  // Setup LRF rebuffer context
  EvalContext lrf_ctx;
  lrf_ctx.arc_delay_calc = sta->arcDelayCalc();
  lrf_ctx.average_delay = avg_delay;
  lrf_ctx.average_leakage = avg_leakage;
  lrf_ctx.PT_tradeoff = 10.0f;
  std::map<std::string, double> rt;
  lrf_ctx.runtime_map = &rt;

  LrRebuffer::initGlobalPreamble(sta, resizer);
  LrRebuffer lrf_rebuffer(resizer, local_sta, &lrf_ctx);
  lrf_rebuffer.init();

  // Helper: update parasitics + timing
  auto updateTiming = [&]() {
    Tcl_Interp *interp = sta->tclInterp();
    Tcl_Eval(interp, "estimate_parasitics -placement");
    sta->delaysInvalid();
    sta->updateTiming(true);
    sta->findRequireds();
  };

  // Helper: worst slack across all sink pins on the net driven by drvr_pin
  auto worstSinkSlack = [&](sta::Pin *drvr_pin) -> double {
    sta::Net *net = network->net(drvr_pin);
    if (!net) return 0.0;
    double worst = 1e30;
    sta::NetPinIterator *npi = network->pinIterator(net);
    while (npi->hasNext()) {
      const sta::Pin *pin = npi->next();
      if (network->isLoad(pin)) {
        sta::Vertex *vtx = graph->pinLoadVertex(pin);
        if (vtx) {
          float s = sta->slack(vtx, sta::MinMax::max());
          if (s < worst) worst = s;
        }
      }
    }
    delete npi;
    return worst == 1e30 ? 0.0 : worst;
  };

  struct ProbeResult {
    std::string name;
    float slack;
    int rsz_bufs;
    double rsz_dwns;
    double rsz_dtns;
    double rsz_sink_slack;  // worst sink slack after RSZ buffer
    int lrf_bufs;
    double lrf_dwns;
    double lrf_dtns;
    double lrf_sink_slack;  // worst sink slack after LRF buffer
    double orig_sink_slack; // worst sink slack before any buffer
  };
  std::vector<ProbeResult> results;

  printf("\n%-35s %7s %8s | %4s %8s %9s %9s | %4s %8s %9s %9s | %s\n",
         "Pin", "Slack", "SinkSlk",
         "RSZ", "dWNS", "dTNS", "dSink",
         "LRF", "dWNS", "dTNS", "dSink", "Winner");

  for (size_t vid : selected) {
    InstVertex *iv = task_arranger->vertex(vid);
    sta::Instance *inst = iv->inst();

    // Find negative-slack driver pin
    sta::InstancePinIterator *pin_iter = network->pinIterator(inst);
    sta::Pin *drvr_pin = nullptr;
    float worst_slack = 0.0f;
    while (pin_iter->hasNext()) {
      sta::Pin *pin = pin_iter->next();
      if (!network->isDriver(pin)) continue;
      sta::Vertex *vtx = graph->pinDrvrVertex(pin);
      if (!vtx) continue;
      float s = sta->slack(vtx, sta::MinMax::max());
      if (s < worst_slack || !drvr_pin) {
        worst_slack = s;
        drvr_pin = pin;
      }
    }
    delete pin_iter;
    if (!drvr_pin || worst_slack >= 0.0f) continue;

    ProbeResult res;
    res.name = network->pathName(inst);
    res.slack = worst_slack;
    res.rsz_bufs = 0;
    res.rsz_dwns = 0; res.rsz_dtns = 0; res.rsz_sink_slack = 0;
    res.lrf_bufs = 0;
    res.lrf_dwns = 0; res.lrf_dtns = 0; res.lrf_sink_slack = 0;
    res.orig_sink_slack = worstSinkSlack(drvr_pin);

    // ── RSZ path ──
    odb::dbDatabase::beginEco(block);
    {
      int before = block->getInsts().size();
      resizer->rebufferNet(drvr_pin);
      res.rsz_bufs = block->getInsts().size() - before;
    }
    if (res.rsz_bufs > 0) {
      updateTiming();
      double wns = sta->worstSlack(sta::MinMax::max());
      double tns = sta->totalNegativeSlack(sta::MinMax::max());
      res.rsz_dwns = (wns - base_wns) * 1e12;
      res.rsz_dtns = (tns - base_tns) * 1e12;
      res.rsz_sink_slack = worstSinkSlack(drvr_pin);
    } else {
      res.rsz_sink_slack = res.orig_sink_slack;
    }
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    updateTiming();

    // ── LRF path ──
    // Build PtGraph for this instance
    PtGraph *pt_graph = local_sta->makePtGraph(inst, true);
    if (pt_graph) {
      lrf_ctx.pt_graph = pt_graph;
      // Find driver PtVertex
      PtVertex *drvr_pv = nullptr;
      for (size_t pi = 0; pi < pt_graph->vertexCount(); pi++) {
        PtVertex &pv = pt_graph->ptVertex(pi);
        if (pv.vertex() && pv.type() == PtVertexType::RefOutput
            && pv.vertex()->pin() == drvr_pin) {
          drvr_pv = &pv;
          break;
        }
      }

      if (drvr_pv) {
        odb::dbDatabase::beginEco(block);
        lrf_rebuffer.rebufferPin(drvr_pin, *drvr_pv);
        if (lrf_rebuffer.bestBnet()) {
          int before = block->getInsts().size();
          lrf_rebuffer.applyBufferingToDb();
          res.lrf_bufs = block->getInsts().size() - before;

          if (res.lrf_bufs > 0) {
            updateTiming();
            double wns = sta->worstSlack(sta::MinMax::max());
            double tns = sta->totalNegativeSlack(sta::MinMax::max());
            res.lrf_dwns = (wns - base_wns) * 1e12;
            res.lrf_dtns = (tns - base_tns) * 1e12;
            res.lrf_sink_slack = worstSinkSlack(drvr_pin);
          } else {
            res.lrf_sink_slack = res.orig_sink_slack;
          }
        } else {
          res.lrf_sink_slack = res.orig_sink_slack;
        }
        lrf_rebuffer.cleanupVirtualBuffer();
        odb::dbDatabase::endEco(block);
        odb::dbDatabase::undoEco(block);
        updateTiming();
      }
    }

    // Determine winner
    const char *winner = "tie";
    if (res.rsz_bufs == 0 && res.lrf_bufs == 0) winner = "no-buf";
    else if (res.rsz_bufs > 0 && res.lrf_bufs == 0) winner = "RSZ-only";
    else if (res.rsz_bufs == 0 && res.lrf_bufs > 0) winner = "LRF-only";
    else if (res.rsz_dwns > res.lrf_dwns + 0.1) winner = "RSZ";
    else if (res.lrf_dwns > res.rsz_dwns + 0.1) winner = "LRF";

    double rsz_dsink = (res.rsz_sink_slack - res.orig_sink_slack) * 1e12;
    double lrf_dsink = (res.lrf_sink_slack - res.orig_sink_slack) * 1e12;
    printf("%-35s %7.1f %8.1f | %4d %+8.1f %+9.1f %+9.1f | %4d %+8.1f %+9.1f %+9.1f | %s\n",
           res.name.c_str(), res.slack * 1e12, res.orig_sink_slack * 1e12,
           res.rsz_bufs, res.rsz_dwns, res.rsz_dtns, rsz_dsink,
           res.lrf_bufs, res.lrf_dwns, res.lrf_dtns, lrf_dsink,
           winner);
    fflush(stdout);
    results.push_back(res);
  }

  // Summary
  int rsz_wins = 0, lrf_wins = 0, ties = 0, nobuf = 0;
  int rsz_improved = 0, lrf_improved = 0;
  int rsz_degraded = 0, lrf_degraded = 0;
  double rsz_dwns_sum = 0, lrf_dwns_sum = 0;
  double rsz_dtns_sum = 0, lrf_dtns_sum = 0;
  for (auto &r : results) {
    if (r.rsz_bufs == 0 && r.lrf_bufs == 0) { nobuf++; continue; }
    if (r.rsz_dwns > 0.1) rsz_improved++;
    if (r.rsz_dwns < -0.1) rsz_degraded++;
    if (r.lrf_dwns > 0.1) lrf_improved++;
    if (r.lrf_dwns < -0.1) lrf_degraded++;
    rsz_dwns_sum += r.rsz_dwns;
    lrf_dwns_sum += r.lrf_dwns;
    rsz_dtns_sum += r.rsz_dtns;
    lrf_dtns_sum += r.lrf_dtns;
    if (r.rsz_dwns > r.lrf_dwns + 0.1) rsz_wins++;
    else if (r.lrf_dwns > r.rsz_dwns + 0.1) lrf_wins++;
    else ties++;
  }

  printf("\n========================================================\n");
  printf("  Summary: %zu pins probed\n", results.size());
  printf("========================================================\n");
  printf("  %-20s %10s %10s\n", "", "RSZ", "LRF");
  printf("  %-20s %10d %10d\n", "Improved WNS", rsz_improved, lrf_improved);
  printf("  %-20s %10d %10d\n", "Degraded WNS", rsz_degraded, lrf_degraded);
  printf("  %-20s %+10.1f %+10.1f\n", "Sum dWNS (ps)", rsz_dwns_sum, lrf_dwns_sum);
  printf("  %-20s %+10.1f %+10.1f\n", "Sum dTNS (ps)", rsz_dtns_sum, lrf_dtns_sum);
  printf("  Wins: RSZ=%d  LRF=%d  Tie=%d  No-buf=%d\n",
         rsz_wins, lrf_wins, ties, nobuf);
  printf("========================================================\n");

  delete incre_sta;
}

void
TestLrf::probeBufferDeep(sta::dbSta* sta,
                          rsz::Resizer *resizer,
                          odb::dbBlock *block,
                          size_t thread_num,
                          const std::vector<std::string> &pin_names)
{
  printf("\n================================================================\n");
  printf(" Deep Probe: RSZ vs LRF-worst vs LRF-sum on %zu specific pins\n",
         pin_names.size());
  printf("================================================================\n");

  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, thread_num);
  LocalSta *local_sta = incre_sta->localSta();
  local_sta->setParasiticsEst(resizer->getEstimateParasitics());
  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());

  sta->updateTiming(true);
  sta->findRequireds();
  double base_wns = sta->worstSlack(sta::MinMax::max());
  double base_tns = sta->totalNegativeSlack(sta::MinMax::max());
  printf("Baseline: WNS=%.3f ps, TNS=%.3f ps\n\n",
         base_wns * 1e12, base_tns * 1e12);

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();
  local_sta->initParallel();

  sta::Network *network = sta->network();
  sta::Graph *graph = sta->graph();

  // Helper: update parasitics + timing
  auto updateTiming = [&]() {
    Tcl_Interp *interp = sta->tclInterp();
    Tcl_Eval(interp, "estimate_parasitics -placement");
    sta->delaysInvalid();
    sta->updateTiming(true);
    sta->findRequireds();
  };

  // Helper: worst slack across all sink pins on the net driven by drvr_pin
  // Helper: collect all load (sink) pins on the net driven by drvr_pin.
  // Must be called BEFORE buffer insertion to capture original sinks.
  auto collectSinkPins = [&](sta::Pin *drvr_pin) -> std::vector<sta::Vertex*> {
    std::vector<sta::Vertex*> sinks;
    sta::Net *net = network->net(drvr_pin);
    if (!net) return sinks;
    sta::NetPinIterator *npi = network->pinIterator(net);
    while (npi->hasNext()) {
      const sta::Pin *pin = npi->next();
      if (network->isLoad(pin)) {
        sta::Vertex *vtx = graph->pinLoadVertex(pin);
        if (vtx) sinks.push_back(vtx);
      }
    }
    delete npi;
    return sinks;
  };

  // Helper: worst slack across a pre-captured set of sink vertices
  auto worstSinkSlack = [&](const std::vector<sta::Vertex*> &sinks) -> double {
    double worst = 1e30;
    for (sta::Vertex *vtx : sinks) {
      float s = sta->slack(vtx, sta::MinMax::max());
      if (s < worst) worst = s;
    }
    return worst == 1e30 ? 0.0 : worst;
  };

  // Helper: sum slack across a pre-captured set of sink vertices
  auto sumSinkSlack = [&](const std::vector<sta::Vertex*> &sinks) -> double {
    double sum = 0.0;
    for (sta::Vertex *vtx : sinks)
      sum += sta->slack(vtx, sta::MinMax::max());
    return sum;
  };

  // Resolve pin names to instances + driver pins
  struct Target {
    sta::Instance *inst;
    sta::Pin *drvr_pin;
    std::string name;
  };
  std::vector<Target> targets;
  for (auto &pname : pin_names) {
    sta::Instance *inst = network->findInstance(pname.c_str());
    if (!inst) {
      printf("WARNING: instance '%s' not found, skipping\n", pname.c_str());
      continue;
    }
    // Find worst-slack driver pin
    sta::InstancePinIterator *pi = network->pinIterator(inst);
    sta::Pin *best_pin = nullptr;
    float best_slack = 1e30;
    while (pi->hasNext()) {
      sta::Pin *pin = pi->next();
      if (network->isDriver(pin)) {
        sta::Vertex *vtx = graph->pinDrvrVertex(pin);
        if (vtx) {
          float s = sta->slack(vtx, sta::MinMax::max());
          if (s < best_slack) { best_slack = s; best_pin = pin; }
        }
      }
    }
    delete pi;
    if (best_pin)
      targets.push_back({inst, best_pin, pname});
    else
      printf("WARNING: no driver pin found for '%s'\n", pname.c_str());
  }

  printf("Resolved %zu / %zu targets\n\n", targets.size(), pin_names.size());

  // Setup LRF rebuffer
  LrRebuffer::initGlobalPreamble(sta, resizer);

  for (auto &t : targets) {
    // Capture original sink pins BEFORE any buffer insertion
    std::vector<sta::Vertex*> orig_sinks = collectSinkPins(t.drvr_pin);
    double orig_worst_sink = worstSinkSlack(orig_sinks);
    double orig_sum_sink = sumSinkSlack(orig_sinks);
    float drvr_slack = sta->slack(graph->pinDrvrVertex(t.drvr_pin),
                                         sta::MinMax::max());

    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("PIN: %s  drvr_slack=%.1f ps\n", t.name.c_str(), drvr_slack * 1e12);
    printf("  Baseline: worst_sink=%.1f ps  sum_sink=%.1f ps  WNS=%.1f ps  TNS=%.1f ps\n",
           orig_worst_sink * 1e12, orig_sum_sink * 1e12,
           base_wns * 1e12, base_tns * 1e12);

    // ── Local slack analysis (via virtual buffer, no DB modification) ──
    PtGraph *pt_graph = local_sta->makePtGraph(t.inst, true);
    if (!pt_graph) {
      printf("  PtGraph failed, skipping\n");
      continue;
    }

    PtVertex *drvr_pv = nullptr;
    for (size_t pi = 0; pi < pt_graph->vertexCount(); pi++) {
      PtVertex &pv = pt_graph->ptVertex(pi);
      if (pv.vertex() && pv.type() == PtVertexType::RefOutput
          && pv.vertex()->pin() == t.drvr_pin) {
        drvr_pv = &pv;
        break;
      }
    }
    if (!drvr_pv) {
      printf("  driver PtVertex not found, skipping\n");
      continue;
    }

    // ── All 3 methods: local + global via TestRebuffer ──
    TestRebuffer::GlobalBaseline bl{base_wns, base_tns, orig_worst_sink, orig_sum_sink, orig_sinks};

    EvalContext ctx;
    ctx.arc_delay_calc = sta->arcDelayCalc();
    ctx.average_delay = avg_delay;
    ctx.average_leakage = avg_leakage;
    ctx.PT_tradeoff = 10.0f;
    std::map<std::string, double> rt;
    ctx.runtime_map = &rt;

    TestRebuffer rebuffer(resizer, local_sta, &ctx);
    TestRebuffer::initGlobalPreamble(sta, resizer);
    rebuffer.init();

    rebuffer.rebufferPinVG(t.drvr_pin, t.inst, block, 0, bl);  // RSZ
    rebuffer.rebufferPinVG(t.drvr_pin, t.inst, block, 1, bl);  // LRF-worst
    rebuffer.rebufferPinVG(t.drvr_pin, t.inst, block, 2, bl);  // LRF-sum

    printf("\n");
    fflush(stdout);
  }

  delete incre_sta;
}

void
TestLrf::probeAllOptions(sta::dbSta* sta, rsz::Resizer *resizer,
                          odb::dbBlock *block, size_t thread_num,
                          const char *pin_name)
{
  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, thread_num);
  LocalSta *local_sta = incre_sta->localSta();
  local_sta->setParasiticsEst(resizer->getEstimateParasitics());
  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
  local_sta->initParallel();

  sta->updateTiming(true);
  sta->findRequireds();

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();

  EvalContext ctx;
  ctx.arc_delay_calc = sta->arcDelayCalc();
  ctx.average_delay = avg_delay;
  ctx.average_leakage = avg_leakage;
  ctx.PT_tradeoff = 10.0f;
  std::map<std::string, double> rt;
  ctx.runtime_map = &rt;

  TestRebuffer probe(resizer, local_sta, &ctx);
  LrRebuffer::initGlobalPreamble(sta, resizer);
  probe.init();

  // Resolve instance + driver pin
  sta::Network *network = sta->network();
  sta::Graph *graph = sta->graph();
  sta::Instance *inst = network->findInstance(pin_name);
  if (!inst) {
    printf("ERROR: instance '%s' not found\n", pin_name);
    delete incre_sta;
    return;
  }

  sta::Pin *drvr_pin = nullptr;
  float best_slack = 1e30;
  sta::InstancePinIterator *pi = network->pinIterator(inst);
  while (pi->hasNext()) {
    sta::Pin *pin = pi->next();
    if (network->isDriver(pin)) {
      sta::Vertex *vtx = graph->pinDrvrVertex(pin);
      if (vtx) {
        float s = sta->slack(vtx, sta::MinMax::max());
        if (s < best_slack) { best_slack = s; drvr_pin = pin; }
      }
    }
  }
  delete pi;
  if (!drvr_pin) {
    printf("ERROR: no driver pin for '%s'\n", pin_name);
    delete incre_sta;
    return;
  }

  // Global baseline
  TestRebuffer::GlobalBaseline baseline;
  baseline.wns = sta->worstSlack(sta::MinMax::max());
  baseline.tns = sta->totalNegativeSlack(sta::MinMax::max());
  baseline.worst_sink = 1e30;
  baseline.sum_sink = 0;
  sta::Net *net = network->net(drvr_pin);
  sta::NetPinIterator *npi = network->pinIterator(net);
  while (npi->hasNext()) {
    const sta::Pin *p = npi->next();
    if (network->isLoad(p)) {
      sta::Vertex *v = graph->pinLoadVertex(p);
      if (v) {
        float s = sta->slack(v, sta::MinMax::max());
        if (s < baseline.worst_sink) baseline.worst_sink = s;
        baseline.sum_sink += s;
      }
    }
  }
  delete npi;

  printf("Baseline: WNS=%.1f TNS=%.1f worst_sink=%.1f sum_sink=%.1f\n",
         baseline.wns * 1e12, baseline.tns * 1e12,
         baseline.worst_sink * 1e12, baseline.sum_sink * 1e12);

  probe.probeAllOptions(drvr_pin, inst, block, baseline);

  delete incre_sta;
}

void
TestLrf::probeAllOptionsBySensitivity(sta::dbSta* sta, rsz::Resizer *resizer,
                                        odb::dbBlock *block, size_t thread_num,
                                        int top_n)
{
  printf("\n========================================================\n");
  printf(" probeAllOptionsBySensitivity: top-%d pins by sensitivity\n", top_n);
  printf("========================================================\n");

  sta->findRequireds();
  IncreSta *incre_sta = new IncreSta(sta, thread_num);
  LocalSta *local_sta = incre_sta->localSta();
  local_sta->setParasiticsEst(resizer->getEstimateParasitics());
  local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
  local_sta->initParallel();

  sta->updateTiming(true);
  sta->findRequireds();

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();

  // Pick top-N sensitivity candidates (same ranking as parallelBuffering)
  std::vector<size_t> selected = incre_sta->bufferingVerticesCandidateBySensitivity(
      resizer, avg_delay, avg_leakage, top_n);
  if (selected.empty()) {
    printf("No buffering candidates. Done.\n");
    delete incre_sta;
    return;
  }

  EvalContext ctx;
  ctx.arc_delay_calc = sta->arcDelayCalc();
  ctx.average_delay = avg_delay;
  ctx.average_leakage = avg_leakage;
  ctx.PT_tradeoff = 10.0f;
  std::map<std::string, double> rt;
  ctx.runtime_map = &rt;

  // Match real-flow slack_margin = 1 + |wns|/clock_period (same formula as
  // ParallelVisitor::init in NetlistTransformation.cc). Without this, probe
  // uses strict gate (margin=1.0) which over-rejects LRF bufs>=1 options.
  float wns = sta->worstSlack(sta::MinMax::max());
  float clock_period = 0.0f;
  for (auto *clock : sta->cmdSdc()->clocks()) {
    if (clock->period() > clock_period) { clock_period = clock->period(); break; }
  }
  float slack_margin = (wns >= 0.0f)
      ? 1.05f
      : std::max(-std::min(wns, 0.0f) / clock_period + 1.0f, 1.05f);
  ctx.slack_margin = slack_margin;
  printf("[probe] slack_margin=%.4f (wns=%.1fps, clk_period=%.1fps)\n",
         slack_margin, wns * 1e12, clock_period * 1e12);

  TestRebuffer probe(resizer, local_sta, &ctx);
  LrRebuffer::initGlobalPreamble(sta, resizer);
  probe.init();

  TaskArranger *task_arranger = local_sta->taskArranger();
  sta::Network *network = sta->network();
  sta::Graph *graph = sta->graph();

  int probed = 0;
  for (size_t vid : selected) {
    InstVertex *iv = task_arranger->vertex(vid);
    sta::Instance *inst = iv->inst();

    sta::Pin *drvr_pin = nullptr;
    float best_slack = 1e30;
    sta::InstancePinIterator *pi = network->pinIterator(inst);
    while (pi->hasNext()) {
      sta::Pin *pin = pi->next();
      if (network->isDriver(pin)) {
        sta::Vertex *vtx = graph->pinDrvrVertex(pin);
        if (vtx) {
          float s = sta->slack(vtx, sta::MinMax::max());
          if (s < best_slack) { best_slack = s; drvr_pin = pin; }
        }
      }
    }
    delete pi;
    if (!drvr_pin) continue;

    // Per-pin baseline
    TestRebuffer::GlobalBaseline baseline;
    baseline.wns = sta->worstSlack(sta::MinMax::max());
    baseline.tns = sta->totalNegativeSlack(sta::MinMax::max());
    baseline.worst_sink = 1e30;
    baseline.sum_sink = 0;
    sta::Net *net = network->net(drvr_pin);
    sta::NetPinIterator *npi = network->pinIterator(net);
    while (npi->hasNext()) {
      const sta::Pin *p = npi->next();
      if (network->isLoad(p)) {
        sta::Vertex *v = graph->pinLoadVertex(p);
        if (v) {
          float s = sta->slack(v, sta::MinMax::max());
          if (s < baseline.worst_sink) baseline.worst_sink = s;
          baseline.sum_sink += s;
        }
      }
    }
    delete npi;

    printf("\n[%d/%zu] Baseline: WNS=%.1f TNS=%.1f worst_sink=%.1f sum_sink=%.1f\n",
           probed + 1, selected.size(),
           baseline.wns * 1e12, baseline.tns * 1e12,
           baseline.worst_sink * 1e12, baseline.sum_sink * 1e12);

    probe.probeAllOptions(drvr_pin, inst, block, baseline);
    probed++;
  }

  printf("\n[probeAllOptionsBySensitivity] done: probed %d/%zu pins\n",
         probed, selected.size());

  delete incre_sta;
}

void
TestLrf::runInitializationStandalone(sta::dbSta* sta,
                                      rsz::Resizer *resizer,
                                      odb::dbBlock *block,
                                      int thread_count,
                                      bool minimize_leakage)
{
  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_count);
  resizer->makeEquivCells();
  ParallelInitializer initializer(sta, incre_sta, resizer, block,
                                  thread_count, minimize_leakage);
  initializer.run();
  // Resync global parasitics/timing so subsequent runLr / test* entries
  // observe the cell swaps performed by the initializer.
  incre_sta->localSta()->updateGlobalParasiticsAndSync(
      resizer->getEstimateParasitics());
  sta->delaysInvalid();
  sta->updateTiming(true);
  delete incre_sta;
}

// ============================================================
//  debugPrecheckAccuracy — Verify precheck predictions one gate at a time.
//
//  After LR convergence, run lmUpdate + precedingResizeCheck to get predicted
//  benefits, then for each selected instance:
//    1. Record global WNS/TNS + PtGraph timing/LM before swap
//    2. Apply single gate sizing (1-hop equiv cell change)
//    3. Update timing
//    4. Record global WNS/TNS + PtGraph timing/LM after swap
//    5. Print detailed comparison
//    6. Undo the swap (restore original state for next instance)
// ============================================================
void
TestLrf::debugPrecheckAccuracy(sta::dbSta* sta,
                               rsz::Resizer *resizer,
                               odb::dbBlock *block,
                               size_t thread_num,
                               float PT_tradeoff,
                               float top_ratio,
                               std::string lr_helper_method)
{
  printf("\n========================================\n");
  printf(" Debug Precheck Accuracy\n");
  printf("========================================\n");
  fflush(stdout);

  sta->findRequireds();
  lrf::IncreSta *incre_sta = new IncreSta(sta, thread_num);
  lrf::LocalSta *local_sta = incre_sta->localSta();

  incre_sta->makeLRHelper(lr_helper_method);
  lrf::LRHelper *lr_helper = incre_sta->lrHelper();

  resizer->makeEquivCells();
  incre_sta->makeSwappableCellsCache(resizer);
  incre_sta->preSaveLibCellLeakage();
  incre_sta->makeEquivCellArray();
  local_sta->initParallel();

  // LM update with current timing state
  incre_sta->lmUpdate();
  sta->findRequireds();

  float avg_delay = incre_sta->averageDelayOnCritPath();
  float avg_leakage = incre_sta->averageLeakage();

  // Run precheck to get predicted benefits
  printf("\n--- Running precedingResizeCheck ---\n");
  fflush(stdout);
  std::vector<size_t> selected = incre_sta->precedingResizeCheck(
      resizer, avg_delay, avg_leakage, PT_tradeoff, top_ratio);

  if (lrfVerbose()) {
    printf("\n--- Selected %zu instances for verification ---\n", selected.size());
  }
  fflush(stdout);

  if (selected.empty()) {
    printf("No instances selected by precheck. Nothing to debug.\n");
    delete incre_sta;
    return;
  }

  sta::dbNetwork *db_network = sta->getDbNetwork();
  sta::Graph *graph = sta->graph();
  sta::DcalcAPIndex dcalc_ap =
      sta->cmdScene()->dcalcAnalysisPtIndex(sta::MinMax::max());

  // Build reverse map: vertex_idx → Instance*
  TaskArranger *task_arranger = local_sta->taskArranger();
  const auto *inst_to_vid = task_arranger->instToVidMap();
  std::unordered_map<size_t, sta::Instance*> vid_to_inst;
  for (auto &[inst, vid] : *inst_to_vid)
    vid_to_inst[vid] = const_cast<sta::Instance*>(inst);

  sta::Scene *cmd_corner = sta->cmdScene();

  // Helper: dump PtGraph edge-centric info for a given instance
  auto dumpPtGraph = [&](sta::Instance *inst, const char *label) {
    PtGraph *pt_graph = local_sta->makePtGraph(inst, true);
    if (!pt_graph) {
      printf("    [%s] PtGraph: could not construct\n", label);
      return;
    }
    sta::ArcDelayCalc *arc_delay_calc = sta->arcDelayCalc()->copy();
    local_sta->findLocalDelays(pt_graph, arc_delay_calc);

    // Compute delay_lm_sum
    float delay_lm_sum = 0.0f;
    pt_graph->delayLmSum(dcalc_ap, delay_lm_sum);

    // Leakage of the ref instance
    float leakage = 0.0f;
    sta::PowerResult pwr = sta->power(inst, cmd_corner);
    leakage = pwr.leakage();

    printf("    [%s] ref=%s  delay_lm_sum=%.6f (×1e12=%.6f)  leakage=%.6e W\n",
           label,
           pt_graph->refGate() ? pt_graph->refGate()->name() : "?",
           delay_lm_sum, delay_lm_sum * 1e12, leakage);

    // Print edges with: global edge name, arc delays, LMs
    for (auto &pt_edge : pt_graph->ptEdges()) {
      if (pt_edge.arcDelayCount() == 0) continue;

      // Global edge name
      const sta::Edge *sta_edge = pt_edge.edge();
      std::string from_name = "(?)";
      std::string to_name = "(?)";
      if (sta_edge) {
        sta::Vertex *from_v = sta_edge->from(graph);
        sta::Vertex *to_v = sta_edge->to(graph);
        if (from_v) from_name = db_network->pathName(from_v->pin());
        if (to_v) to_name = db_network->pathName(to_v->pin());
      }

      bool is_wire = pt_edge.isWire();
      const sta::ArcDelay *delays = pt_edge.arcDelays();
      const sta::LMValue *lms = pt_edge.arcLms();
      size_t arc_count = pt_edge.arcDelayCount();

      if (arc_count == 0) continue;

      // Aggregate: max delay, max LM, sum(delay*lm) across all arcs
      float max_delay = 0, max_lm = 0, dlm_sum = 0;
      for (size_t a = 0; a < arc_count; a++) {
        float d = sta::delayAsFloat(delays[a]);
        float l = lms ? lms[a] : 0.0f;
        max_delay = std::max(max_delay, d);
        max_lm = std::max(max_lm, l);
        dlm_sum += d * l;
      }

      printf("      %s %s → %s  delay=%.1fps lm=%.4f d*lm=%.6f",
             is_wire ? "WIRE" : "GATE",
             from_name.c_str(), to_name.c_str(),
             max_delay * 1e12, max_lm, dlm_sum * 1e12);

      // Print per-arc detail if gate edge with multiple arcs
      if (!is_wire && arc_count > 1) {
        printf("  [%zu arcs:", arc_count);
        for (size_t a = 0; a < std::min(arc_count, size_t(4)); a++) {
          printf(" d=%.1f/lm=%.4f", sta::delayAsFloat(delays[a]) * 1e12,
                 lms ? lms[a] : 0.0f);
        }
        if (arc_count > 4) printf(" ...");
        printf("]");
      }
      printf("\n");
    }

    // Print head vertex (RefOutput) slack
    for (size_t vid : pt_graph->sortedVertexIds()) {
      PtVertex &pv = pt_graph->ptVertex(vid);
      if (pv.type() != PtVertexType::RefOutput) continue;
      if (!pv.vertex()) continue;
      float slack = sta::delayAsFloat(
          sta->slack(pv.vertex(), sta::MinMax::max()));
      std::string pin_name = db_network->pathName(pv.pin());
      printf("      HEAD_SLACK: %s  slack=%.1fps\n", pin_name.c_str(), slack * 1e12);
    }

    delete arc_delay_calc;
  };

  // Test each selected instance (cap at 20)
  size_t test_count = std::min(selected.size(), size_t(20));
  int match = 0, mismatch = 0;

  for (size_t idx = 0; idx < test_count; idx++) {
    size_t vertex_idx = selected[idx];
    auto it = vid_to_inst.find(vertex_idx);
    if (it == vid_to_inst.end()) continue;
    sta::Instance *inst = it->second;
    if (!inst) continue;

    sta::LibertyCell *orig_cell = db_network->libertyCell(inst);
    if (!orig_cell) continue;

    printf("\n--- [%zu/%zu] %s (%s) ---\n",
           idx+1, test_count,
           db_network->pathName(inst), orig_cell->name());

    // Record before state
    sta::Slack wns_before = sta->worstSlack(sta::MinMax::max());
    sta::Slack tns_before = sta->totalNegativeSlack(sta::MinMax::max());

    printf("  BEFORE: WNS=%.3fps TNS=%.3fps\n",
           wns_before * 1e12, tns_before * 1e12);
    dumpPtGraph(inst, "BEFORE");

    // Apply single gate sizing via ECO
    odb::dbDatabase::beginEco(block);

    // Create a visitor and do single gate sizing
    auto *visitor = new ParallelVisitor(sta, local_sta, resizer);
    auto resize_op = std::make_unique<ResizeOperator>(sta, local_sta);
    resize_op->setEquivCellArray(incre_sta->equivCellArray(),
                                  incre_sta->equivCellPosMap());
    visitor->setOperator(std::move(resize_op));
    visitor->init(avg_delay, avg_leakage,
                  sta::delayAsFloat(wns_before), PT_tradeoff,
                  nullptr);

    bool sized = visitor->singleGateSizing(inst);

    // Get new cell
    sta::LibertyCell *new_cell = db_network->libertyCell(inst);

    if (sized && new_cell != orig_cell) {
      // Update timing
      local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
      sta->delaysInvalid();
      sta->updateTiming(true);
      sta->findRequireds();

      sta::Slack wns_after = sta->worstSlack(sta::MinMax::max());
      sta::Slack tns_after = sta->totalNegativeSlack(sta::MinMax::max());

      printf("  AFTER:  WNS=%.3fps TNS=%.3fps  cell=%s→%s\n",
             wns_after * 1e12, tns_after * 1e12,
             orig_cell->name(), new_cell->name());
      dumpPtGraph(inst, "AFTER");

      float wns_delta = (wns_after - wns_before) * 1e12;
      float tns_delta = (tns_after - tns_before) * 1e12;
      bool improved = (wns_after > wns_before) ||
                      (wns_after == wns_before && tns_after > tns_before);

      printf("  VERDICT: WNS_delta=%+.3fps TNS_delta=%+.3fps → %s\n",
             wns_delta, tns_delta,
             improved ? "IMPROVED" : "REGRESSED");

      if (improved) match++;
      else mismatch++;
    } else {
      printf("  SKIPPED: singleGateSizing returned no change\n");
    }

    // Undo the swap
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    local_sta->updateGlobalParasiticsAndSync(resizer->getEstimateParasitics());
    sta->delaysInvalid();
    sta->updateTiming(true);
    sta->findRequireds();

    delete visitor;
    fflush(stdout);
  }

  printf("\n========================================\n");
  printf(" Summary: %d improved, %d regressed out of %zu tested\n",
         match, mismatch, test_count);
  printf("========================================\n");
  fflush(stdout);

  delete incre_sta;
}

}  // namespace lrf

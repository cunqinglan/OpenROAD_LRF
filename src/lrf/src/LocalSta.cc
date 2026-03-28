#include <mutex>
#include <cstring>
#include <string>

#include "sta/Sta.hh"
#include "sta/Corner.hh"
#include "sta/MinMax.hh"
#include "sta/Graph.hh"
#include "sta/TimingArc.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "EquivCells.hh"
#include "sta/Delay.hh"
#include "sta/TimingRole.hh"
#include "sta/ClkNetwork.hh"
#include "LocalParasitics.hh"
#include "sta/Corner.hh"
#include "sta/Sdc.hh"
#include "sta/InputDrive.hh"
#include "sta/Parasitics.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "LocalSearch.hh"
#include "TaskArranger.hh"
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "TaskArranger.hh"
#include "sta/PortDirection.hh"
#include "search/Tag.hh"
#include "search/TagGroup.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/FuncExpr.hh"
#include "sta/LeakagePower.hh"
#include "sta/Liberty.hh"
#include "sta/DelayFloat.hh"
  
#include <stdexcept>



namespace lrf {
using namespace sta;
static const Slew default_slew = 0.0;

// Global mutex to protect OpenDB/STA network object access
// Declared extern here, defined in LocalParasitics.cc
extern std::mutex g_odb_sta_access_mutex;

LocalSta::LocalSta(sta::dbSta *sta) :
  GraphDelayCalc(sta),
  sta_(sta),
  collected_(false),
  sorted_(false),
  estimate_parasitics_(nullptr),
  local_parasitics_(new LocalParasitics(sta)),
  task_arranger_(new TaskArranger(sta)),
  pred_(new SearchMEEPred(sta)),
  search_pred_(new SearchPredNonLatch2(sta))
{
  printf("LocalSta::LocalSta created\n");
  fflush(stdout);
  parasitics_set_ = false;
}

LocalSta::~LocalSta()
{
  // Clean up locally created PtGraph instances to avoid memory leaks.
  for (PtGraph *g : local_graphs_) {
    delete g;
  }
  local_graphs_.clear();
  delete local_parasitics_;
  delete task_arranger_;
  delete pred_;
  delete search_pred_;
}

void 
LocalSta::copyState(const Sta *sta)
{
  GraphDelayCalc::copyState(sta);
  local_parasitics_->copyState(sta);
  task_arranger_->copyState(sta);
  sorted_ = false;
}

void
LocalSta::setParasiticsEst(est::EstimateParasitics *estimate_parasitics) {
  estimate_parasitics_ = estimate_parasitics;
  parasitics_set_ = true;
}

void
LocalSta::updateGlobalParasiticsAndSync(est::EstimateParasitics *est_parasitics)
{
  est_parasitics->updateWireParasiticsNoDeleteNetwork();
  local_parasitics_->initParasiticMapFromBase();
}

void
LocalSta::collectLocalGraph(Instance *inst, InstanceSet &local_instances)
{
  if (network_->libertyCell(inst)->hasSequentials()) {
    // For sequential cells, skip
    throw std::runtime_error("LocalSta::collectLocalGraph: Sequential cells not supported");
  }
  local_instances.insert(inst);
  InstancePinIterator *pin_iter = network_->pinIterator(inst);
  PinSet visited_pins(network_);
  while (pin_iter->hasNext()) {
    Pin *pin = pin_iter->next();
    if (network_->isDriver(pin)) {
      collectLocalFanouts(pin, local_instances);
    } else if (network_->isLoad(pin)) {
      collectLocalFaninSiblings(pin, visited_pins, local_instances);
    }
  }
  delete pin_iter;
  collected_ = true;
}

void
LocalSta::collectLocalVertices(Instance *inst, VertexSet &local_vertices)
{
  if (network_->libertyCell(inst)->hasSequentials()) {
    // For sequential cells, skip
    throw std::runtime_error("LocalSta::collectLocalVertices: Sequential cells not supported");
  }
  InstancePinIterator *pin_iter = network_->pinIterator(inst);
  while (pin_iter->hasNext()) {
    Pin *pin = pin_iter->next();
    if (network_->isDriver(pin)) {
      sta::Vertex *drvr_vertex = graph_->pinDrvrVertex(pin);
      if (search_pred_->searchTo(drvr_vertex)) {
        local_vertices.insert(drvr_vertex);
        collectLocalFanoutVertices(drvr_vertex, local_vertices);
      }
    }
    if (network_->isLoad(pin)) {
      sta::Vertex *load_vertex = graph_->pinLoadVertex(pin);
      if (load_vertex == nullptr)
        continue;
      // Always include ref instance's own load pins for complete
      // RefInput annotation; fanin expansion is still gated by searchFrom.
      local_vertices.insert(load_vertex);
      if (search_pred_->searchFrom(load_vertex)) {
        collectLocalFaninSiblingVertices(load_vertex, local_vertices);
      }
    }
  }
  delete pin_iter;
}

void
LocalSta::collectLocalFanoutVertices(sta::Vertex *drvr_vertex, 
                                     VertexSet &local_vertices)
{
  if (!graph_ || drvr_vertex == nullptr 
          || !network_->isDriver(drvr_vertex->pin())) {
    throw std::runtime_error("LocalSta::collectLocalFanoutVertices invalid input");
  }

  VertexOutEdgeIterator edge_iter(drvr_vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge *out_edge = edge_iter.next();
    if (!out_edge->isWire() || !search_pred_->searchThru(out_edge))
      continue;
    Vertex *load_vertex = out_edge->to(graph_);
    if (!network_->isLoad(load_vertex->pin()))
      continue;
    local_vertices.insert(load_vertex);

    VertexOutEdgeIterator in_inst_edge_iter(load_vertex, graph_);
    while (in_inst_edge_iter.hasNext()) {
      Edge *in_inst_edge = in_inst_edge_iter.next();
      Vertex *out_driver_vertex = in_inst_edge->to(graph_);
      if (!search_pred_->searchThru(in_inst_edge) || 
                      !search_pred_->searchFrom(load_vertex)) {
        continue;
      }
      if (!network_->isDriver(out_driver_vertex->pin())) {
        printf("Warining: LocalSta::collectLocalFanoutVertices: from driver %s vertex %s of edge %s is not a driver\n",
                drvr_vertex->to_string(graph_).c_str(),
               out_driver_vertex->to_string(graph_).c_str(),
               in_inst_edge->to_string(graph_).c_str());
        fflush(stdout);
        continue;
      }
      // We avoid collecting latches in the local graph.
      if (search_pred_->searchThru(in_inst_edge) && 
                      search_pred_->searchTo(out_driver_vertex))
        local_vertices.insert(out_driver_vertex);
    }
  }
}

void 
LocalSta::collectLocalFaninSiblingVertices(Vertex *load_vertex, 
                                           VertexSet &local_vertices)
{
  if (!graph_ || load_vertex == nullptr 
          || !network_->isLoad(load_vertex->pin())) {
    throw std::runtime_error("LocalSta::collectLocalFaninSiblingVertices invalid input");
  }

  Pin *load_pin = load_vertex->pin();
  PinSet visited_pins(network_);
  PinSeq loads, drvrs;
  FindNetDrvrLoads visitor(load_pin, visited_pins, loads, drvrs, network_);
  network_->visitConnectedPins(load_pin, visitor);

  for (auto drvr_pin : drvrs) {
    Vertex *drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
    if (drvr_vertex == nullptr)
      continue;
    if (!search_pred_->searchTo(drvr_vertex)) {
      // Still collect the driver so RefInput won't become a root without RefDriver
      local_vertices.insert(drvr_vertex);
      continue;
    }
    local_vertices.insert(drvr_vertex);
    VertexInEdgeIterator in_edge_iter(drvr_vertex, graph_);
    while (in_edge_iter.hasNext()) {
      Edge *in_edge = in_edge_iter.next();
      Vertex *pred_vertex = in_edge->from(graph_);
      if (search_pred_->searchThru(in_edge) &&
          search_pred_->searchFrom(pred_vertex))
        local_vertices.insert(pred_vertex);
    }
  }

  for (auto load_pin : loads) {
    if (load_pin == load_vertex->pin())
      continue;
    Vertex *sibling_load_vertex = graph_->pinLoadVertex(load_pin);
    if (sibling_load_vertex
        && search_pred_->searchFrom(sibling_load_vertex)) {
      local_vertices.insert(sibling_load_vertex);
      // Collect sibling driver vertices, skip check edges and latch edges
      VertexOutEdgeIterator in_inst_edge_iter(sibling_load_vertex, graph_);
      while (in_inst_edge_iter.hasNext()) {
        Edge *sibling_inst_edge = in_inst_edge_iter.next();
        Vertex *sibling_drvr_vertex = sibling_inst_edge->to(graph_);
        if (search_pred_->searchThru(sibling_inst_edge) && 
                        search_pred_->searchTo(sibling_drvr_vertex)) {
          if (!network_->isDriver(sibling_drvr_vertex->pin())) {
            printf("Warining: LocalSta::collectLocalFaninSiblingVertices: vertex %s is not a driver\n",
                  sibling_drvr_vertex->to_string(graph_).c_str());
            continue;
          }
          
          local_vertices.insert(sibling_drvr_vertex);
        }
      }
    }
  }
}

void 
LocalSta::collectLocalFanouts(Pin *drvr_pin, InstanceSet &local_instances)
{
  if (graph_ == nullptr) {
    printf("LocalSta::collectLocalFanouts graph pointer is nullptr\n");
    fflush(stdout);
    return;
  }

  if (drvr_pin == nullptr) {
    printf("LocalSta::collectLocalFanouts drvr_pin is nullptr\n");
    fflush(stdout);
    return;
  }

  VertexId vertex_id = network_->vertexId(drvr_pin);
  if (vertex_id == vertex_id_null) {
    printf("LocalSta::collectLocalFanouts vertex_id is 0 for pin %s\n",
           network_->name(drvr_pin));
    fflush(stdout);
    return;
  }

  Vertex *vertex = graph_->pinDrvrVertex(drvr_pin);
  if (vertex == nullptr) {
    printf("LocalSta::collectLocalFanouts vertex is nullptr for pin %s\n",
           network_->name(drvr_pin));
    fflush(stdout);
    return;
  }

  VertexOutEdgeIterator edge_iter(vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge *out_edge = edge_iter.next();
    Vertex *load_vertex = out_edge->to(graph_);
    Instance *load_inst = network_->instance(load_vertex->pin());
    if (load_inst)
      local_instances.insert(load_inst);
  }
}

// Compute average leakage across all when-conditions.
// This is a simple average that does NOT consider input duty cycle.
// Same approach as Resizer::cellLeakage.
float
LocalSta::cellAvgLeakage(sta::LibertyCell *cell)
{
  // 1. Try cell-level default leakage first
  float leakage = 0.0f;
  bool exists;
  cell->leakagePower(leakage, exists);
  if (exists) {
    return leakage;
  }

  // 2. Average all conditional leakage groups
  sta::LeakagePowerSeq *leakages = cell->leakagePowers();
  if (!leakages || leakages->empty()) {
    return 0.0f;
  }

  float total_leakage = 0.0f;
  int count = 0;
  for (sta::LeakagePower *leak : *leakages) {
    float pwr = leak->power();
    if (pwr > 0.0f) {
      total_leakage += pwr;
      count++;
    }
  }
  return count > 0 ? total_leakage / count : 0.0f;
}

// Compute buffer/inverter leakage weighted by input duty cycle.
// For a buffer with when conditions like:
//   when: "(A * Y)"   → input=1, output=1  → probability = input_duty
//   when: "(!A * !Y)" → input=0, output=0  → probability = 1 - input_duty
// For an inverter:
//   when: "(A * !Y)"  → input=1, output=0  → probability = input_duty
//   when: "(!A * Y)"  → input=0, output=1  → probability = 1 - input_duty
//
// We evaluate each when-expression by assigning:
//   P(input_port = 1) = input_duty
//   P(output_port = 1) = output_duty  (same as input_duty for buffer,
//                                       1-input_duty for inverter)
float
LocalSta::cellLeakageWithDuty(sta::LibertyCell *cell,
                              float input_duty)
{
  sta::LibertyPort *in_port, *out_port;
  cell->bufferPorts(in_port, out_port);

  // Determine output duty based on cell function
  sta::FuncExpr *func = out_port->function();
  bool is_inverter = (func
                      && func->op() == sta::FuncExpr::op_not
                      && func->left()->op() == sta::FuncExpr::op_port);
  float output_duty = is_inverter ? (1.0f - input_duty) : input_duty;

  // Lambda to evaluate P(when=true) given port duties
  std::function<float(sta::FuncExpr*)> evalProb;
  evalProb = [&](sta::FuncExpr *expr) -> float {
    switch (expr->op()) {
      case sta::FuncExpr::op_port: {
        sta::LibertyPort *port = expr->port();
        if (port == in_port)
          return input_duty;
        else if (port == out_port)
          return output_duty;
        return 0.5f;
      }
      case sta::FuncExpr::op_not:
        return 1.0f - evalProb(expr->left());
      case sta::FuncExpr::op_and:
        return evalProb(expr->left()) * evalProb(expr->right());
      case sta::FuncExpr::op_or: {
        float pa = evalProb(expr->left());
        float pb = evalProb(expr->right());
        return pa + pb - pa * pb;
      }
      case sta::FuncExpr::op_xor: {
        float pa = evalProb(expr->left());
        float pb = evalProb(expr->right());
        return pa * (1.0f - pb) + (1.0f - pa) * pb;
      }
      case sta::FuncExpr::op_one:
        return 1.0f;
      case sta::FuncExpr::op_zero:
        return 0.0f;
    }
    return 0.5f;
  };

  // Weighted leakage sum
  float cond_leakage = 0.0f;
  bool found_cond = false;
  float uncond_leakage = 0.0f;
  bool found_uncond = false;
  float cond_duty_sum = 0.0f;

  for (sta::LeakagePower *leak : *cell->leakagePowers()) {
    sta::FuncExpr *when = leak->when();
    if (when) {
      float prob = evalProb(when);
      cond_leakage += leak->power() * prob;
      if (leak->power() > 0.0f)
        cond_duty_sum += prob;
      found_cond = true;
    } else {
      uncond_leakage += leak->power();
      found_uncond = true;
    }
  }

  float leakage = 0.0f;

  // Cell-level default leakage covers the remaining probability space
  float cell_leakage;
  bool cell_leakage_exists;
  cell->leakagePower(cell_leakage, cell_leakage_exists);
  if (cell_leakage_exists) {
    float remaining_duty = 1.0f - cond_duty_sum;
    cell_leakage *= remaining_duty;
  }

  if (found_cond)
    leakage = cond_leakage;
  else if (found_uncond)
    leakage = uncond_leakage;

  if (cell_leakage_exists)
    leakage += cell_leakage;

  return leakage;
}

void
LocalSta::collectLocalFaninSiblings(Pin *load_pin, PinSet &visited_pins, 
                                 InstanceSet &local_instances)
{
  PinSeq loads, drvrs;
  FindNetDrvrLoads visitor(load_pin, visited_pins, loads, drvrs, network_);
  network_->visitConnectedPins(load_pin, visitor);

  for (auto drvr_pin : drvrs) {
    Instance *drvr_inst = network_->instance(drvr_pin);
    if (!drvr_inst || network_->isTopInstance(drvr_inst))
      return;
  }
  
  for (auto drvr_pin : drvrs) {
    if (drvr_pin == load_pin)
      continue;
    Instance *drvr_inst = network_->instance(drvr_pin);
    if (drvr_inst)
      local_instances.insert(drvr_inst);
  }
  for (auto fanin_pins : loads) {
    Instance *load_inst = network_->instance(fanin_pins);
    if (load_inst)
      local_instances.insert(load_inst);
  }
}

// void 
// LocalSta::makePtGraph(PtGraph *pt_graph, Instance *inst, 
//                           DcalcAnalysisPt *dcalc_ap)
// {
//   InstanceSet local_instances(sta_->network());
//   collectLocalGraph(inst, local_instances);
//   pt_graph->makeGraph(local_instances, inst);
//   if (dcalc_ap == nullptr) {
//     Corner *corner = sta_->corners()->findCorner(0);
//     dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
//     if (dcalc_ap == nullptr) {
//       throw std::runtime_error("LocalSta::makePtGraph: No dcalc analysis point found");
//     }
//   }
//   pt_graph->setDcalcAnalysisPt(dcalc_ap);
// }

void
LocalSta::makePtGraph(PtGraph *pt_graph, Instance *inst, 
                      DcalcAnalysisPt *dcalc_ap)
{
  VertexSet local_vertices(graph_);
  collectLocalVertices(inst, local_vertices);
  pt_graph->makeGraph(local_vertices, inst);
  if (dcalc_ap == nullptr) {
    Corner *corner = sta_->corners()->findCorner("default");
    dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
    if (dcalc_ap == nullptr) {
      throw std::runtime_error("LocalSta::makePtGraph: No dcalc analysis point found");
    }
  }
  pt_graph->setDcalcAnalysisPt(dcalc_ap);
}

PtGraph *
LocalSta::makePtGraph(Instance *inst, bool update_timing_first)
{
  PtGraph *pt_graph = new PtGraph(sta_);
  makePtGraph(pt_graph, inst);
  if (update_timing_first) {
    Level top_level = pt_graph->topVertexLevel();
    findDelays(top_level);
    search_->findArrivals(top_level);
    pt_graph->initVertexAndEdges();
  }
  // Initialize PtPiElmore for all drivers in the PtGraph.
  local_parasitics_->recomputePtParasitics(pt_graph);
  std::lock_guard<std::mutex> lock(pt_graph_vector_mutex_);
  local_graphs_.push_back(pt_graph);
  return pt_graph;
}

void
LocalSta::topoSortVertices(PtGraph *pt_graph)
{
  pt_graph->topoSortVertices();
}

void
LocalSta::findLocalDelays(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc)
{
  for (VertexId vertex_id : pt_graph->sortedVertexIds()) {
    PtVertex &pt_vertex = pt_graph->ptVertex(vertex_id);
    findVertexDelays(vertex_id,  arc_delay_calc, pt_graph);
  }
}

void 
LocalSta::seedRootSlew(PtVertex &pt_vertex, PtGraph *pt_graph, 
                       ArcDelayCalc *arc_delay_calc)
{
  if (!pt_vertex.hasBase()) {
    // Virtual vertex as root: no slew to seed, delays will be
    // computed when arrival propagation reaches it.
    if (pt_vertex.type() != PtVertexType::Sentinel)
      throw std::runtime_error("LocalSta::seedRootSlew: Virtual root vertex type must be Sentinal");
    return;
  }
  Vertex *vertex = pt_vertex.vertex();

  if (pt_vertex.type() == PtVertexType::RefDriver
      || pt_vertex.type() == PtVertexType::RefInput) {
    if (vertex->isDriver(network_)) {
      seedDrvrSlew(pt_vertex, pt_graph, arc_delay_calc);
    } else {
      loadSlewFromGraph(pt_vertex, pt_graph);
    }
  } else {
    loadSlewFromGraph(pt_vertex, pt_graph);
  }
}

void
LocalSta::seedDrvrSlew(PtVertex &pt_drvr_vertex, PtGraph *pt_graph,
                       ArcDelayCalc *arc_delay_calc)
{
  const Pin *drvr_pin = pt_drvr_vertex.vertex()->pin();
  InputDrive *drive = 0;
  if (network_->isTopLevelPort(drvr_pin)) {
    Port *port = network_->port(drvr_pin);
    drive = sdc_->findInputDrive(port);
  }
  for (const RiseFall *rf : RiseFall::range()) {
    for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
      if (drive) {
  const MinMax *cnst_min_max = dcalc_ap->constraintMinMax();
	const LibertyCell *drvr_cell;
	const LibertyPort *from_port, *to_port;
	float *from_slews;
	drive->driveCell(rf, cnst_min_max, drvr_cell, from_port,
			 from_slews, to_port);
  if (drvr_cell) {
    printf("Warning: LocalSta::seedDrvrSlew: Input drive seeding not implemented yet\n");
    // if (from_port == nullptr) {
    //   from_port = driveCellDefaultFromPort(drvr_cell, to_port);
    // }
    // findInputDriverDelay(drvr_cell, drvr_pin, drvr_vertex, rf,
			      //  from_port, from_slews, to_port, dcalc_ap);
  } else
    seedNoDrvrCellSlew(pt_drvr_vertex, drvr_pin, rf, drive, 
              dcalc_ap, arc_delay_calc, pt_graph);
      } else {
        seedNoDrvrSlew(pt_drvr_vertex, rf, dcalc_ap, arc_delay_calc, pt_graph);
      }
    }
  }
}

void
LocalSta::seedNoDrvrCellSlew(PtVertex &pt_drvr_vertex,
                              const Pin *drvr_pin,
                              const RiseFall *rf,
                              const InputDrive *drive,
                              const DcalcAnalysisPt *dcalc_ap,
                              ArcDelayCalc *arc_delay_calc,
                              PtGraph *pt_graph)
{
  DcalcAPIndex ap_index = dcalc_ap->index();
  const MinMax *cnst_min_max = dcalc_ap->constraintMinMax();
  Slew slew = default_slew;
  float drive_slew;
  bool exists;
  drive->slew(rf, cnst_min_max, drive_slew, exists);
  if (exists)
    slew = drive_slew;
  else {
    // Top level bidirect driver uses load slew unless
    // bidirect instance paths are disabled.
    printf("Warning: LocalSta::seedNoDrvrCellSlew: Input drive slew not found for pin %s\n",
           network_->name(drvr_pin));
           fflush(stdout);
  }
  Delay drive_delay = delay_zero;
  float drive_res;
  drive->driveResistance(rf, cnst_min_max, drive_res, exists);
  const Parasitic *parasitic;
  float load_cap;
  localParasiticLoad(pt_drvr_vertex, rf, dcalc_ap, nullptr, load_cap, parasitic, pt_graph);
  if (exists) {
    drive_delay = load_cap * drive_res;
    slew = load_cap * drive_res;
  }
  const MinMax *slew_min_max = dcalc_ap->slewMinMax();
  if (pt_drvr_vertex.vertex()->slewAnnotated(rf, slew_min_max)) {
      slew = graph_->slew(pt_drvr_vertex.vertex(), rf, ap_index);
  }

  pt_graph->setSlew(pt_drvr_vertex, rf, ap_index, slew);
  LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(pt_drvr_vertex, pt_graph);
  ArcDcalcResult dcalc_result =
    arc_delay_calc->inputPortDelay(drvr_pin, delayAsFloat(slew), rf, parasitic,
                                   load_pin_index_map, dcalc_ap);
  annotateLoadDelays(pt_drvr_vertex, rf, dcalc_result, load_pin_index_map, 
                     drive_delay, false, dcalc_ap, pt_graph);
  arc_delay_calc->finishDrvrPin();
}

void
LocalSta::seedNoDrvrSlew(PtVertex &pt_drvr_vertex,
                             const RiseFall *rf,
                             const DcalcAnalysisPt *dcalc_ap,
                             ArcDelayCalc *arc_delay_calc,
                             PtGraph *pt_graph)
{
  const sta::Pin *drvr_pin = pt_drvr_vertex.vertex()->pin();
  sta::Vertex *drvr_vertex = pt_drvr_vertex.vertex();
  const MinMax *slew_min_max = dcalc_ap->slewMinMax();
  DcalcAPIndex ap_index = dcalc_ap->index();
  Slew slew(0.0);
  // Top level bidirect driver uses load slew unless
  // bidirect instance paths are disabled.
  if (bidirectDrvrSlewFromLoad(drvr_pin)) {
    Vertex *load_vertex = graph_->pinLoadVertex(drvr_pin);
    slew = graph_->slew(load_vertex, rf, ap_index);
  } else if (drvr_vertex->slewAnnotated(rf, slew_min_max)) {
     slew = graph_->slew(drvr_vertex, rf, ap_index);
  }

  // Use local slew
  pt_graph->setSlew(pt_drvr_vertex, rf, ap_index, slew);

  float load_cap = 0.0f;
  const Parasitic *parasitic = nullptr;
  localParasiticLoad(pt_drvr_vertex, rf, dcalc_ap, nullptr,
                     load_cap, parasitic, pt_graph);
  LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(pt_drvr_vertex, pt_graph);
  ArcDcalcResult dcalc_result =
    arc_delay_calc->inputPortDelay(drvr_pin, delayAsFloat(slew), rf,
                                   const_cast<Parasitic*>(parasitic),
                                   load_pin_index_map, dcalc_ap);
  annotateLoadDelays(pt_drvr_vertex, rf, dcalc_result, load_pin_index_map, delay_zero, false, dcalc_ap, pt_graph);
  arc_delay_calc->finishDrvrPin();
}

void
LocalSta::seedLoadSlew(PtVertex &pt_load_vertex, PtGraph *pt_graph,
                       ArcDelayCalc *arc_delay_calc)
{
  Vertex *vertex = pt_load_vertex.vertex();
  const Pin *pin = vertex->pin();
  ClockSet *clks = sdc_->findLeafPinClocks(pin);
  loadSlewFromGraph(pt_load_vertex, pt_graph);
  for (const RiseFall *rf : RiseFall::range()) {
    for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
      const MinMax *slew_min_max = dcalc_ap->slewMinMax();
      if (!vertex->slewAnnotated(rf, slew_min_max)) {
	float slew = 0.0;
	if (clks) {
	  slew = slew_min_max->initValue();
	  ClockSet::Iterator clk_iter(clks);
	  while (clk_iter.hasNext()) {
	    Clock *clk = clk_iter.next();
	    float clk_slew = clk->slew(rf, slew_min_max);
	    if (slew_min_max->compare(clk_slew, slew))
	      slew = clk_slew;
	  }
	}
	DcalcAPIndex ap_index = dcalc_ap->index();
	pt_graph->setSlew(pt_load_vertex, rf, ap_index, Slew(slew));
      }
    }
  }
}

void 
LocalSta::loadSlewFromGraph(PtVertex &root_pt_vertex, PtGraph *pt_graph)
{
  Vertex *root_vertex = root_pt_vertex.vertex();
  for (const RiseFall *rf : RiseFall::range()) {
    for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
      Slew slew = graph_->slew(root_vertex, rf, dcalc_ap->index());
      pt_graph->setSlew(root_pt_vertex, rf, dcalc_ap->index(), slew);
    }
  }
}

void 
LocalSta::findInputDriverDelay(const LibertyCell *drvr_cell,
                              const Pin *drvr_pin,
                              Vertex *drvr_vertex,
                              const RiseFall *rf,
                              const LibertyPort *from_port,
                              float *from_slews,
                              const LibertyPort *to_port,
                              const DcalcAnalysisPt *dcalc_ap)
{
  for (TimingArcSet *arc_set : drvr_cell->timingArcSets(from_port, to_port)) {
    for (TimingArc *arc : arc_set->arcs()) {
      if (arc->toEdge()->asRiseFall() == rf) {
        float from_slew = from_slews[arc->fromEdge()->index()];
        findInputArcDelay(drvr_pin, drvr_vertex, arc, from_slew, dcalc_ap);
      }
    }
  }
  arc_delay_calc_->finishDrvrPin();
}

int 
LocalSta::findPortIndex(const LibertyCell *cell,
                        const LibertyPort *port)
{
  int index = 0;
  LibertyCellPortIterator port_iter(cell);
  while (port_iter.hasNext()) {
    LibertyPort *cell_port = port_iter.next();
    if (cell_port == port)
      return index;
    index++;
  }
  return 0;
}


void 
LocalSta::findVertexDelays(VertexId pt_vertex_id,
                           ArcDelayCalc *arc_delay_calc,
                           PtGraph *pt_graph)
{
  PtVertex &pt_vertex = pt_graph->ptVertex(pt_vertex_id);
  if (pt_vertex.isRoot()) {
    seedRootSlew(pt_vertex, pt_graph, arc_delay_calc);
  } else if (pt_vertex.isDriver()) {
    if (pt_vertex.hasBase()) {
      Pin *pin = pt_vertex.vertex()->pin();
      if (network_->isLeaf(pin)) {
        LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(pt_vertex, pt_graph);
        findDriverDelays(pt_vertex, arc_delay_calc,
                         load_pin_index_map, pt_graph);
      }
    } else {
      // Virtual driver vertex
      LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(pt_vertex, pt_graph);
      findDriverDelays(pt_vertex, arc_delay_calc,
                       load_pin_index_map, pt_graph);
    }
  }
}


void 
LocalSta::findDriverDelays(PtVertex &drvr_pt_vertex,
                           ArcDelayCalc *arc_delay_calc,
                           LoadPinIndexMap &load_pin_index_map,
                           PtGraph *pt_graph)
{
  initLoadSlews(drvr_pt_vertex, pt_graph);
  // Compute delays for each arc from the driver vertex.
  findDriverDelays1(drvr_pt_vertex, nullptr, arc_delay_calc, 
                    load_pin_index_map, pt_graph);
}

void 
LocalSta::initSlew(PtVertex &pt_vertex, PtGraph *pt_graph)
{
  for (const RiseFall *rf : RiseFall::range()) {
    for (const DcalcAnalysisPt *dcalc_ap : sta_->corners()->dcalcAnalysisPts()) {
      const MinMax *slew_min_max = dcalc_ap->slewMinMax();
      Slew slew_init_value(slew_min_max->initValue());
      DcalcAPIndex ap_index = dcalc_ap->index();
      pt_graph->setSlew(pt_vertex, rf, ap_index, slew_init_value);
    }
  }
}

void 
LocalSta::findDriverDelays1(PtVertex &drvr_pt_vertex,
                            MultiDrvrNet *multi_drvr_net,
                            ArcDelayCalc *arc_delay_calc,
                            LoadPinIndexMap &load_pin_index_map,
                            PtGraph *pt_graph)
{
  initSlew(drvr_pt_vertex, pt_graph);
  initWireDelays(drvr_pt_vertex, pt_graph);
  std::array<bool, RiseFall::index_count> delay_exists = {false, false};
  PtVertexInEdgeIterator in_edge_iter(drvr_pt_vertex.objectIdx(), pt_graph);
  while (in_edge_iter.hasNext()) {
    PtEdge &pt_edge = in_edge_iter.next();

    // Skip pruned sibling arcs (LM < threshold) to avoid expensive
    // liberty table lookups for second-order timing edges.
    if (pt_edge.isSiblingSkipped())
      continue;

    // PtGraph edges already passed searchThru at construction time.
    // Avoid dereferencing pt_edge.edge() here because the underlying
    // sta::Edge* may have been invalidated by a concurrent replaceCell.
    bool pass_predicates;
    if (pt_edge.hasBase()) {
      Vertex *from_vertex = pt_graph->ptVertex(pt_edge.ptFromId()).vertex();
      pass_predicates = search_pred_->searchFrom(from_vertex)
                        && !pt_edge.role()->isLatchDtoQ();
    } else {
      pass_predicates = !pt_edge.role()->isLatchDtoQ();
    }

    if (pass_predicates)
      findDriverEdgeDelays(drvr_pt_vertex, multi_drvr_net, pt_edge,
                           arc_delay_calc, load_pin_index_map,
                           delay_exists, pt_graph);
  }
  for (const RiseFall *rf : RiseFall::range()) {
    if (!delay_exists[rf->index()]) {
      zeroSlewAndWireDelays(drvr_pt_vertex, rf, pt_graph);
    }
  }
}

void 
LocalSta::zeroSlewAndWireDelays(PtVertex &drvr_pt_vertex,
                           const RiseFall *rf,
                           PtGraph *pt_graph)
{
  Vertex *drvr_vertex = drvr_pt_vertex.vertex();
  for (const DcalcAnalysisPt *dcalc_ap : sta_->corners()->dcalcAnalysisPts()) {
    DcalcAPIndex ap_index = dcalc_ap->index();
    const MinMax *slew_min_max = dcalc_ap->slewMinMax();
    // Init drvr slew.
    bool drvr_slew_annotated = drvr_vertex
        ? drvr_vertex->slewAnnotated(rf, slew_min_max) : false;
    if (!drvr_slew_annotated) {
      pt_graph->setSlew(drvr_pt_vertex, rf, ap_index, slew_min_max->initValue());
    }

    // Init wire delays and slews.
    PtVertexOutEdgeIterator edge_iter(drvr_pt_vertex.objectIdx(), pt_graph);
    while (edge_iter.hasNext()) {
      PtEdge &pt_edge = edge_iter.next();
      if (pt_edge.isWire()) {
        PtVertex &load_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
        Vertex *load_vertex = load_pt_vertex.vertex();
        Edge *wire_edge = pt_edge.edge();
        // VirtualWireEdge has pre-set delays from buildVirtualBuffer;
        // treat as annotated to avoid zeroing them.
        bool wire_annotated = (pt_edge.type() == PtEdgeType::VirtualWireEdge)
            || (wire_edge && graph_->wireDelayAnnotated(wire_edge, rf, ap_index));
        if (!wire_annotated) {
          pt_graph->setWireArcDelay(pt_edge, rf, ap_index, delay_zero);
        }
        bool load_slew_annotated = load_vertex
            ? load_vertex->slewAnnotated(rf, slew_min_max) : false;
        if (!load_slew_annotated) {
          pt_graph->setSlew(load_pt_vertex, rf, ap_index, 0.0);
        }
      }
    }
  }
}

void 
LocalSta::findDriverEdgeDelays(PtVertex &drvr_pt_vertex,
                               const MultiDrvrNet *multi_drvr_net,
                               PtEdge &pt_edge,
                               ArcDelayCalc *arc_delay_calc,
                               LoadPinIndexMap &load_pin_index_map,
                               std::array<bool, RiseFall::index_count> &delay_exists,
                               PtGraph *pt_graph)
{ 
  // If both vertices belong to ref instance, use ref cell's timing
  TimingArcSet *ref_arc_set = pt_edge.timingArcSet();
  if (ref_arc_set == nullptr){
    printf("ERROR findDriverEdgeDelays: timingArcSet is nullptr for edge %u "
           "(from %u to %u), type=%d, hasBase=%d, isWire=%d\n",
           pt_edge.objectIdx(), pt_edge.ptFromId(), pt_edge.ptToId(),
           (int)pt_edge.type(), (int)pt_edge.hasBase(), (int)pt_edge.isWire());
    fflush(stdout);
    throw std::runtime_error("LocalSta::findDriverEdgeDelays: timingArcSet is nullptr");
  }
  
  for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
    for (const TimingArc *arc : ref_arc_set->arcs()) {
      findDriverArcDelays(drvr_pt_vertex, multi_drvr_net, pt_edge, 
                         arc, dcalc_ap, arc_delay_calc,
                         load_pin_index_map, pt_graph);
      delay_exists[arc->toEdge()->asRiseFall()->index()] = true;
    }
  }
  return;
}

void 
LocalSta::findDriverArcDelays(PtVertex &drvr_pt_vertex,
                              PtEdge &pt_edge,
                              const TimingArc *arc,
                              const DcalcAnalysisPt *dcalc_ap,
                              ArcDelayCalc *arc_delay_calc,
                              PtGraph *pt_graph)
{
  Vertex *drvr_vertex = drvr_pt_vertex.vertex();
  MultiDrvrNet *multi_drvr = drvr_vertex ? multiDrvrNet(drvr_vertex) : nullptr;
  LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(drvr_pt_vertex, pt_graph);
  findDriverArcDelays(drvr_pt_vertex, multi_drvr, pt_edge, arc, dcalc_ap,
                      arc_delay_calc, load_pin_index_map, pt_graph);
}

void 
LocalSta::findDriverArcDelays(PtVertex &drvr_pt_vertex,
                              const MultiDrvrNet *multi_drvr_net,
                              PtEdge &pt_edge,
                              const TimingArc *arc,
                              const DcalcAnalysisPt *dcalc_ap,
                              ArcDelayCalc *arc_delay_calc,
                              LoadPinIndexMap &load_pin_index_map,
                              PtGraph *pt_graph)
{
  const RiseFall *from_rf = arc->fromEdge()->asRiseFall();
  const RiseFall *drvr_rf = arc->toEdge()->asRiseFall();
  if (from_rf && drvr_rf) {
    const Pin *drvr_pin = drvr_pt_vertex.pin();
    // For virtual vertices, fall back to proxy vertex pin for PVT lookup in gateDelay
    const Pin *dcalc_pin = drvr_pin;
    if (!dcalc_pin && drvr_pt_vertex.proxyVertex())
      dcalc_pin = drvr_pt_vertex.proxyVertex()->pin();
    if (!dcalc_pin)
      throw std::runtime_error("LocalSta::findDriverArcDelays: virtual vertex has no pin and no proxy vertex");
    const Parasitic *parasitic = nullptr;
    float load_cap = 0.0f;

    localParasiticLoad(drvr_pt_vertex, drvr_rf, dcalc_ap, multi_drvr_net,
                       load_cap, parasitic, pt_graph);

    if (multi_drvr_net == nullptr) {
      PtVertex &from_pt_vertex = pt_graph->ptVertex(pt_edge.ptFromId());
      const Slew in_slew = edgeFromLocalSlew(from_pt_vertex, from_rf, pt_edge,
                                            dcalc_ap, pt_graph);
      ArcDcalcResult dcalc_result;
      dcalc_result = arc_delay_calc->gateDelay(
                          dcalc_pin, arc, in_slew, load_cap, parasitic,
                          load_pin_index_map, dcalc_ap);

      annotateDelaysSlews(pt_edge, arc, dcalc_result,
                          load_pin_index_map, dcalc_ap, pt_graph);
    } else {
      // ArcDcalcArg dcalc_args = makeArcDcalcArgs(drvr_pt_vertex,
                                  // multi_drvr_net, pt_edge, arc,
                                  // dcalc_ap, arc_delay_calc, pt_graph);
      printf("LocalSta::findDriverArcDelays multi-driver net not implemented\n");
      fflush(stdout);
    }
    arc_delay_calc->finishDrvrPin();
  }
  // debug_info_.push_back(debug_info);  
}

bool
LocalSta::annotateDelaysSlews(PtEdge &pt_edge,
                         const TimingArc *arc,
                         ArcDcalcResult &dcalc_result,
                         LoadPinIndexMap &load_pin_index_map,
                         const DcalcAnalysisPt *dcalc_ap,
                         PtGraph *pt_graph)
{
  bool delay_changed = annotateDelaySlew(pt_edge, arc,
                  dcalc_result.gateDelay(),
                  dcalc_result.drvrSlew(), dcalc_ap, pt_graph);
  if (!pt_edge.role()->isLatchDtoQ()) {
    PtVertex &to_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
    delay_changed |= annotateLoadDelays(to_pt_vertex, arc->toEdge()->asRiseFall(),
                       dcalc_result, load_pin_index_map,
                       delay_zero, true, dcalc_ap, pt_graph);
  }
  return delay_changed;
}

bool 
LocalSta::annotateLoadDelays(PtVertex &drvr_pt_vertex,
                             const RiseFall *to_rf,
                             ArcDcalcResult &dcalc_result,
                             LoadPinIndexMap &load_pin_index_map,
                             const ArcDelay &extra_delay,
                             bool merge,
                             const DcalcAnalysisPt *dcalc_ap,
                             PtGraph *pt_graph)
{
  Vertex *drvr_vertex = drvr_pt_vertex.vertex();
  bool load_changed = false;
  DcalcAPIndex ap_index = dcalc_ap->index();
  const MinMax * slew_min_max = dcalc_ap->slewMinMax();
  PtVertexOutEdgeIterator edge_iter(drvr_pt_vertex, pt_graph);
  while (edge_iter.hasNext()) {
    PtEdge &wire_pt_edge = edge_iter.next();
    if (wire_pt_edge.isWire()) {
      PtVertex &load_pt_vertex = pt_graph->ptVertex(wire_pt_edge.ptToId());
      Vertex *load_vertex = load_pt_vertex.vertex();
      Pin *load_pin = load_vertex ? load_vertex->pin() : nullptr;

      if (!load_pin) {
        // Virtual load: try PtPiElmore Elmore delay for wire delay + load slew
        PtPiElmore *pt_pi = pt_graph->findPtParasitic(
            drvr_pt_vertex.objectIdx(), to_rf, ap_index);
        if (pt_pi) {
          bool exists;
          float elmore = pt_pi->findElmoreByVertexId(
              load_pt_vertex.objectIdx(), exists);
          if (exists && elmore > 0.0f) {
            Slew drvr_slew = dcalc_result.drvrSlew();
            // Same formula as DmpCeff::dspfWireDelaySlew
            LibertyPort *load_port = load_pt_vertex.libertyPort();
            LibertyLibrary *load_lib = load_port
                ? load_port->libertyCell()->libertyLibrary() : nullptr;
            float vth = 0.5f, vl = 0.2f, vh = 0.8f, slew_derate = 1.0f;
            if (load_lib) {
              vth = load_lib->inputThreshold(to_rf);
              vl = load_lib->slewLowerThreshold(to_rf);
              vh = load_lib->slewUpperThreshold(to_rf);
              slew_derate = load_lib->slewDerateFromLibrary();
            }
            ArcDelay wire_delay = -elmore * log(1.0 - vth);
            Slew load_slew = drvr_slew
                + elmore * log((1.0 - vl) / (1.0 - vh)) / slew_derate;

            pt_graph->setWireArcDelay(wire_pt_edge, to_rf, ap_index, wire_delay);
            const Slew &cur_slew = pt_graph->slew(load_pt_vertex, to_rf, ap_index);
            if (!merge || delayGreater(load_slew, cur_slew, slew_min_max, this)) {
              pt_graph->setSlew(load_pt_vertex, to_rf, ap_index, load_slew);
              load_changed = true;
            }
            continue;
          }
        }
        // Fallback: wire delay = 0, load slew = driver slew
        Slew drvr_slew = dcalc_result.drvrSlew();
        const Slew &cur_slew = pt_graph->slew(load_pt_vertex, to_rf, ap_index);
        if (!merge || delayGreater(drvr_slew, cur_slew, slew_min_max, this)) {
          pt_graph->setSlew(load_pt_vertex, to_rf, ap_index, drvr_slew);
          load_changed = true;
        }
        continue;
      }

      // Skip load pins not in the map (hierarchical pins)
      if (load_pin_index_map.find(load_pin) == load_pin_index_map.end())
        continue;
      size_t load_idx = load_pin_index_map[load_pin];

      ArcDelay wire_delay = dcalc_result.wireDelay(load_idx);
      Slew load_slew = dcalc_result.loadSlew(load_idx);
      bool load_slew_annotated = load_vertex->slewAnnotated(to_rf, slew_min_max);
      bool drvr_slew_annotated = drvr_vertex
          ? drvr_vertex->slewAnnotated(to_rf, slew_min_max) : false;
      if (!load_slew_annotated) {
        if (drvr_slew_annotated) {
          Slew drvr_slew = graph_->slew(drvr_vertex, to_rf, ap_index);
          pt_graph->setSlew(load_pt_vertex, to_rf, ap_index, drvr_slew);
          load_changed = true;
        } else {
          const Slew &slew = pt_graph->slew(load_pt_vertex, to_rf, ap_index);
          if (!merge || delayGreater(load_slew, slew, slew_min_max, this)) {
            pt_graph->setSlew(load_pt_vertex, to_rf, ap_index, load_slew);
            load_changed = true;
          }
        }
      }
      Edge *wire_edge = wire_pt_edge.edge();
      bool wire_annotated = wire_edge
          ? graph_->wireDelayAnnotated(wire_edge, to_rf, ap_index) : false;
      if (!wire_annotated) {
        const ArcDelay &delay = pt_graph->wireArcDelay(wire_pt_edge, to_rf, ap_index);
        ArcDelay wire_delay_extra = wire_delay + extra_delay;
        const MinMax *delay_min_max = dcalc_ap->delayMinMax();
        if (!merge || delayGreater(wire_delay_extra, delay, delay_min_max, this)) {
          pt_graph->setWireArcDelay(wire_pt_edge, to_rf, ap_index, wire_delay_extra);
          load_changed = true;
        }
      }
    }
  }
  return load_changed;
}

bool 
LocalSta::annotateDelaySlew(PtEdge &pt_edge,
                         const TimingArc *arc,
                         ArcDelay &gate_delay,
                         Slew &gate_slew,
                         const DcalcAnalysisPt *dcalc_ap,
                         PtGraph *pt_graph)
{
  bool delay_changed = false;
  DcalcAPIndex ap_index = dcalc_ap->index();
  PtVertex &drvr_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
  Vertex *drvr_vertex = drvr_pt_vertex.vertex();
  const RiseFall *drvr_rf = arc->toEdge()->asRiseFall();
  const Slew drvr_slew = pt_graph->slew(drvr_pt_vertex, drvr_rf, ap_index);
  const MinMax *slew_min_max = dcalc_ap->slewMinMax();
  bool slew_annotated = drvr_vertex ? drvr_vertex->slewAnnotated(drvr_rf, slew_min_max) : false;
  if (delayGreater(gate_slew, drvr_slew, slew_min_max, this)
      && !slew_annotated
      && !pt_edge.role()->isLatchDtoQ())
    pt_graph->setSlew(drvr_pt_vertex, drvr_rf, ap_index, gate_slew);
  bool delay_annotated = pt_edge.hasBase()
      ? graph_->arcDelayAnnotated(pt_edge.edge(), arc, ap_index) : false;
  if (!delay_annotated) {
    const ArcDelay &prev_gate_delay = 
                              pt_graph->arcDelay(pt_edge, arc, ap_index);
    float prev_gate_delay1 = delayAsFloat(prev_gate_delay);
    float gate_delay1 = delayAsFloat(gate_delay);
    if (prev_gate_delay1 == 0.0 || (abs(gate_delay1 - prev_gate_delay1) 
        / gate_delay1) > incremental_delay_tolerance_) 
      delay_changed = true;
    pt_graph->setArcDelay(pt_edge, arc, ap_index, gate_delay);
  }
  return delay_changed;
}

// ArcDcalcArgSeq 
// LocalSta::makeArcDcalcArgs(PtVertex &drvr_pt_vertex,
//                            const MultiDrvrNet *multi_drvr_net,
//                            PtEdge &pt_edge,
//                            const TimingArc *arc,
//                            const DcalcAnalysisPt *dcalc_ap,
//                            ArcDelayCalc *arc_delay_calc,
//                            PtGraph *pt_graph)
// {
//   // Make arc dcalc args for all parallel drivers on multi-driver net.
//   return ArcDcalcArgSeq();
// }

float
LocalSta::computeVirtualLoadCap(PtVertex &drvr_pt_vertex,
                                const RiseFall *drvr_rf,
                                const DcalcAnalysisPt *dcalc_ap,
                                PtGraph *pt_graph)
{
  float load_cap = 0.0f;
  const Corner *corner = dcalc_ap->corner();
  const MinMax *min_max = dcalc_ap->constraintMinMax();

  // Driver output pin capacitance (self-cap of the output port)
  LibertyPort *drvr_port = nullptr;
  const Pin *drvr_pin = drvr_pt_vertex.pin();
  if (drvr_pin) {
    drvr_port = network_->libertyPort(drvr_pin);
  } else {
    drvr_port = drvr_pt_vertex.libertyPort();
  }
  if (drvr_port) {
    float dcap = drvr_port->capacitance(drvr_rf, min_max);
    load_cap += dcap;
    // printf("[DEBUG computeVirtualLoadCap] drvr_port=%s self_cap=%.6f pF\n",
    //        drvr_port->name(), dcap * 1e12);
  } else {
    printf("[WARNING computeVirtualLoadCap] drvr_port=NULL (pin=%p)\n",
           (void*)drvr_pin);
  }

  // Downstream load pin capacitances
  PtVertexOutEdgeIterator edge_iter(drvr_pt_vertex.objectIdx(), pt_graph);
  int load_count = 0;
  while (edge_iter.hasNext()) {
    PtEdge &pt_edge = edge_iter.next();
    if (pt_edge.isWire()) {
      PtVertex &load_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
      LibertyPort *load_port = nullptr;
      std::string load_name;

      if (load_pt_vertex.hasBase()) {
        // Real load: use network->libertyPort
        if (const Pin *load_pin = load_pt_vertex.pin()) {
          load_port = network_->libertyPort(load_pin);
          load_name = network_->name(load_pin);
        }
      } else {
        // Virtual load: use libertyPort directly
        load_port = load_pt_vertex.libertyPort();
        load_name = load_port ? load_port->name() : "?";
      }

      if (load_port) {
        float pin_cap = load_port->capacitance(drvr_rf, min_max);
        load_cap += pin_cap;
        // printf("[DEBUG computeVirtualLoadCap] load=%s cap=%.6f\n",
        //        load_name.c_str(), pin_cap * 1e12);
        load_count++;
      }
    }
  }
  // printf("[DEBUG computeVirtualLoadCap] total_cap=%.6f (loads=%d)\n",
  //        load_cap * 1e12, load_count);
  return load_cap;
}

void
LocalSta::initWireDelays(PtVertex &drvr_pt_vertex, PtGraph *pt_graph)
{
  pt_graph->initWireDelays(drvr_pt_vertex);
}

void 
LocalSta::initLoadSlews(PtVertex &pt_vertex, PtGraph *pt_graph)
{
  pt_graph->initLoadSlews(pt_vertex);
}

LoadPinIndexMap
LocalSta::makeLoadPinIndexMap(Vertex *drvr_vertex)
{
  LoadPinIndexMap load_pin_index_map(network_);
  size_t load_idx = 0;
  VertexOutEdgeIterator edge_iter(drvr_vertex, graph_);
  while (edge_iter.hasNext()) {
    Edge *wire_edge = edge_iter.next();
    if (wire_edge->isWire()) {
      Vertex *load_vertex = wire_edge->to(graph_);
      const Pin *load_pin = load_vertex->pin();

      // Only skip hierarchical (dbModITerm) pins.
      // Top-level ports (dbBTerm) are safe for delay calculation.
      if (network_->isHierarchical(load_pin)) {
        continue;
      }

      load_pin_index_map[load_pin] = load_idx;
      load_idx++;
    }
  }
  return load_pin_index_map;
}

LoadPinIndexMap
LocalSta::makeLoadPinIndexMap(PtVertex &drvr_pt_vertex, PtGraph *pt_graph)
{
  LoadPinIndexMap load_pin_index_map(network_);
  size_t load_idx = 0;
  PtVertexOutEdgeIterator edge_iter(drvr_pt_vertex.objectIdx(), pt_graph);
  while (edge_iter.hasNext()) {
    PtEdge &pt_edge = edge_iter.next();
    if (pt_edge.isWire()) {
      PtVertex &load_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
      const Pin *load_pin = load_pt_vertex.pin();

      if (load_pin == nullptr) {
        // Virtual load: no pin to map, skip (will use index directly)
        continue;
      }

      if (network_->isHierarchical(load_pin)) {
        continue;
      }

      load_pin_index_map[load_pin] = load_idx;
      load_idx++;
    }
  }
  return load_pin_index_map;
}

Slew
LocalSta::edgeFromLocalSlew(const PtVertex &from_pt_vertex,
                       const RiseFall *from_rf,
                       const PtEdge &pt_edge,
                       const DcalcAnalysisPt *dcalc_ap,
                       PtGraph *pt_graph)
{
  return edgeFromLocalSlew(from_pt_vertex, from_rf, pt_edge.role(),
                      dcalc_ap, pt_graph);
}

Slew
LocalSta::edgeFromLocalSlew(const PtVertex &from_pt_vertex,
                       const RiseFall *from_rf,
                       const TimingRole *role,
                       const DcalcAnalysisPt *dcalc_ap,
                       PtGraph *pt_graph)
{
  Vertex *from_vertex = from_pt_vertex.vertex();
  if (from_vertex
      && role->genericRole() == TimingRole::regClkToQ()
      && clk_network_->isIdealClock(from_vertex->pin())) {
    return clk_network_->idealClkSlew(from_vertex->pin(), from_rf,
                                      dcalc_ap->slewMinMax());
  } else {
    return pt_graph->slew(from_pt_vertex, from_rf, dcalc_ap->index());
  }
}

void
LocalSta::graphPop()
{
  if (!local_graphs_.empty()) {
    PtGraph *pt_graph = local_graphs_.back();
    local_graphs_.pop_back();
    delete pt_graph;
  }
}

float
LocalSta::delayLmSum(Instance *inst, const MinMax *minmax)
{
  PtGraph *pt_graph = makePtGraph(inst, true);
  float delay_lambda_sum;
  pt_graph->delayLmSum(minmax, delay_lambda_sum);
  graphPop();
  return delay_lambda_sum;
}

float
LocalSta::refgateDelayLmSum(PtGraph *pt_graph)
{
  float delay_lambda_sum;
  pt_graph->refgateDelayLmSum(delay_lambda_sum, nullptr);
  return delay_lambda_sum;
}

float 
LocalSta::delayLmSum(PtGraph *pt_graph, DcalcAnalysisPt *dcalc_ap)
{
  float delay_lm_sum;
  pt_graph->delayLmSum(dcalc_ap, delay_lm_sum);
  return delay_lm_sum;
}

float
LocalSta::delayLmSum(PtGraph *pt_graph)
{
  sta::DcalcAnalysisPt *dcalc_ap = pt_graph->dcalcAnalysisPt();
  float delay_lm_sum;
  pt_graph->delayLmSum(dcalc_ap, delay_lm_sum);
  return delay_lm_sum;
}

DelayLmSumResult
LocalSta::delayLmSum(PtGraph *pt_graph,
                     DcalcAnalysisPt *dcalc_ap,
                     bool collect_vecs)
{
  DelayLmSumResult result;
  pt_graph->delayLmSum(dcalc_ap, &result, collect_vecs);
  return result;
}

float 
LocalSta::maxInputSlew(const Pin* input_pin,
                            const Corner* corner) const
{
  LibertyPort *port = network_->libertyPort(input_pin);
  float limit;
  bool exists;
  sta_->findSlewLimit(port, corner, MinMax::max(), limit, exists);
  if (!exists || limit == 0.0) {
    // Fixup for nangate45: This library doesn't specify any max transition on
    // input pins which indirectly causes issues for the resizer when
    // repairing driver pin transitions.
    //
    // To address, if there's no max tran on the port directly, use the
    // library default (the default only applies to output pins per the
    // Liberty spec, as a workaround we apply it to input pins too).
    port->libertyLibrary()->defaultMaxSlew(limit, exists);
    if (!exists) {
      limit = INF;
    }
  }
  return limit;
}

DelayLmSumResult
LocalSta::initAndGetLocalTimingCost(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc)
{
  // During pt graph creation, delays from original graph are copied 
  // to pt graph. So here we just need to sum up the delays.
  const Corner *corner = corners_->findCorner("default");
  DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
  return delayLmSum(pt_graph, dcalc_ap, false);
}

DelayLmSumResult
LocalSta::increAndGetLocalTimingCost(PtGraph *pt_graph,
                                     ArcDelayCalc *arc_delay_calc,
                                     LibertyCell *equiv_cell)
{
  if (!virtualReplaceCell(pt_graph, equiv_cell)) {
    // Incompatible cell: return max cost to reject this candidate.
    return DelayLmSumResult{};
  }
  findLocalDelays(pt_graph, arc_delay_calc);
  findLocalArrivals(pt_graph);
  findLocalRequireds(pt_graph);
  const Corner *corner = corners_->findCorner("default");
  DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
  return delayLmSum(pt_graph, dcalc_ap, false);
}

void
LocalSta::updateLocalTiming(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc)
{
  findLocalDelays(pt_graph, arc_delay_calc);
  findLocalArrivals(pt_graph);
  findLocalRequireds(pt_graph);
}

// Recompute PtGraph-local PiElmore parasitics after cell swap.
void
LocalSta::recomputeLocalParasitics(PtGraph *pt_graph)
{
  local_parasitics_->recomputePtParasitics(pt_graph);
}

void
LocalSta::recomputeSinglePtParasitic(PtGraph *pt_graph, VertexId drvr_vid)
{
  local_parasitics_->recomputeSinglePtParasitic(pt_graph, drvr_vid);
}

void
LocalSta::syncParasiticNetworkFromGlobal(const Net *net)
{
  local_parasitics_->syncParasiticNetworkFromGlobal(net);
}


Slack
LocalSta::localSlackAroundRef(PtGraph *pt_graph)
{
  Slack local_slack = 0.0;
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    // We offer two options for local slack calculation:
    if (pt_vertex.type() == PtVertexType::RefDriver
        || pt_vertex.type() == PtVertexType::RefOutput) {
      PtVertexPathIterator path_iter(pt_vertex, this);
      while (path_iter.hasNext()) {
        Path *path = path_iter.next();
        // We should select the wanted analysis point here.
        if (path->dcalcAnalysisPt(this) == pt_graph->dcalcAnalysisPt()) {
          Slack slack = path->slack(this);
          if (slack > 0.0) continue; // Only consider negative slack
          local_slack += slack;
          // printf("LocalSta::localSlack: Vertex %s path: %s, arrival = %f, required = %f, slack = %f\n",
          //        pt_vertex.vertex()->to_string(graph_).c_str(),
          //        path->to_string(sta_).c_str(),
          //        path->arrival() * 1.0e12,
          //        path->required() * 1.0e12,
          //        slack * 1.0e12);
          //        fflush(stdout);
        }
      }
    }
  }
  return local_slack;
}

Slack
LocalSta::localSlackAtEndpoints(PtGraph *pt_graph)
{
  Slack local_slack;
  // pending implementation
  local_slack = 0.0;
  return local_slack;
}

Slack
LocalSta::localSlackOnSinks(PtGraph *pt_graph)
{
  // Collect actual sink PtVertices: load pins on nets driven by RefOutput.
  // Walk RefOutput → wire out edges in sta graph → to vertex → ptVertex.
  std::vector<PtVertex*> sink_vertices;
  for (auto& pv : pt_graph->ptVertices()) {
    if (pv.type() != PtVertexType::RefOutput || !pv.vertex())
      continue;
    sta::VertexOutEdgeIterator out_iter(pv.vertex(), graph_);
    while (out_iter.hasNext()) {
      sta::Edge *edge = out_iter.next();
      if (!edge->isWire())
        continue;
      sta::Vertex *load_vertex = edge->to(graph_);
      PtVertex *load_pv = pt_graph->ptVertex(load_vertex);
      if (load_pv && load_pv->hasBase())
        sink_vertices.push_back(load_pv);
    }
  }

  Slack local_slack = 0.0;
  for (PtVertex *pt_vp : sink_vertices) {
    PtVertex &pt_vertex = *pt_vp;

    sta::Path *pt_paths = pt_vertex.paths();
    if (!pt_paths)
      continue;

    sta::Vertex *sta_vertex = pt_vertex.vertex();
    sta::Path *sta_paths = sta_vertex->paths();
    if (!sta_paths)
      continue;

    // Verify tag group match
    sta::TagGroup *pt_tg = search_->tagGroup(pt_vertex.tagGroupIndex());
    sta::TagGroup *sta_tg = search_->tagGroup(sta_vertex);
    if (!pt_tg || !sta_tg || pt_tg->index() != sta_tg->index()) {
      printf("Warning: localSlackOnSinks: tag group mismatch on vertex %s "
             "(pt_tg=%p idx=%d, sta_tg=%p idx=%d)\n",
             sta_vertex->to_string(graph_).c_str(),
             pt_tg, pt_tg ? (int)pt_tg->index() : -1,
             sta_tg, sta_tg ? (int)sta_tg->index() : -1);
      fflush(stdout);
      continue;
    }

    size_t path_count = pt_tg->pathCount();
    for (size_t i = 0; i < path_count; i++) {
      if (pt_paths[i].dcalcAnalysisPt(this) != pt_graph->dcalcAnalysisPt())
        continue;
      sta::Slack slack = sta_paths[i].required() - pt_paths[i].arrival();
      if (sta::delayInf(slack))
        continue;
      if (slack > 0.0)
        continue;
      local_slack += slack;
    }
  }
  return local_slack;
}

void
LocalSta::localParasiticLoad(PtVertex &drvr_pt_vertex,
                          const RiseFall *rf,
                          const DcalcAnalysisPt *dcalc_ap,
                          const MultiDrvrNet *multi_drvr_net,
                          // Return values
                          float &load_cap,
                          const Parasitic *&parasitic,
                          PtGraph *pt_graph)
{
  parasitic = nullptr;
  load_cap = 0.0f;

  const Pin *drvr_pin = drvr_pt_vertex.pin();

  // PtGraph-local PiElmore parasitic (highest priority)
  PtPiElmore *pt_pi = pt_graph->findPtParasitic(
      drvr_pt_vertex.objectIdx(), rf, dcalc_ap->index());
  if (pt_pi && pt_pi->capacitance() > 0.0f) {
    parasitic = pt_pi;
    load_cap = pt_pi->capacitance();
    return;
  }

  // PtPiElmore missing — try to recompute for this driver
  local_parasitics_->recomputeSinglePtParasitic(pt_graph,
                                                 drvr_pt_vertex.objectIdx());
  pt_pi = pt_graph->findPtParasitic(
      drvr_pt_vertex.objectIdx(), rf, dcalc_ap->index());
  if (pt_pi && pt_pi->capacitance() > 0.0f) {
    parasitic = pt_pi;
    load_cap = pt_pi->capacitance();
    return;
  }

  // Virtual driver or driver with virtual buffer downstream
  if (!drvr_pin || drvr_pt_vertex.hasVirtualBuffer()) {
    load_cap = computeVirtualLoadCap(drvr_pt_vertex, rf, dcalc_ap, pt_graph);
    return;
  }

  // Fallback for uncomputed parasitic of real drivers.
  if (network_->net(drvr_pin) == nullptr) {
    load_cap = 0.0;
  } else {
    bool has_net_load;
    float fanout;
    float pin_cap, wire_cap;
    netCaps(drvr_pin, rf, dcalc_ap, multi_drvr_net,
          pin_cap, wire_cap, fanout, has_net_load);
    load_cap = pin_cap + wire_cap;
    // Recompute failed
    // printf("Error: localParasiticLoad: recompute failed for pin %s, using fallback\n",
    //       drvr_pin ? network_->name(drvr_pin) : "(virtual)");
    // fflush(stdout);
  }
}


void
LocalSta::printLocalParasitics(PtGraph *pt_graph) const
{
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.objectIdx() == pt_vertex_id_null)
      continue;
    if (pt_vertex.type() != PtVertexType::RefDriver)
      continue;
    Vertex *vertex = pt_vertex.vertex();
    if (pt_vertex.isDriver() && vertex) {
      for (const RiseFall *rf : RiseFall::range()) {
        for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
          PtPiElmore *pt_pi = pt_graph->findPtParasitic(
              pt_vertex.objectIdx(), rf, dcalc_ap->index());
          if (pt_pi) {
            printf("%s::printLocalParasitics: Pin %s, RF %s, AP %u, with cap %f\n",
                   debug_label_.c_str(),
                   network_->name(vertex->pin()),
                   rf->to_string().c_str(),
                   dcalc_ap->index(),
                   pt_pi->capacitance() * 1.0e15);
            fflush(stdout);
          }
        }
      }
    }
  }
}

void 
LocalSta::printParasitics(PtGraph *pt_graph) const
{
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.objectIdx() == pt_vertex_id_null)
      continue;
    if (pt_vertex.type() != PtVertexType::RefDriver)
      continue;
    Vertex *vertex = pt_vertex.vertex();
    if (pt_vertex.isDriver()) {
      for (const RiseFall *rf : RiseFall::range()) {
        for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
          const Parasitic *parasitic = 
            arc_delay_calc_->findParasitic(vertex->pin(), rf, dcalc_ap);
          float load_cap = local_parasitics_->capacitance(parasitic);
          if (parasitic != nullptr) {
            printf("LOCALSTA::printLocalParasitics: Pin %s, RF %s, AP %u, with cap %f\n",
                   network_->name(vertex->pin()),
                   rf->to_string().c_str(),
                   dcalc_ap->index(),
                   load_cap * 1.0e15);
            fflush(stdout);
          }
        }
      }
    }
  }
}

void
LocalSta::printLocalArrivals(PtGraph *pt_graph) const
{
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    PtVertexPathIterator path_iter(pt_vertex, this);
    while (path_iter.hasNext()) {
      Path *path = path_iter.next();
      printf("%s::printLocalArrivals: Vertex %s arrival path: %s, arrival = %f\n",
             debug_label_.c_str(),
             pt_vertex.vertex()->to_string(graph_).c_str(),
             path->to_string(sta_).c_str(),
             path->arrival() * 1.0e12);
    }
  }
  fflush(stdout);
}

void 
LocalSta::printLocalRequireds(PtGraph *pt_graph) const
{
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    PtVertexPathIterator path_iter(pt_vertex, this);
    while (path_iter.hasNext()) {
      Path *path = path_iter.next();
      printf("%s::printLocalRequireds: Vertex %s required path: %s, required = %f\n",
             debug_label_.c_str(),
             pt_vertex.vertex()->to_string(graph_).c_str(),
             path->to_string(sta_).c_str(),
             path->required() * 1.0e12);
    }
  }
  fflush(stdout);
}

void 
LocalSta::printLocalTiming(PtGraph *pt_graph) const
{
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    std::string vname = pt_vertex.vertex()
        ? pt_vertex.vertex()->to_string(graph_)
        : ("virtual_" + std::to_string(pt_vertex.objectIdx()));
    printf("%s::printLocalTiming: Vertex %s\n",
           debug_label_.c_str(),
           vname.c_str());
    PtVertexPathIterator path_iter(pt_vertex, this);
    while (path_iter.hasNext()) {
      Path *path = path_iter.next();
      printf("  Path: %s, arrival = %f, required = %f\n",
             path->to_string(sta_).c_str(),
             path->arrival() * 1.0e12,
             path->required() * 1.0e12);
    }
  }
  fflush(stdout);
}

void
LocalSta::printLocalSlews(PtGraph *pt_graph) const
{
  printf("LocalSta::printLocalSlews: \n");
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::Sentinel)
      continue;
    std::string vname = pt_vertex.vertex()
        ? pt_vertex.vertex()->to_string(graph_)
        : ("virtual_" + std::to_string(pt_vertex.objectIdx()));
    for (const RiseFall *rf : RiseFall::range()) {
      for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
        Slew slew = pt_graph->slew(pt_vertex, rf, dcalc_ap->index());
        printf("Vertex %s, RF %s, AP %u, slew = %f\n",
               vname.c_str(),
               rf->to_string().c_str(),
               dcalc_ap->index(),
               slew * 1.0e12);
      }
    }
  }
  fflush(stdout);
}

/////////////////////////////////////////////////////
// Legality checking methods
/////////////////////////////////////////////////////
float 
LocalSta::getPinMaxSlewLimit(sta::Pin *pin, sta::LibertyCell *lib_cell)
{
  if (pin == nullptr || lib_cell == nullptr)
    throw std::runtime_error("LocalSta::getPinMaxSlewLimit: pin or lib_cell is nullptr");
  sta::dbNetwork *network = sta_->getDbNetwork();
  const char *port_name = network->portName(pin);
  sta::LibertyPort *sta_port = lib_cell->findLibertyPort(port_name);
  if (sta_port == nullptr)
    throw std::runtime_error("LocalSta::getPinMaxSlewLimit: sta_port is nullptr");
  odb::dbMTerm *db_iterm = network->staToDb(sta_port);
  if (db_iterm == nullptr) {
    printf("LocalSta::getPinMaxSlewLimit: db_iterm is nullptr for port %s of pin %s\n",
           port_name,
           network->name(pin));
    // If DB mapping fails, try to get limit from Liberty port directly
    sta::LibertyLibrary *lib = network->defaultLibertyLibrary();
    bool max_slew_exists;
    float max_slew = 0.0;
    sta_port->slewLimit(MinMax::max(), max_slew, max_slew_exists);
    if (!max_slew_exists) {
      lib->defaultMaxSlew(max_slew, max_slew_exists);
      if (!max_slew_exists)
        max_slew = INF;
    }
    return max_slew;
  }
  sta::LibertyLibrary *lib = network->defaultLibertyLibrary();
  bool max_slew_exists;
  float max_slew = 0.0;
  if (!db_iterm->getSigType().isSupply()) {
    sta_port->slewLimit(MinMax::max(), max_slew, max_slew_exists);
    if (!max_slew_exists) {
      lib->defaultMaxSlew(max_slew, max_slew_exists);
      if (!max_slew_exists)
        max_slew = INF;
    }
  } else {
    printf("LocalSta::getPinMaxSlewLimit: Supply pin %s, setting max_slew to INF\n",
           network->name(pin));
    max_slew = INF;
  }
  return max_slew;
}

float
LocalSta::getPinMaxCapLimit(sta::Pin *pin, sta::LibertyCell *lib_cell)
{
  if (pin == nullptr || lib_cell == nullptr) {
    throw std::runtime_error("LocalSta::getPinMaxCapLimit: pin or lib_cell is nullptr");
  }
  sta::dbNetwork *network = sta_->getDbNetwork();
  const char *port_name = network->portName(pin);
  sta::LibertyPort *sta_port = lib_cell->findLibertyPort(port_name);
  if (sta_port == nullptr) 
    throw std::runtime_error("LocalSta::getPinMaxCapLimit: sta_port is nullptr");
  odb::dbMTerm *db_iterm = network->staToDb(sta_port);
  if (db_iterm == nullptr) {
    printf("LocalSta::getPinMaxCapLimit: db_iterm is nullptr for port %s of pin %s\n",
           port_name,
           network->name(pin));
    // If DB mapping fails, try to get limit from Liberty port directly
    sta::LibertyLibrary *lib = network->defaultLibertyLibrary();
    float max_cap = 0.0;
    bool max_cap_exists;
    sta_port->capacitanceLimit(sta::MinMax::max(), max_cap, max_cap_exists);
    if (!max_cap_exists) {
      lib->defaultMaxCapacitance(max_cap, max_cap_exists);
      if (!max_cap_exists)
        max_cap = INF;
    }
    return max_cap;
  }
  sta::LibertyLibrary *lib = network->defaultLibertyLibrary();
  float max_cap = 0.0;
  bool max_cap_exists;
  if (!db_iterm->getSigType().isSupply()) {
    sta_port->capacitanceLimit(sta::MinMax::max(), max_cap, max_cap_exists);
    if (!max_cap_exists) {
      lib->defaultMaxCapacitance(max_cap, max_cap_exists);
    if (!max_cap_exists)
        max_cap = INF;
    }
  }
  return max_cap;
}

float
LocalSta::getPinSlew(sta::Pin *pin, const sta::Corner *corner,
                     const sta::MinMax *min_max, PtGraph *pt_graph)
{
  sta::Vertex *vertex, *bidir_vertex;
  graph_->pinVertices(pin, vertex, bidir_vertex);
  if (vertex == nullptr)
    throw std::runtime_error("LocalSta::getPinSlew: vertex is nullptr");
  PtVertex *pt_vertex = pt_graph->ptVertex(vertex);
  if (pt_vertex == nullptr)
    return 0.0;
  float max_vertex_slew = 0.0;
  for (const RiseFall *rf : RiseFall::range()) {
    float vertex_slew = pt_graph->slew(*pt_vertex, rf, corner->findDcalcAnalysisPt(min_max)->index());
    if (vertex_slew > max_vertex_slew)
      max_vertex_slew = vertex_slew;
  }
  return max_vertex_slew;
}

const Pin*
LocalSta::findNetParasiticDrvrPin(sta::Net *net) const
{
  const Pin *load_pin = nullptr;
  sta::NetConnectedPinIterator *pin_iter = network_->connectedPinIterator(net);
  while (pin_iter->hasNext()) {
    const Pin *pin = pin_iter->next();
    if (network_->isDriver(pin)) {
      delete pin_iter;
      return pin;
    }
    if (network_->isLoad(pin))
      load_pin = pin;
  }
  delete pin_iter;
  return load_pin;
}

sta::LibertyPort *
LocalSta::findTargetPort(const PtVertex &ptv,
                         sta::LibertyCell *to_lib_cell) const
{
  sta::LibertyPort *port = ptv.libertyPort();
  if (!port)
    return nullptr;
  if (!to_lib_cell)
    return port;
  return to_lib_cell->findLibertyPort(port->name());
}

float
LocalSta::getVertexMaxSlew(PtGraph *pt_graph, PtVertex &ptv,
                           sta::DcalcAnalysisPt *dcalc_ap)
{
  float max_slew = 0.0;
  for (const RiseFall *rf : RiseFall::range()) {
    float s = pt_graph->slew(ptv, rf, dcalc_ap->index());
    if (s > max_slew)
      max_slew = s;
  }
  return max_slew;
}

bool
LocalSta::checkFanoutLoadSlew(PtGraph *pt_graph, VertexId drvr_id,
                              sta::DcalcAnalysisPt *dcalc_ap)
{
  PtVertexOutEdgeIterator out_iter(drvr_id, pt_graph);
  while (out_iter.hasNext()) {
    PtEdge &pt_edge = out_iter.next();
    if (pt_edge.type() == PtEdgeType::Sentinel || !pt_edge.isWire())
      continue;
    PtVertex &load_ptv = pt_graph->ptVertex(pt_edge.ptToId());
    sta::LibertyPort *load_port = load_ptv.libertyPort();
    if (!load_port) {
      if (const Pin *load_pin = load_ptv.pin()) {
        load_port = network_->libertyPort(load_pin);
      } else {
        // Virtual load without liberty port: skip slew check (assume it's legal).
        continue;
      }
    }
    if (getVertexMaxSlew(pt_graph, load_ptv, dcalc_ap)
        > getPortMaxSlewLimit(load_port))
      return false;
  }
  return true;
}

float
LocalSta::getPortMaxSlewLimit(sta::LibertyPort *port)
{
  if (!port)
    return INF;
  sta::dbNetwork *network = sta_->getDbNetwork();
  odb::dbMTerm *db_mterm = network->staToDb(port);
  if (db_mterm && db_mterm->getSigType().isSupply())
    return INF;
  float max_slew = 0.0;
  bool exists;
  port->slewLimit(MinMax::max(), max_slew, exists);
  if (!exists) {
    sta::LibertyLibrary *lib = port->libertyCell()
        ? port->libertyCell()->libertyLibrary() : nullptr;
    if (lib)
      lib->defaultMaxSlew(max_slew, exists);
    if (!exists)
      max_slew = INF;
  }
  return max_slew;
}

float
LocalSta::getPortMaxCapLimit(sta::LibertyPort *port)
{
  if (!port)
    return INF;
  sta::dbNetwork *network = sta_->getDbNetwork();
  odb::dbMTerm *db_mterm = network->staToDb(port);
  if (db_mterm && db_mterm->getSigType().isSupply())
    return INF;
  float max_cap = 0.0;
  bool exists;
  port->capacitanceLimit(sta::MinMax::max(), max_cap, exists);
  if (!exists) {
    sta::LibertyLibrary *lib = port->libertyCell()
        ? port->libertyCell()->libertyLibrary() : nullptr;
    if (lib)
      lib->defaultMaxCapacitance(max_cap, exists);
    if (!exists)
      max_cap = INF;
  }
  return max_cap;
}

bool
LocalSta::legalCheckBeforeSwap(sta::Instance *inst,
                               sta::LibertyCell *to_lib_cell,
                               const sta::Corner *corner,
                               const sta::MinMax *min_max,
                               PtGraph *pt_graph)
{
  // Check input slew and output load cap legality before swap.
  if (corner == nullptr)
    corner = corners_->findCorner("default");
  if (min_max == nullptr)
    min_max = sta::MinMax::max();
  sta::DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(min_max);

  for (auto &ptv : pt_graph->ptVertices()) {
    if (ptv.type() == PtVertexType::RefOutput) {
      sta::LibertyPort *to_port = findTargetPort(ptv, to_lib_cell);
      if (!to_port)
        continue;
      float cap_limit = getPortMaxCapLimit(to_port);
      float load_cap = getLoadCap(ptv, corner, min_max, pt_graph);
      if (load_cap > cap_limit)
        return false;
    }
  }
  return true;
}

bool
LocalSta::legalCheckAfterSwap(sta::Instance *inst,
                              sta::LibertyCell *to_lib_cell,
                              const sta::Corner *corner,
                              const sta::MinMax *min_max,
                              PtGraph *pt_graph)
{
  // Check output slew and input load cap legality after swap.
  // After virtualReplaceCell + updateRefPorts, RefInput/RefOutput ports
  // already point to the new cell's ports.
  if (corner == nullptr)
    corner = corners_->findCorner("default");
  if (min_max == nullptr)
    min_max = sta::MinMax::max();
  sta::DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(min_max);

  for (auto &ptv : pt_graph->ptVertices()) {
    sta::LibertyPort *port = ptv.libertyPort();
    if (!port)
      continue;

    if (ptv.type() == PtVertexType::RefOutput) {
      // Output slew + fanout load slew
      if (getVertexMaxSlew(pt_graph, ptv, dcalc_ap) > getPortMaxSlewLimit(port))
        return false;
      if (!checkFanoutLoadSlew(pt_graph, ptv.objectIdx(), dcalc_ap))
        return false;
    }
    else if (ptv.type() == PtVertexType::RefDriver) {
      // RefInput slew is checked via checkFanoutLoadSlew below
      // (RefInput is a wire fanout of RefDriver, and its libertyPort
      // is already updated to the new cell's port by updateRefPorts).
      // Driver output cap + slew + fanout load slew (including siblings)
      if (getLoadCap(ptv, corner, min_max, pt_graph) > getPortMaxCapLimit(port))
        return false;
      if (getVertexMaxSlew(pt_graph, ptv, dcalc_ap) > getPortMaxSlewLimit(port))
        return false;
      if (!checkFanoutLoadSlew(pt_graph, ptv.objectIdx(), dcalc_ap))
        return false;
    }
  }
  return true;
}


/////////////////////////////////////////////////////
// LRSInstanceVisitor methods
/////////////////////////////////////////////////////
LRSInstanceVisitor::LRSInstanceVisitor(LocalSta *local_sta) :
  local_sta_(local_sta),
  inst_(nullptr),
  local_graph_(nullptr),
  delay_arc_calc_(local_sta->getSta()->arcDelayCalc()->copy())
{
}

LRSInstanceVisitor::~LRSInstanceVisitor()
{
  delete local_graph_;
  delete delay_arc_calc_;
}

void
LRSInstanceVisitor::visit(Instance *inst)
{
  inst_ = inst;
  local_graph_ = new PtGraph(local_sta_->getSta());
  local_sta_->makePtGraph(local_graph_, inst_);
}

LRSInstanceVisitor 
*LRSInstanceVisitor::copy() const
{
  return new LRSInstanceVisitor(local_sta_);
}

bool
LocalSta::virtualReplaceCell(PtGraph *pt_graph, LibertyCell *new_cell)
{
  // If it's nullptr, we use original ref lib cell of pt graph
  if (new_cell) {
    if (!sta::equivCellsArcs(pt_graph->refGate(), new_cell)) {
      return false;
    }
    pt_graph->setRefGate(new_cell);
    pt_graph->updateTimingArcSets();
    pt_graph->updateRefPorts();
    recomputeLocalParasitics(pt_graph);
  } else {
    if (pt_graph->refGate() == nullptr) {
      throw std::runtime_error("LocalSta::virtualReplaceCell: pt_graph ref gate is nullptr");
    }
  }
  return true;
}

void 
LocalSta::findLocalArrivals(PtGraph *pt_graph)
{
  LocalArrivalVisitor arrival_visitor(this, pt_graph, debug_label_);
  arrival_visitor.findLocalArrivals();
}

void 
LocalSta::findLocalRequireds(PtGraph *pt_graph)
{
  LocalRequiredVisitor required_visitor(this, pt_graph);
  required_visitor.findLocalRequireds();
}

void 
LocalSta::initParallel()
{
  task_arranger_->init();
}

void 
LocalSta::runResize(rsz::Resizer *resizer, ParallelLrVisitor *visitor)
{
  // task_arranger_->enableTopologyCheck(true);
  task_arranger_->visitOrdered(sta_, this, resizer, visitor);
}

void
LocalSta::runResize(rsz::Resizer *resizer, ParallelVisitor *visitor)
{
  task_arranger_->visitOrdered(sta_, this, resizer, visitor);
}

sta::Path *
LocalSta::ptVertexWorstSlackPath(PtVertex &pt_vertex, const sta::MinMax *min_max) const
{
  Path *worst_slack_path = nullptr;
  sta::Slack worst_slack = sta::MinMax::min()->initValue();
  PtVertexPathIterator path_iter(pt_vertex, this);
  while (path_iter.hasNext()) {
    sta::Path *path = path_iter.next();
    const Tag *tag = path->tag(this);
    sta::Slack path_slack = path->slack(this);
    if (tag->pathAnalysisPt(this)->pathMinMax() == min_max
        && (!path->tag(this)->isGenClkSrcPath() 
            && delayLess(path_slack, worst_slack, this))) {
      worst_slack = path_slack;
      worst_slack_path = path;
    }
  }
  return worst_slack_path;
}

} // namespace lrf

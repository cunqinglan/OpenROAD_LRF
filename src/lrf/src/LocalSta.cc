#include "sta/Sta.hh"
#include "sta/Corner.hh"
#include "sta/MinMax.hh"
#include "sta/Graph.hh"
#include "sta/TimingArc.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Delay.hh"
#include "sta/SearchPred.hh"
#include "sta/TimingRole.hh"
#include "sta/ClkNetwork.hh"
#include "LocalParasitics.hh"
#include "sta/Corner.hh"
#include "sta/Parasitics.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "LocalSearch.hh"

namespace lrf {
using namespace sta;


LocalSta::LocalSta(Sta *sta) :
  GraphDelayCalc(sta),
  sta_(sta),
  collected_(false),
  sorted_(false),
  estimate_parasitics_(nullptr),
  local_parasitics_(new LocalParasitics(sta))
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
}

void 
LocalSta::copyState(const Sta *sta)
{
  GraphDelayCalc::copyState(sta);
  sorted_ = false;
}

void 
LocalSta::setParasiticsEst(est::EstimateParasitics *estimate_parasitics) {
  estimate_parasitics_ = estimate_parasitics;
  parasitics_set_ = true;
}

void 
LocalSta::collectLocalGraph(Instance *inst, InstanceSet &local_instances)
{
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
    local_instances.insert(load_inst);
  }
}

void
LocalSta::collectLocalFaninSiblings(Pin *load_pin, PinSet &visited_pins, 
                                 InstanceSet &local_instances)
{
  PinSeq loads, drvrs;
  FindNetDrvrLoads visitor(load_pin, visited_pins, loads, drvrs, network_);
  network_->visitConnectedPins(load_pin, visitor);
  
  for (auto drvr_pin : drvrs) {
    if (drvr_pin == load_pin)
      continue;
    local_instances.insert(network_->instance(drvr_pin));
  }
  for (auto fanin_pins : loads) {
    local_instances.insert(network_->instance(fanin_pins));
  }
}

void
LocalSta::makePtGraph(PtGraph *pt_graph, Instance *inst)
{
  InstanceSet local_instances(sta_->network());
  collectLocalGraph(inst, local_instances);
  pt_graph->makeGraph(local_instances, inst);
}

PtGraph *
LocalSta::makePtGraph(Instance *inst, bool update_timing_first)
{
  PtGraph *pt_graph = new PtGraph(sta_);
  makePtGraph(pt_graph, inst);
  if (update_timing_first) {
    Level top_level = pt_graph->topVertexLevel();
    findDelays(top_level);
    pt_graph->initVertexAndEdges();
  }
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
  recomputeLocalParasitics(pt_graph);
  for (VertexId vertex_id : pt_graph->sortedVertexIds()) {
    findVertexDelays(vertex_id,  arc_delay_calc, pt_graph);
  }
}

void 
LocalSta::seedRootSlew(PtVertex &pt_vertex, PtGraph *pt_graph)
{
  Vertex *vertex = pt_vertex.vertex();
  if (vertex->isDriver(network_)) {
    printf("Local::seedRootSlew seeding drvr slew for vertex %s\n",
           vertex->to_string(sta_).c_str());
    fflush(stdout);
  } else {
    loadSlewFromGraph(pt_vertex, pt_graph);
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
  Vertex *vertex = pt_vertex.vertex();
  if (!pt_vertex.hasFanin()) {
    seedRootSlew(pt_vertex, pt_graph);
  } else {
    Pin *pin = vertex->pin();
    if (network_->isLeaf(pin)) {
      if (vertex->isDriver(network_)) {
        LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(vertex);
        DrvrLoadSlews load_slews_prev;
        // For a gate, compute its delay in different 
        // [arcs, corners, rise/fall].
        findDriverDelays(pt_vertex, arc_delay_calc,
                         load_pin_index_map, pt_graph);
        
      }
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
    Edge *edge = pt_edge.edge();
    Vertex *from_vertex = pt_graph->ptVertex(pt_edge.ptFromId()).vertex();

    if (search_pred_->searchFrom(from_vertex)
	&& search_pred_->searchThru(edge)
        && !edge->role()->isLatchDtoQ())
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
    if (!drvr_vertex->slewAnnotated(rf, slew_min_max)) {
      DcalcAPIndex ap_index = dcalc_ap->index();
      pt_graph->setSlew(drvr_pt_vertex, rf, ap_index, slew_min_max->initValue());
    }
    
    // Init wire delays and slews.
    PtVertexOutEdgeIterator edge_iter(drvr_pt_vertex.objectIdx(), pt_graph);
    while (edge_iter.hasNext()) {
      PtEdge &pt_edge = edge_iter.next();
      Edge *wire_edge = pt_edge.edge();
      if (wire_edge->isWire()) {
        PtVertex &load_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
        Vertex *load_vertex = load_pt_vertex.vertex();
        if (!graph_->wireDelayAnnotated(wire_edge, rf, ap_index)) {
          pt_graph->setWireArcDelay(pt_edge, rf, ap_index, delay_zero);
        }
        if (!load_vertex->slewAnnotated(rf, slew_min_max)) {
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
    printf("LocalSta::findDriverEdgeDelays: timingArcSet is nullptr for edge %s\n",
           pt_edge.edge()->to_string(graph_).c_str());
           fflush(stdout);
    ref_arc_set = pt_edge.edge()->timingArcSet();
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
  MultiDrvrNet *multi_drvr = multiDrvrNet(drvr_vertex);
  LoadPinIndexMap load_pin_index_map = makeLoadPinIndexMap(drvr_vertex);
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
  Instance *drvr_inst = network_->instance(drvr_pt_vertex.vertex()->pin());
  std::string debug_info = "";
  bool debug = false;
  if (std::string(network_->name(drvr_inst)) == "g213657")
    debug = true;
  if (debug) {
   debug_info = std::string("LOCALSTACHECK: Driver Instance: ") 
             + network_->name(drvr_inst)
             + " LibCell: " + pt_graph->refGate()->name()
             + ", Arc: " + arc->to_string() 
             + "DcalcAP: " + std::to_string(dcalc_ap->index());
  }
    

  Vertex *drvr_vertex = drvr_pt_vertex.vertex();
  const RiseFall *from_rf = arc->fromEdge()->asRiseFall();
  const RiseFall *drvr_rf = arc->toEdge()->asRiseFall();
  if (from_rf && drvr_rf) {
    const Pin *drvr_pin = drvr_vertex->pin();
    const Parasitic *parasitic;
    float load_cap;
    localParasiticLoad(drvr_pin, drvr_rf, dcalc_ap, multi_drvr_net, 
                       load_cap, parasitic);
    if (debug) {
      debug_info += ", Load Cap: " + std::to_string(local_parasitics_->capacitance(parasitic) * 1e15) + "fF";
    }

    if (multi_drvr_net == nullptr) {
      PtVertex &from_pt_vertex = pt_graph->ptVertex(pt_edge.ptFromId());
      const Slew in_slew = edgeFromSlew(from_pt_vertex, from_rf, pt_edge, 
                                        dcalc_ap, pt_graph);
      ArcDcalcResult dcalc_result = arc_delay_calc->gateDelay(
                          drvr_pin, arc, in_slew, load_cap, parasitic,
                          load_pin_index_map, dcalc_ap);
      annotateDelaysSlews(pt_edge, arc, dcalc_result,
                          load_pin_index_map, dcalc_ap, pt_graph);
      if (debug) {
        debug_info += ", In Slew: " + std::to_string(1e12 * in_slew)
                      + ", Gate Delay: " + std::to_string(1e12 * (dcalc_result.gateDelay()))
                      + ", Drvr Slew: " + std::to_string(1e12 * (dcalc_result.drvrSlew())) + "\n";
      }
    } else {
      // ArcDcalcArg dcalc_args = makeArcDcalcArgs(drvr_pt_vertex,
                                  // multi_drvr_net, pt_edge, arc,
                                  // dcalc_ap, arc_delay_calc, pt_graph);
      printf("LocalSta::findDriverArcDelays multi-driver net not implemented\n");
      fflush(stdout);
    }
    arc_delay_calc->finishDrvrPin();
  }
  debug_info_.push_back(debug_info);  
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
  Edge *edge = pt_edge.edge();
  if (!edge->role()->isLatchDtoQ()) {
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
    Edge *wire_edge = wire_pt_edge.edge();
    if (wire_edge->isWire()) {
      PtVertex &load_pt_vertex = pt_graph->ptVertex(wire_pt_edge.ptToId());
      Vertex *load_vertex = load_pt_vertex.vertex();
      Pin *load_pin = load_vertex->pin();
      size_t load_idx = load_pin_index_map[load_pin];
      ArcDelay wire_delay = dcalc_result.wireDelay(load_idx);
      Slew load_slew = dcalc_result.loadSlew(load_idx);
      if (!load_vertex->slewAnnotated(to_rf, slew_min_max)) {
    if (drvr_vertex->slewAnnotated(to_rf, slew_min_max)) {
      const Slew &drvr_slew = pt_graph->slew(drvr_pt_vertex, to_rf, ap_index);
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
      if (!graph_->wireDelayAnnotated(wire_edge, to_rf, ap_index)) {
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
  Edge *edge = pt_edge.edge();
  DcalcAPIndex ap_index = dcalc_ap->index();
  PtVertex &drvr_pt_vertex = pt_graph->ptVertex(pt_edge.ptToId());
  Vertex *drvr_vertex = drvr_pt_vertex.vertex();
  const RiseFall *drvr_rf = arc->toEdge()->asRiseFall();
  const Slew drvr_slew = pt_graph->slew(drvr_pt_vertex, drvr_rf, ap_index);
  const MinMax *slew_min_max = dcalc_ap->slewMinMax();
  if (delayGreater(gate_slew, drvr_slew, slew_min_max, this)
      && !drvr_vertex->slewAnnotated(drvr_rf, slew_min_max)
      && !edge->role()->isLatchDtoQ()) 
    pt_graph->setSlew(drvr_pt_vertex, drvr_rf, ap_index, gate_slew);
  if (!graph_->arcDelayAnnotated(pt_edge.edge(), arc, ap_index)) {
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
      load_pin_index_map[load_pin] = load_idx;
      load_idx++;
    }
  }
  return load_pin_index_map;
}

Slew
LocalSta::edgeFromSlew(const PtVertex &from_pt_vertex,
                       const RiseFall *from_rf,
                       const PtEdge &pt_edge,
                       const DcalcAnalysisPt *dcalc_ap,
                       PtGraph *pt_graph)
{
  const Edge *edge = pt_edge.edge();
  return edgeFromSlew(from_pt_vertex, from_rf, edge->role(), 
                      dcalc_ap, pt_graph);
}

Slew
LocalSta::edgeFromSlew(const PtVertex &from_pt_vertex,
                       const RiseFall *from_rf,
                       const TimingRole *role,
                       const DcalcAnalysisPt *dcalc_ap,
                       PtGraph *pt_graph)
{
  Vertex *from_vertex = from_pt_vertex.vertex();
  if (role->genericRole() == TimingRole::regClkToQ()
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
LocalSta::delayLmSum(PtGraph *pt_graph, DcalcAnalysisPt *dcalc_ap)
{
  float delay_lm_sum;
  pt_graph->delayLmSum(dcalc_ap, delay_lm_sum);
  return delay_lm_sum;
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

LocalCost
LocalSta::initAndGetLocalTimingCost(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc)
{
  printf("LocalSta::initAndGetLocalTimingCost computing local delays\n");
  fflush(stdout);
  // During pt graph creation, delays from original graph are copied 
  // to pt graph. So here we just need to sum up the delays.
  const Corner *corner = corners_->findCorner("default");
  DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
  return delayLmSum(pt_graph, dcalc_ap);
}

LocalCost
LocalSta::increAndGetLocalTimingCost(PtGraph *pt_graph, 
                                     ArcDelayCalc *arc_delay_calc,
                                     LibertyCell *equiv_cell)
{
  printf("LocalSta::increAndGetLocalTimingCost recomputing local delays\n");
  fflush(stdout);
  virtualReplaceCell(pt_graph, equiv_cell);
  findLocalDelays(pt_graph, arc_delay_calc);
  findLocalArrivals(pt_graph);
  findLocalRequireds(pt_graph);
  const Corner *corner = corners_->findCorner("default");
  DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(MinMax::max());
  return delayLmSum(pt_graph, dcalc_ap);
}

// Recompute local parasitics after cell swap
void 
LocalSta::recomputeLocalParasitics(PtGraph *pt_graph)
{
  printf("LocalSta::recomputeLocalParasitics recomputing local parasitics\n");
  fflush(stdout);
  local_parasitics_->recomputeLocalParasitics(pt_graph);
}

Slack
LocalSta::localSlackAroundRef(PtGraph *pt_graph)
{
  printf("LocalSta::localSlack computing local slacks\n");
  fflush(stdout);
  Slack local_slack = 0.0;
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.vertex() == nullptr)
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
          local_slack += slack;
        }
      }
    }
  }
  printf("LocalSta::localSlack total local slack = %f\n",
         local_slack * 1.0e12);
  fflush(stdout);
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

void 
LocalSta::localParasiticLoad(const Pin *drvr_pin,
                          const RiseFall *rf,
                          const DcalcAnalysisPt *dcalc_ap,
                          const MultiDrvrNet *multi_drvr_net,
                          // Return values
                          float &load_cap,
                          const Parasitic *&parasitic) const
{
  bool has_net_load;
  float fanout;
  float pin_cap, wire_cap;
  netCaps(drvr_pin, rf, dcalc_ap, multi_drvr_net,
          pin_cap, wire_cap, fanout, has_net_load);

  parasitic = local_parasitics_->findLocalParasitic(drvr_pin, rf, dcalc_ap);
  if (!has_net_load && parasitic != nullptr) {
    if (!local_parasitics_->isPiModel(parasitic)) {
      printf("LocalSta::localParasiticLoad: Non-PI model parasitic found for pin %s\n",
             network_->name(drvr_pin));
      fflush(stdout);
      return;
    }
    load_cap = local_parasitics_->capacitance(parasitic);
  }
  else {
    load_cap = pin_cap + wire_cap;
    throw std::runtime_error("LocalSta::localParasiticLoad: Net load not supported yet");
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
    if (network_->isDriver(vertex->pin())) {
      for (const RiseFall *rf : RiseFall::range()) {
        for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
          const Parasitic *parasitic = 
            local_parasitics_->findLocalParasitic(vertex->pin(), rf, dcalc_ap);
          float load_cap = local_parasitics_->capacitance(parasitic);
          if (parasitic != nullptr) {
            printf("%s::printLocalParasitics: Pin %s, RF %s, AP %u, with cap %f\n",
                   debug_label_.c_str(),
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
LocalSta::printParasitics(PtGraph *pt_graph) const
{
  for (auto& pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.objectIdx() == pt_vertex_id_null)
      continue;
    if (pt_vertex.type() != PtVertexType::RefDriver)
      continue;
    Vertex *vertex = pt_vertex.vertex();
    if (network_->isDriver(vertex->pin())) {
      for (const RiseFall *rf : RiseFall::range()) {
        for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
          const Parasitic *parasitic = 
            arc_delay_calc_->findParasitic(vertex->pin(), rf, dcalc_ap);
          float load_cap = local_parasitics_->capacitance(parasitic);
          if (parasitic != nullptr) {
            printf("OpenSta::printLocalParasitics: Pin %s, RF %s, AP %u, with cap %f\n",
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
    if (pt_vertex.vertex() == nullptr)
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
    if (pt_vertex.vertex() == nullptr)
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
    if (pt_vertex.vertex() == nullptr)
      continue;
    printf("%s::printLocalTiming: Vertex %s\n",
           debug_label_.c_str(),
           pt_vertex.vertex()->to_string(graph_).c_str());
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
    if (pt_vertex.vertex() == nullptr)
      continue;
    for (const RiseFall *rf : RiseFall::range()) {
      for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
        Slew slew = pt_graph->slew(pt_vertex, rf, dcalc_ap->index());
        printf("Vertex %s, RF %s, AP %u, slew = %f\n",
               pt_vertex.vertex()->to_string(graph_).c_str(),
               rf->to_string().c_str(),
               dcalc_ap->index(),
               slew * 1.0e12);
      }
    }
  }
  fflush(stdout);
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

void 
LocalSta::virtualReplaceCell(PtGraph *pt_graph, LibertyCell *new_cell)
{
  pt_graph->setRefGate(new_cell);
  pt_graph->updateTimingArcSets();
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


} // namespace lrf

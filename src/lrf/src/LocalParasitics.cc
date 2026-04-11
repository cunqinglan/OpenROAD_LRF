
#include <cstdio>
#include <mutex>

#include "LocalParasitics.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Corner.hh"
#include "PtGraph.hh"
#include "sta/Sdc.hh"
#include "sta/ClkNetwork.hh"

#include "LocalReduceParasitic.hh"
#include "PtPiElmore.hh"

namespace lrf {

// Global mutex to protect access to OpenDB/STA objects which may not be thread-safe
std::mutex g_odb_sta_access_mutex;

using sta::Parasitic;
using sta::ParasiticNode;
using sta::Pin;
using sta::RiseFall;
using sta::Corner;
using sta::MinMax;
using sta::ParasiticAnalysisPt;
using sta::ConcreteParasitic;
using sta::StaState;

LocalParasitics::LocalParasitics(StaState* state, bool parallelism_exists) :
  ConcreteParasitics(state),
  parallelism_exists_(parallelism_exists),
  copy_helper_(new ParasiticCopyHelper(state))
{
  initParasiticMapFromBase();
}

LocalParasitics::~LocalParasitics()
{
  delete copy_helper_;
}

void
LocalParasitics::initParasiticMapFromBase()
{
  if (!corners_) {
    printf("DEBUG: LocalParasitics::initParasiticMapFromBase: corners_ is null\n");
    fflush(stdout);
    return;
  }

  ConcreteParasitics *global = dynamic_cast<ConcreteParasitics*>(parasitics_);
  if (global != nullptr) {
    global_parasitic_network_map_ = &global->parasitic_network_map_;
  }
}

void
LocalParasitics::recomputePtParasitics(PtGraph *pt_graph)
{
  pt_graph->clearPtParasitics();
  for (const auto &pt_vertex : pt_graph->ptVertices()) {
    if (pt_vertex.type() != PtVertexType::RefDriver
        && pt_vertex.type() != PtVertexType::RefOutput)
      continue;
    if (!pt_vertex.vertex() || !pt_vertex.vertex()->pin())
      continue;
    const Pin *drvr_pin = pt_vertex.vertex()->pin();
    // Ideal clock nets have no parasitic network by design (skipped in
    // EstimateParasitics). Continue silently to avoid spurious errors.
    if (clk_network_->isIdealClock(drvr_pin))
      continue;
    const Net *net = findParasiticNet(drvr_pin);
    for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
      ParasiticAnalysisPt *ap = dcalc_ap->parasiticAnalysisPt();
      Parasitic *parasitic_network = findLocalParasiticNetwork(net, ap);
      if (!parasitic_network)
        continue;
      ParasiticNode *drvr_node =
          parasitics_->findParasiticNode(parasitic_network, drvr_pin);
      if (!drvr_node)
        continue;
      for (const RiseFall *rf : RiseFall::range()) {
        PtPiElmore &pt_pi = pt_graph->makePtParasitic(
            pt_vertex.objectIdx(), rf, dcalc_ap->index());
        pt_pi.clear();
        LocalReduceToPiElmore reducer(this, pt_graph);
        reducer.makePtPiElmore(parasitic_network, drvr_pin, drvr_node,
                               ap->couplingCapFactor(), rf,
                               dcalc_ap->corner(),
                               dcalc_ap->constraintMinMax(), ap,
                               pt_pi);
      }
    }
  }
}

void
LocalParasitics::recomputeSinglePtParasitic(PtGraph *pt_graph, VertexId drvr_vid)
{
  pt_graph->clearPtParasitics(drvr_vid);
  const PtVertex &pt_vertex = pt_graph->ptVertex(drvr_vid);
  if (!pt_vertex.vertex() || !pt_vertex.vertex()->pin())
    return;
  const Pin *drvr_pin = pt_vertex.vertex()->pin();
  // Ideal clock nets have no parasitic network by design (skipped in
  // EstimateParasitics). Return silently to avoid spurious errors.
  if (clk_network_->isIdealClock(drvr_pin))
    return;
  const Net *net = findParasiticNet(drvr_pin);
  for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
    ParasiticAnalysisPt *ap = dcalc_ap->parasiticAnalysisPt();
    Parasitic *parasitic_network = findLocalParasiticNetwork(net, ap);
    if (!parasitic_network)
      continue;
    ParasiticNode *drvr_node =
        parasitics_->findParasiticNode(parasitic_network, drvr_pin);
    if (!drvr_node)
      continue;
    for (const RiseFall *rf : RiseFall::range()) {
      PtPiElmore &pt_pi = pt_graph->makePtParasitic(
          drvr_vid, rf, dcalc_ap->index());
      pt_pi.clear();
      LocalReduceToPiElmore reducer(this, pt_graph);
      reducer.makePtPiElmore(parasitic_network, drvr_pin, drvr_node,
                             ap->couplingCapFactor(), rf,
                             dcalc_ap->corner(),
                             dcalc_ap->constraintMinMax(), ap,
                             pt_pi);
    }
  }
}


Parasitic *
LocalParasitics::findLocalParasiticNetwork(const Net *net, const ParasiticAnalysisPt *ap) const
{
  if (global_parasitic_network_map_ && !global_parasitic_network_map_->empty()) {
    ConcreteParasiticNetwork **parasitic_array =
      global_parasitic_network_map_->findKey(net);
    if (!parasitic_array) {
      const char *unconnected_net_name = "UNCONNECTED";
      if (!network_->name(net) || !strstr(network_->name(net), unconnected_net_name)) {
        printf("Error: LocalParasitics::findLocalParasiticNetwork: No parasitic array found for net %s\n",
                network_->name(net));
        fflush(stdout);
      }
      return nullptr;
    }
    ConcreteParasiticNetwork *parasitic = parasitic_array[ap->index()];
    if (!parasitic) {
      parasitic = parasitic_array[ap->indexMax()];
      if (parasitic == nullptr) {
        printf("Error: LocalParasitics::findLocalParasiticNetwork: No parasitic found for net %s\n",
               network_->name(net));
        fflush(stdout);
        return nullptr;
      }
    }
    return parasitic;
  }
  printf("Error: LocalParasitics::findLocalParasiticNetwork: global parasitic network map is null or empty\n");
  fflush(stdout);
  return nullptr;
}

float
LocalParasitics::pinCapacitance(const Pin *pin,
                                const RiseFall *rf,
                                const Corner *corner,
                                const MinMax *min_max) const
{
  float pin_cap = 0.0;
  if (pin) {
    Port *port = network_->port(pin);
    LibertyPort *lib_port = network_->libertyPort(port);
    if (lib_port) {
      pin_cap = sdc_->pinCapacitance(pin, rf, corner, min_max);
    }
    else if (network_->isTopLevelPort(pin))
      pin_cap = sdc_->portExtCap(port, rf, corner, min_max);
  }
  return pin_cap;
}

float
LocalParasitics::pinCapacitance(const ParasiticNode *node,
                                const RiseFall *rf,
                                const Corner *corner,
                                const MinMax *min_max) const
{
  const Pin *pin = parasitics_->pin(node);
  float pin_cap = 0.0;
  if (pin) {
    Port *port = network_->port(pin);
    LibertyPort *lib_port = network_->libertyPort(port);
    if (lib_port) {
      pin_cap = sdc_->pinCapacitance(pin, rf, corner, min_max);
    }
    else if (network_->isTopLevelPort(pin))
      pin_cap = sdc_->portExtCap(port, rf, corner, min_max);
  }
  return pin_cap;
}

////////////////////////////////////////////////////////
// Functions for ParasiticCopyHelper
////////////////////////////////////////////////////////
ParasiticCopyHelper::ParasiticCopyHelper(StaState* state) : StaState(state)
{
}

Parasitic *
ParasiticCopyHelper::getCopy(const Parasitic *from_parasitic)
{
  return new Parasitic();
}

ConcreteParasitic *
ParasiticCopyHelper::getCopy(const ConcreteParasitic *from_parasitic)
{
  return nullptr;
}

ConcretePiElmore *
ParasiticCopyHelper::getCopy(const ConcretePiElmore *from_parasitic)
{
  ConcretePiElmore *copy = new ConcretePiElmore(from_parasitic->c2(),
                                                from_parasitic->rpi(),
                                                from_parasitic->c1());
  for (const auto &entry : from_parasitic->loads()) {
    const Pin *load_pin = entry.first;
    float load_cap = entry.second;
    copy->setElmore(load_pin, load_cap);
  }
  return copy;
}

}  // namespace lrf

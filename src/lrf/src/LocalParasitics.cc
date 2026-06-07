
#include <cstdio>
#include <mutex>

#include "LocalParasitics.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"
#include "sta/Scene.hh"
#include "PtGraph.hh"
#include "sta/Sdc.hh"
#include "sta/ClkNetwork.hh"
#include "sta/Mode.hh"

#include "LocalReduceParasitic.hh"
#include "PtPiElmore.hh"
#include "PtElmoreCeff.hh"
#include "LocalDmpDelayCalc.hh"  // useElmoreCeff()

namespace lrf {

// Global mutex to protect access to OpenDB/STA objects which may not be thread-safe
std::mutex g_odb_sta_access_mutex;

using sta::Parasitic;
using sta::ParasiticNode;
using sta::Pin;
using sta::RiseFall;
using sta::Scene;
using sta::MinMax;
using sta::ConcreteParasitic;
using sta::StaState;

LocalParasitics::LocalParasitics(StaState* state, bool parallelism_exists) :
  ConcreteParasitics("LocalParasitics", "", state),
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
  // No-op: parasitics are now per-Scene (scene->parasitics(min_max)); the old
  // global ConcreteParasiticNetworkMap cache is gone.
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
    if (modes()[0]->clkNetwork()->isIdealClock(drvr_pin))
      continue;
    const Net *net = findParasiticNet(drvr_pin);
    for (sta::Scene *scene : scenes()) {
     for (const MinMax *min_max : MinMax::range()) {
      sta::DcalcAPIndex ap_index = scene->dcalcAnalysisPtIndex(min_max);
      Parasitic *parasitic_network = findLocalParasiticNetwork(net, scene, min_max);
      if (!parasitic_network)
        continue;
      ParasiticNode *drvr_node =
          this->findParasiticNode(parasitic_network, drvr_pin);
      if (!drvr_node)
        continue;
      const float coupling = scene->parasitics(min_max)->couplingCapFactor();
      for (const RiseFall *rf : RiseFall::range()) {
        LocalReduceToPiElmore reducer(this, pt_graph);
        if (useElmoreCeff()) {
          // EC-only path: skip PtPiElmore allocation entirely.
          PtElmoreCeff &pt_ec = pt_graph->makePtElmoreCeff(
              pt_vertex.objectIdx(), rf, ap_index);
          reducer.makePtElmoreCeffOnly(parasitic_network, drvr_pin, drvr_node,
                                        coupling, rf,
                                        scene, min_max,
                                        pt_ec);
        }
        else {
          PtPiElmore &pt_pi = pt_graph->makePtParasitic(
              pt_vertex.objectIdx(), rf, ap_index);
          pt_pi.clear();
          reducer.makePtPiElmore(parasitic_network, drvr_pin, drvr_node,
                                 coupling, rf,
                                 scene, min_max,
                                 pt_pi);
        }
      }
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
  if (modes()[0]->clkNetwork()->isIdealClock(drvr_pin))
    return;
  const Net *net = findParasiticNet(drvr_pin);
  for (sta::Scene *scene : scenes()) {
   for (const MinMax *min_max : MinMax::range()) {
    sta::DcalcAPIndex ap_index = scene->dcalcAnalysisPtIndex(min_max);
    Parasitic *parasitic_network = findLocalParasiticNetwork(net, scene, min_max);
    if (!parasitic_network)
      continue;
    ParasiticNode *drvr_node =
        this->findParasiticNode(parasitic_network, drvr_pin);
    if (!drvr_node)
      continue;
    const float coupling = scene->parasitics(min_max)->couplingCapFactor();
    for (const RiseFall *rf : RiseFall::range()) {
      LocalReduceToPiElmore reducer(this, pt_graph);
      if (useElmoreCeff()) {
        // EC-only: skip PtPiElmore alloc / Pi math.
        PtElmoreCeff &pt_ec = pt_graph->makePtElmoreCeff(
            drvr_vid, rf, ap_index);
        reducer.makePtElmoreCeffOnly(parasitic_network, drvr_pin, drvr_node,
                                      coupling, rf,
                                      scene, min_max,
                                      pt_ec);
      }
      else {
        PtPiElmore &pt_pi = pt_graph->makePtParasitic(
            drvr_vid, rf, ap_index);
        pt_pi.clear();
        reducer.makePtPiElmore(parasitic_network, drvr_pin, drvr_node,
                               coupling, rf,
                               scene, min_max,
                               pt_pi);
      }
    }
   }
  }
}


PtPiElmore *
LocalParasitics::ensurePtPiElmore(PtGraph *pt_graph,
                                  VertexId drvr_vid,
                                  const RiseFall *rf,
                                  int ap_index)
{
  // Fast path: already built (e.g. env=off main reduce path, or a previous
  // Bakoglu hit on this net within the same LR iteration).
  if (PtPiElmore *existing =
          pt_graph->findPtParasitic(drvr_vid, rf, ap_index))
    return existing;

  // env=on main reduce skips Pi build (only PtElmoreCeff). Bakoglu needs
  // Pi parameters (rpi/c1+c2 + per-load Elmore). Build it lazily for just
  // this (vid, rf, ap_index) so the rest of env=on stays Pi-free.
  const PtVertex &pt_vertex = pt_graph->ptVertex(drvr_vid);
  if (!pt_vertex.vertex() || !pt_vertex.vertex()->pin())
    return nullptr;
  const Pin *drvr_pin = pt_vertex.vertex()->pin();
  if (modes()[0]->clkNetwork()->isIdealClock(drvr_pin))
    return nullptr;

  // Recover the (scene, min_max) whose dcalc analysis point index == ap_index.
  // Parasitics are per-scene now; scan scenes x min/max. Cheap.
  sta::Scene *found_scene = nullptr;
  const MinMax *found_mm = nullptr;
  for (sta::Scene *scene : scenes()) {
    for (const MinMax *mm : MinMax::range()) {
      if (scene->dcalcAnalysisPtIndex(mm) == ap_index) {
        found_scene = scene;
        found_mm = mm;
        break;
      }
    }
    if (found_scene) break;
  }
  if (!found_scene) return nullptr;

  const Net *net = findParasiticNet(drvr_pin);
  Parasitic *parasitic_network = findLocalParasiticNetwork(net, found_scene, found_mm);
  if (!parasitic_network) return nullptr;

  ParasiticNode *drvr_node =
      this->findParasiticNode(parasitic_network, drvr_pin);
  if (!drvr_node) return nullptr;

  const float coupling = found_scene->parasitics(found_mm)->couplingCapFactor();
  PtPiElmore &pt_pi = pt_graph->makePtParasitic(drvr_vid, rf, ap_index);
  pt_pi.clear();
  LocalReduceToPiElmore reducer(this, pt_graph);
  reducer.makePtPiElmore(parasitic_network, drvr_pin, drvr_node,
                         coupling, rf,
                         found_scene, found_mm,
                         pt_pi);
  return &pt_pi;
}

Parasitic *
LocalParasitics::findLocalParasiticNetwork(const Net *net, const Scene *scene, const MinMax *min_max) const
{
  // Parasitics are per-scene (scene->parasitics(min_max) holds one
  // ConcreteParasiticNetwork per net). Look it up directly.
  sta::Parasitics *par = scene->parasitics(min_max);
  if (par) {
    return par->findParasiticNetwork(net);
  }
  return nullptr;
}

float
LocalParasitics::pinCapacitance(const Pin *pin,
                                const RiseFall *rf,
                                const Scene *corner,
                                const MinMax *min_max) const
{
  float pin_cap = 0.0;
  if (pin) {
    Port *port = network_->port(pin);
    LibertyPort *lib_port = network_->libertyPort(port);
    if (lib_port) {
      pin_cap = corner->sdc()->pinCapacitance(pin, rf, corner, min_max);
    }
    else if (network_->isTopLevelPort(pin))
      pin_cap = corner->sdc()->portExtCap(port, rf, min_max);
  }
  return pin_cap;
}

float
LocalParasitics::pinCapacitance(const ParasiticNode *node,
                                const RiseFall *rf,
                                const Scene *corner,
                                const MinMax *min_max) const
{
  const Pin *pin = this->pin(node);
  float pin_cap = 0.0;
  if (pin) {
    Port *port = network_->port(pin);
    LibertyPort *lib_port = network_->libertyPort(port);
    if (lib_port) {
      pin_cap = corner->sdc()->pinCapacitance(pin, rf, corner, min_max);
    }
    else if (network_->isTopLevelPort(pin))
      pin_cap = corner->sdc()->portExtCap(port, rf, min_max);
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

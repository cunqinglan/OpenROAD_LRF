
#include <cstdio>
#include <mutex>

#include "LocalParasitics.hh"
#include "parasitics/ReduceParasitics.hh"
#include "parasitics/ConcreteParasitics.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Corner.hh"
#include "PtGraph.hh"
#include "sta/Sdc.hh"

#include "LocalReduceParasitic.hh"
#include "LocalParasitics.hh"
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
  // Pre-assign space for local driver parasitics
  initParasiticMapFromBase();
}

LocalParasitics::~LocalParasitics()
{
  for (auto &entry : local_drvr_parasitic_map_) {
    ConcreteParasitic **parasitic_array = entry.second;
    int ap_count = corners_->parasiticAnalysisPtCount();
    int ap_rf_count = ap_count * RiseFall::index_count;
    for (int i = 0; i < ap_rf_count; i++) {
      delete parasitic_array[i];
    }
    delete [] parasitic_array;
  }
  delete copy_helper_;
}

void 
LocalParasitics::initParasiticMapFromBase() 
{
  if (!corners_) {
    // printf("DEBUG: LocalParasitics::initParasiticMapFromBase: corners_ is null\n");
    // fflush(stdout);
    return;
  }

  ConcreteParasitics *global = dynamic_cast<ConcreteParasitics*>(parasitics_);

  if (global != nullptr) {
    if (!global->drvr_parasitic_map_.empty()) {
      for (const auto& [pin, array] : global->drvr_parasitic_map_) {
        int ap_count = corners_->parasiticAnalysisPtCount();
        int ap_rf_count = ap_count * RiseFall::index_count;
        ConcreteParasitic **local_array = new ConcreteParasitic*[ap_rf_count];
        for (int i = 0; i < ap_rf_count; i++){
          if (array && array[i]) {
            if (array[i]->isPiElmore()) {
              ConcretePiElmore *pi_elmore = dynamic_cast<ConcretePiElmore*>(array[i]);
              if (pi_elmore && copy_helper_)
                  local_array[i] = copy_helper_->getCopy(pi_elmore);
              else 
                  local_array[i] = nullptr;
            } else {
               local_array[i] = nullptr;
            }
          } else
            local_array[i] = nullptr;
        }
        local_drvr_parasitic_map_[pin] = local_array;
      }
    }
    local_parasitic_network_map_ = global->parasitic_network_map_;
  }
}

Parasitic *
LocalParasitics::reduceToLocalPiElmore(const Parasitic *parasitic_network,
                                       const Pin *drvr_pin,
                                       const RiseFall *rf,
                                       const Corner *corner,
                                       const PtGraph *pt_graph,
                                //  In fact, cnst_min_max = dcalc.minmax()
                                //  ap = dcalc.parasiticAnalysisPt()
                                       const MinMax *cnst_min_max,
                                       const ParasiticAnalysisPt *ap)
{
  ParasiticNode *drvr_node =
    parasitics_->findParasiticNode(parasitic_network, drvr_pin);
  if (drvr_node) {
    return makeLocalPiElmore(parasitic_network, drvr_pin, drvr_node,
                             ap->couplingCapFactor(), rf,
                             corner,  pt_graph, cnst_min_max, ap);
  }
  return nullptr;
}

Parasitic *
LocalParasitics::makeLocalPiElmore(const Parasitic *parasitic_network,
                                   const Pin *drvr_pin,
                                   ParasiticNode *drvr_node,
                                   float coupling_cap_factor,
                                   const RiseFall *rf,
                                   const Corner *corner,
                                   const PtGraph *pt_graph,
                                   const MinMax *min_max,
                                   const ParasiticAnalysisPt *ap)
{
  // Protect access to OpenDB/STA network objects which may have internal state
  std::lock_guard<std::mutex> lock(g_odb_sta_access_mutex);
  float c2, rpi, c1;
  LocalReduceToPiElmore reducer(this, pt_graph);
  reducer.reduceToPi(parasitic_network, drvr_pin, drvr_node,
                     coupling_cap_factor, rf, corner, min_max, ap,
                     c2, rpi, c1);

  ConcreteParasitic **parasitic_array = 
    local_drvr_parasitic_map_.findKey(drvr_pin);
  if (!parasitic_array) {
    if (parallelism_exists_) {
      throw std::runtime_error("Error: LocalParasitics::makeLocalPiElmore: Do not use it where parallelism exists\n");
    }
    int ap_count = corners_->parasiticAnalysisPtCount();
    int ap_rf_count = ap_count * RiseFall::index_count;
    parasitic_array = new ConcreteParasitic*[ap_rf_count];
    for (int i = 0; i < ap_rf_count; i++)
      parasitic_array[i] = nullptr;
    local_drvr_parasitic_map_[drvr_pin] = parasitic_array;
  }
  int ap_rf_index = parasiticAnalysisPtIndex(ap, rf);
  ConcreteParasitic *existing_parasitic = parasitic_array[ap_rf_index];
  ConcretePiElmore *local_pi_elmore = nullptr;
  if (existing_parasitic) {
    if (!existing_parasitic->isPiElmore()) {
      printf("Error: Existing parasitic is not PiElmore\n");
      fflush(stdout);
      return nullptr;
    }
    local_pi_elmore = dynamic_cast<ConcretePiElmore*>(existing_parasitic);
    local_pi_elmore->setPiModel(c2, rpi, c1);
  }
  else {
    local_pi_elmore = new ConcretePiElmore(c2, rpi, c1);
    parasitic_array[ap_rf_index] = local_pi_elmore;
  }
  
  reducer.reduceElmoreDfs(drvr_pin, drvr_node, 0, 0.0, local_pi_elmore);
  return local_pi_elmore;
}

void 
LocalParasitics::recomputeLocalParasitics(PtGraph *pt_graph)
{
  for (const auto &pt_vertex: pt_graph->ptVertices()) {
    if (pt_vertex.type() == PtVertexType::RefDriver
        || pt_vertex.type() == PtVertexType::RefOutput) {
      const Net *net = findParasiticNet(pt_vertex.vertex()->pin());
      for (const DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
    ParasiticAnalysisPt *ap = dcalc_ap->parasiticAnalysisPt();
    // Use global parasitics to find the parasitic network
    Parasitic *drvr_parsitic_network = 
        findLocalParasiticNetwork(net, ap);
    if (drvr_parsitic_network) {
      reduceLocalParasitic(drvr_parsitic_network, pt_graph, pt_vertex, 
                           dcalc_ap);
    } else {
      // For rst nets, PI nets or special nets, there may be no parasitic network
      // if (network_->name(net) != "(null)")
      // printf("Warning: LocalParasitics::recomputeLocalParasitics: No parasitic network found for driver net %s.\n"
      //         "              This net drives vertex %s\n",
      //        network_->name(net),
      //        network_->name(pt_vertex.vertex()->pin()));
      // fflush(stdout);
    }
      }
    }
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
  if (!local_parasitic_network_map_.empty()) {
    ConcreteParasiticNetwork **parasitic_array = 
      local_parasitic_network_map_.findKey(net);
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
  printf("Error: LocalParasitics::findLocalParasiticNetwork: local_parasitic_network_map_ is empty\n");
  fflush(stdout);
  return nullptr;
}

void 
LocalParasitics::reduceLocalParasitic(
                          Parasitic* parasitic_network,
                          PtGraph *pt_graph, 
                          const PtVertex &pt_drvr_vertex,
                          const DcalcAnalysisPt *dcalc_ap)
{
  ParasiticAnalysisPt *ap = dcalc_ap->parasiticAnalysisPt();
  const Pin *drvr_pin = pt_drvr_vertex.vertex()->pin();
  for (const RiseFall *rf : RiseFall::range()) {
    reduceToLocalPiElmore(parasitic_network, drvr_pin, rf,
                          dcalc_ap->corner(), 
                          pt_graph, dcalc_ap->constraintMinMax(), ap);
  }
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

Parasitic *
LocalParasitics::findLocalParasitic(const Pin *drvr_pin, const RiseFall *rf, const DcalcAnalysisPt *ap)
{
  ParasiticAnalysisPt *parasitic_ap = ap->parasiticAnalysisPt();
  if (!local_drvr_parasitic_map_.empty()) {
    int ap_rf_index = parasiticAnalysisPtIndex(parasitic_ap, rf);
    ConcreteParasitic **parasitic_array = 
      local_drvr_parasitic_map_.findKey(drvr_pin);
    
    if (!parasitic_array) {
      // printf("Error: LocalParasitics::findLocalParasitic: No parasitic array found for driver pin %s\n",
      //        network_->name(drvr_pin));
      fflush(stdout);
      return nullptr;
    }
    ConcreteParasitic *parasitic = parasitic_array[ap_rf_index];
    if (!parasitic) {
      printf("Error: LocalParasitics::findLocalParasitic: No parasitic found for driver pin %s\n",
             network_->name(drvr_pin));
      fflush(stdout);
      return nullptr;
    }
    if (!parasitic->isPiElmore()) {
      printf("Error: LocalParasitics::findLocalParasitic: Parasitic is not PiElmore for driver pin %s\n",
             network_->name(drvr_pin));
      fflush(stdout);
      return nullptr;
    }
    return parasitic;
  }
  return nullptr;
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

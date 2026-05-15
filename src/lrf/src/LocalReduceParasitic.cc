// OpenSTA, Static Timing Analyzer
// Copyright (c) 2025, Parallax Software, Inc.
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
// 
// The origin of this software must not be misrepresented; you must not
// claim that you wrote the original software.
// 
// Altered source versions must be plainly marked as such, and must not be
// misrepresented as being the original software.
// 
// This notice may not be removed or altered from any source distribution.

#include "LocalReduceParasitic.hh"

#include "sta/Error.hh"
#include "sta/Debug.hh"
#include "sta/MinMax.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/Sdc.hh"
#include "sta/Scene.hh"
#include "sta/Parasitics.hh"
#include "PtGraph.hh"
#include "PtPiElmore.hh"

namespace lrf {

using std::max;
using namespace sta;

LocalReduceToPi::LocalReduceToPi(StaState *sta, const PtGraph *pt_graph) :
  StaState(sta),
  coupling_cap_multiplier_(1.0),
  rf_(nullptr),
  corner_(nullptr),
  min_max_(nullptr),
  pin_caps_one_value_(true),
  pt_graph_(pt_graph)
{
}

// "Modeling the Driving-Point Characteristic of Resistive
// Interconnect for Accurate Delay Estimation", Peter O'Brien and
// Thomas Savarino, Proceedings of the 1989 Design Automation
// Conference.
void
LocalReduceToPi::reduceToPi(const Parasitic *parasitic_network,
                       const Pin *drvr_pin,
		       ParasiticNode *drvr_node,
		       float coupling_cap_factor,
		       const RiseFall *rf,
		       const Scene *corner,
		       const MinMax *min_max,
		       float &c2,
		       float &rpi,
		       float &c1)
{
  corner_ = corner;
  min_max_ = min_max;
  parasitics_ = corner->parasitics(min_max);
  includes_pin_caps_ = parasitics_->includesPinCaps(parasitic_network),
  coupling_cap_multiplier_ = coupling_cap_factor;
  rf_ = rf;
  resistor_map_ = parasitics_->parasiticNodeResistorMap(parasitic_network);
  capacitor_map_ = parasitics_->parasiticNodeCapacitorMap(parasitic_network);

  double y1, y2, y3, dcap;
  double max_resistance = 0.0;
  reducePiDfs(drvr_pin, drvr_node, nullptr, 0.0,
              y1, y2, y3, dcap, max_resistance);

  if (y2 == 0.0 && y3 == 0.0) {
    // Capacitive load.
    c1 = y1;
    c2 = 0.0;
    rpi = 0.0;
  }
  else {
    c1 = y2 * y2 / y3;
    c2 = y1 - y2 * y2 / y3;
    rpi = -y3 * y3 / (y2 * y2 * y2);
  }
  debugPrint(debug_, "parasitic_reduce", 2,
             " Pi model c2=%.3g rpi=%.3g c1=%.3g max_r=%.3g",
             c2, rpi, c1, max_resistance);
}

// Find admittance moments.
void
LocalReduceToPi::reducePiDfs(const Pin *drvr_pin,
			ParasiticNode *node,
			ParasiticResistor *from_res,
			double src_resistance,
			double &y1,
			double &y2,
			double &y3,
			double &dwn_cap,
                        double &max_resistance)
{
  double coupling_cap = 0.0;
  ParasiticCapacitorSeq &capacitors = capacitor_map_[node];
  for (ParasiticCapacitor *capacitor : capacitors)
    coupling_cap += parasitics_->value(capacitor);
  dwn_cap = parasitics_->nodeGndCap(node)
    + coupling_cap * coupling_cap_multiplier_
    + localPinCapacitance(node);

  y1 = dwn_cap;
  y2 = y3 = 0.0;
  max_resistance = max(max_resistance, src_resistance);

  visit(node);
  ParasiticResistorSeq &resistors = resistor_map_[node];
  for (ParasiticResistor *resistor : resistors) {
    if (!isLoopResistor(resistor)) {
      ParasiticNode *onode = parasitics_->otherNode(resistor, node);
      // One commercial extractor creates resistors with identical from/to nodes.
      if (onode != node
          && resistor != from_res) {
        if (isVisited(onode)) {
          // Resistor loop.
          debugPrint(debug_, "parasitic_reduce", 2, " loop detected thru resistor %zu",
                     parasitics_->id(resistor));
          markLoopResistor(resistor);
        }
        else {
          double r = parasitics_->value(resistor);
          double yd1, yd2, yd3, dcap;
          reducePiDfs(drvr_pin, onode, resistor, src_resistance + r,
                      yd1, yd2, yd3, dcap, max_resistance);
          // Rule 3.  Upstream traversal of a series resistor.
          // Rule 4.  Parallel admittances add.
          y1 += yd1;
          y2 += yd2 - r * yd1 * yd1;
          y3 += yd3 - 2 * r * yd1 * yd2 + r * r * yd1 * yd1 * yd1;
          dwn_cap += dcap;
        }
      }
    }
  }

  setDownstreamCap(node, dwn_cap);
  leave(node);
  debugPrint(debug_, "parasitic_reduce", 3,
             " node %s y1=%.3g y2=%.3g y3=%.3g cap=%.3g",
             parasitics_->name(node), y1, y2, y3, dwn_cap);
}

float
LocalReduceToPi::pinCapacitance(ParasiticNode *node)
{
  const Pin *pin = parasitics_->pin(node);
  float pin_cap = 0.0;
  if (pin) {
    Port *port = network_->port(pin);
    LibertyPort *lib_port = network_->libertyPort(port);
    if (lib_port) {
      if (!includes_pin_caps_) {
	pin_cap = corner_->sdc()->pinCapacitance(pin, rf_, corner_, min_max_);
	pin_caps_one_value_ &= lib_port->capacitanceIsOneValue();
      }
    }
    else if (network_->isTopLevelPort(pin))
      pin_cap = corner_->sdc()->portExtCap(port, rf_, corner_, min_max_);
  }
  return pin_cap;
}

float
LocalReduceToPi::localPinCapacitance(ParasiticNode *node)
{
  const Pin *pin = parasitics_->pin(node);
  float pin_cap = 0.0;

  if (pin) {
    // Top-level ports have no liberty cell; handle them via SDC directly.
    if (network_->isTopLevelPort(pin)) {
      Port *port = network_->port(pin);
      if (port)
        pin_cap = corner_->sdc()->portExtCap(port, rf_, corner_, min_max_);
      return pin_cap;
    }
    // Safety: check vertexId before calling pinLoadVertex.
    // Pins without a valid vertex (unconnected, hierarchical, or stale after undoEco)
    // would cause pinLoadVertex → ObjectTable::pointer(null) to segfault.
    sta::VertexId vid = network_->vertexId(pin);
    if (vid == sta::object_id_null) {
      return pin_cap;
    }
    sta::Vertex *sta_vtx = graph_->vertex(vid);
    
    const PtVertex *pt_vp = (sta_vtx && pt_graph_)
        ? pt_graph_->ptVertex(sta_vtx) : nullptr;
    if (pt_vp) {
      // Instance pins in PtGraph: use cached liberty port,
      // avoiding network_->port() which can crash on stale pins.
      if (!includes_pin_caps_) {
        pin_cap = pt_graph_->getRefPinCapacitance(*pt_vp, rf_, corner_, min_max_);
        sta::LibertyPort *lp = pt_vp->libertyPort();
        if (lp)
          pin_caps_one_value_ &= lp->capacitanceIsOneValue();
      }
    }
    else {
      // Pin not in PtGraph (should be rare after collectLocal fix).
      Port *port = network_->port(pin);
      if (!port)
        return pin_cap;
      LibertyPort *lib_port = network_->libertyPort(port);
      if (lib_port) {
        if (!includes_pin_caps_) {
          pin_cap = corner_->sdc()->pinCapacitance(pin, rf_, corner_, min_max_);
          pin_caps_one_value_ &= lib_port->capacitanceIsOneValue();
        }
      }
    }
  }
  return pin_cap;
}

void
LocalReduceToPi::visit(ParasiticNode *node)
{
  visited_nodes_.insert(node);
}

bool
LocalReduceToPi::isVisited(ParasiticNode *node)
{
  return visited_nodes_.hasKey(node);
}

void
LocalReduceToPi::leave(ParasiticNode *node)
{
  visited_nodes_.erase(node);
}

bool
LocalReduceToPi::isLoopResistor(ParasiticResistor *resistor)
{
  return loop_resistors_.hasKey(resistor);
}

void
LocalReduceToPi::markLoopResistor(ParasiticResistor *resistor)
{
  loop_resistors_.insert(resistor);
}

void
LocalReduceToPi::setDownstreamCap(ParasiticNode *node,
			     float cap)
{
  node_values_[node] = cap;
}

float
LocalReduceToPi::downstreamCap(ParasiticNode *node)
{
  return node_values_[node];
}

////////////////////////////////////////////////////////////////

LocalReduceToPiElmore::LocalReduceToPiElmore(StaState *sta, const PtGraph *pt_graph) :
  LocalReduceToPi(sta, pt_graph)
{
}

Parasitic *
LocalReduceToPiElmore::makePiElmore(const Parasitic *parasitic_network,
			       const Pin *drvr_pin,
			       ParasiticNode *drvr_node,
			       float coupling_cap_factor,
			       const RiseFall *rf,
			       const Scene *corner,
			       const MinMax *min_max)
{
  float c2, rpi, c1;
  reduceToPi(parasitic_network, drvr_pin, drvr_node, coupling_cap_factor,
             rf, corner, min_max, c2, rpi, c1);
  Parasitic *pi_elmore = parasitics_->makePiElmore(drvr_pin, rf, min_max,
						   c2, rpi, c1);
  parasitics_->setIsReducedParasiticNetwork(pi_elmore, true);
  reduceElmoreDfs(drvr_pin, drvr_node, 0, 0.0, pi_elmore);
  return pi_elmore;
}

// Find elmore delays on 2nd DFS search using downstream capacitances
// set by reducePiDfs.
void
LocalReduceToPiElmore::reduceElmoreDfs(const Pin *drvr_pin,
				  ParasiticNode *node,
				  ParasiticResistor *from_res,
				  double elmore,
				  Parasitic *pi_elmore)
{
  const Pin *pin = parasitics_->pin(node);
  if (from_res && pin) {
    if (network_->isLoad(pin)) {
      debugPrint(debug_, "parasitic_reduce", 2, " Load %s elmore=%.3g",
                 network_->pathName(pin),
                 elmore);
      parasitics_->setElmore(pi_elmore, pin, elmore);
    }
  }
  visit(node);
  ParasiticResistorSeq &resistors = resistor_map_[node];
  for (ParasiticResistor *resistor : resistors) {
    ParasiticNode *onode = parasitics_->otherNode(resistor, node);
    if (resistor != from_res
        && !isVisited(onode)
        && !isLoopResistor(resistor)) {
      float r = parasitics_->value(resistor);
      double onode_elmore = elmore + r * downstreamCap(onode);
      reduceElmoreDfs(drvr_pin, onode, resistor, onode_elmore, pi_elmore);
    }
  }
  leave(node);
}

////////////////////////////////////////////////////////////////

void
LocalReduceToPiElmore::makePtPiElmore(const Parasitic *parasitic_network,
                                      const Pin *drvr_pin,
                                      ParasiticNode *drvr_node,
                                      float coupling_cap_factor,
                                      const RiseFall *rf,
                                      const Scene *corner,
                                      const MinMax *min_max,
                                      PtPiElmore &result)
{
  float c2, rpi, c1;
  reduceToPi(parasitic_network, drvr_pin, drvr_node, coupling_cap_factor,
             rf, corner, min_max, c2, rpi, c1);
  result.setPiModel(c2, rpi, c1);
  reduceElmoreDfsToPt(drvr_pin, drvr_node, nullptr, 0.0, result);
}

void
LocalReduceToPiElmore::reduceElmoreDfsToPt(const Pin *drvr_pin,
                                           ParasiticNode *node,
                                           ParasiticResistor *from_res,
                                           double elmore,
                                           PtPiElmore &result)
{
  const Pin *pin = parasitics_->pin(node);
  if (from_res && pin) {
    // Safety: verify pin has a valid vertex before dereferencing.
    // Parasitic nodes may hold stale pin pointers after buffer insertion/undoEco.
    sta::VertexId vid_check = network_->vertexId(pin);
    if (vid_check != sta::object_id_null && network_->isLoad(pin)) {
      sta::Vertex *load_vertex = graph_->vertex(vid_check);
      const PtVertex *pt_v = load_vertex
          ? pt_graph_->ptVertex(load_vertex) : nullptr;
      VertexId vid = pt_v ? pt_v->objectIdx() : sta::object_id_null;
      result.addLoad(vid, pin, elmore);
    }
  }
  visit(node);
  ParasiticResistorSeq &resistors = resistor_map_[node];
  for (ParasiticResistor *resistor : resistors) {
    ParasiticNode *onode = parasitics_->otherNode(resistor, node);
    if (resistor != from_res
        && !isVisited(onode)
        && !isLoopResistor(resistor)) {
      float r = parasitics_->value(resistor);
      double onode_elmore = elmore + r * downstreamCap(onode);
      reduceElmoreDfsToPt(drvr_pin, onode, resistor, onode_elmore, result);
    }
  }
  leave(node);
}

} // namespace

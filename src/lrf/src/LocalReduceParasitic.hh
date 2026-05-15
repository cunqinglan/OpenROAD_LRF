#pragma once
#include <map>
#include "sta/NetworkClass.hh"
#include "sta/StaState.hh"
#include "sta/Parasitics.hh"

namespace sta {
class Parasitic;
class ParasiticNode;
class ParasiticResistor;
class Pin;
class RiseFall;
class Scene;
class MinMax;
}

namespace lrf {
class PtPiElmore;
}

namespace lrf {
using namespace sta;

typedef std::map<ParasiticNode*, double> ParasiticNodeValueMap;
typedef std::map<ParasiticResistor*, double> ResistorCurrentMap;
typedef std::set<ParasiticResistor*> ParasiticResistorSet;
typedef std::set<ParasiticNode*> ParasiticNodeSet;

class PtGraph;

class LocalReduceToPi : public StaState
{
public:
  LocalReduceToPi(StaState *sta, const PtGraph *pt_graph);
  void reduceToPi(const Parasitic *parasitic_network,
                  const Pin *drvr_pin,
		  ParasiticNode *drvr_node,
		  float coupling_cap_factor,
		  const RiseFall *rf,
		  const Scene *corner,
		  const MinMax *min_max,
		  float &c2,
		  float &rpi,
		  float &c1);
  bool pinCapsOneValue() { return pin_caps_one_value_; }
  float downstreamCap(ParasiticNode *node);

protected:
  void reducePiDfs(const Pin *drvr_pin,
		   ParasiticNode *node,
		   ParasiticResistor *from_res,
                   double src_resistance,
		   double &y1,
		   double &y2,
		   double &y3,
		   double &dwn_cap,
                   double &max_resistance);
  void visit(ParasiticNode *node);
  bool isVisited(ParasiticNode *node);
  void leave(ParasiticNode *node);
  void setDownstreamCap(ParasiticNode *node,
			float cap);
  float pinCapacitance(ParasiticNode *node);
  bool isLoopResistor(ParasiticResistor *resistor);
  void markLoopResistor(ParasiticResistor *resistor);
  float localPinCapacitance(ParasiticNode *node);

  bool includes_pin_caps_;
  float coupling_cap_multiplier_;
  const RiseFall *rf_;
  const Scene *corner_;
  const MinMax *min_max_;
  Parasitics *parasitics_ = nullptr;
  ParasiticNodeResistorMap resistor_map_;
  ParasiticNodeCapacitorMap capacitor_map_;

  ParasiticNodeSet visited_nodes_;
  ParasiticNodeValueMap node_values_;
  ParasiticResistorSet loop_resistors_;
  bool pin_caps_one_value_;

  const PtGraph *pt_graph_;
};


class LocalReduceToPiElmore : public LocalReduceToPi
{
public:
  LocalReduceToPiElmore(StaState *sta, const PtGraph *pt_graph);
  Parasitic *makePiElmore(const Parasitic *parasitic_network,
                          const Pin *drvr_pin,
                          ParasiticNode *drvr_node,
                          float coupling_cap_factor,
                          const RiseFall *rf,
                          const Scene *corner,
                          const MinMax *min_max);
  void reduceElmoreDfs(const Pin *drvr_pin,
		       ParasiticNode *node,
		       ParasiticResistor *from_res,
		       double elmore,
		       Parasitic *pi_elmore);

  // Reduce parasitic network into a PtGraph-local PtPiElmore.
  // Uses the same reduceToPi DFS but stores results into PtPiElmore
  // instead of the global parasitic map.
  void makePtPiElmore(const Parasitic *parasitic_network,
                      const Pin *drvr_pin,
                      ParasiticNode *drvr_node,
                      float coupling_cap_factor,
                      const RiseFall *rf,
                      const Scene *corner,
                      const MinMax *min_max,
                      PtPiElmore &result);
  void reduceElmoreDfsToPt(const Pin *drvr_pin,
                           ParasiticNode *node,
                           ParasiticResistor *from_res,
                           double elmore,
                           PtPiElmore &result);
};


} // namespace lrf
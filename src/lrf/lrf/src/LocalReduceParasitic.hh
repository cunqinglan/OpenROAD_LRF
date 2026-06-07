#pragma once
#include <cstdint>
#include <unordered_map>

#include "sta/Map.hh"
#include "sta/NetworkClass.hh"
#include "sta/StaState.hh"
#include "sta/Parasitics.hh"

#include "PtElmoreCeff.hh"  // for kInvalidTreeNodeIdx in default args

namespace sta {
class Parasitic;
class ParasiticNode;
class ParasiticResistor;
class Pin;
class RiseFall;
class Corner;
class MinMax;
class ParasiticAnalysisPt;
}

namespace lrf {
class PtPiElmore;
class PtElmoreCeff;
}

namespace lrf {
using namespace sta;

typedef Map<ParasiticNode*, double> ParasiticNodeValueMap;
typedef Map<ParasiticResistor*, double> ResistorCurrentMap;
typedef Set<ParasiticResistor*> ParasiticResistorSet;
typedef Set<ParasiticNode*> ParasiticNodeSet;

class PtGraph;

class LocalReduceToPi : public StaState
{
public:
  LocalReduceToPi(StaState *sta, const PtGraph *pt_graph);
  // ec_sink (optional): if non-null, the Pi reduction DFS additionally
  // emits the full RC tree topology into ec_sink in a single pass. Used
  // by LocalParasitics when LRF_USE_ELMORECEFF is on, to avoid walking
  // the parasitic network twice (once for Pi, once for ElmoreCeff).
  void reduceToPi(const Parasitic *parasitic_network,
                  const Pin *drvr_pin,
		  ParasiticNode *drvr_node,
		  float coupling_cap_factor,
		  const RiseFall *rf,
		  const Corner *corner,
		  const MinMax *min_max,
		  const ParasiticAnalysisPt *ap,
		  float &c2,
		  float &rpi,
		  float &c1,
		  PtElmoreCeff *ec_sink = nullptr);
  bool pinCapsOneValue() { return pin_caps_one_value_; }
  float downstreamCap(ParasiticNode *node);

protected:
  // ec_sink/ec_parent_idx/branch_R_from_parent: dual-output mode for
  // ElmoreCeff. When ec_sink is non-null, pre-order pushes a tree node
  // (parent, branch_R, local_cap) into ec_sink->tree_ and records the
  // pin → tree-idx mapping for the per-load Elmore pass.
  void reducePiDfs(const Pin *drvr_pin,
		   ParasiticNode *node,
		   ParasiticResistor *from_res,
                   double src_resistance,
		   double &y1,
		   double &y2,
		   double &y3,
		   double &dwn_cap,
                   double &max_resistance,
                   PtElmoreCeff *ec_sink = nullptr,
                   uint32_t ec_parent_idx = kInvalidTreeNodeIdx,
                   double branch_R_from_parent = 0.0);
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
  const Corner *corner_;
  const MinMax *min_max_;
  const ParasiticAnalysisPt *ap_;
  ParasiticNodeResistorMap resistor_map_;
  ParasiticNodeCapacitorMap capacitor_map_;

  ParasiticNodeSet visited_nodes_;
  ParasiticNodeValueMap node_values_;
  ParasiticResistorSet loop_resistors_;
  bool pin_caps_one_value_;

  // Transient pin→tree-idx map populated during dual-output reducePiDfs;
  // consumed by reduceElmoreDfsToPt to emit per-load Elmore into both
  // PtPiElmore and PtElmoreCeff. Cleared at the start of each reduce.
  std::unordered_map<const sta::Pin *, uint32_t> ec_pin_to_tree_idx_;

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
                          const Corner *corner,
                          const MinMax *min_max,
                          const ParasiticAnalysisPt *ap);
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
                      const Corner *corner,
                      const MinMax *min_max,
                      const ParasiticAnalysisPt *ap,
                      PtPiElmore &result);

  // Dual-output variant: walks the parasitic network ONCE and emits
  // BOTH PtPiElmore (Pi reduction + per-load Elmore) and PtElmoreCeff
  // (full RC tree topology + per-load Elmore). Used by LocalParasitics
  // when LRF_USE_ELMORECEFF is on, to avoid duplicate DFS walks.
  void makePtPiElmoreAndCeff(const Parasitic *parasitic_network,
                             const Pin *drvr_pin,
                             ParasiticNode *drvr_node,
                             float coupling_cap_factor,
                             const RiseFall *rf,
                             const Corner *corner,
                             const MinMax *min_max,
                             const ParasiticAnalysisPt *ap,
                             PtPiElmore &result_pi,
                             PtElmoreCeff &result_ec);

  // ec_sink optional: when non-null, also emits per-load Elmore into
  // ec_sink->loads_, indexed by the pin → tree-idx map populated during
  // reducePiDfs's pre-order topology push.
  void reduceElmoreDfsToPt(const Pin *drvr_pin,
                           ParasiticNode *node,
                           ParasiticResistor *from_res,
                           double elmore,
                           PtPiElmore &result,
                           PtElmoreCeff *ec_sink = nullptr);

  // Pure ElmoreCeff path — does NOT compute Pi admittance moments
  // (y2/y3), does NOT derive c2/rpi/c1, does NOT allocate/touch any
  // PtPiElmore. Single DFS that:
  //   - pushes tree topology pre-order into result_ec.tree_
  //   - records loads inline (tree_node_idx known at push time)
  //   - accumulates total_cap
  //   - marks loop resistors
  // Then result_ec.precomputeMoments() fills delay[n] and impulse_sq[n]
  // via Algorithm 1.
  //
  // Per-load PtRcLoad.elmore is left at 0: in env=on, gateDelay reads
  // tree_[load.tree_node_idx].delay (Eq.15 wire delay), not load.elmore.
  void makePtElmoreCeffOnly(const Parasitic *parasitic_network,
                            const Pin *drvr_pin,
                            ParasiticNode *drvr_node,
                            float coupling_cap_factor,
                            const RiseFall *rf,
                            const Corner *corner,
                            const MinMax *min_max,
                            const ParasiticAnalysisPt *ap,
                            PtElmoreCeff &result_ec);

protected:
  // Single DFS used by makePtElmoreCeffOnly. Pre-order push to tree_,
  // inline load record (uses the just-assigned my_idx), recursive
  // descent with loop detection. No y2/y3, no downstream_cap caching,
  // no PtPiElmore touched.
  void topologyAndLoadsDfs(const Pin *drvr_pin,
                           ParasiticNode *node,
                           ParasiticResistor *from_res,
                           uint32_t parent_tree_idx,
                           float branch_R_from_parent,
                           float &total_cap_acc,
                           PtElmoreCeff &result_ec);
};


// Sibling to LocalReduceToPiElmore. Produces PtElmoreCeff: same Pi
// reduction + per-load Elmore as PtPiElmore, PLUS the full RC tree
// topology (post-order vector of {parent_idx, branch_R, local_cap})
// needed by Phase B1.3's Algorithm 2 and Phase B2's Eq.15 moments.
//
// The DFS shape mirrors reduceElmoreDfsToPt; it bolts a topology-recording
// (The env=on PtElmoreCeff build path goes through
// LocalReduceToPiElmore::makePtPiElmoreAndCeff above — a single DFS emits
// BOTH PtPiElmore and PtElmoreCeff. There is intentionally no standalone
// `LocalReduceToElmoreCeff` class: it would semantically be a sibling
// of LocalReduceToPi, not a subclass, but extracting a shared base just
// for that is not worth the churn while env=off remains the default. If
// B4 ever flips the default and drops the env=off path, the dual-output
// hack disappears at the same time and a clean ElmoreCeff-only reducer
// can be reintroduced then.)

} // namespace lrf
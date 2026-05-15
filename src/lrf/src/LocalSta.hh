#pragma once

#include <map>
#include <string>
#include "sta/Sta.hh"
#include "PtGraph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Network.hh"
#include "sta/StaState.hh"
#include "est/EstimateParasitics.h"
#include "lrf/LrfClass.hh"
#include <map>
#include "LocalParasitics.hh"
#include "sta/Delay.hh"
#include "sta/SearchPred.hh"
#include "sta/SdcClass.hh"

#include <map>
#include <vector>

namespace rsz {
  class Resizer;
}

namespace est {
  class EstimateParasitics;
}

namespace sta {
  class dbSta;
}

namespace lrf {

using namespace sta;

class ConcreteParasitic;
class ConcreteParasiticNetwork;
class TaskArranger;
typedef std::map<const Pin*, ConcreteParasitic**> ConcreteParasiticMap;

typedef float LocalCost;

class LocalParasitics;
class ParallelVisitor;

class LocalSta: public GraphDelayCalc {
public:
  LocalSta(dbSta *sta);
  ~LocalSta();

  virtual void copyState(const Sta *sta);

  void collectLocalGraph(Instance *inst, InstanceSet &local_instances);
  void collectLocalVertices(Instance *inst, VertexSet &local_vertices);
  // FF variant of collectLocalVertices: accepts sequential ref instances and
  // skips fanin-sibling expansion on clock pins (otherwise every sister FF on
  // the same clock leaf would join the local graph as a SiblingLoad). The
  // CK pin's own load vertex is still inserted so the CK→D setup check edge
  // resolves both endpoints.
  void collectLocalVerticesFF(Instance *inst, VertexSet &local_vertices);
  // Lightweight: ref-instance pins + direct wire fanout loads only
  // (no fanin sibling collection, no downstream driver traversal).
  void collectDriverFanoutOnly(Instance *inst, VertexSet &local_vertices);
  void makePtGraph(PtGraph *pt_graph, Instance *inst,
                          sta::Scene *scene = nullptr, const sta::MinMax *min_max = nullptr);
  // Lightweight PtGraph: skips fanin siblings; no pruneInsignificantSiblings.
  void makePtGraphDriverOnly(PtGraph *pt_graph, Instance *inst,
                             sta::Scene *scene = nullptr, const sta::MinMax *min_max = nullptr);
  // FF PtGraph: accepts sequential ref instance, bounded clock-pin
  // expansion, and adds CK→D setup CheckEdges via PtGraph::addCheckEdgesForRefInst.
  void makePtGraphFF(PtGraph *pt_graph, Instance *inst,
                     sta::Scene *scene = nullptr, const sta::MinMax *min_max = nullptr);
  PtGraph *makePtGraph(Instance *inst, bool update_timing_first = false);

  sta::dbSta *getSta() { return sta_; }
  TaskArranger *taskArranger() { return task_arranger_; }
  bool equivCellsMade() const { return equiv_cells_made_; }
  void setEquivCellsMade(bool made) { equiv_cells_made_ = made; }

  // Delay calculation methods
  void findLocalDelays(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  void initLocalDelays(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  // Compute setup-time on every CheckEdge in the FF-mode PtGraph.
  // CK slew is read from the global sta::Graph (CK driver is not in
  // PtGraph by design); D slew is read from the PtGraph cache after
  // findLocalDelays has propagated the upstream wire arc to D.
  // Setup delay is stored in the CheckEdge's arc_delays_ via setArcDelay.
  void findLocalCheckDelays(PtGraph *pt_graph,
                            ArcDelayCalc *arc_delay_calc);
  // Σ over CheckEdges of (setup_delay × LM(wire-in edge to D)). Computes
  // the FF-specific Δsetup × LM contribution that the regular delayLmSum
  // misses (because LR maintains LM=0 on check edges). The LM coefficient
  // is taken from the wire edge feeding D, which carries the endpoint
  // rescaling of the upstream combinational path's LM.
  float computeSetupLmSum(PtGraph *pt_graph);
  // FF variant of increAndGetLocalTimingCost: virtually replace cell,
  // recompute findLocalDelays + findLocalCheckDelays, and return
  // delay_lm_sum (combinational, regClkToQ, wire) + computeSetupLmSum.
  DelayLmSumResult
  increAndGetLocalTimingCostFF(PtGraph *pt_graph,
                               ArcDelayCalc *arc_delay_calc,
                               sta::LibertyCell *equiv_cell,
                               std::map<std::string, double> *runtime_map = nullptr);
  float maxInputSlew(const Pin* input,
                            const Scene* corner) const;
  void setParasiticsEst(est::EstimateParasitics *estimate_parasitics);
  void updateGlobalParasiticsAndSync(est::EstimateParasitics *est_parasitics);
  // Sync local parasitic map from global (without re-estimating).
  void syncParasiticMapFromGlobal();
  void setAnalysisPoints(const std::vector<sta::DcalcAPIndex> &dcalc_ap_set);
  void setDebugLabel(const std::string &label) { debug_label_ = label; }

  // Power APIs
  // Average leakage across all when-conditions (no duty weighting)
  float cellAvgLeakage(sta::LibertyCell *cell);
  // Leakage weighted by driver pin duty cycle (P(input=1))
  float cellLeakageWithDuty(sta::LibertyCell *cell, float input_duty);

  // Functions for Searching arrivals and required times
  void findLocalArrivals(PtGraph *pt_graph);
  void findLocalRequireds(PtGraph *pt_graph);
  // Full version: checks hasVirtualBuffer tag and recomputes load cap if needed
  void localParasiticLoad(PtVertex &drvr_pt_vertex,
                          const RiseFall *rf,
                          const sta::Scene *scene, const sta::MinMax *min_max,
                          const MultiDrvrNet *multi_drvr_net,
                          // Return values
                          float &load_cap,
                          const Parasitic *&parasitic,
                          PtGraph *pt_graph);

  // print informations of local graphs for debug purpose
  void printLocalParasitics(PtGraph *pt_graph) const;
  void printParasitics(PtGraph *pt_graph) const;
  void printLocalArrivals(PtGraph *pt_graph) const;
  void printLocalRequireds(PtGraph *pt_graph) const;
  void printLocalTiming(PtGraph *pt_graph) const;
  void printLocalSlews(PtGraph *pt_graph) const;

  // Functions for parallel LR
  void initParallel();
  void runResize(rsz::Resizer *resizer, ParallelVisitor *visitor);

  // Functions for ERC check
  float getPinMaxSlewLimit(sta::Pin *pin, sta::LibertyCell *cell);
  float getPinMaxCapLimit(sta::Pin *pin, sta::LibertyCell *lib_cell);
  float getPortMaxSlewLimit(sta::LibertyPort *port);
  float getPortMaxCapLimit(sta::LibertyPort *port);
  // Get max slew across rise/fall for a PtVertex
  float getVertexMaxSlew(PtGraph *pt_graph, PtVertex &ptv,
                         const sta::Scene *scene, const sta::MinMax *min_max);
  // Check slew limits for all wire-fanout loads of a driver PtVertex.
  // slew_limit_scale multiplies the library slew limit. Default 0.9 = 10%
  // headroom, reserved so the optimizer rejects cells that would ship
  // post-GRT slew violations after the placement → global_routing RC shift.
  bool checkFanoutLoadSlew(PtGraph *pt_graph, VertexId drvr_id,
                           const sta::Scene *scene, const sta::MinMax *min_max,
                           float slew_limit_scale = 0.95f);
  // Sum (load_slew - limit)_+ across all wire-fanout loads of drvr.
  float fanoutLoadSlewViolation(PtGraph *pt_graph, VertexId drvr_id,
                                const sta::Scene *scene, const sta::MinMax *min_max,
                                float slew_limit_scale = 0.95f);
  sta::LibertyPort *findTargetPort(const PtVertex &ptv,
                                   sta::LibertyCell *to_lib_cell) const;
  float getPinSlew(sta::Pin *pin, const sta::Scene *corner,
                     const sta::MinMax *min_max, PtGraph *pt_graph);
  float getLoadCap(PtVertex &drvr_pt_vertex, const sta::Scene *corner,
                   const sta::MinMax *min_max, PtGraph *pt_graph);
  bool legalCheckBeforeSwap(sta::Instance *inst, 
                            sta::LibertyCell *to_lib_cell,
                            const sta::Scene *corner,
                            const sta::MinMax *min_max,
                            PtGraph *pt_graph);
  // slew_limit_scale: multiplier on the library slew limit. Default 0.9 =
  // 10% headroom, matching LrConfig::slew_margin's default. Reserved so the
  // optimizer rejects cells that would ship post-GRT slew violations after
  // the placement → global_routing RC shift.
  bool legalCheckAfterSwap(sta::Instance *inst,
                           sta::LibertyCell *to_lib_cell,
                           const sta::Scene *corner,
                           const sta::MinMax *min_max,
                           PtGraph *pt_graph,
                           float slew_limit_scale = 0.95f);

  // ERC relaxation: instead of boolean legal/illegal, return summed
  // violation magnitude (max(0, value - limit)) split by type.
  // slew/cap_limit_scale multiply the library slew/cap limits: a scale of
  // 0.85 flags violations once the value exceeds 85% of the limit, giving
  // 15% physical headroom that survives post-GR RC shift. Tighter scale
  // → earlier penalization → more conservative sizing.
  // slew is in seconds, cap is in farads. Caller normalizes to taste.
  struct ViolationSum { float slew = 0.0f; float cap = 0.0f; };
  ViolationSum violationSumBeforeSwap(sta::Instance *inst,
                                      sta::LibertyCell *to_lib_cell,
                                      const sta::Scene *corner,
                                      const sta::MinMax *min_max,
                                      PtGraph *pt_graph,
                                      float cap_limit_scale = 1.0f);
  ViolationSum violationSumAfterSwap(sta::Instance *inst,
                                     sta::LibertyCell *to_lib_cell,
                                     const sta::Scene *corner,
                                     const sta::MinMax *min_max,
                                     PtGraph *pt_graph,
                                     float slew_limit_scale = 1.0f,
                                     float cap_limit_scale = 1.0f);

  // Violation check functions - public interfaces
  void checkSlew(const sta::Pin *pin,
                 const sta::LibertyCell *lib_cell,
                 const sta::Scene *corner,
                 const sta::MinMax *min_max,
                 bool check_clks,
                 PtGraph *pt_graph,
                 // Return values
                 const sta::Scene *&corner1,
                 const sta::RiseFall *&rf1,
                 float &slew1,
                 float &limit1,
                 float &slack1) const;

  sta::Path *ptVertexWorstSlackPath(PtVertex &pt_vertex, const sta::MinMax *min_max) const;
  sta::Path *ptVertexWorstSlackPath(PtVertex &pt_vertex,
                                    const sta::Scene *scene, const sta::MinMax *min_max) const;

  // Public API for operators
  DelayLmSumResult initAndGetLocalTimingCost(PtGraph *pt_graph, sta::ArcDelayCalc *arc_delay_calc);
  DelayLmSumResult increAndGetLocalTimingCost(PtGraph *pt_graph,
                                    sta::ArcDelayCalc *arc_delay_calc,
                                    sta::LibertyCell *equiv_cell,
                                    std::map<std::string, double> *runtime_map = nullptr);
  sta::Slack localSlackAroundRef(PtGraph *pt_graph);
  sta::Slack localSlackOnSinks(PtGraph *pt_graph);
  sta::Slack localWorstSlackOnSinks(PtGraph *pt_graph);
  void recomputeSinglePtParasitic(PtGraph *pt_graph, sta::VertexId drvr_vid);
  bool virtualReplaceCell(PtGraph *pt_graph, sta::LibertyCell *new_cell);
  // Swap ref cell with selective parasitic recompute: skip RefOutput
  // drivers whose output port cap is unchanged after cell swap.
  bool virtualReplaceCellSelective(PtGraph *pt_graph, sta::LibertyCell *new_cell);

protected:
  const Pin *findNetParasiticDrvrPin(sta::Net *net) const;
  void collectLocalFanouts(Pin *drvr_pin, InstanceSet &local_instances);
  void collectLocalFaninSiblings(Pin *pin, PinSet &visited_pins, 
                                 InstanceSet &local_instances);
  void collectLocalFanoutVertices(sta::Vertex *drvr_vertex, 
                                  sta::VertexSet &local_vertices);
  void collectLocalFaninSiblingVertices(sta::Vertex *load_vertex, 
                                      sta::VertexSet &local_vertices);
  void topoSortVertices(PtGraph *pt_graph);

  // Violation check helper functions
  void checkSlew1(const sta::Pin *pin,
                  Vertex *vertex,
                  const sta::LibertyCell *lib_cell,
                  const sta::Scene *corner,
                  const sta::MinMax *min_max,
                  bool check_clks,
                  PtGraph *pt_graph,
                  // Return values
                  const sta::Scene *&corner1,
                  const sta::RiseFall *&rf1,
                  float &slew1,
                  float &limit1,
                  float &slack1) const;
  
  void checkSlew2(const sta::Pin *pin,
                  Vertex *vertex,
                  const sta::LibertyCell *lib_cell,
                  const sta::Scene *corner,
                  const sta::MinMax *min_max,
                  const ClockSet &clks,
                  PtGraph *pt_graph,
                  // Return values
                  const sta::Scene *&corner1,
                  const sta::RiseFall *&rf1,
                  float &slew1,
                  float &limit1,
                  float &slack1) const;
  
  void checkSlew3(const sta::Pin *pin,
                  Vertex *vertex,
                  const sta::LibertyCell *lib_cell,
                  const sta::Scene *corner,
                  const sta::RiseFall *rf,
                  const sta::MinMax *min_max,
                  float limit,
                  PtGraph *pt_graph,
                  // Return values
                  const sta::Scene *&corner1,
                  const sta::RiseFall *&rf1,
                  float &slew1,
                  float &slack1,
                  float &limit1) const;


  void localFindSlewLimit(const sta::LibertyPort *lib_port,
                          const sta::Scene *corner,
                          const sta::MinMax *min_max,
                          // Return values
                          float &limit,
                          bool &exists) const;

  void localFindSlewLimit(const sta::Pin *pin,
                          const sta::LibertyCell *lib_cell,
                          const sta::Scene *corner,
                          const sta::MinMax *min_max,
                          const sta::RiseFall *rf,
                          const ClockSet &clks,
                          // Return values
                          float &limit,
                          bool &exists) const;

  sta::ClockSet clockDomains(const sta::Vertex *vertex) const;

  // Delay calculation methods
  void seedRootSlews();
  void zeroSlewAndWireDelays(PtVertex &drvr_pt_vertex,
                           const sta::RiseFall *rf,
                           PtGraph *pt_graph);
  void loadSlewFromGraph(PtVertex &pt_vertex, PtGraph *pt_graph);
  void findVertexDelays(VertexId pt_vertex_id, 
                        ArcDelayCalc *arc_delay_calc,
                        PtGraph *pt_graph);
  void seedRootSlew(PtVertex &pt_vertex, PtGraph *pt_graph, 
                   ArcDelayCalc *arc_delay_calc);
  void seedDrvrSlew(PtVertex &pt_vertex, PtGraph *pt_graph, 
                   ArcDelayCalc *arc_delay_calc);
  void seedLoadSlew(PtVertex &pt_vertex, PtGraph *pt_graph, 
                   ArcDelayCalc *arc_delay_calc);
  void seedNoDrvrSlew(PtVertex &pt_drvr_vertex,
                             const RiseFall *rf,
                             const sta::Scene *scene, const sta::MinMax *min_max,
                             ArcDelayCalc *arc_delay_calc,
                             PtGraph *pt_graph);
  void seedNoDrvrCellSlew(PtVertex &pt_drvr_vertex,
                          const Pin *drvr_pin,
                          const RiseFall *rf,
                          const InputDrive *drive,
                          const sta::Scene *scene, const sta::MinMax *min_max,
                          ArcDelayCalc *arc_delay_calc,
                          PtGraph *pt_graph);
  int findPortIndex(const LibertyCell *cell,
                    const LibertyPort *port);
  void findInputDriverDelay(const LibertyCell *drvr_cell,
                            const Pin *drvr_pin,
                            Vertex *drvr_vertex,
                            const RiseFall *rf,
                            const LibertyPort *from_port,
                            float *from_slews,
                            const LibertyPort *to_port,
                            const sta::Scene *scene, const sta::MinMax *min_max);
  LoadPinIndexMap makeLoadPinIndexMap(Vertex *drvr_vertex);
  LoadPinIndexMap makeLoadPinIndexMap(PtVertex &drvr_pt_vertex, PtGraph *pt_graph);
  MultiDrvrNet *findMultiDrvrNet(Vertex *drvr_vertex);
  void findDriverDelays(PtVertex &drvr_pt_vertex,
                        ArcDelayCalc *arc_delay_calc,
                        LoadPinIndexMap &load_pin_index_map,
                        PtGraph *pt_graph);
  void initLoadSlews(PtVertex &pt_vertex, PtGraph *pt_graph);
  void findDriverDelays1(PtVertex &drvr_pt_vertex,
                         MultiDrvrNet *multi_drvr_net,
                         ArcDelayCalc *arc_delay_calc,
                         LoadPinIndexMap &load_pin_index_map,
                         PtGraph *pt_graph);
  void initSlew(PtVertex &pt_vertex, PtGraph *pt_graph);
  void initWireDelays(PtVertex &drvr_pt_vertex, PtGraph *pt_graph);
  void findDriverEdgeDelays(PtVertex &drvr_pt_vertex,
                               const MultiDrvrNet *multi_drvr_net,
                               PtEdge &pt_edge,
                               ArcDelayCalc *arc_delay_calc,
                               LoadPinIndexMap &load_pin_index_map,
                               std::array<bool, 
                               RiseFall::index_count> &delay_exists,
                               PtGraph *pt_graph);
  void findDriverArcDelays(PtVertex &drvr_pt_vertex,
                        PtEdge &pt_edge,
                        const TimingArc *arc,
                        const sta::Scene *scene, const sta::MinMax *min_max,
                        ArcDelayCalc *arc_delay_calc,
                        PtGraph *pt_graph);
  void findDriverArcDelays(PtVertex &drvr_pt_vertex,
                        const MultiDrvrNet *multi_drvr_net,
                        PtEdge &pt_edge,
                        const TimingArc *arc,
                        const sta::Scene *scene, const sta::MinMax *min_max,
                        ArcDelayCalc *arc_delay_calc,
                        LoadPinIndexMap &load_pin_index_map,
                        PtGraph *pt_graph);
  bool annotateDelaysSlews(PtEdge &pt_edge,
                         const TimingArc *arc,
                         ArcDcalcResult &dcalc_result,
                         LoadPinIndexMap &load_pin_index_map,
                         const sta::Scene *scene, const sta::MinMax *min_max,
                         PtGraph *pt_graph);
  bool annotateDelaySlew(PtEdge &pt_edge,
                        const TimingArc *arc,
                        ArcDelay &gate_delay,
                        Slew &gate_slew,
                        const sta::Scene *scene, const sta::MinMax *min_max,
                        PtGraph *pt_graph);
  Slew edgeFromLocalSlew(const PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    const PtEdge &pt_edge,
                    const sta::Scene *scene, const sta::MinMax *min_max,
                    PtGraph *pt_graph);
  Slew edgeFromLocalSlew(const PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    const TimingRole *role,
                    const sta::Scene *scene, const sta::MinMax *min_max,
                    PtGraph *pt_graph);
  bool annotateLoadDelays(PtVertex &drvr_pt_vertex,
                          const RiseFall *to_rf,
                          ArcDcalcResult &dcalc_result,
                          LoadPinIndexMap &load_pin_index_map,
                          const ArcDelay &extra_delay,
                          bool merge,
                          const sta::Scene *scene, const sta::MinMax *min_max,
                          PtGraph *pt_graph);
  float computeVirtualLoadCap(PtVertex &drvr_pt_vertex,
                              const RiseFall *drvr_rf,
                              const sta::Scene *scene, const sta::MinMax *min_max,
                              PtGraph *pt_graph);

  float delayLmSum(Instance *inst, const MinMax *minmax);
  DelayLmSumResult delayLmSum(PtGraph *pt_graph,
                     bool collect_vecs);
  float refgateDelayLmSum(PtGraph *pt_graph);
  void graphPop();
  void setSta(dbSta *sta) { sta_ = sta; }
  // initAndGetLocalTimingCost, increAndGetLocalTimingCost, localSlackAroundRef
  // moved to public section above
  void updateLocalTiming(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  Slack localSlackAtEndpoints(PtGraph *pt_graph);
  // localSlackOnSinks / localWorstSlackOnSinks moved to public section
  
  ////////////////////////////////////////////////////////
  // Deal with parasitics
  ////////////////////////////////////////////////////////
  // Each time a cell is swapped, the pi model of its fanin 
  // will change largely. So the parasitic network and its
  // reduced pi model need to be recomputed.
  void recomputeLocalParasitics(PtGraph *pt_graph);
  // recomputeSinglePtParasitic moved to public section
  void loadLocalParasitics(const Pin *drvr_pin,
                           const RiseFall *rf,
                           const sta::Scene *scene, const sta::MinMax *min_max,
                           const MultiDrvrNet *multi_drvr_net,
                           ArcDelayCalc *arc_delay_calc,
                           float *load_cap,
                           const Parasitic *&parasitic) const;

  void AnnotateRefFaninVertex(PtGraph *pt_graph);

  // Not finished function
  // ArcDcalcArgSeq makeArcDcalcArgs(PtVertex &drvr_pt_vertex,
  //                          const MultiDrvrNet *multi_drvr_net,
  //                          PtEdge &pt_edge,
  //                          const TimingArc *arc,
  //                          const sta::Scene *scene, const sta::MinMax *min_max,
  //                          ArcDelayCalc *arc_delay_calc,
  //                          PtGraph *pt_graph);

  //////////////////////////////////////////////////////////////
  // Functions for Searching arrivals and required times
  //////////////////////////////////////////////////////////////
  // Use tagGroup of search to initialize paths_ of PtGraph
  void initPtGraphPaths(PtGraph *pt_graph);

  dbSta *sta_;

  bool collected_;
  bool sorted_;
  bool parasitics_set_;

  InstanceSet local_fanins_;
  InstanceSet local_fanout_siblings_;

  VertexSeq local_vertices_;
  VertexSeq root_vertices_;
  std::vector<PtGraph*> local_graphs_;
  est::EstimateParasitics *estimate_parasitics_;
  LocalParasitics *local_parasitics_;
  TaskArranger *task_arranger_;
  bool equiv_cells_made_ = false;
  SearchPred *pred_;
  SearchPred *search_pred_;
  std::mutex pt_graph_vector_mutex_;

  std::string debug_label_ = "LocalSTA";
  bool debug_ = false;
public:
  void setDebug(bool d) { debug_ = d; }
  bool debug() const { return debug_; }
protected:

private:
  friend class IncreSta;
  friend class TestLrf;
  friend class LrRebuffer;
  friend class TestRebuffer;
public:
  // Print per-sink arrival comparison: local (PtGraph) vs global (OpenSTA)
  // for max/default/rise path. Shows which sinks have arrival mismatch.
  void printPerSinkArrivals(PtGraph *pt_graph, const char *label);
};


class LRSInstanceVisitor {
public:
  LRSInstanceVisitor(LocalSta *local_sta);
  ~LRSInstanceVisitor();
  LRSInstanceVisitor *copy() const;
  void visit(Instance *inst);

protected:
  LocalSta *local_sta_;
  Instance *inst_;
  PtGraph *local_graph_;
  ArcDelayCalc *delay_arc_calc_;

  // Containers for parasitics
  ConcreteParasiticMap drvr_parasitic_map_;
  std::vector<sta::DcalcAPIndex> dcalc_ap_set_;

private:
  friend class TestLrf;
};


} // namespace lrf

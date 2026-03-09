#pragma once

#include "sta/Sta.hh"
#include "PtGraph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Network.hh"
#include "sta/StaState.hh"
#include "est/EstimateParasitics.h"
#include "lrf/LrfClass.hh"
#include "sta/Map.hh"
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
typedef Map<const Pin*, ConcreteParasitic**> ConcreteParasiticMap;
typedef Map<const Net*, ConcreteParasiticNetwork**> ConcreteParasiticNetworkMap;

typedef float LocalCost;

class LocalParasitics;
class ParallelLrVisitor;

class LocalSta: public GraphDelayCalc {
public:
  LocalSta(dbSta *sta);
  ~LocalSta();

  virtual void copyState(const Sta *sta);

  void collectLocalGraph(Instance *inst, InstanceSet &local_instances);
  void collectLocalVertices(Instance *inst, VertexSet &local_vertices);
  void makePtGraph(PtGraph *pt_graph, Instance *inst, 
                          DcalcAnalysisPt *dcalc_ap = nullptr);
  PtGraph *makePtGraph(Instance *inst, bool update_timing_first = false);

  sta::dbSta *getSta() { return sta_; }
  TaskArranger *taskArranger() { return task_arranger_; }
  bool equivCellsMade() const { return equiv_cells_made_; }
  void setEquivCellsMade(bool made) { equiv_cells_made_ = made; }

  // Delay calculation methods
  void findLocalDelays(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  void initLocalDelays(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  float maxInputSlew(const Pin* input,
                            const Corner* corner) const;
  void setParasiticsEst(est::EstimateParasitics *estimate_parasitics);
  void setAnalysisPoints(const std::vector<const DcalcAnalysisPt*> &dcalc_ap_set);
  void setDebugLabel(const std::string &label) { debug_label_ = label; }

  // Power APIs
  // Average leakage across all when-conditions (no duty weighting)
  float cellAvgLeakage(sta::LibertyCell *cell);
  // Leakage weighted by driver pin duty cycle (P(input=1))
  float cellLeakageWithDuty(sta::LibertyCell *cell, float input_duty);

  // Functions for Searching arrivals and required times
  void findLocalArrivals(PtGraph *pt_graph);
  void findLocalRequireds(PtGraph *pt_graph);
  void localParasiticLoad(const Pin *drvr_pin,
                          const RiseFall *rf,
                          const DcalcAnalysisPt *dcalc_ap,
                          const MultiDrvrNet *multi_drvr_net,
                          // Return values
                          float &load_cap,
                          const Parasitic *&parasitic) const;

  // print informations of local graphs for debug purpose
  void printLocalParasitics(PtGraph *pt_graph) const;
  void printParasitics(PtGraph *pt_graph) const;
  void printLocalArrivals(PtGraph *pt_graph) const;
  void printLocalRequireds(PtGraph *pt_graph) const;
  void printLocalTiming(PtGraph *pt_graph) const;
  void printLocalSlews(PtGraph *pt_graph) const;

  // Functions for parallel LR
  void initParallel();
  void runResize(rsz::Resizer *resizer, ParallelLrVisitor *visitor);

  // Functions for ERC check
  float getPinMaxSlewLimit(sta::Pin *pin, sta::LibertyCell *cell);
  float getPinMaxCapLimit(sta::Pin *pin, sta::LibertyCell *lib_cell);
  float getPinSlew(sta::Pin *pin, const sta::Corner *corner,
                     const sta::MinMax *min_max, PtGraph *pt_graph);
  float getNetCap(sta::Net *net, const sta::Corner *corner,
                  const sta::MinMax *min_max, PtGraph *pt_graph);
  bool legalCheckBeforeSwap(sta::Instance *inst, 
                            sta::LibertyCell *to_lib_cell,
                            const sta::Corner *corner,
                            const sta::MinMax *min_max,
                            PtGraph *pt_graph);
  bool legalCheckAfterSwap(sta::Instance *inst, 
                           sta::LibertyCell *to_lib_cell,
                           const sta::Corner *corner,
                           const sta::MinMax *min_max,
                           PtGraph *pt_graph);

  // Violation check functions - public interfaces
  void checkSlew(const sta::Pin *pin,
                 const sta::LibertyCell *lib_cell,
                 const sta::Corner *corner,
                 const sta::MinMax *min_max,
                 bool check_clks,
                 PtGraph *pt_graph,
                 // Return values
                 const sta::Corner *&corner1,
                 const sta::RiseFall *&rf1,
                 float &slew1,
                 float &limit1,
                 float &slack1) const;

  void localCheckCapacitance(const sta::Pin *pin,
                              const sta::LibertyCell *lib_cell,
                              const sta::Corner *corner,
                              const sta::MinMax *min_max,
                              // Return values
                              const sta::Corner *&corner1,
                              const sta::RiseFall *&rf1,
                              float &capacitance1,
                              float &limit1,
                              float &slack1) const;
  sta::Path *ptVertexWorstSlackPath(PtVertex &pt_vertex, const sta::MinMax *min_max) const;

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
                  const sta::Corner *corner,
                  const sta::MinMax *min_max,
                  bool check_clks,
                  PtGraph *pt_graph,
                  // Return values
                  const sta::Corner *&corner1,
                  const sta::RiseFall *&rf1,
                  float &slew1,
                  float &limit1,
                  float &slack1) const;
  
  void checkSlew2(const sta::Pin *pin,
                  Vertex *vertex,
                  const sta::LibertyCell *lib_cell,
                  const sta::Corner *corner,
                  const sta::MinMax *min_max,
                  const ClockSet &clks,
                  PtGraph *pt_graph,
                  // Return values
                  const sta::Corner *&corner1,
                  const sta::RiseFall *&rf1,
                  float &slew1,
                  float &limit1,
                  float &slack1) const;
  
  void checkSlew3(const sta::Pin *pin,
                  Vertex *vertex,
                  const sta::LibertyCell *lib_cell,
                  const sta::Corner *corner,
                  const sta::RiseFall *rf,
                  const sta::MinMax *min_max,
                  float limit,
                  PtGraph *pt_graph,
                  // Return values
                  const sta::Corner *&corner1,
                  const sta::RiseFall *&rf1,
                  float &slew1,
                  float &slack1,
                  float &limit1) const;


  void localFindSlewLimit(const sta::LibertyPort *lib_port,
                          const sta::Corner *corner,
                          const sta::MinMax *min_max,
                          // Return values
                          float &limit,
                          bool &exists) const;

  void localFindSlewLimit(const sta::Pin *pin,
                          const sta::LibertyCell *lib_cell,
                          const sta::Corner *corner,
                          const sta::MinMax *min_max,
                          const sta::RiseFall *rf,
                          const ClockSet &clks,
                          // Return values
                          float &limit,
                          bool &exists) const;

  void localCheckCapacitance1(const sta::Pin *pin,
                               const sta::LibertyCell *lib_cell,
                               const sta::Corner *corner,
                               const sta::MinMax *min_max,
                               // Return values
                               const sta::Corner *&corner1,
                               const sta::RiseFall *&rf1,
                               float &capacitance1,
                               float &limit1,
                               float &slack1) const;

  void localFindCapLimit(const sta::Pin *pin,
                         const sta::LibertyCell *lib_cell,
                         const sta::Corner *corner,
                         const sta::MinMax *min_max,
                         // Return values
                         float &limit,
                         bool &exists) const;

  void localCheckCapacitance(const sta::Pin *pin,
                              const sta::LibertyCell *lib_cell,
                              const sta::Corner *corner,
                              const sta::MinMax *min_max,
                              const sta::RiseFall *rf,
                              float limit,
                              // Return values
                              const sta::Corner *&corner1,
                              const sta::RiseFall *&rf1,
                              float &capacitance1,
                              float &slack1,
                              float &limit1) const;

  void connectedCap(const Pin *drvr_pin,
                     const sta::RiseFall *rf,
                     const sta::Corner *corner,
                     const sta::MinMax *min_max,
                     float &load_cap) const;

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
                             const DcalcAnalysisPt *dcalc_ap,
                             ArcDelayCalc *arc_delay_calc,
                             PtGraph *pt_graph);
  void seedNoDrvrCellSlew(PtVertex &pt_drvr_vertex,
                          const Pin *drvr_pin,
                          const RiseFall *rf,
                          const InputDrive *drive,
                          const DcalcAnalysisPt *dcalc_ap,
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
                            const DcalcAnalysisPt *dcalc_ap);
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
                        const DcalcAnalysisPt *dcalc_ap,
                        ArcDelayCalc *arc_delay_calc,
                        PtGraph *pt_graph);
  void findDriverArcDelays(PtVertex &drvr_pt_vertex,
                        const MultiDrvrNet *multi_drvr_net,
                        PtEdge &pt_edge,
                        const TimingArc *arc,
                        const DcalcAnalysisPt *dcalc_ap,
                        ArcDelayCalc *arc_delay_calc,
                        LoadPinIndexMap &load_pin_index_map,
                        PtGraph *pt_graph);
  bool annotateDelaysSlews(PtEdge &pt_edge,
                         const TimingArc *arc,
                         ArcDcalcResult &dcalc_result,
                         LoadPinIndexMap &load_pin_index_map,
                         const DcalcAnalysisPt *dcalc_ap,
                         PtGraph *pt_graph);
  bool annotateDelaySlew(PtEdge &pt_edge,
                        const TimingArc *arc,
                        ArcDelay &gate_delay,
                        Slew &gate_slew,
                        const DcalcAnalysisPt *dcalc_ap,
                        PtGraph *pt_graph);
  Slew edgeFromLocalSlew(const PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    const PtEdge &pt_edge,
                    const DcalcAnalysisPt *dcalc_ap,
                    PtGraph *pt_graph);
  Slew edgeFromLocalSlew(const PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    const TimingRole *role,
                    const DcalcAnalysisPt *dcalc_ap,
                    PtGraph *pt_graph);
  bool annotateLoadDelays(PtVertex &drvr_pt_vertex,
                          const RiseFall *to_rf,
                          ArcDcalcResult &dcalc_result,
                          LoadPinIndexMap &load_pin_index_map,
                          const ArcDelay &extra_delay,
                          bool merge,
                          const DcalcAnalysisPt *dcalc_ap,
                          PtGraph *pt_graph);
  float computeVirtualLoadCap(PtVertex &drvr_pt_vertex,
                              const RiseFall *drvr_rf,
                              const DcalcAnalysisPt *dcalc_ap,
                              PtGraph *pt_graph);

  float delayLmSum(Instance *inst, const MinMax *minmax);
  float delayLmSum(PtGraph *pt_graph, DcalcAnalysisPt *dcalc_ap);
  DelayLmSumResult delayLmSum(PtGraph *pt_graph,
                     DcalcAnalysisPt *dcalc_ap, 
                     bool collect_vecs);
  float delayLmSum(PtGraph *pt_graph);
  float delayLmSum(sta::Instance *inst, PtGraph *pt_graph);
  float refgateDelayLmSum(PtGraph *pt_graph);
  void graphPop();
  void setSta(dbSta *sta) { sta_ = sta; }
  DelayLmSumResult initAndGetLocalTimingCost(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  DelayLmSumResult increAndGetLocalTimingCost(PtGraph *pt_graph,
                                    ArcDelayCalc *arc_delay_calc,
                                    LibertyCell *equiv_cell);
  void updateLocalTiming(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  Slack localSlackAroundRef(PtGraph *pt_graph);
  Slack localSlackAtEndpoints(PtGraph *pt_graph);
  
  ////////////////////////////////////////////////////////
  // Deal with parasitics
  ////////////////////////////////////////////////////////
  // Each time a cell is swapped, the pi model of its fanin 
  // will change largely. So the parasitic network and its
  // reduced pi model need to be recomputed.
  void recomputeLocalParasitics(PtGraph *pt_graph);

  ////////////////////////////////////////////////////////
  // Swapping cells virtually
  ////////////////////////////////////////////////////////
  void virtualReplaceCell(PtGraph *pt_graph, LibertyCell *new_cell);
  void loadLocalParasitics(const Pin *drvr_pin,
                           const RiseFall *rf,
                           const DcalcAnalysisPt *dcalc_ap,
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
  //                          const DcalcAnalysisPt *dcalc_ap,
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

private:
  friend class IncreSta;
  friend class TestLrf;
  friend class LrRebuffer;
  friend class ParallelLrVisitor;
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
  ConcreteParasiticNetworkMap parasitic_network_map_;
  std::vector<DcalcAnalysisPt*> dcalc_ap_set_;

private:
  friend class TestLrf;
};


} // namespace lrf

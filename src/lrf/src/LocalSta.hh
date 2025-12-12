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

#include <map>
#include <vector>

namespace rsz {
  class Resizer;
}

namespace est {
  class EstimateParasitics;
}

namespace lrf {

using namespace sta;

class ConcreteParasitic;
class ConcreteParasiticNetwork;
typedef Map<const Pin*, ConcreteParasitic**> ConcreteParasiticMap;
typedef Map<const Net*, ConcreteParasiticNetwork**> ConcreteParasiticNetworkMap;

typedef float LocalCost;

class LocalParasitics;
class ParallelLrVisitor;

class LocalSta: public GraphDelayCalc {
public:
  LocalSta(Sta *sta);
  ~LocalSta();

  virtual void copyState(const Sta *sta);

  void collectLocalGraph(Instance *inst, InstanceSet &local_instances);
  
  void makePtGraph(PtGraph *pt_graph, Instance *inst);
  PtGraph *makePtGraph(Instance *inst, bool update_timing_first = false);

  Sta *getSta() { return sta_; }
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

protected:
  void collectLocalFanouts(Pin *drvr_pin, InstanceSet &local_instances);
  void collectLocalFaninSiblings(Pin *pin, PinSet &visited_pins, 
                                 InstanceSet &local_instances);
  void topoSortVertices(PtGraph *pt_graph);

  // Delay calculation methods
  void seedRootSlews();
  void zeroSlewAndWireDelays(PtVertex &drvr_pt_vertex,
                           const RiseFall *rf,
                           PtGraph *pt_graph);
  void loadSlewFromGraph(PtVertex &pt_vertex, PtGraph *pt_graph);
  void findVertexDelays(VertexId pt_vertex_id, 
                        ArcDelayCalc *arc_delay_calc,
                        PtGraph *pt_graph);
  void seedRootSlew(PtVertex &pt_vertex, PtGraph *pt_graph);
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
  Slew edgeFromSlew(const PtVertex &from_pt_vertex,
                    const RiseFall *from_rf,
                    const PtEdge &pt_edge,
                    const DcalcAnalysisPt *dcalc_ap,
                    PtGraph *pt_graph);
  Slew edgeFromSlew(const PtVertex &from_pt_vertex,
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
  
  float delayLmSum(Instance *inst, const MinMax *minmax);
  float delayLmSum(PtGraph *pt_graph, DcalcAnalysisPt *dcalc_ap);
  void graphPop();
  void setSta(Sta *sta) { sta_ = sta; }
  LocalCost initAndGetLocalTimingCost(PtGraph *pt_graph, ArcDelayCalc *arc_delay_calc);
  LocalCost increAndGetLocalTimingCost(PtGraph *pt_graph, 
                                    ArcDelayCalc *arc_delay_calc,
                                    LibertyCell *equiv_cell);
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

private:
  Sta *sta_;

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
  bool equiv_cells_made_ = false;

  std::string debug_label_ = "LocalSTA";

  friend class IncreSta;
  friend class TestLrf;
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

#pragma once
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "parasitics/ConcreteParasitics.hh"
#include "sta/Map.hh"
#include "sta/StaState.hh"

namespace sta {
  class Parasitics;
  class Parasitic;
  class DmpCeffDelayCalc;
  class ConcreteParasitic;
  class ConcreteParasiticNetwork;
  class ParasiticNode;
  class StaState;
  class ConcretePiElmore;


  typedef Map<const Pin*, ConcreteParasitic**> ConcreteParasiticMap;
  typedef Map<const Net*, ConcreteParasiticNetwork**> ConcreteParasiticNetworkMap;

}

namespace lrf {

using namespace sta;

class PtGraph;
class PtVertex;
class ParasiticCopyHelper;

class LocalParasitics: public ConcreteParasitics
{
public:
  LocalParasitics(sta::StaState* state, bool parallelism_exists = true);
  virtual ~LocalParasitics();
  void initParasiticMapFromBase();
  void reduceparasitic(const Parasitic *parasitic_network,
                        const Net *net,
                        const Corner *corner,
                        const MinMaxAll *min_max);
  Parasitic *reduceParasitic(const Parasitic *parasitic_network,
                             const Pin *drvr_pin,
                             const RiseFall *rf,
                             const DcalcAnalysisPt *ap);
  Parasitic *reduceToLocalPiElmore(const Parasitic *parasitic_network,
                              const Pin *drvr_pin,
                              const RiseFall *rf,
                              const Corner *corner,
                              const PtGraph *pt_graph,
                              const MinMax *min_max,
                              const ParasiticAnalysisPt *ap);
  Parasitic *makeLocalPiElmore(const Parasitic *parasitic_network,
                               const Pin *drvr_pin,
                               ParasiticNode *drvr_node,
                               float coupling_cap_factor,
                               const RiseFall *rf,
                               const Corner *corner,
                               const PtGraph *pt_graph,
                               const MinMax *min_max,
                               const ParasiticAnalysisPt *ap);
  Parasitic *findLocalParasiticNetwork(const Net *net, const ParasiticAnalysisPt *ap) const;
  Parasitic *findLocalParasiticNetwork(const Pin *drvr_pin);
  Parasitic *findLocalParasitic(const Pin *drvr_pin, const RiseFall *rf, const DcalcAnalysisPt *ap);
  // When run this in parallel, pt graph should be conflicted graph
  void recomputeLocalParasitics(PtGraph *pt_graph);
  void reduceLocalParasitic(Parasitic* parasitic_network,
                            PtGraph *pt_graph,
                            const PtVertex &pt_drvr_vertex,
                            const DcalcAnalysisPt *dcalc_ap);
  // Reduce parasitic networks into PtGraph-local PtPiElmore objects.
  void recomputePtParasitics(PtGraph *pt_graph);
  // Rebuild PtPiElmore for a single driver vertex from the original parasitic network.
  void recomputeSinglePtParasitic(PtGraph *pt_graph, sta::VertexId drvr_vid);
  // Parasitic *

protected:
  float pinCapacitance(const ParasiticNode *node, 
                        const RiseFall *rf,
                        const Corner *corner,
                        const MinMax *min_max) const;
  float pinCapacitance(const Pin *pin,
                        const RiseFall *rf,
                        const Corner *corner,
                        const MinMax *min_max) const;


  ConcreteParasiticMap local_drvr_parasitic_map_;
  ConcreteParasiticNetworkMap local_parasitic_network_map_;
  bool parallelism_exists_ = true;
  bool initialized_ = false;
  ParasiticCopyHelper *copy_helper_;
  
private:
};

// This helper is used to copy parasitics of different types
class ParasiticCopyHelper: public StaState
{
public:
  ParasiticCopyHelper(StaState* state);
  ~ParasiticCopyHelper() = default;

  Parasitic *getCopy(const Parasitic *from_parasitic);
  ConcreteParasitic *getCopy(const ConcreteParasitic *from_parasitic);
  ConcreteParasiticNetwork *getCopy(const ConcreteParasiticNetwork *from_parasitic_network);
  ConcretePiElmore *getCopy(const ConcretePiElmore *from_pi_elmore);
};

}  // namespace lrf
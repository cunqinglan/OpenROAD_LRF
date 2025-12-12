

#include "sta/Search.hh"
#include "search/TagGroup.hh"

#include "lrf/LocalSta.hh"
#include "lrf/LocalSearch.hh"
#include "lrf/PtGraph.hh"



namespace lrf
{
CombinationArrivalVisitor::CombinationArrivalVisitor(StaState *state,
                                                    PtGraph *pt_graph)
  : sta_(state),
    pt_graph_(pt_graph)
{
}

CombinationArrivalVisitor::~CombinationArrivalVisitor()
{
}

void 
CombinationArrivalVisitor::visit(PtVertex &pt_vertex)
{
  if (!pt_vertex.hasFanin())
    seedLocalRootArrivals(pt_vertex);
  else
    findVertexArrival(pt_vertex);
}

void 
CombinationArrivalVisitor::seedLocalRootArrivals(PtVertex &pt_vertex)
{
  // Since we have initialize all paths for the local
  // root vertices, just skip seeding here.
  // Maybe in the future we can add some checks
  return;
}

void
CombinationArrivalVisitor::findVertexArrival(PtVertex &pt_vertex)
{
  



}

} // namespace lrf
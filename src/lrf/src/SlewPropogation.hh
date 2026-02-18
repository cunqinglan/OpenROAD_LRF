
#pragma once

#include "sta/Graph.hh"
#include "sta/Delay.hh"


namespace lrf {

class SlewPropagation : public dbStaState
{
public:
  SlewPropagation(sta::dbSta *db_sta);
  ~SlewPropagation();

  // Propagate slew changes from the given vertex to its fanout vertices.
  // Returns true if any slews were updated, false otherwise.
  bool propagateSlewFromVertex(sta::Vertex *vertex);
};

class SlewPropagateVisitor : public BfsFwdIterator
{
public:
  SlewPropagateVisitor(sta::dbSta *db_sta, SearchPred *search_pred);
  ~SlewPropagateVisitor();

  // Override to propagate slew changes along the edge to adjacent vertices.
  void enqueueAdjacentVertices(sta::Vertex *vertex,
                              SearchPred *search_pred,
                              Level to_level) override;
  
}


} // namespace lrf
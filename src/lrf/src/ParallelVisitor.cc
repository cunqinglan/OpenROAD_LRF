#include "ParallelVisitor.hh"
#include "sta/GraphDelayCalc.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "sta/Liberty.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "PtGraph.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/TimingArc.hh"
#include "sta/Search.hh"
#include "search/TagGroup.hh"
#include "sta/EquivCells.hh"

namespace sta {
}



namespace lrf {

typedef float LocalCost;

ParallelLrVisitor::ParallelLrVisitor(sta::dbSta *db_sta, LocalSta *local_sta) :
  db_sta_(db_sta),
  ref_inst_(nullptr),
  local_sta_(local_sta),
  arc_delay_calc_(local_sta_->arcDelayCalc()->copy())
{
  // Since this visitor is created in serial, 
  // make equivalent cells here is safe.
  slack_before_swap_ = sta::MinMax::max()->initValue();
}

ParallelLrVisitor::~ParallelLrVisitor()
{
  delete arc_delay_calc_;
}

void 
ParallelLrVisitor::visit(sta::Instance *inst)
{
  printf("ParallelLrVisitor::visit instance %s\n",
         db_sta_->network()->pathName(inst));
  fflush(stdout);
  best_cell_ = nullptr;
  // The visit do following things:
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  // 4. Submmit the best cell swap to the resizer.
  visited_instances_.push_back(db_sta_->network()->pathName(inst));
  sta::LibertyCell *cell = db_sta_->network()->libertyCell(inst);
  if (cell) {
    sta::LibertyCellSeq *equiv_cells = db_sta_->equivCells(cell);
    if (equiv_cells == nullptr) {
      printf("ParallelLrVisitor::visit no equiv cells for %s\n",
             cell->name());
      fflush(stdout);
      return;
    }
    pt_graph_ = local_sta_->makePtGraph(inst, false);
    // Compute Original delays
    LocalCost original_cost = local_sta_->
                initAndGetLocalTimingCost(pt_graph_, arc_delay_calc_);
    LocalCost best_cost = original_cost;
    // Initialize the slack before swap
    slack_before_swap_ = local_sta_->localSlackAroundRef(pt_graph_);
    
    best_cell_ = cell;
    for (sta::LibertyCell *equiv_cell : *equiv_cells) {
      // This first virtual swap the cell in pt graph,
      // then recompute local delays, arrivals, requireds.
      LocalCost swapped_cost = local_sta_->
        increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, equiv_cell);
      sta::Slack swapped_slack = 
                      local_sta_->localSlackAroundRef(pt_graph_);
      // Do local slack check
      if (swapped_cost < best_cost 
          && swapped_slack >= slack_before_swap_) {
        best_cost = swapped_cost;
        best_cell_ = equiv_cell;
      }
    }
  }
}

ParallelLrVisitor *
ParallelLrVisitor::copy() const
{
  return new ParallelLrVisitor(db_sta_, local_sta_);
}

void
ParallelLrVisitor::printVisitedInstNames() const
{
  for (const std::string &inst_name : visited_instances_) {
    printf("Visited instance: %s\n", inst_name.c_str());
  }
}


void
ParallelLrVisitor::applyChangesToDb(rsz::Resizer *resizer)
{
  // First apply best cell type changes to OpenROAD
  if (best_cell_ && pt_graph_->refInstance()) {
    printf("Applying best cell %s to instance %s\n",
           best_cell_->name(),
           db_sta_->network()->pathName(pt_graph_->refInstance()));
    fflush(stdout);
    sta::LibertyCell *from_lib_cell = 
                db_sta_->network()->libertyCell(pt_graph_->refInstance());
    if (best_cell_->name() == from_lib_cell->name()) {
      // No change needed
      return;
    } else {
      if (!sta::equivCellsArcs(from_lib_cell, best_cell_)) {
        // No change needed
        return;
      }
      db_sta_->replaceCell(pt_graph_->refInstance(), best_cell_);
    }
  }
  // Then update timing information from PtGraph to sta::Graph.
  // updateTimingFromPtGraph();
}

void
ParallelLrVisitor::updateTimingFromPtGraph()
{
  for (VertexId vertex_id : pt_graph_->sortedVertexIds()) {
    updateVertexInfo(vertex_id);
  }
  for (const PtEdge &pt_edge : pt_graph_->ptEdges()) {
    updateEdgeInfo(pt_edge.objectIdx());
  }
}

void 
ParallelLrVisitor::updateVertexInfo(sta::VertexId vertex_id)
{
  PtVertex &pt_vertex = pt_graph_->ptVertex(vertex_id);
  if (!pt_vertex.vertex()) {
    return;
  }
  sta::Vertex *sta_vertex = db_sta_->graph()->vertex(vertex_id);
  // Update slews
  sta::Slew *slews[pt_vertex.slewCount()];
  sta::Slew *pt_graph_slews = pt_vertex.slews();
  sta::Graph *sta_graph = db_sta_->graph();
  for (const sta::RiseFall *rf : sta::RiseFall::range()) {
    for (int i = 0; i < sta_graph->apCount(); i++) {
      sta::Slew slew = pt_graph_->slew(pt_vertex, rf, i);
      sta_graph->setSlew(sta_vertex, rf, i, slew);
    }
  }

  // update paths
  // Do not replace the path array, as it causes ownership issues with prev_path_.
  // Instead, copy the updated timing values (arrival, required) back to the existing paths.
  sta::Path *pt_paths = pt_vertex.paths();
  sta::Path *sta_paths = sta_vertex->paths();
  
  if (pt_paths && sta_paths) {
    sta::TagGroup *pt_tag_group = pt_graph_->tagGroup(pt_vertex);
    sta::TagGroup *sta_tag_group = db_sta_->search()->tagGroup(sta_vertex);
    if (pt_tag_group->index() != sta_tag_group->index()) {
      printf("TagGroup mismatch for vertex %s: PtTagGroup index %u, StaTagGroup index %u\n",
             sta_vertex->to_string(db_sta_).c_str(),
             pt_tag_group->index(),
             sta_tag_group->index());
      // We neglect many cases here for simplicity.
      // So when tag groups do not match, we skip 
      // updating paths, and just keep the existing ones.
      return;
    }
    size_t path_count = pt_tag_group->pathCount();
    
    for (size_t i = 0; i < path_count; i++) {
      sta_paths[i].setArrival(pt_paths[i].arrival());
      sta_paths[i].setRequired(pt_paths[i].required());
    }
  }
}

void 
ParallelLrVisitor::updateEdgeInfo(sta::EdgeId edge_id)
{
  PtEdge &pt_edge = pt_graph_->edge(edge_id);
  if (!pt_edge.edge()) {
    return;
  }
  sta::Edge *sta_edge = db_sta_->graph()->edge(edge_id);
  // Update arc delays
  for (const sta::TimingArc *arc : sta_edge->timingArcSet()->arcs()) {
    for (int i = 0; i < db_sta_->graph()->apCount(); i++) {
      sta::ArcDelay delay = pt_graph_->arcDelay(pt_edge, arc, i);
      db_sta_->graph()->setArcDelay(sta_edge, arc, i, delay);
    }
  }
}

} // namespace lrf



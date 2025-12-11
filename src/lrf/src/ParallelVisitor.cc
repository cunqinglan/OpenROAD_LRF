#include "ParallelVisitor.hh"
#include "sta/GraphDelayCalc.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "sta/Liberty.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"



namespace sta {
}



namespace lrf {

typedef float LocalCost;

ParallelLrVisitor::ParallelLrVisitor(sta::dbSta *db_sta, 
        sta::Instance * ref_inst, LocalSta *local_sta, Resizer *resizer) :
  db_sta_(db_sta),
  ref_inst_(ref_inst),
  local_sta_(local_sta),
  arc_delay_calc_(local_sta->arc_delay_calc_->copy()),
  resizer_(resizer)
{
  // Since this visitor is created in serial, 
  // make equivalent cells here is safe.
  if (!local_sta_->equiv_cells_made()) {
    resizer_->makeEquivCells();
    local_sta_->setEquivCellsMade(true);
  }
}

ParallelLrVisitor::~ParallelLrVisitor()
{
  delete arc_delay_calc_;
}

void 
ParallelLrVisitor::visit(Instance *inst)
{
  // The visit do three things:
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  LibertyCell *cell = db_sta_->network()->libertyCell(inst);
  if (cell) {
    LibertyCellSeq *equiv_cells = resizer_->getSwappableCells(cell);
    if (equiv_cells == nullptr) {
      printf("ParallelLrVisitor::visit no equiv cells for %s\n",
             cell->name().c_str());
      fflush(stdout);
      return;
    }
    PtGraph *pt_graph = local_sta_->makePtGraph(inst, false);
    // Compute Original delays
    LocalCost original_cost = local_sta_->initAndGetLocalTimingCost(pt_graph, arc_delay_calc_);
    LocalCost best_cost = original_cost;
    LibertyCell *best_cell = cell;
    for (LibertyCell *equiv_cell : *equiv_cells) {
      printf("ParallelLrVisitor::visit finding delays for equiv cell %s\n",
             equiv_cell->name().c_str());
      fflush(stdout);
      local_sta_->virtualSwap(pt_graph, inst, equiv_cell);
      LocalCost swapped_cost = local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_);
      if (swapped_cost < best_cost) {
        best_cost = swapped_cost;
        best_cell = equiv_cell;
      }
    }
  }
}



} // namespace lrf



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

ParallelLrVisitor::ParallelLrVisitor(sta::dbSta *db_sta, LocalSta *local_sta, rsz::Resizer *resizer) :
  db_sta_(db_sta),
  ref_inst_(nullptr),
  local_sta_(local_sta),
  arc_delay_calc_(local_sta_->arcDelayCalc()->copy()),
  resizer_(resizer)
{
  // Since this visitor is created in serial, 
  // make equivalent cells here is safe.
  if (!local_sta_->equivCellsMade()) {
    resizer_->makeEquivCells();
    local_sta_->setEquivCellsMade(true);
  }
  slack_before_swap_ = sta::MinMax::max()->initValue();
}

ParallelLrVisitor::~ParallelLrVisitor()
{
  delete arc_delay_calc_;
}

void 
ParallelLrVisitor::visit(sta::Instance *inst)
{
  // The visit do following things:
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  // 4. Submmit the best cell swap to the resizer.
  sta::LibertyCell *cell = db_sta_->network()->libertyCell(inst);
  if (cell) {
    sta::LibertyCellSeq equiv_cells = resizer_->getSwappableCells(cell);
    // if (equiv_cells == nullptr) {
    //   printf("ParallelLrVisitor::visit no equiv cells for %s\n",
    //          cell->name());
    //   fflush(stdout);
    //   return;
    // }
    PtGraph *pt_graph = local_sta_->makePtGraph(inst, false);
    // Compute Original delays
    LocalCost original_cost = local_sta_->
                initAndGetLocalTimingCost(pt_graph, arc_delay_calc_);
    LocalCost best_cost = original_cost;
    // Initialize the slack before swap
    slack_before_swap_ = local_sta_->localSlackAroundRef(pt_graph);
    
    sta::LibertyCell *best_cell = cell;
    for (sta::LibertyCell *equiv_cell : equiv_cells) {
      printf("ParallelLrVisitor::visit finding delays for equiv cell %s\n",
             equiv_cell->name());
      fflush(stdout);
      // This first virtual swap the cell in pt graph,
      // then recompute local delays, arrivals, requireds.
      LocalCost swapped_cost = local_sta_->
        increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, equiv_cell);
      sta::Slack swapped_slack = 
                      local_sta_->localSlackAroundRef(pt_graph);
      // Do local slack check
      if (swapped_cost < best_cost 
          && swapped_slack >= slack_before_swap_) {
        best_cost = swapped_cost;
        best_cell = equiv_cell;
      }
    }
  }
}

// void
// TestLrf::testParallelVisitor(std::vector<char*> &inst_names, sta::dbSta* sta, 
//                                rsz::Resizer *resizer, odb::dbBlock *block)
// {
//   IncreSta *incre_sta = new IncreSta(sta);
//   LocalSta *local_sta = incre_sta->localSta();
//   sta::dbNetwork *db_network = sta->getDbNetwork();
//   resizer->makeEquivCells();

//   // 只测 2~3 个线程
//   sta::DispatchQueue dq(/*thread_count=*/3);
//   dq.setThreadCount(3);

//   // 为每个实例准备一个 ParallelLrVisitor，并投递到队列
//   size_t n = std::min<size_t>(inst_names.size(), 6); // 随便挑几个任务
//   std::vector<std::unique_ptr<ParallelLrVisitor>> visitors;
//   visitors.reserve(n);

//   for (size_t i = 0; i < n; ++i) {
//     char *inst_name = inst_names[i];
//     odb::dbInst *db_inst = block->findInst(inst_name);
//     if (!db_inst) {
//       printf("Instance %s not found in the block.\n", inst_name);
//       continue;
//     }
//     sta::Instance *sta_inst = db_network->dbToSta(db_inst);
//     sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
//     resizer->getSwappableCells(orig_cell);

//     visitors.emplace_back(std::make_unique<ParallelLrVisitor>(sta, local_sta, resizer));

//     // 投递一个任务，执行 visit
//     dq.dispatch([vis = visitors.back().get(), sta_inst]() {
//       vis->visit(sta_inst);
//     });
//   }

//   // 等待所有任务完成
//   dq.finishTasks();
// }

// void
// TestLrf::testParallelVisitor(std::vector<char*> &inst_names, sta::dbSta* sta, 
//                                rsz::Resizer *resizer, odb::dbBlock *block)
// {
//   IncreSta *incre_sta = new IncreSta(sta);
//   LocalSta *local_sta = incre_sta->localSta();
//   sta::dbNetwork *db_network = sta->getDbNetwork();
//   resizer->makeEquivCells();

//   // 只启动 2~3 个线程
//   const size_t thread_count = std::min<size_t>(3, inst_names.size());
//   std::vector<std::thread> threads;
//   std::vector<std::unique_ptr<ParallelLrVisitor>> visitors;
//   visitors.reserve(thread_count);

//   for (size_t i = 0; i < thread_count; ++i) {
//     char *inst_name = inst_names[i];
//     odb::dbInst *db_inst = block->findInst(inst_name);
//     if (!db_inst) {
//       printf("Instance %s not found in the block.\n", inst_name);
//       continue;
//     }
//     sta::Instance *sta_inst = db_network->dbToSta(db_inst);
//     sta::LibertyCell *orig_cell = sta->network()->libertyCell(sta_inst);
//     resizer->getSwappableCells(orig_cell);

//     visitors.emplace_back(std::make_unique<ParallelLrVisitor>(sta, local_sta, resizer));
//     threads.emplace_back([vis = visitors.back().get(), sta_inst]() {
//       vis->visit(sta_inst);
//     });
//   }

//   for (auto &t : threads) {
//     if (t.joinable()) t.join();
//   }
// }

} // namespace lrf



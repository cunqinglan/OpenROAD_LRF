#include "ParallelVisitor.hh"
#include "sta/GraphDelayCalc.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "sta/Liberty.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "PtGraph.hh"
#include "sta/Delay.hh"
#include "sta/Sdc.hh"
#include "sta/Clock.hh"
#include "sta/Graph.hh"
#include "sta/TimingArc.hh"
#include "sta/Search.hh"
#include "search/TagGroup.hh"
#include "sta/EquivCells.hh"
#include "lrf/TestLrf.hh"
#include "ParallelLibData.hh"

#include <vector>
#include <mutex>
#include <algorithm>
#include "sta/Fuzzy.hh"

namespace sta {
}



namespace lrf {

extern std::mutex g_odb_sta_access_mutex;

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

bool 
ParallelLrVisitor::checkVisitorStatus() const
{
  if (db_sta_ == nullptr || local_sta_ == nullptr || arc_delay_calc_ == nullptr) {
    return false;
  }
  if (swappable_cells_cache_->empty() || inst_info_map_->empty()) {
    return false;
  }
  return true;
}

float
ParallelLrVisitor::swapCost(float delay_lm_sum, float power)
{
  float swap_cost = 100 * delay_lm_sum / average_delay_ 
                    + power / average_leakage_;
  return swap_cost;
}

bool 
ParallelLrVisitor::singleGateSizing(sta::Instance *inst)
{
  best_cell_ = nullptr;
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  // 4. Submmit the best cell swap to the resizer.
  visited_instances_.push_back(db_sta_->network()->pathName(inst));
  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (ori_cell) {
    auto info_it = inst_info_map_->find(inst);
    if (info_it == inst_info_map_->end()) {
      // Instance skipped during pre-calculation (likely no swappable cells), not an error.
      printf("ParallelLrVisitor::visit instance %s of type %s not found in inst_info_map_, skipped\n",
             db_sta_->network()->pathName(inst),
             ori_cell->name());
      fflush(stdout);
      return false;
    }
    sta::LibertyCellSeq *equiv_cells = info_it->second->equiv_cells;
    
    if (equiv_cells == nullptr || equiv_cells->empty()) {
      printf("ParallelLrVisitor::visit no equiv cells for %s\n",
             ori_cell->name());
      fflush(stdout);
      equiv_cells = db_sta_->equivCells(ori_cell);
      if (equiv_cells == nullptr) {
        printf("ParallelLrVisitor::visit no equiv cells from db_sta_ for %s\n",
               ori_cell->name());
        fflush(stdout);
      }
      return false;
    } 

    // printf("ParallelLrVisitor::visit instance %s of type %s with %lu legal equivalent cells\n",
    //      db_sta_->network()->pathName(inst),
    //      db_sta_->network()->libertyCell(inst)->name(),
    //      legal_equiv_cells.size());
    // fflush(stdout);

    pt_graph_ = local_sta_->makePtGraph(inst, false);
    
    best_cell_ = ori_cell;
    bool orig_inequiv = false;
    std::vector<float> vec_cost_slack(equiv_cells->size() * 2, std::numeric_limits<float>::max());
    float best_cost = std::numeric_limits<float>::max();
    int cnt = 0;
    for (sta::LibertyCell *equiv_cell : *equiv_cells) {
      if (!sta::equivCellsArcs(ori_cell, equiv_cell)) {
        throw std::runtime_error("ParallelLrVisitor::visit found non-equivalent cell in equiv_cells");
      }
      float leakage = (*inst_info_map_)[inst]->cell_leakages[cnt];
      float delay_lm_sum = local_sta_->
        increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, equiv_cell).delay_lm_sum;
      float swapped_cost = swapCost(delay_lm_sum, leakage);
      sta::Slack swapped_slack = 
                      local_sta_->localSlackAroundRef(pt_graph_);
      vec_cost_slack[cnt * 2] = swapped_cost;
      vec_cost_slack[cnt * 2 + 1] = swapped_slack;
      if (equiv_cell == ori_cell) {
        // For original cell, update best cost directly
        slack_before_swap_ = swapped_slack;
        orig_inequiv = true;
      }
      cnt++;
      
      // printf("ParallelLrVisitor::visit metics: from delay_lm_sum %f to %f for cell %s, slack before swap %f, after swap %f\n",
      //        best_cost * 1e12,
      //        swapped_cost * 1e12,
      //        equiv_cell->name(),
      //        slack_before_swap_ * 1e12,
      //        swapped_slack * 1e12);
      // fflush(stdout);
    }
    for (size_t i = 0; i < equiv_cells->size(); i++) {
      float cost = vec_cost_slack[i * 2];
      float slack = vec_cost_slack[i * 2 + 1];
      if (cost < best_cost
          && slack >= slack_before_swap_ * 1.05) {
        best_cell_ = (*equiv_cells)[i];
        best_cost = cost;
      }
    }
    if (best_cell_ == ori_cell || !orig_inequiv) {
      return false;
    }
    // First compute the final timing after choosing best cell
    if (best_cell_ != equiv_cells->at(equiv_cells->size() - 1)) 
      local_sta_->increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, best_cell_);
    return true;
  } 
  printf("ParallelLrVisitor::visit no liberty cell for instance %s\n",
         db_sta_->network()->pathName(inst));
  fflush(stdout);
  return false;
}

bool
ParallelLrVisitor::equivVtCells(sta::LibertyCell *cell1, sta::LibertyCell *cell2)
{
  sta::dbNetwork *network = db_sta_->getDbNetwork();
  odb::dbMaster *master1 = network->staToDb(cell1);
  odb::dbMaster *master2 = network->staToDb(cell2);
  if (master1 == nullptr || master2 == nullptr) {
    return false;
  }
  if (!fuzzyEqual(master1->getArea(), master2->getArea())) {
    return false;
  }
  if (master1->getSite() != master2->getSite()) {
    return false;
  }
  if (!sta::stringEqIf(cell1->footprint(), cell2->footprint())) {
    return false;
  }
  if (cell1->userFunctionClass() && cell2->userFunctionClass() &&
    !sta::stringEqIf(cell1->userFunctionClass(), cell2->userFunctionClass())) {
    return false;
  }
  return true;
}

std::vector<std::pair<sta::LibertyCell*, std::pair<size_t, size_t>>>
ParallelLrVisitor::getLegalEquivCells(
    std::vector<sta::LibertyCellSeq> *equiv_cells_vec,
    sta::LibertyCell *ori_cell)
{
  std::vector<std::pair<sta::LibertyCell*, std::pair<size_t, size_t>>> legal_equiv_cells;
  size_t scope_count = 4;
  size_t i = 0, j;
  for (const sta::LibertyCellSeq &cell_seq : *equiv_cells_vec) {
    j = 0;
    if (cell_seq.size() == 0) {
      i++;
      continue;
    }
    for (sta::LibertyCell *equiv_cell : cell_seq) {
      if (ori_cell == equiv_cell) {
        size_t start_index = (j > scope_count) ? (j - scope_count) : 0;
        size_t end_index = std::min(cell_seq.size() - 1, j + scope_count);
        // Add cells with same VT as orig into legal_equiv_cells
        for (size_t k = start_index; k <= end_index; k++) {
          sta::LibertyCell *near_cell = cell_seq[k];
          legal_equiv_cells.push_back(std::make_pair(near_cell, std::make_pair(i, k)));
        }
        if (i > 0) {
          // Also add cells from lower VT
          sta::LibertyCellSeq lower_vt_seq = (*equiv_cells_vec)[i - 1];
          j = 0;
          for (sta::LibertyCell *lower_vt_cell : lower_vt_seq) {
            if (equivVtCells(ori_cell, lower_vt_cell)) {
              size_t start_index = (j > scope_count) ? (j - scope_count) : 0;
              size_t end_index = std::min(lower_vt_seq.size() - 1, j + scope_count);
              // Add nearby cells with lower VT into legal_equiv_cells
              for (size_t k = start_index; k <= end_index; k++) {
                sta::LibertyCell *near_cell = lower_vt_seq[k];
                legal_equiv_cells.push_back(std::make_pair(near_cell, std::make_pair(i - 1, k)));
              }
              break;
            }
            j++;
          }
          j = 0;
          // Also add cells from higher VT
          if (i + 1 < equiv_cells_vec->size()) {
            sta::LibertyCellSeq higher_vt_seq = (*equiv_cells_vec)[i + 1];
            for (sta::LibertyCell *higher_vt_cell : higher_vt_seq) {
              if (equivVtCells(ori_cell, higher_vt_cell)) {
                size_t start_index = (j > scope_count) ? (j - scope_count) : 0;
                size_t end_index = std::min(higher_vt_seq.size() - 1, j + scope_count);
                // Add nearby cells with higher VT into legal_equiv_cells
                for (size_t k = start_index; k <= end_index; k++) {
                  sta::LibertyCell *near_cell = higher_vt_seq[k];
                  legal_equiv_cells.push_back(std::make_pair(near_cell, std::make_pair(i + 1, k)));
                }
                return legal_equiv_cells;
              }
              j++;
            }
          }
        }
        break;
      }
      j++;
    }
    i++;
  }
  return legal_equiv_cells;
}

bool 
ParallelLrVisitor::singleGateSizingV1(sta::Instance *inst)
{
  best_cell_ = nullptr;
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  // 4. Submmit the best cell swap to the resizer.
  visited_instances_.push_back(db_sta_->network()->pathName(inst));
  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (ori_cell) {
    if (parallel_lib_data_ == nullptr || parallel_lib_data_->inst_to_vid_map_ == nullptr) {
      throw std::runtime_error("ParallelLrVisitor::singleGateSizingV1 parallel_lib_data/inst_to_vid_map_ is null");
    }
    const auto vid_it = parallel_lib_data_->inst_to_vid_map_->find(inst);
    if (vid_it == parallel_lib_data_->inst_to_vid_map_->end()) {
      printf("ParallelLrVisitor::singleGateSizingV1 inst %s not found in inst_to_vid_map_\n",
             db_sta_->network()->pathName(inst));
      fflush(stdout);
      return false;
    }
    ParallelLocalCellInfo &cell_info = parallel_lib_data_->cell_info_vec_[vid_it->second];
    std::vector<sta::LibertyCellSeq> *equiv_cells_vec = cell_info.equiv_cells;
    
    if (equiv_cells_vec == nullptr || equiv_cells_vec->empty() ||
        (equiv_cells_vec->size() == 1 && (*equiv_cells_vec)[0].size() == 1)) {
      printf("ParallelLrVisitor::singleGateSizingV1 no equiv cells for %s, inst = %s\n",
             ori_cell->name(),
             db_sta_->network()->pathName(inst));
      fflush(stdout);
      return false;
    }
    std::vector<std::pair<sta::LibertyCell*, std::pair<size_t, size_t>>> 
                    legal_equiv_cells = getLegalEquivCells(equiv_cells_vec, ori_cell);
    if (legal_equiv_cells.size() < 2) {
      printf("ParallelLrVisitor::singleGateSizingV1 for inst %s no legal equiv cells for %s\n",
             db_sta_->network()->pathName(inst),
             ori_cell->name());
      fflush(stdout);
      return false;
    }

    // printf("ParallelLrVisitor::visit instance %s of type %s with %lu legal equivalent cells\n",
    //      db_sta_->network()->pathName(inst),
    //      db_sta_->network()->libertyCell(inst)->name(),
    //      legal_equiv_cells.size());
    // fflush(stdout);

    pt_graph_ = local_sta_->makePtGraph(inst, false);
    
    best_cell_ = ori_cell;
    float best_cost = std::numeric_limits<float>::max();
    std::vector<float> vec_cost_slack(legal_equiv_cells.size() * 2, std::numeric_limits<float>::max());
    bool orig_inequiv = false;
    int cnt = 0;
    for (auto &equiv_cell_pair : legal_equiv_cells) {
      sta::LibertyCell *equiv_cell = equiv_cell_pair.first;
      float leakage = cell_info.cell_leakages[equiv_cell_pair.second.first][equiv_cell_pair.second.second]; 
      float delay_lm_sum = local_sta_->
        increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, equiv_cell).delay_lm_sum;
      float swapped_cost = swapCost(delay_lm_sum, leakage);
      sta::Slack swapped_slack = 
                      local_sta_->localSlackAroundRef(pt_graph_);
      vec_cost_slack[cnt * 2] = swapped_cost;
      vec_cost_slack[cnt * 2 + 1] = swapped_slack;
      if (equiv_cell == ori_cell) {
        // For original cell, update best cost directly
        slack_before_swap_ = swapped_slack;
        orig_inequiv = true;
      }
      cnt++;
      
      // printf("ParallelLrVisitor::visit metics: from delay_lm_sum %f to %f for cell %s, slack before swap %f, after swap %f\n",
      //        best_cost * 1e12,
      //        swapped_cost * 1e12,
      //        equiv_cell->name(),
      //        slack_before_swap_ * 1e12,
      //        swapped_slack * 1e12);
      // fflush(stdout);
    }
    if (!orig_inequiv) {
      printf("ParallelLrVisitor::singleGateSizingV1 for inst %s original cell %s not in legal equiv cells\n",
             db_sta_->network()->pathName(inst),
             ori_cell->name());
      fflush(stdout);
      throw std::runtime_error("Original cell not in legal equiv cells");
    }
    for (size_t i = 0; i < legal_equiv_cells.size(); i++) {
      float cost = vec_cost_slack[i * 2];
      float slack = vec_cost_slack[i * 2 + 1];
      if (cost < best_cost
          && slack >= slack_before_swap_ * slack_margin_) {
        best_cell_ = legal_equiv_cells[i].first;
        best_cost = cost;
      }
    }
    if (best_cell_ == ori_cell) {
      return false;
    }
    // First compute the final timing after choosing best cell
    if (best_cell_ != legal_equiv_cells[legal_equiv_cells.size() - 1].first) 
      local_sta_->increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, best_cell_);
    return true;
  } 
  printf("ParallelLrVisitor::visit no liberty cell for instance %s\n",
         db_sta_->network()->pathName(inst));
  fflush(stdout);
  return false;
}

bool
ParallelLrVisitor::visit(sta::Instance *inst)
{
  // std::lock_guard<std::mutex> lock(g_odb_sta_access_mutex);
  if (!checkVisitorStatus()) {
    throw std::runtime_error("ParallelLrVisitor::visit visitor status invalid");
  }
  // if (!parallel_lib_data_) {
  //   printf("ParallelLrVisitor::visit parallel_lib_data_ is null\n");
  //   fflush(stdout);
  //   return false;
  // }
  return singleGateSizing(inst);
  // return singleGateSizingV1(inst);
}

bool 
ParallelLrVisitor::visit(sta::Instance *inst,
                             TimingRecord &timing_record)
{
  // std::lock_guard<std::mutex> lock(g_odb_sta_access_mutex);
  best_cell_ = nullptr;
  timing_record.inst = inst;
  // The visit do following things:
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  // 4. Submmit the best cell swap to the resizer.
  visited_instances_.push_back(db_sta_->network()->pathName(inst));
  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (ori_cell) {
    timing_record.orig_cell = ori_cell;
    sta::LibertyCellSeq *equiv_cells = db_sta_->equivCells(ori_cell);
    if (equiv_cells == nullptr) {
      printf("ParallelLrVisitor::visit no equiv cells for %s\n",
             ori_cell->name());
      fflush(stdout);
      return false;
    } 
    // Examine if all equiv cells are legal
    sta::LibertyCellSeq legal_equiv_cells;
    for (sta::LibertyCell *equiv_cell : *equiv_cells) {
      if (sta::equivCellsArcs(ori_cell, equiv_cell)) {
        legal_equiv_cells.push_back(equiv_cell);
      }
    }
    if (legal_equiv_cells.size() < 2) {
      printf("ParallelLrVisitor::visit no legal equiv cells for %s\n",
             ori_cell->name());
      fflush(stdout);
      return false;
    }

    printf("ParallelLrVisitor::visit instance %s of type %s with %lu legal equivalent cells\n",
         db_sta_->network()->pathName(inst),
         db_sta_->network()->libertyCell(inst)->name(),
         legal_equiv_cells.size());
    fflush(stdout);

    pt_graph_ = local_sta_->makePtGraph(inst, false);
    // Compute Original delays
    DelayLmSumResult original_result = local_sta_->
                initAndGetLocalTimingCost(pt_graph_, arc_delay_calc_);
    // Initialize the slack before swap
    slack_before_swap_ = local_sta_->localSlackAroundRef(pt_graph_);
    
    // Record original timing
    // GraphTiming orig_cell_timing;
    // orig_cell_timing.cell = ori_cell;
    // recordGraphTimingFromPtGraph(db_sta_, pt_graph_, orig_cell_timing);
    // timing_record.liberty_timing_map[std::string(ori_cell->name())] = orig_cell_timing;
    
    best_cell_ = ori_cell;
    DelayLmSumResult best_result = original_result;
    int cnt = 1;
    for (sta::LibertyCell *equiv_cell : legal_equiv_cells) {
      cnt++;
      if (cnt > 2) break; // Only test first 1 equiv cells
      // This first virtual swap the cell in pt graph,
      // then recompute local delays, arrivals, requireds.
      // printf("ParallelLrVisitor::visit testing equiv cell %s for instance %s\n",
      //        equiv_cell->name(),
      //        db_sta_->network()->pathName(inst));

      DelayLmSumResult swapped_result = local_sta_->
        increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, equiv_cell);

      LocalCost swapped_cost = swapped_result.delay_lm_sum;
      sta::Slack swapped_slack = 
                      local_sta_->localSlackAroundRef(pt_graph_);
      
      GraphTiming cell_type_timing;
      cell_type_timing.cell = equiv_cell;
      // Record timing after computing slack (to ensure full propagation)
      recordGraphTimingFromPtGraph(db_sta_, pt_graph_, cell_type_timing);
      timing_record.liberty_timing_map[std::string(equiv_cell->name())] = cell_type_timing;

      // Do local slack check
      printf("from delay_lm_sum %f to %f for cell %s, slack before swap %f, after swap %f\n",
             best_result.delay_lm_sum * 1e12,
             swapped_cost * 1e12,
             equiv_cell->name(),
             slack_before_swap_ * 1e12,
             swapped_slack * 1e12);
      fflush(stdout);
      if (swapped_cost < best_result.delay_lm_sum
          && swapped_slack >= slack_before_swap_ * 1.1 ) {
        best_cell_ = equiv_cell;
        best_result = swapped_result;
      }
    }
    if (best_cell_ == ori_cell) {
      // printf("ParallelLrVisitor::visit no better cell found for instance %s with cell %s, delaylmsum = %f\n",
      //       db_sta_->network()->pathName(inst),
      //       ori_cell->name(),
      //       best_result.delay_lm_sum * 1e12);
      // fflush(stdout);
      return false;
    }
    return true;
  } 
  printf("Warning: ParallelLrVisitor::visit no liberty cell for instance %s\n",
         db_sta_->network()->pathName(inst));
  fflush(stdout);
  return false;
}

ParallelLrVisitor *
ParallelLrVisitor::copy() const
{
  ParallelLrVisitor *new_visitor = new ParallelLrVisitor(db_sta_, local_sta_);
  new_visitor->setAverageDelay(average_delay_);
  new_visitor->setAverageLeakage(average_leakage_);
  new_visitor->setSwappableCellsCache(swappable_cells_cache_);
  new_visitor->setInstInfoMap(inst_info_map_);
  new_visitor->setSlackMargin(slack_margin_);
  new_visitor->setPTTradeoff(PT_tradeoff_);
  new_visitor->setParallelLibData(parallel_lib_data_);
  new_visitor->setClockPeriod(clock_period_);
  return new_visitor;
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
    sta::LibertyCell *from_lib_cell = 
                db_sta_->network()->libertyCell(pt_graph_->refInstance());
    if (!sta::equivCellsArcs(from_lib_cell, best_cell_)) {
      // No change needed
      printf("ParallelLrVisitor::applyChangesToDb skipping instance %s swap from cell %s to cell %s due to arc mismatch\n",
              db_sta_->network()->pathName(pt_graph_->refInstance()),
              from_lib_cell->name(),
              best_cell_->name());
      fflush(stdout);
      return;
    }
    // printf("ParallelLrVisitor::applyChangesToDb swapping instance %s from cell %s to cell %s\n",
    //         db_sta_->network()->pathName(pt_graph_->refInstance()),
    //         from_lib_cell->name(),
    //         best_cell_->name());
    // fflush(stdout);
    db_sta_->replaceCell(pt_graph_->refInstance(), best_cell_);
  }
  updateTimingFromPtGraph();
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
  if (!pt_vertex.vertex() || (pt_vertex.type() != PtVertexType::RefInput)
       && (pt_vertex.type() != PtVertexType::RefOutput)) {
    return;
  }
  sta::Vertex *sta_vertex = pt_vertex.vertex();
  // Update slews
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
  PtVertex &pt_to_vertex = pt_graph_->ptVertex(pt_edge.ptToId());
  if (!pt_edge.edge() || (pt_to_vertex.type() != PtVertexType::RefOutput)
   && (pt_to_vertex.type() != PtVertexType::RefDriver)) {
    return;
  }
  sta::Edge *sta_edge = pt_edge.edge();
  // Update arc delays
  for (const sta::TimingArc *arc : sta_edge->timingArcSet()->arcs()) {
    for (int i = 0; i < db_sta_->graph()->apCount(); i++) {
      sta::ArcDelay delay = pt_graph_->arcDelay(pt_edge, arc, i);
      db_sta_->graph()->setArcDelay(sta_edge, arc, i, delay);
    }
  }
}

void 
ParallelLrVisitor::recordGraphTimingFromPtGraphPara(sta::dbSta* sta, PtGraph *pt_graph, GraphTiming &graph_timing)
{
  printf("LocalSta::Recording Graph Timing from PtGraph for cell %s\n", 
          graph_timing.cell ? graph_timing.cell->name() : "nullptr");
  fflush(stdout);
  // First copy slews and paths from pt_graph's vertex to graph_timing
  for (PtVertex &pt_vertex : pt_graph->ptVertices()) {
    if (!pt_vertex.vertex()) continue;
    // First copy slews from pt_vertex to graph_timing
    std::string vertex_name = pt_vertex.vertex()->name(sta->network());
    TimingInfo vertex_timing_info;
    vertex_timing_info.type = TimingType::VERTEX;
    const sta::Slew *slews = pt_vertex.slews();
    vertex_timing_info.slews.clear();
    for (int i = 0; i < pt_vertex.slewCount(); ++i) {
      vertex_timing_info.slews.push_back(slews[i]);
    }
    // Then copy paths (including arrivals and requireds)
    sta::Path *pt_paths = pt_vertex.paths();
    vertex_timing_info.paths.clear();
    int path_count = pt_graph->tagGroup(pt_vertex)->pathCount();
    for (int i = 0; i < path_count; ++i) {
      sta::Path path = pt_paths[i];
      vertex_timing_info.paths.push_back(path);
    }
    vertex_timing_info.tag_group_index = pt_vertex.tagGroupIndex();
    graph_timing.vertex_timing_map[vertex_name] = vertex_timing_info;
  }

  // Second copy delays from pt_graph's edges to graph_timing
  for (const PtEdge &pt_edge : pt_graph->ptEdges()) {
    if (!pt_edge.edge()) continue;
    // First copy delays from pt_edge to graph_timing
    std::string edge_name = pt_edge.edge()->to_string(sta->network());
    TimingInfo edge_timing_info;
    edge_timing_info.type = TimingType::EDGE;
    const sta::ArcDelay *delays = pt_edge.arcDelays();
    for (int i = 0; i < pt_edge.arcDelayCount(); ++i) {
      edge_timing_info.delays.push_back(delays[i]);
    }
    graph_timing.edge_timing_map[edge_name] = edge_timing_info;
  }
}

void 
ParallelLrVisitor::init(float average_delay, float average_power, float wns,
  float PT_tradeoff,
  std::unordered_map<LibertyCell*, LibertyCellSeq*> *cache,
  std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map)
{
  average_delay_ = average_delay;
  average_leakage_ = average_power;
  float clock_period = 0.0;
  for (auto *clock : *db_sta_->sdc()->clocks()) {
    if (clock->period() > clock_period) {
      clock_period = clock->period();
      break;
    }
  }
  if (wns >= 0.0f) {
    slack_margin_ = 1.05f;
  } else
    slack_margin_ = std::min((-std::min(wns, 0.0f) / clock_period + 1.0f), 1.05f);
  PT_tradeoff_ = PT_tradeoff;
  printf("slack_margin: %f\n", slack_margin_);
  fflush(stdout);
  swappable_cells_cache_ = cache;
  inst_info_map_ = inst_info_map;
}

void
ParallelLrVisitor::init(float averge_delay, float average_power, float wns, 
    float PT_tradeoff, ParallelLibData *parallel_lib_data)
{
  average_delay_ = averge_delay;
  average_leakage_ = average_power;
  float clock_period = 0.0;
  for (auto *clock : *db_sta_->sdc()->clocks()) {
    if (clock->period() > clock_period) {
      clock_period = clock->period();
      break;
    }
  }
  clock_period_ = clock_period;
  slack_margin_ = std::max((-std::min(wns, 0.0f) / clock_period + 1.0f), 1.05f);
  PT_tradeoff_ = PT_tradeoff;
  printf("slack_margin: %f\n", slack_margin_);
  fflush(stdout);
  parallel_lib_data_ = parallel_lib_data;
}

bool
ParallelLrVisitor::bufferInsertion(sta::Instance *inst)
{
  // 1. Vitually insert buffers at the output net of the instance
  // 2. Compute the local timing cost after buffer insertion
  // 3. If cost improved, keep the buffer insertion
  // 4. Submmit the buffer insertion
  
  return false;
}

} // namespace lrf



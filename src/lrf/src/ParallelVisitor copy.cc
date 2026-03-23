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
#include "lrf/TestLrf.hh"

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

bool 
ParallelLrVisitor::checkVisitorStatus() const
{
  if (db_sta_ == nullptr || local_sta_ == nullptr || arc_delay_calc_ == nullptr
      || swappable_cells_cache_ == nullptr || inst_info_map_ == nullptr) {
    return false;
  }
  return true;
}

bool
ParallelLrVisitor::visit(sta::Instance *inst)
{
  if (!checkVisitorStatus()) {
    throw std::runtime_error("ParallelLrVisitor::visit visitor status invalid");
  }
  best_cell_ = nullptr;
  // The visit do following things:
  // 1. Get the target instance and set up a ptgraph for it.
  // 2. For each equivalent cell, virtual swap the instance to the cell,
  //    and compute the local timing cost.
  // 3. Keep track of the best cell and cost.
  // 4. Submmit the best cell swap to the resizer.
  visited_instances_.push_back(db_sta_->network()->pathName(inst));
  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (ori_cell) {
    sta::LibertyCellSeq *equiv_cells = swappable_cells_cache_->at(ori_cell);
    if (equiv_cells == nullptr) {
      // printf("Warning: ParallelLrVisitor::visit no equiv cells cached for %s, regenerating\n",
             // ori_cell->name());
      fflush(stdout);
      db_sta_->equivCells(ori_cell);
    }
    if (equiv_cells == nullptr) {
      // printf("ParallelLrVisitor::visit no equiv cells for %s\n",
             // ori_cell->name());
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
      // printf("ParallelLrVisitor::visit no legal equiv cells for %s\n",
             // ori_cell->name());
      fflush(stdout);
      return false;
    }

    // printf("ParallelLrVisitor::visit instance %s of type %s with %lu legal equivalent cells\n",
    //      db_sta_->network()->pathName(inst),
    //      db_sta_->network()->libertyCell(inst)->name(),
    //      legal_equiv_cells.size());
    // fflush(stdout);

    pt_graph_ = local_sta_->makePtGraph(inst, false);
    // Compute Original delays
    DelayLmSumResult original_result = local_sta_->
                increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, ori_cell);
    // Initialize the slack before swap
    slack_before_swap_ = local_sta_->localSlackAroundRef(pt_graph_);
    
    best_cell_ = ori_cell;
    DelayLmSumResult best_result = original_result;
    for (sta::LibertyCell *equiv_cell : legal_equiv_cells) {
      // This first virtual swap the cell in pt graph,
      // then recompute local delays, arrivals, requireds.
      if (ori_cell == equiv_cell) {
        continue;
      }
      DelayLmSumResult swapped_result = local_sta_->
        increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, equiv_cell);
      LocalCost swapped_cost = swapped_result.delay_lm_sum;
      sta::Slack swapped_slack = 
                      local_sta_->localSlackAroundRef(pt_graph_);
      // Do local slack check
      // printf("from delay_lm_sum %f to %f for cell %s, slack before swap %f, after swap %f\n",
      //        best_result.delay_lm_sum * 1e12,
      //        swapped_cost * 1e12,
      //        equiv_cell->name(),
      //        slack_before_swap_ * 1e12,
      //        swapped_slack * 1e12);
      // fflush(stdout);
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
    // First compute the final timing after choosing best cell
    local_sta_->increAndGetLocalTimingCost(pt_graph_, arc_delay_calc_, best_cell_);
    return true;
  } 
  // printf("ParallelLrVisitor::visit no liberty cell for instance %s\n",
         // db_sta_->network()->pathName(inst));
  fflush(stdout);
  return false;
}

bool 
ParallelLrVisitor::visit(sta::Instance *inst,
                             TimingRecord &timing_record)
{
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
      // printf("ParallelLrVisitor::visit no equiv cells for %s\n",
             // ori_cell->name());
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
      // printf("ParallelLrVisitor::visit no legal equiv cells for %s\n",
             // ori_cell->name());
      fflush(stdout);
      return false;
    }

    // printf("ParallelLrVisitor::visit instance %s of type %s with %lu legal equivalent cells\n",
         // db_sta_->network()->pathName(inst),
         // db_sta_->network()->libertyCell(inst)->name(),
         // legal_equiv_cells.size());
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
             // equiv_cell->name(),
             // db_sta_->network()->pathName(inst));

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
      // printf("from delay_lm_sum %f to %f for cell %s, slack before swap %f, after swap %f\n",
             // best_result.delay_lm_sum * 1e12,
             // swapped_cost * 1e12,
             // equiv_cell->name(),
             // slack_before_swap_ * 1e12,
             // swapped_slack * 1e12);
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
  // printf("Warning: ParallelLrVisitor::visit no liberty cell for instance %s\n",
         // db_sta_->network()->pathName(inst));
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
  return new_visitor;
}

void
ParallelLrVisitor::printVisitedInstNames() const
{
  for (const std::string &inst_name : visited_instances_) {
    // printf("Visited instance: %s\n", inst_name.c_str());
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
      // printf("ParallelLrVisitor::applyChangesToDb skipping instance %s swap from cell %s to cell %s due to arc mismatch\n",
              // db_sta_->network()->pathName(pt_graph_->refInstance()),
              // from_lib_cell->name(),
              // best_cell_->name());
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
  if (!pt_vertex.vertex() || pt_vertex.type() != PtVertexType::RefInput
      || pt_vertex.type() != PtVertexType::RefOutput
     ) {
    return;
  }
  sta::Vertex *sta_vertex = pt_vertex.vertex();
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
      // printf("TagGroup mismatch for vertex %s: PtTagGroup index %u, StaTagGroup index %u\n",
             // sta_vertex->to_string(db_sta_).c_str(),
             // pt_tag_group->index(),
             // sta_tag_group->index());
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
  if (!pt_edge.edge() || pt_to_vertex.type() != PtVertexType::RefOutput
|| pt_to_vertex.type() != PtVertexType::RefDriver) {
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
  // printf("LocalSta::Recording Graph Timing from PtGraph for cell %s\n", 
          // graph_timing.cell ? graph_timing.cell->name() : "nullptr");
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
ParallelLrVisitor::init(float average_delay, float average_power, 
  std::unordered_map<LibertyCell*, LibertyCellSeq*> *cache,
  std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map)
{
  average_delay_ = average_delay;
  average_leakage_ = average_power;
  swappable_cells_cache_ = cache;
  inst_info_map_ = inst_info_map;
}

} // namespace lrf



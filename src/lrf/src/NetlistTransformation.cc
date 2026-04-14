#include "NetlistTransformation.hh"
#include "PlacementDensityMap.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "LrRebuffer.hh"
#include "TaskArranger.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/Graph.hh"
#include "sta/EquivCells.hh"
#include "sta/Sdc.hh"
#include "sta/Clock.hh"
#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "rsz/Resizer.hh"

#include <chrono>
#include <algorithm>

namespace lrf {

// ═══════════════════════════════════════════════════════════
// EvalContext
// ═══════════════════════════════════════════════════════════

float
EvalContext::swapCost(float delay_lm_sum, float power,
                      float density_cost) const
{
  return PT_tradeoff * delay_lm_sum / average_delay
       + power / average_leakage
       + density_weight * density_cost / average_area;
}

// ═══════════════════════════════════════════════════════════
// MoveOption
// ═══════════════════════════════════════════════════════════

void
MoveOption::updateIfBetter(Type t, float c, float s,
                           float slack_before, float slack_margin,
                           sta::LibertyCell *cell,
                           rsz::BufferedNetPtr bnet)
{
  if (s < slack_before * slack_margin)
    return;
  if (c < cost) {
    type = t;
    cost = c;
    slack = s;
    target_cell = cell;
    buffer_tree = bnet;
  }
}

// ═══════════════════════════════════════════════════════════
// updateTimingFromPtGraph — free function
// ═══════════════════════════════════════════════════════════

void
updateTimingFromPtGraph(PtGraph *pt_graph)
{
  for (VertexId vertex_id : pt_graph->sortedVertexIds()) {
    PtVertex &pt_vertex = pt_graph->ptVertex(vertex_id);
    if (!pt_vertex.vertex())
      continue;
    sta::Vertex *sta_vertex = pt_vertex.vertex();
    PtVertexType type = pt_vertex.type();
    if (type == PtVertexType::RefInput
     || type == PtVertexType::RefOutput
     || type == PtVertexType::SiblingLoad) {
      pt_graph->writeSlewToGraph(pt_vertex, sta_vertex);
      pt_graph->writePathsToGraph(pt_vertex, sta_vertex);
    }
  }
}

// ═══════════════════════════════════════════════════════════
// ResizeOperator
// ═══════════════════════════════════════════════════════════

ResizeOperator::ResizeOperator(sta::dbSta *db_sta, LocalSta *local_sta)
  : db_sta_(db_sta), local_sta_(local_sta)
{
}

void
ResizeOperator::setEquivCellArray(LibertyCellArray *array, PosMap *pos_map)
{
  equiv_cell_array_ = array;
  equiv_cell_pos_map_ = pos_map;
}

void
ResizeOperator::setInstInfoMap(
    std::unordered_map<sta::Instance*, LocalCellInfo*> *map)
{
  inst_info_map_ = map;
}

std::vector<sta::LibertyCell*>
ResizeOperator::collectCandidates(sta::LibertyCell *ori_cell) const
{
  std::vector<sta::LibertyCell*> candidates;
  if (!equiv_cell_array_ || !equiv_cell_pos_map_)
    return candidates;

  auto pos_it = equiv_cell_pos_map_->find(ori_cell);
  if (pos_it == equiv_cell_pos_map_->end())
    return candidates;

  const CellArrayPos &pos = pos_it->second;
  const int cur_row = pos.row;
  const int cur_col = pos.col;
  const int group_start = pos.group_start;
  const int group_end = pos.group_end;
  const int num_cols = static_cast<int>((*equiv_cell_array_)[cur_row].size());

  for (int dr = -row_padding_; dr <= row_padding_; dr++) {
    int r = cur_row + dr;
    if (r < group_start || r >= group_end)
      continue;
    for (int dc = -col_padding_; dc <= col_padding_; dc++) {
      int c = cur_col + dc;
      if (c >= 0 && c < num_cols) {
        sta::LibertyCell *cell = (*equiv_cell_array_)[r][c];
        if (cell)
          candidates.push_back(cell);
      }
    }
  }
  return candidates;
}

float
ResizeOperator::lookupLeakage(sta::Instance *inst,
                              sta::LibertyCell *cand) const
{
  if (!inst_info_map_)
    return 0.0f;
  auto info_it = inst_info_map_->find(inst);
  if (info_it == inst_info_map_->end())
    return 0.0f;
  LocalCellInfo *cell_info = info_it->second;
  sta::LibertyCellSeq *equiv = cell_info->equiv_cells;
  if (!equiv)
    return 0.0f;
  for (size_t j = 0; j < equiv->size(); j++) {
    if ((*equiv)[j] == cand)
      return cell_info->cell_leakages[j];
  }
  return 0.0f;
}

MoveOption
ResizeOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                         EvalContext &ctx)
{
  MoveOption result;

  // Skip dont_touch instances
  odb::dbInst *db_inst = db_sta_->getDbNetwork()->staToDb(inst);
  if (db_inst && db_inst->isDoNotTouch())
    return result;

  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (!ori_cell)
    return result;

  // --- Determine pruning mode ---
  enum class EvalMode { FULL, PRUNED, REORDER };
  EvalMode mode = EvalMode::FULL;
  CellPruningState *pstate = nullptr;

  if (pruning_control_ && pruning_control_->enabled) {
    auto it = pruning_control_->state.find(inst);
    if (it != pruning_control_->state.end() && !it->second.ordered_cells.empty()) {
      pstate = &it->second;
      pstate->iters_since_reorder++;
      if (pstate->iters_since_reorder >= pstate->M) {
        mode = EvalMode::REORDER;
      } else {
        bool has_ori = false;
        for (auto *c : pstate->ordered_cells) {
          if (c == ori_cell) { has_ori = true; break; }
        }
        mode = has_ori ? EvalMode::PRUNED : EvalMode::REORDER;
      }
    }
  }

  // --- Build candidate list ---
  std::vector<sta::LibertyCell*> candidates;
  if (mode == EvalMode::PRUNED) {
    candidates = pstate->ordered_cells;
  } else {
    candidates = collectCandidates(ori_cell);
  }
  if (candidates.size() < 2)
    return result;

  auto start_eval = std::chrono::high_resolution_clock::now();

  // Precompute density at this cell's location (shared across candidates).
  float local_density = 0.0f;
  float ori_area = ori_cell->area();
  if (ctx.density_map && ctx.density_weight > 0.0f) {
    local_density = ctx.density_map->getDensity(db_inst);
  }

  // Pass 1: evaluate all candidates, store (cost, slack) pairs
  std::vector<float> vec_cost_slack(candidates.size() * 2,
                                    std::numeric_limits<float>::max());
  float slack_before = 0.0f;

  for (size_t i = 0; i < candidates.size(); i++) {
    sta::LibertyCell *cand = candidates[i];

    auto t_lc0 = std::chrono::high_resolution_clock::now();
    bool legal_before = local_sta_->legalCheckBeforeSwap(
        inst, cand, nullptr, nullptr, pt_graph);
    if (ctx.runtime_map) {
      auto t_lc1 = std::chrono::high_resolution_clock::now();
      (*ctx.runtime_map)["legalCheckBeforeSwap"] +=
          std::chrono::duration<double>(t_lc1 - t_lc0).count();
    }
    if (!legal_before && cand != ori_cell)
      continue;

    float leakage = lookupLeakage(inst, cand);
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, cand, ctx.runtime_map).delay_lm_sum;

    auto t_lc2 = std::chrono::high_resolution_clock::now();
    bool legal_after = local_sta_->legalCheckAfterSwap(
        inst, cand, nullptr, nullptr, pt_graph);
    if (ctx.runtime_map) {
      auto t_lc3 = std::chrono::high_resolution_clock::now();
      (*ctx.runtime_map)["legalCheckAfterSwap"] +=
          std::chrono::duration<double>(t_lc3 - t_lc2).count();
    }
    if (!legal_after && cand != ori_cell)
      continue;

    // Density penalty: Dd = (cand_area - ori_area) * Φ(x,y)
    float density_cost = (cand->area() - ori_area) * local_density;
    float cost = ctx.swapCost(delay_lm_sum, leakage, density_cost);
    float slack = local_sta_->localSlackAroundRef(pt_graph);
    vec_cost_slack[i * 2] = cost;
    vec_cost_slack[i * 2 + 1] = slack;

    if (cand == ori_cell)
      slack_before = slack;
  }

  auto end_eval = std::chrono::high_resolution_clock::now();
  if (ctx.runtime_map) {
    (*ctx.runtime_map)["equiv_cell_check"] +=
        std::chrono::duration<double>(end_eval - start_eval).count();
    (*ctx.runtime_map)["equiv_cell_count"] += candidates.size();
  }

  // Pass 2: pick best (with slack margin check)
  for (size_t i = 0; i < candidates.size(); i++) {
    float cost = vec_cost_slack[i * 2];
    float slack = vec_cost_slack[i * 2 + 1];
    result.updateIfBetter(MoveOption::RESIZE_ONLY, cost, slack,
                          slack_before, slack_margin_, candidates[i], nullptr);
  }

  // --- Update pruning state (FULL or REORDER: evaluated full neighborhood) ---
  if (pruning_control_ && mode != EvalMode::PRUNED) {
    std::vector<std::pair<float, sta::LibertyCell*>> cost_cells;
    for (size_t i = 0; i < candidates.size(); i++) {
      float cost = vec_cost_slack[i * 2];
      float slack = vec_cost_slack[i * 2 + 1];
      if (cost < std::numeric_limits<float>::max()
          && slack >= slack_before * slack_margin_) {
        cost_cells.push_back({cost, candidates[i]});
      }
    }
    std::sort(cost_cells.begin(), cost_cells.end());

    size_t keep = std::max(static_cast<size_t>(2),
                           static_cast<size_t>(cost_cells.size() * pruning_control_->P));
    keep = std::min(keep, cost_cells.size());

    CellPruningState &ps = pruning_control_->state[inst];

    // Adaptive M: large jump in ori_cell rank → diverging → reorder sooner
    if (mode == EvalMode::REORDER) {
      int jump = static_cast<int>(cost_cells.size());
      for (size_t i = 0; i < cost_cells.size(); i++) {
        if (cost_cells[i].second == ori_cell) {
          jump = static_cast<int>(i);
          break;
        }
      }
      ps.M = (jump <= static_cast<int>(keep))
           ? std::min(ps.M + 1, 10)   // converging
           : std::max(ps.M - 1, 1);   // diverging
    }

    ps.ordered_cells.clear();
    for (size_t i = 0; i < keep; i++)
      ps.ordered_cells.push_back(cost_cells[i].second);
    ps.iters_since_reorder = 0;
  }

  // If best is the original cell, no change
  if (result.target_cell == ori_cell) {
    result.type = MoveOption::NONE;
    return result;
  }

  if (!result.hasChange())
    return result;

  // Recompute final timing for best cell
  if (result.target_cell != candidates.back())
    local_sta_->increAndGetLocalTimingCost(pt_graph, ctx.arc_delay_calc,
                                           result.target_cell);
  return result;
}

std::vector<MoveOption>
ResizeOperator::evaluateTopN(PtGraph *pt_graph, sta::Instance *inst,
                             EvalContext &ctx, int n)
{
  std::vector<MoveOption> results;

  // Skip dont_touch instances
  odb::dbInst *db_inst = db_sta_->getDbNetwork()->staToDb(inst);
  if (db_inst && db_inst->isDoNotTouch())
    return results;

  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (!ori_cell)
    return results;

  std::vector<sta::LibertyCell*> candidates = collectCandidates(ori_cell);
  if (candidates.size() < 2)
    return results;

  auto start_eval = std::chrono::high_resolution_clock::now();

  // Precompute density at this cell's location (shared across candidates).
  float local_density = 0.0f;
  float ori_area = ori_cell->area();
  if (ctx.density_map && ctx.density_weight > 0.0f) {
    local_density = ctx.density_map->getDensity(db_inst);
  }

  // Pass 1: evaluate all candidates, store (cost, slack) pairs
  std::vector<float> vec_cost_slack(candidates.size() * 2,
                                    std::numeric_limits<float>::max());
  float slack_before = 0.0f;

  for (size_t i = 0; i < candidates.size(); i++) {
    sta::LibertyCell *cand = candidates[i];

    auto t_lc0 = std::chrono::high_resolution_clock::now();
    bool legal_before = local_sta_->legalCheckBeforeSwap(
        inst, cand, nullptr, nullptr, pt_graph);
    if (ctx.runtime_map) {
      auto t_lc1 = std::chrono::high_resolution_clock::now();
      (*ctx.runtime_map)["legalCheckBeforeSwap"] +=
          std::chrono::duration<double>(t_lc1 - t_lc0).count();
    }
    if (!legal_before && cand != ori_cell)
      continue;

    float leakage = lookupLeakage(inst, cand);
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, cand, ctx.runtime_map).delay_lm_sum;

    auto t_lc2 = std::chrono::high_resolution_clock::now();
    bool legal_after = local_sta_->legalCheckAfterSwap(
        inst, cand, nullptr, nullptr, pt_graph);
    if (ctx.runtime_map) {
      auto t_lc3 = std::chrono::high_resolution_clock::now();
      (*ctx.runtime_map)["legalCheckAfterSwap"] +=
          std::chrono::duration<double>(t_lc3 - t_lc2).count();
    }
    if (!legal_after && cand != ori_cell)
      continue;

    float density_cost = (cand->area() - ori_area) * local_density;
    float cost = ctx.swapCost(delay_lm_sum, leakage, density_cost);
    float slack = local_sta_->localSlackAroundRef(pt_graph);
    vec_cost_slack[i * 2] = cost;
    vec_cost_slack[i * 2 + 1] = slack;

    if (cand == ori_cell)
      slack_before = slack;
  }

  auto end_eval = std::chrono::high_resolution_clock::now();
  if (ctx.runtime_map) {
    (*ctx.runtime_map)["equiv_cell_check"] +=
        std::chrono::duration<double>(end_eval - start_eval).count();
    (*ctx.runtime_map)["equiv_cell_count"] += candidates.size();
  }

  // Pass 2: collect top-N (with slack margin check), sorted ascending by cost
  struct CandEntry {
    float cost;
    sta::LibertyCell *cell;
  };
  std::vector<CandEntry> valid;
  float ori_cost = std::numeric_limits<float>::max();

  for (size_t i = 0; i < candidates.size(); i++) {
    float cost = vec_cost_slack[i * 2];
    float slack = vec_cost_slack[i * 2 + 1];
    if (candidates[i] == ori_cell) {
      ori_cost = cost;
      continue;
    }
    if (cost >= std::numeric_limits<float>::max())
      continue;
    if (slack < slack_before * slack_margin_)
      continue;
    valid.push_back({cost, candidates[i]});
  }

  std::sort(valid.begin(), valid.end(),
            [](const CandEntry &a, const CandEntry &b) {
              return a.cost < b.cost;
            });

  int count = std::min(n, static_cast<int>(valid.size()));
  for (int i = 0; i < count; i++) {
    if (valid[i].cost < ori_cost) {
      MoveOption mo;
      mo.type = MoveOption::RESIZE_ONLY;
      mo.cost = valid[i].cost;
      mo.target_cell = valid[i].cell;
      results.push_back(mo);
    }
  }

  return results;
}

std::unique_ptr<LrOperator>
ResizeOperator::copy() const
{
  auto op = std::make_unique<ResizeOperator>(db_sta_, local_sta_);
  op->equiv_cell_array_ = equiv_cell_array_;
  op->equiv_cell_pos_map_ = equiv_cell_pos_map_;
  op->inst_info_map_ = inst_info_map_;
  op->slack_margin_ = slack_margin_;
  op->col_padding_ = col_padding_;
  op->row_padding_ = row_padding_;
  op->pruning_control_ = pruning_control_;
  return op;
}

void
ResizeOperator::apply(const MoveOption &move, PtGraph *pt_graph,
                      std::map<std::string, double> &runtime_map)
{
  auto start = std::chrono::steady_clock::now();
  if (move.target_cell && pt_graph->refInstance()) {
    sta::LibertyCell *from_cell =
        db_sta_->network()->libertyCell(pt_graph->refInstance());
    if (!sta::equivCellPorts(from_cell, move.target_cell)
        || !sta::equivCellFuncs(from_cell, move.target_cell)) {
      printf("ResizeOperator::apply skipping %s: "
             "port/function mismatch\n",
             db_sta_->network()->pathName(pt_graph->refInstance()));
      fflush(stdout);
      return;
    }
    db_sta_->replaceCell(pt_graph->refInstance(), move.target_cell);
  }
  auto end = std::chrono::steady_clock::now();
  runtime_map["swap"] += std::chrono::duration<double>(end - start).count();
  runtime_map["applyDb"] +=
      std::chrono::duration<double>(end - start).count();
}

// ═══════════════════════════════════════════════════════════
// ResizePrecheckOperator
// ═══════════════════════════════════════════════════════════

MoveOption
ResizePrecheckOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                                 EvalContext &ctx)
{
  MoveOption result;
  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (!ori_cell)
    return result;

  // Precheck uses smaller neighborhood
  int saved_col = col_padding_;
  int saved_row = row_padding_;
  col_padding_ = 1;
  row_padding_ = 1;
  std::vector<sta::LibertyCell*> candidates = collectCandidates(ori_cell);
  col_padding_ = saved_col;
  row_padding_ = saved_row;

  if (candidates.size() < 2)
    return result;

  auto start_eval = std::chrono::high_resolution_clock::now();

  // Precompute density at this cell's location.
  float local_density = 0.0f;
  float ori_area = ori_cell->area();
  if (ctx.density_map && ctx.density_weight > 0.0f) {
    odb::dbInst *db_inst = db_sta_->getDbNetwork()->staToDb(inst);
    if (db_inst)
      local_density = ctx.density_map->getDensity(db_inst);
  }

  // Build O(1) leakage lookup
  std::unordered_map<sta::LibertyCell*, float> leakage_cache;
  if (inst_info_map_) {
    auto info_it = inst_info_map_->find(inst);
    if (info_it != inst_info_map_->end()) {
      LocalCellInfo *cell_info = info_it->second;
      sta::LibertyCellSeq *equiv = cell_info->equiv_cells;
      if (equiv) {
        for (size_t j = 0; j < equiv->size(); j++)
          leakage_cache[(*equiv)[j]] = cell_info->cell_leakages[j];
      }
    }
  }

  struct CandResult {
    float cost  = std::numeric_limits<float>::max();
    float slack = 0.0f;
  };
  std::vector<CandResult> cand_results(candidates.size());

  float ori_cost = std::numeric_limits<float>::max();
  float ori_slack = 0.0f;

  // Pass 1: evaluate all candidates, record (cost, slack)
  for (size_t i = 0; i < candidates.size(); i++) {
    sta::LibertyCell *cand = candidates[i];

    if (!local_sta_->legalCheckBeforeSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float leakage = 0.0f;
    auto lk_it = leakage_cache.find(cand);
    if (lk_it != leakage_cache.end())
      leakage = lk_it->second;

    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, cand, ctx.runtime_map).delay_lm_sum;

    if (!local_sta_->legalCheckAfterSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float density_cost = (cand->area() - ori_area) * local_density;
    float cost = ctx.swapCost(delay_lm_sum, leakage, density_cost);
    float slack = local_sta_->localSlackAroundRef(pt_graph);
    cand_results[i] = {cost, slack};

    if (cand == ori_cell) {
      ori_cost = cost;
      ori_slack = slack;
    }
  }

  auto end_eval = std::chrono::high_resolution_clock::now();
  if (ctx.runtime_map) {
    (*ctx.runtime_map)["equiv_cell_check"] +=
        std::chrono::duration<double>(end_eval - start_eval).count();
    (*ctx.runtime_map)["equiv_cell_count"] += candidates.size();
  }

  if (ori_cost == std::numeric_limits<float>::max())
    return result;

  // Pass 2: find best cost with correct ori_slack for slack protection
  float best_cost = ori_cost;
  for (size_t i = 0; i < candidates.size(); i++) {
    if (candidates[i] == ori_cell)
      continue;
    const CandResult &r = cand_results[i];
    if (r.cost == std::numeric_limits<float>::max())
      continue;  // was skipped (illegal)
    if (r.cost < best_cost && r.slack >= ori_slack * slack_margin_)
      best_cost = r.cost;
  }

  if (best_cost < ori_cost) {
    result.cost = ori_cost - best_cost;
  }
  return result;
}

std::unique_ptr<LrOperator>
ResizePrecheckOperator::copy() const
{
  auto op = std::make_unique<ResizePrecheckOperator>(db_sta_, local_sta_);
  op->equiv_cell_array_ = equiv_cell_array_;
  op->equiv_cell_pos_map_ = equiv_cell_pos_map_;
  op->inst_info_map_ = inst_info_map_;
  op->slack_margin_ = slack_margin_;
  op->col_padding_ = col_padding_;
  op->row_padding_ = row_padding_;
  return op;
}

// ═══════════════════════════════════════════════════════════
// BufferOperator
// ═══════════════════════════════════════════════════════════

BufferOperator::BufferOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                               rsz::Resizer *resizer, EvalContext *ctx)
  : db_sta_(db_sta), local_sta_(local_sta), resizer_(resizer)
{
  if (ctx) {
    rebuffer_ = std::make_unique<LrRebuffer>(resizer, local_sta, ctx);
    rebuffer_->init();
  }
  // If ctx is nullptr, rebuffer_ stays null until setEvalContext() is called.
}

MoveOption
BufferOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                         EvalContext &ctx)
{
  MoveOption result;
  if (!rebuffer_) {
    printf("Error: BufferOperator's rebuffer is not initialized.\n");
    return result;
  }
      

  // Sync pt_graph into eval context so LrRebuffer sees the right graph
  ctx.pt_graph = pt_graph;

  // Collect RefOutput driver pins before rebufferPin (may reallocate)
  struct DrvrInfo { sta::Pin *pin; VertexId vid; };
  std::vector<DrvrInfo> drvr_infos;
  for (size_t i = 0; i < pt_graph->vertexCount(); i++) {
    PtVertex &pv = pt_graph->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput)
      drvr_infos.push_back({pv.vertex()->pin(), pv.objectIdx()});
  }

  if (drvr_infos.size() != 1)
    return result;

  rebuffer_->rebufferPin(drvr_infos[0].pin,
                         pt_graph->ptVertex(drvr_infos[0].vid));

  if (!rebuffer_->bestBnet())
    return result;

  result.type = MoveOption::BUFFER_ONLY;
  result.cost = rebuffer_->bestCost();
  result.buffer_tree = rebuffer_->bestBnet();
  return result;
}

void
BufferOperator::setEvalContext(EvalContext *ctx)
{
  // Recreate rebuffer with new context (per-thread copy)
  rebuffer_ = std::make_unique<LrRebuffer>(resizer_, local_sta_, ctx);
  rebuffer_->init();
}

std::unique_ptr<LrOperator>
BufferOperator::copy() const
{
  // copy() creates with nullptr ctx; caller must call setEvalContext() after
  auto op = std::make_unique<BufferOperator>(db_sta_, local_sta_,
                                             resizer_, nullptr);
  return op;
}

void
BufferOperator::apply(const MoveOption &move, PtGraph *pt_graph,
                      std::map<std::string, double> &runtime_map)
{
  auto start = std::chrono::steady_clock::now();
  if (rebuffer_) {
    int count = rebuffer_->applyBufferingToDb();
    runtime_map["buffer_count"] += count;
  }
  auto end = std::chrono::steady_clock::now();
  runtime_map["applyDb"] +=
      std::chrono::duration<double>(end - start).count();
}

// ═══════════════════════════════════════════════════════════
// BufferSensitivityOperator
// ═══════════════════════════════════════════════════════════

bool
BufferSensitivityOperator::skipInstance(sta::Instance *inst) const
{
  // Skip instances whose driver pins all have non-negative slack —
  // buffer insertion only targets negative-slack paths.  This avoids
  // the full PtGraph construction for the (usually large) non-critical
  // majority.
  sta::Network *network = db_sta_->network();
  sta::Graph *graph = db_sta_->graph();
  sta::InstancePinIterator *iter = network->pinIterator(inst);
  bool all_positive = true;
  while (iter->hasNext()) {
    sta::Pin *pin = iter->next();
    if (network->isDriver(pin)) {
      sta::Vertex *vtx = graph->pinDrvrVertex(pin);
      if (vtx && db_sta_->vertexSlack(vtx, sta::MinMax::max()) < 0.0f) {
        all_positive = false;
        break;
      }
    }
  }
  delete iter;
  if (local_sta_->debug()) {
    printf("[DBG-SKIP] inst=%s all_positive=%d\n",
           db_sta_->network()->pathName(inst), all_positive);
  }
  return all_positive;
}

MoveOption
BufferSensitivityOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                                    EvalContext &ctx)
{
  MoveOption result;
  if (!rebuffer_)
    return result;

  sta::Pin *drvr_pin = nullptr;
  PtVertex *drvr_pv = nullptr;
  for (size_t i = 0; i < pt_graph->vertexCount(); i++) {
    PtVertex &pv = pt_graph->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput) {
      drvr_pin = pv.vertex()->pin();
      drvr_pv = &pv;
      break;
    }
  }
  if (!drvr_pin || !drvr_pv)
    return result;

  float score = rebuffer_->computeNetSensitivity(
      drvr_pin, *drvr_pv, ctx.average_delay, ctx.average_leakage);

  result.cost = score;
  return result;
}

std::unique_ptr<LrOperator>
BufferSensitivityOperator::copy() const
{
  auto op = std::make_unique<BufferSensitivityOperator>(
      db_sta_, local_sta_, resizer_, nullptr);
  return op;
}

// ═══════════════════════════════════════════════════════════
// BufferRszOperator
// ═══════════════════════════════════════════════════════════

BufferRszOperator::BufferRszOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                                     rsz::Resizer *resizer, EvalContext *ctx)
  : db_sta_(db_sta), local_sta_(local_sta), resizer_(resizer)
{
  if (ctx) {
    rebuffer_ = std::make_unique<LrRebuffer>(resizer, local_sta, ctx);
    rebuffer_->init();
  }
}

bool
BufferRszOperator::skipInstance(sta::Instance *inst) const
{
  return false;  // operators which do actual operation never skip, 
                  // since they are required to update timing.
}

MoveOption
BufferRszOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                            EvalContext &ctx)
{
  MoveOption result;
  if (!rebuffer_)
    return result;

  // Find the worst-slack driver pin on this instance.
  sta::Network *network = db_sta_->network();
  sta::Graph *graph = db_sta_->graph();
  sta::Pin *worst_pin = nullptr;
  float worst_slack = 0.0f;

  sta::InstancePinIterator *iter = network->pinIterator(inst);
  while (iter->hasNext()) {
    sta::Pin *pin = iter->next();
    if (!network->isDriver(pin))
      continue;
    sta::Vertex *vtx = graph->pinDrvrVertex(pin);
    if (!vtx)
      continue;
    float slack = db_sta_->vertexSlack(vtx, sta::MinMax::max());
    if (slack < worst_slack) {
      worst_slack = slack;
      worst_pin = pin;
    }
  }
  delete iter;

  if (!worst_pin)
    return result;

  // Heavy computation: makeBufferedNet + bufferForTiming + recoverArea
  // Result stored in rebuffer_->best_bnet_ / drvr_pin_
  if (rebuffer_->prepareRszBnet(worst_pin)) {
    result.type = MoveOption::BUFFER_ONLY;
    result.cost = 0.0f;
  }
  return result;
}

void
BufferRszOperator::apply(const MoveOption &move, PtGraph *pt_graph,
                         std::map<std::string, double> &runtime_map)
{
  if (!rebuffer_ || !rebuffer_->bestBnet())
    return;

  auto start = std::chrono::steady_clock::now();
  int count = rebuffer_->applyBufferingToDb();
  auto end = std::chrono::steady_clock::now();
  runtime_map["buffer_count"] += count;
  runtime_map["applyDb"] +=
      std::chrono::duration<double>(end - start).count();
}

void
BufferRszOperator::setEvalContext(EvalContext *ctx)
{
  rebuffer_ = std::make_unique<LrRebuffer>(resizer_, local_sta_, ctx);
  rebuffer_->init();
}

std::unique_ptr<LrOperator>
BufferRszOperator::copy() const
{
  // copy() creates with nullptr ctx; caller must call setEvalContext() after
  auto op = std::make_unique<BufferRszOperator>(db_sta_, local_sta_,
                                                resizer_, nullptr);
  return op;
}

// ═══════════════════════════════════════════════════════════
// BufferSdpOperator — LRF slack-DP rebuffering.
// Strict mirror of BufferOperator: pin selection via thread-local pt_graph
// RefOutput vertices (no global STA read), requires exactly 1 driver per
// instance. Only difference: invokes prepareSlackDpBnet (slack-DP +
// recoverLrCost) instead of rebufferPin (cost-DP).
// ═══════════════════════════════════════════════════════════

BufferSdpOperator::BufferSdpOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                                      rsz::Resizer *resizer, EvalContext *ctx)
  : db_sta_(db_sta), local_sta_(local_sta), resizer_(resizer)
{
  if (ctx) {
    rebuffer_ = std::make_unique<LrRebuffer>(resizer, local_sta, ctx);
    rebuffer_->init();
  }
}

MoveOption
BufferSdpOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                             EvalContext &ctx)
{
  MoveOption result;
  if (!rebuffer_) {
    printf("Error: BufferSdpOperator's rebuffer is not initialized.\n");
    return result;
  }

  // Sync pt_graph into eval context so LrRebuffer sees the right graph.
  ctx.pt_graph = pt_graph;

  // Collect RefOutput driver pins from thread-local pt_graph (no global STA).
  struct DrvrInfo { sta::Pin *pin; VertexId vid; };
  std::vector<DrvrInfo> drvr_infos;
  for (size_t i = 0; i < pt_graph->vertexCount(); i++) {
    PtVertex &pv = pt_graph->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput)
      drvr_infos.push_back({pv.vertex()->pin(), pv.objectIdx()});
  }

  if (drvr_infos.size() != 1)
    return result;

  // Heavy computation: slack-DP bufferForTiming + recoverLrCost.
  // Pass vid directly (not PtVertex&) so internal pt_graph mutations
  // (virtual buffer insertion in evaluateOption etc.) never invalidate
  // the reference — prepareSlackDpBnet re-looks-up PtVertex via vid
  // fresh each time it's needed.
  rebuffer_->prepareSlackDpBnet(drvr_infos[0].pin, drvr_infos[0].vid);

  if (!rebuffer_->bestBnet())
    return result;

  result.type = MoveOption::BUFFER_ONLY;
  result.cost = rebuffer_->bestCost();
  result.buffer_tree = rebuffer_->bestBnet();
  return result;
}

void
BufferSdpOperator::setEvalContext(EvalContext *ctx)
{
  rebuffer_ = std::make_unique<LrRebuffer>(resizer_, local_sta_, ctx);
  rebuffer_->init();
}

std::unique_ptr<LrOperator>
BufferSdpOperator::copy() const
{
  auto op = std::make_unique<BufferSdpOperator>(db_sta_, local_sta_,
                                                resizer_, nullptr);
  return op;
}

void
BufferSdpOperator::apply(const MoveOption &move, PtGraph *pt_graph,
                          std::map<std::string, double> &runtime_map)
{
  auto start = std::chrono::steady_clock::now();
  if (rebuffer_) {
    int count = rebuffer_->applyBufferingToDb();
    runtime_map["buffer_count"] += count;
  }
  auto end = std::chrono::steady_clock::now();
  runtime_map["applyDb"] +=
      std::chrono::duration<double>(end - start).count();
}

// ═══════════════════════════════════════════════════════════
// CombinedOperator
// ═══════════════════════════════════════════════════════════

CombinedOperator::CombinedOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                                   rsz::Resizer *resizer, EvalContext *ctx)
  : db_sta_(db_sta), local_sta_(local_sta)
{
  resize_op_ = std::make_unique<ResizeOperator>(db_sta, local_sta);
  buffer_op_ = std::make_unique<BufferOperator>(db_sta, local_sta,
                                                resizer, ctx);
}

void
CombinedOperator::setEquivCellArray(LibertyCellArray *array, PosMap *pos_map)
{
  resize_op_->setEquivCellArray(array, pos_map);
}

void
CombinedOperator::setInstInfoMap(
    std::unordered_map<sta::Instance*, LocalCellInfo*> *map)
{
  resize_op_->setInstInfoMap(map);
}

void
CombinedOperator::setSlackMargin(float margin)
{
  resize_op_->setSlackMargin(margin);
}

MoveOption
CombinedOperator::tryBufferingOnCandidates(
    PtGraph *pt_graph, sta::Instance *inst, EvalContext &ctx,
    const std::vector<MoveOption> &resize_candidates,
    sta::LibertyCell *ori_cell, float ori_cost)
{
  MoveOption result;
  LrRebuffer *rebuffer = buffer_op_ ? buffer_op_->rebuffer() : nullptr;
  if (!rebuffer)
    return result;

  // Collect RefOutput driver pin
  sta::Pin *drvr_pin = nullptr;
  VertexId drvr_vid = sta::object_id_null;
  for (size_t i = 0; i < pt_graph->vertexCount(); i++) {
    PtVertex &pv = pt_graph->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput) {
      drvr_pin = pv.vertex()->pin();
      drvr_vid = pv.objectIdx();
      break;
    }
  }
  if (!drvr_pin || drvr_vid == sta::object_id_null)
    return result;

  // Phase 2a: Build buffer tree + 2 rounds coarse (cell-independent, cached)
  ctx.pt_graph = pt_graph;
  rsz::BufferedNetPtr cached_bnet = rebuffer->prepareBufferOptions(
      drvr_pin, pt_graph->ptVertex(drvr_vid));
  if (!cached_bnet) {
    if (ctx.debug)
      printf("[DBG-BUF] %s: prepareBufferOptions returned null\n",
             db_sta_->network()->pathName(inst));
    return result;
  }

  // Build candidate list: top-N resize cells + original cell
  struct BufCandidate {
    sta::LibertyCell *cell;
    bool is_original;
  };
  std::vector<BufCandidate> buf_candidates;
  for (auto &mo : resize_candidates)
    buf_candidates.push_back({mo.target_cell, false});
  buf_candidates.push_back({ori_cell, true});

  float best_buf_cost = std::numeric_limits<float>::max();
  sta::LibertyCell *best_buf_cell = nullptr;
  bool best_buf_is_original = false;

  // Phase 2b: For each candidate, evaluate precisely
  for (auto &bc : buf_candidates) {
    if (!bc.cell)
      continue;

    local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, bc.cell);

    rebuffer->evaluateBufferOnCandidate(drvr_vid, cached_bnet);

    if (rebuffer->bestBnet()) {
      float cost = rebuffer->bestCost();
      if (ctx.debug)
        printf("[DBG-BUF] %s: cell=%s is_orig=%d buf_cost=%.3e ori_cost=%.3e\n",
               db_sta_->network()->pathName(inst), bc.cell->name(),
               bc.is_original, cost, ori_cost);
      if (cost < best_buf_cost) {
        best_buf_cost = cost;
        best_buf_cell = bc.cell;
        best_buf_is_original = bc.is_original;
      }
    } else {
      if (ctx.debug)
        printf("[DBG-BUF] %s: cell=%s evaluateBufferOnCandidate -> null bestBnet\n",
               db_sta_->network()->pathName(inst), bc.cell->name());
    }
    rebuffer->cleanupVirtualBuffer();
  }

  if (!best_buf_cell || best_buf_cost >= ori_cost)
    return result;

  // Re-evaluate the winner to populate best_bnet_ for applyChangesToDb
  local_sta_->increAndGetLocalTimingCost(
      pt_graph, ctx.arc_delay_calc, best_buf_cell);
  rebuffer->evaluateBufferOnCandidate(drvr_vid, cached_bnet);

  if (!rebuffer->bestBnet()) {
    rebuffer->cleanupVirtualBuffer();
    return result;
  }

  if (best_buf_is_original) {
    result.type = MoveOption::BUFFER_ONLY;
    result.target_cell = nullptr;
  } else {
    result.type = MoveOption::COMBINED;
    result.target_cell = best_buf_cell;
  }
  result.cost = best_buf_cost;
  result.buffer_tree = rebuffer->bestBnet();
  return result;
}

MoveOption
CombinedOperator::tryBufferingOnTop1AndSmaller(
    PtGraph *pt_graph, sta::Instance *inst, EvalContext &ctx,
    const std::vector<MoveOption> &resize_candidates,
    sta::LibertyCell *ori_cell, float baseline_cost)
{
  MoveOption result;
  LrRebuffer *rebuffer = buffer_op_ ? buffer_op_->rebuffer() : nullptr;
  if (!rebuffer || !resize_op_->equiv_cell_array_ || !resize_op_->equiv_cell_pos_map_) {
    if (ctx.runtime_map)
      (*ctx.runtime_map)["buf_reject_no_rebuffer"] += 1.0;
    return result;
  }

  // Collect driver pin
  sta::Pin *drvr_pin = nullptr;
  VertexId drvr_vid = sta::object_id_null;
  for (size_t i = 0; i < pt_graph->vertexCount(); i++) {
    PtVertex &pv = pt_graph->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput) {
      drvr_pin = pv.vertex()->pin();
      drvr_vid = pv.objectIdx();
      break;
    }
  }
  if (!drvr_pin || drvr_vid == sta::object_id_null) {
    if (ctx.runtime_map)
      (*ctx.runtime_map)["buf_reject_no_driver"] += 1.0;
    return result;
  }

  // Build candidate list: top-1 + one-size-smaller + original
  struct BufCandidate {
    sta::LibertyCell *cell;
    bool is_original;
  };
  std::vector<BufCandidate> buf_candidates;

  sta::LibertyCell *top1_cell = nullptr;
  if (!resize_candidates.empty()) {
    top1_cell = resize_candidates[0].target_cell;
    buf_candidates.push_back({top1_cell, false});

    // Find one-size-smaller (col - 1) for top1
    auto pos_it = resize_op_->equiv_cell_pos_map_->find(top1_cell);
    if (pos_it != resize_op_->equiv_cell_pos_map_->end()) {
      const CellArrayPos &pos = pos_it->second;
      if (pos.col > 0) {
        sta::LibertyCell *smaller = (*resize_op_->equiv_cell_array_)[pos.row][pos.col - 1];
        if (smaller && smaller != top1_cell && smaller != ori_cell)
          buf_candidates.push_back({smaller, false});
      }
    }
  }
  buf_candidates.push_back({ori_cell, true});

  // Direct rebufferPin for each candidate (precise evaluation, no coarse filtering)
  ctx.pt_graph = pt_graph;

  float best_buf_cost = std::numeric_limits<float>::max();
  sta::LibertyCell *best_buf_cell = nullptr;
  bool best_buf_is_original = false;

  for (auto &bc : buf_candidates) {
    if (!bc.cell) continue;

    local_sta_->increAndGetLocalTimingCost(pt_graph, ctx.arc_delay_calc, bc.cell);
    rebuffer->rebufferPin(drvr_pin, pt_graph->ptVertex(drvr_vid));

    if (rebuffer->bestBnet()) {
      float cost = rebuffer->bestCost();
      if (cost < best_buf_cost) {
        best_buf_cost = cost;
        best_buf_cell = bc.cell;
        best_buf_is_original = bc.is_original;
      }
    }
    rebuffer->cleanupVirtualBuffer();
  }

  if (!best_buf_cell) {
    if (ctx.runtime_map)
      (*ctx.runtime_map)["buf_reject_no_valid_option"] += 1.0;
    return result;
  }
  // Removed baseline_cost gate: bufferForTiming already does fair comparison
  // between buffers=0 and buffers>0 options within the same evaluateOption
  // framework. If best option has buffers, it genuinely beat no-buffer.

  // Re-evaluate winner to rebuild rebuffer internal state for apply
  local_sta_->increAndGetLocalTimingCost(pt_graph, ctx.arc_delay_calc, best_buf_cell);
  rebuffer->rebufferPin(drvr_pin, pt_graph->ptVertex(drvr_vid));
  if (!rebuffer->bestBnet()) {
    rebuffer->cleanupVirtualBuffer();
    if (ctx.runtime_map)
      (*ctx.runtime_map)["buf_reject_reeval_fail"] += 1.0;
    return result;
  }

  // Check if best option actually has buffers
  int buf_count_in_tree = 0;
  if (rebuffer->bestBnet()) {
    visitTree(
      [&](auto& recurse, int level, const rsz::BufferedNetPtr& node) -> int {
        switch (node->type()) {
          case rsz::BufferedNetType::buffer: buf_count_in_tree++; return recurse(node->ref());
          case rsz::BufferedNetType::junction: return recurse(node->ref()) + recurse(node->ref2());
          case rsz::BufferedNetType::wire: case rsz::BufferedNetType::via: return recurse(node->ref());
          default: return 0;
        }
      }, rebuffer->bestBnet());
  }

  if (buf_count_in_tree == 0) {
    // bestBnet has no buffers — this is just a resize, not a real buffer insertion
    if (ctx.runtime_map)
      (*ctx.runtime_map)["buf_accept_no_real_buffer"] += 1.0;
    rebuffer->cleanupVirtualBuffer();
    return result;  // return empty, let resize-only path handle it
  }

  if (ctx.runtime_map)
    (*ctx.runtime_map)["buf_accept_with_buffer"] += 1.0;

  if (best_buf_is_original) {
    result.type = MoveOption::BUFFER_ONLY;
    result.target_cell = nullptr;
  } else {
    result.type = MoveOption::COMBINED;
    result.target_cell = best_buf_cell;
  }
  result.cost = best_buf_cost;
  result.buffer_tree = rebuffer->bestBnet();
  return result;
}

MoveOption
CombinedOperator::evaluate(PtGraph *pt_graph, sta::Instance *inst,
                           EvalContext &ctx)
{
  MoveOption best;
  sta::LibertyCell *ori_cell = db_sta_->network()->libertyCell(inst);
  if (!ori_cell)
    return best;

  // Route based on allow_resize/allow_buffer flags from TaskArranger move mask
  if (ctx.allow_resize && !ctx.allow_buffer)
    return resize_op_->evaluate(pt_graph, inst, ctx);
  if (!ctx.allow_resize && ctx.allow_buffer)
    return buffer_op_->evaluate(pt_graph, inst, ctx);
  if (!ctx.allow_resize && !ctx.allow_buffer)
    return best;

  // Both allowed: full combined path
  // ── Phase 1: Evaluate resize candidates, get top-1 ──
  std::vector<MoveOption> top_n = resize_op_->evaluateTopN(pt_graph, inst, ctx, 1);

  // Compute original cost
  float ori_cost;
  float ori_delay_lm_sum, ori_leakage;
  {
    ori_delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, ori_cell).delay_lm_sum;
    ori_leakage = resize_op_->lookupLeakage(inst, ori_cell);
    ori_cost = ctx.swapCost(ori_delay_lm_sum, ori_leakage);
  }

  if (ctx.debug) {
    float delay_part = ctx.PT_tradeoff * ori_delay_lm_sum / ctx.average_delay;
    float leak_part = ori_leakage / ctx.average_leakage;
    printf("[DBG-COMBINED] %s ori_cell=%s delay_lm=%.3e leak=%.3e "
           "delay_part=%.3e leak_part=%.3e ratio=%.2f ori_cost=%.3e "
           "PT=%.1f avg_delay=%.3e avg_leak=%.3e\n",
           db_sta_->network()->pathName(inst), ori_cell->name(),
           ori_delay_lm_sum, ori_leakage,
           delay_part, leak_part,
           leak_part > 0 ? delay_part / leak_part : 0.0f,
           ori_cost, ctx.PT_tradeoff, ctx.average_delay, ctx.average_leakage);
  }

  // Best resize-only candidate
  if (!top_n.empty() && top_n[0].cost < ori_cost)
    best = top_n[0];

  // ── Phase 2: Buffering on top-1 + one-size-smaller + original ──
  // Use the better of ori_cost and resize_cost as threshold so buffer must
  // beat both to be accepted.
  float baseline_cost = best.hasChange() ? best.cost : ori_cost;
  auto start_buf = std::chrono::high_resolution_clock::now();

  MoveOption buf_result = tryBufferingOnTop1AndSmaller(
      pt_graph, inst, ctx, top_n, ori_cell, baseline_cost);

  if (ctx.runtime_map) {
    auto end_buf = std::chrono::high_resolution_clock::now();
    (*ctx.runtime_map)["buffer_insertion"] +=
        std::chrono::duration<double>(end_buf - start_buf).count();
  }

  // Buffer already beat baseline (resize or original) — accept directly.
  if (buf_result.hasChange())
    best = buf_result;

  // Ensure PtGraph in correct state for the winner
  if (best.hasChange() && best.target_cell) {
    local_sta_->increAndGetLocalTimingCost(pt_graph, ctx.arc_delay_calc,
                                           best.target_cell);
  }

  return best;
}

void
CombinedOperator::setEvalContext(EvalContext *ctx)
{
  if (buffer_op_)
    buffer_op_->setEvalContext(ctx);
}

std::unique_ptr<LrOperator>
CombinedOperator::copy() const
{
  auto op = std::make_unique<CombinedOperator>(
      db_sta_, local_sta_,
      buffer_op_ ? buffer_op_->resizer() : nullptr,
      nullptr);  // ctx=nullptr; caller must call setEvalContext() after
  op->resize_op_->setEquivCellArray(resize_op_->equiv_cell_array_,
                                    resize_op_->equiv_cell_pos_map_);
  op->resize_op_->setInstInfoMap(resize_op_->inst_info_map_);
  op->resize_op_->setSlackMargin(resize_op_->slack_margin_);
  op->resize_op_->setColPadding(resize_op_->col_padding_);
  op->resize_op_->setRowPadding(resize_op_->row_padding_);
  return op;
}

void
CombinedOperator::apply(const MoveOption &move, PtGraph *pt_graph,
                         std::map<std::string, double> &runtime_map)
{
  switch (move.type) {
    case MoveOption::RESIZE_ONLY:
      resize_op_->apply(move, pt_graph, runtime_map);
      break;
    case MoveOption::BUFFER_ONLY:
      buffer_op_->apply(move, pt_graph, runtime_map);
      break;
    case MoveOption::COMBINED:
      resize_op_->apply(move, pt_graph, runtime_map);
      buffer_op_->apply(move, pt_graph, runtime_map);
      break;
    case MoveOption::NONE:
      break;
  }
}

// ═══════════════════════════════════════════════════════════
// ParallelVisitor
// ═══════════════════════════════════════════════════════════

ParallelVisitor::ParallelVisitor(sta::dbSta *db_sta, LocalSta *local_sta,
                                 rsz::Resizer *resizer)
  : db_sta_(db_sta), local_sta_(local_sta), resizer_(resizer)
{
  eval_ctx_.arc_delay_calc = local_sta_->arcDelayCalc()->copy();
  eval_ctx_.runtime_map = &runtime_map_;
}

ParallelVisitor::~ParallelVisitor()
{
  delete eval_ctx_.arc_delay_calc;
}

void
ParallelVisitor::init(float average_delay, float average_power, float wns,
                      float PT_tradeoff,
                      std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map)
{
  eval_ctx_.average_delay = average_delay;
  eval_ctx_.average_leakage = average_power;
  eval_ctx_.PT_tradeoff = PT_tradeoff;

  // Compute slack_margin from WNS and clock period
  float clock_period = 0.0f;
  for (auto *clock : *db_sta_->sdc()->clocks()) {
    if (clock->period() > clock_period) {
      clock_period = clock->period();
      break;
    }
  }
  float slack_margin = (wns >= 0.0f)
      ? 1.05f
      : std::max(-std::min(wns, 0.0f) / clock_period + 1.0f, 1.05f);
  printf("slack_margin: %f\n", slack_margin);
  fflush(stdout);

  // Also expose via EvalContext so LrRebuffer::evaluateOption gate uses the
  // same margin (otherwise buffering candidates get rejected by strict gate
  // while same margin is applied on the resize / size-up path).
  eval_ctx_.slack_margin = slack_margin;

  // Propagate to operator via virtual interface
  if (operator_) {
    operator_->setSlackMargin(slack_margin);
    operator_->setInstInfoMap(inst_info_map);
  }
}

bool
ParallelVisitor::visit(sta::Instance *inst, sta::VertexId vid)
{
  auto start_time = std::chrono::steady_clock::now();
  best_move_ = MoveOption{};

  // Early skip: operator can reject this instance before PtGraph construction
  // (e.g. BufferSensitivityOperator skips slack >= 0 instances).
  // precheck_results_ keeps its pre-initialized -inf value for skipped entries.
  if (operator_ && operator_->skipInstance(inst)) {
    runtime_map_["skip_count"] += 1.0;
    return false;
  }

  // Build PtGraph (visitor-owned, freed at next visit or destructor)
  auto start_pt = std::chrono::high_resolution_clock::now();
  pt_graph_.reset(new PtGraph(db_sta_));
  const bool driver_only = operator_
      && operator_->ptGraphLevel() == LrOperator::PtGraphLevel::DriverOnly;
  if (driver_only)
    local_sta_->makePtGraphDriverOnly(pt_graph_.get(), inst);
  else
    local_sta_->makePtGraph(pt_graph_.get(), inst);
  if (!driver_only)
    pt_graph_->pruneInsignificantSiblings();
  auto end_pt = std::chrono::high_resolution_clock::now();
  runtime_map_["pt_graph_construction"] +=
      std::chrono::duration<double>(end_pt - start_pt).count();

  eval_ctx_.pt_graph = pt_graph_.get();

  // Set move mask from TaskArranger if available
  if (task_arranger_ && vid != sta::object_id_null) {
    auto *v = task_arranger_->vertex(vid);
    eval_ctx_.allow_resize = v->doResize();
    eval_ctx_.allow_buffer = v->doBuffer();
  } else {
    eval_ctx_.allow_resize = true;
    eval_ctx_.allow_buffer = true;
  }

  // Evaluate via operator
  if (operator_)
    best_move_ = operator_->evaluate(pt_graph_.get(), inst, eval_ctx_);

  // Precheck mode: store cost in results vector, don't apply to DB
  if (precheck_results_ && vid != sta::object_id_null) {
    float cost_change = (best_move_.cost < std::numeric_limits<float>::max())
                            ? best_move_.cost : 0.0f;
    (*precheck_results_)[vid] = {inst, cost_change, static_cast<size_t>(vid)};
    auto end_time = std::chrono::steady_clock::now();
    runtime_map_["precheck"] +=
        std::chrono::duration<double>(end_time - start_time).count();
    return false;  // no DB changes in precheck
  }

  // Always write back timing so downstream instances see up-to-date
  // slew/arrival on shared vertices, even when no resize/buffer is chosen.
  updateTimingFromPtGraph(pt_graph_.get());

  // Track visit/change counts for pruning K detection
  resize_visit_count_++;
  if (best_move_.hasChange())
    resize_change_count_++;

  auto end_time = std::chrono::steady_clock::now();
  runtime_map_["visit"] +=
      std::chrono::duration<double>(end_time - start_time).count();
  return best_move_.hasChange();
}

bool
ParallelVisitor::singleGateSizing(sta::Instance *inst)
{
  if (visit(inst, sta::object_id_null)) {
    applyChangesToDb(nullptr);
    return true;
  }
  return false;
}

void
ParallelVisitor::visitSlewOnly(sta::Instance *inst)
{
  pt_graph_.reset(new PtGraph(db_sta_));
  local_sta_->makePtGraph(pt_graph_.get(), inst);
  local_sta_->findLocalDelays(pt_graph_.get(), eval_ctx_.arc_delay_calc);
  local_sta_->findLocalArrivals(pt_graph_.get());
  local_sta_->findLocalRequireds(pt_graph_.get());
  updateTimingFromPtGraph(pt_graph_.get());
}

void
ParallelVisitor::applyChangesToDb(rsz::Resizer *resizer)
{
  std::lock_guard<std::mutex> lock(g_odb_sta_access_mutex);
  if (operator_)
    operator_->apply(best_move_, pt_graph_.get(), runtime_map_);
}

ParallelVisitor *
ParallelVisitor::copy() const
{
  auto *v = new ParallelVisitor(db_sta_, local_sta_, resizer_);
  v->eval_ctx_.average_delay = eval_ctx_.average_delay;
  v->eval_ctx_.average_leakage = eval_ctx_.average_leakage;
  v->eval_ctx_.PT_tradeoff = eval_ctx_.PT_tradeoff;
  v->eval_ctx_.density_map = eval_ctx_.density_map;
  v->eval_ctx_.density_weight = eval_ctx_.density_weight;
  v->eval_ctx_.average_area = eval_ctx_.average_area;
  v->task_arranger_ = task_arranger_;
  v->precheck_results_ = precheck_results_;

  if (operator_) {
    v->operator_ = operator_->copy();
    v->operator_->setEvalContext(&v->eval_ctx_);
  }
  return v;
}

void
ParallelVisitor::printRuntimeProfile() const
{
  printf("ParallelVisitor Runtime Profile:\n");
  for (const auto &entry : runtime_map_) {
    printf("  %-30s: %.6f seconds\n", entry.first.c_str(), entry.second);
  }
  auto safe_get = [&](const std::string &key) -> double {
    auto it = runtime_map_.find(key);
    return (it != runtime_map_.end()) ? it->second : 0.0;
  };
  double equiv_count = safe_get("equiv_cell_count");
  double equiv_time = safe_get("equiv_cell_check");
  if (equiv_count > 0) {
    printf("  Average equiv cell check time: %.9f seconds\n",
           equiv_time / equiv_count);
  }
  // Print percentage breakdown within equiv_cell_check
  if (equiv_time > 0.0) {
    printf("\n  --- equiv_cell_check breakdown (%%  of %.3fs) ---\n", equiv_time);
    const char *sub_keys[] = {
      "legalCheckBeforeSwap", "vrc_setRefGate", "vrc_recomputeParasitics",
      "findLocalDelays", "findLocalArrivals", "findLocalRequireds",
      "delayLmSum", "legalCheckAfterSwap"
    };
    double accounted = 0.0;
    for (const char *key : sub_keys) {
      double val = safe_get(key);
      accounted += val;
      printf("    %-30s: %8.4fs  (%5.1f%%)\n", key, val,
             val / equiv_time * 100.0);
    }
    double unaccounted = equiv_time - accounted;
    if (unaccounted > 0.001) {
      printf("    %-30s: %8.4fs  (%5.1f%%)\n", "(other/overhead)", unaccounted,
             unaccounted / equiv_time * 100.0);
    }
  }
}

}  // namespace lrf

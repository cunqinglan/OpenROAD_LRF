#include "NetlistTransformation.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "LrRebufferV2.hh"
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
EvalContext::swapCost(float delay_lm_sum, float power) const
{
  return PT_tradeoff * delay_lm_sum / average_delay
       + power / average_leakage;
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
    PtVertexType type = pt_vertex.type();
    if (type == PtVertexType::RefInput
     || type == PtVertexType::RefOutput
     || type == PtVertexType::SiblingLoad) {
      pt_graph->writeSlewToGraph(pt_vertex, pt_vertex.vertex());
      pt_graph->writePathsToGraph(pt_vertex, pt_vertex.vertex());
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

  std::vector<sta::LibertyCell*> candidates = collectCandidates(ori_cell);
  if (candidates.size() < 2)
    return result;

  auto start_eval = std::chrono::high_resolution_clock::now();

  // Pass 1: evaluate all candidates, store (cost, slack) pairs
  std::vector<float> vec_cost_slack(candidates.size() * 2,
                                    std::numeric_limits<float>::max());
  float slack_before = 0.0f;

  for (size_t i = 0; i < candidates.size(); i++) {
    sta::LibertyCell *cand = candidates[i];

    if (!local_sta_->legalCheckBeforeSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float leakage = lookupLeakage(inst, cand);
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, cand).delay_lm_sum;

    if (!local_sta_->legalCheckAfterSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float cost = ctx.swapCost(delay_lm_sum, leakage);
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

  // Pass 1: evaluate all candidates, store (cost, slack) pairs
  std::vector<float> vec_cost_slack(candidates.size() * 2,
                                    std::numeric_limits<float>::max());
  float slack_before = 0.0f;

  for (size_t i = 0; i < candidates.size(); i++) {
    sta::LibertyCell *cand = candidates[i];

    if (!local_sta_->legalCheckBeforeSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float leakage = lookupLeakage(inst, cand);
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, cand).delay_lm_sum;

    if (!local_sta_->legalCheckAfterSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float cost = ctx.swapCost(delay_lm_sum, leakage);
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
  auto mid = std::chrono::steady_clock::now();
  runtime_map["swap"] += std::chrono::duration<double>(mid - start).count();
  updateTimingFromPtGraph(pt_graph);
  auto end = std::chrono::steady_clock::now();
  runtime_map["writeTimingToDb"] +=
      std::chrono::duration<double>(end - start).count();
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
        pt_graph, ctx.arc_delay_calc, cand).delay_lm_sum;

    if (!local_sta_->legalCheckAfterSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float cost = ctx.swapCost(delay_lm_sum, leakage);
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
    rebuffer_ = std::make_unique<LrRebufferV2>(resizer, local_sta, ctx);
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
      

  // Sync pt_graph into eval context so LrRebufferV2 sees the right graph
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
  rebuffer_ = std::make_unique<LrRebufferV2>(resizer_, local_sta_, ctx);
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
  LrRebufferV2 *rebuffer = buffer_op_ ? buffer_op_->rebuffer() : nullptr;
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
  if (!cached_bnet)
    return result;

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
      if (cost < best_buf_cost) {
        best_buf_cost = cost;
        best_buf_cell = bc.cell;
        best_buf_is_original = bc.is_original;
      }
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
  // ── Phase 1: Evaluate resize candidates, get top-2 ──
  std::vector<MoveOption> top_n = resize_op_->evaluateTopN(pt_graph, inst, ctx, 2);

  // Compute original cost
  float ori_cost;
  {
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, ctx.arc_delay_calc, ori_cell).delay_lm_sum;
    float leakage = resize_op_->lookupLeakage(inst, ori_cell);
    ori_cost = ctx.swapCost(delay_lm_sum, leakage);
  }

  // Best resize-only candidate
  if (!top_n.empty() && top_n[0].cost < ori_cost)
    best = top_n[0];

  // ── Phase 2: Two-phase buffering on top-2 candidates ──
  auto start_buf = std::chrono::high_resolution_clock::now();

  MoveOption buf_result = tryBufferingOnCandidates(
      pt_graph, inst, ctx, top_n, ori_cell, ori_cost);

  if (ctx.runtime_map) {
    auto end_buf = std::chrono::high_resolution_clock::now();
    (*ctx.runtime_map)["buffer_insertion"] +=
        std::chrono::duration<double>(end_buf - start_buf).count();
  }

  // ── Phase 3: Decision — pick best among resize-only vs buffer results ──
  if (buf_result.hasChange() && buf_result.cost < best.cost)
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

  // Build PtGraph (visitor-owned, freed at next visit or destructor)
  auto start_pt = std::chrono::high_resolution_clock::now();
  pt_graph_.reset(new PtGraph(db_sta_));
  local_sta_->makePtGraph(pt_graph_.get(), inst);
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
    printf("  %s: %.6f seconds\n", entry.first.c_str(), entry.second);
  }
  double equiv_count = runtime_map_.at("equiv_cell_count");
  double equiv_time = runtime_map_.at("equiv_cell_check");
  if (equiv_count > 0) {
    printf("  Average equiv cell check time: %.9f seconds\n",
           equiv_time / equiv_count);
  }
}

}  // namespace lrf

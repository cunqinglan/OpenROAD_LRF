#include "EcoController.hh"

#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "lrf/IncreSta.hh"
#include "LocalSta.hh"
#include "TaskArranger.hh"
#include "est/EstimateParasitics.h"

namespace lrf {

EcoController::EcoController(const EcoConfig &config,
                             IncreSta *incre_sta,
                             sta::dbSta *sta,
                             odb::dbBlock *block,
                             rsz::Resizer *resizer)
  : config_(config),
    incre_sta_(incre_sta),
    sta_(sta),
    block_(block),
    resizer_(resizer),
    local_sta_(incre_sta->localSta())
{
}

EcoDecision
EcoController::decide(size_t iter,
                      const IterationHelper::Metrics &cur,
                      const IterationHelper::Metrics &best)
{
  // Hard wall-clock limit: terminate immediately if exceeded.
  if (config_.max_runtime_seconds > 0) {
    double elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - wall_start_).count();
    if (elapsed > config_.max_runtime_seconds) {
      printf("ECO: wall-clock limit reached (%.0fs > %.0fs), terminating at iter %zu.\n",
             elapsed, config_.max_runtime_seconds, iter + 1);
      fflush(stdout);
      return EcoDecision::TERMINATE;
    }
  }

  double cur_wns = cur.wns_ps / 1e12;
  double best_wns = best.wns_ps / 1e12;

  // Track whether we've ever reached positive WNS (timing met).
  // Once met, any regression back to WNS < 0 triggers immediate termination
  // to protect the achieved timing closure.
  if (best_wns >= 0.0)
    reached_positive_wns_ = true;
  if (reached_positive_wns_ && cur_wns < 0.0) {
    printf("ECO: WNS regressed below 0 (%.3f ps) after reaching timing closure, "
           "terminating at iter %zu.\n", cur.wns_ps, iter + 1);
    fflush(stdout);
    return EcoDecision::TERMINATE;
  }

  bool improved = (cur_wns > best_wns && cur_wns < 0)
      || (cur_wns >= 0.0 && (cur_wns > best_wns || cur.leakage < best.leakage));

  if (improved) {
    consecutive_reverts_ = 0;
    total_accepts_++;
    return EcoDecision::ACCEPT;
  }

  // Not improved.
  if (iter < config_.warmup_iters) {
    // Warmup: only revert when both WNS and TNS regressed.
    // If only one metric worsened, accept and keep going — the netlist may
    // still be making useful progress along the other dimension.
    bool wns_worse = (cur.wns_ps < best.wns_ps);
    bool tns_worse = (cur.tns_ps < best.tns_ps);
    if (wns_worse && tns_worse) {
      total_reverts_++;
      return EcoDecision::REVERT_WARMUP;
    }
    // Partial regression — treat as accept during warmup.
    consecutive_reverts_ = 0;
    total_accepts_++;
    return EcoDecision::ACCEPT;
  }

  if (!in_eco_) {
    in_eco_ = true;
    first_eco_entry_ = true;
  }

  consecutive_reverts_++;
  total_reverts_++;

  if (consecutive_reverts_ > config_.max_eco_reverts)
    return EcoDecision::TERMINATE;

  return EcoDecision::REVERT;
}

float
EcoController::updateRatio(EcoDecision decision)
{
  float current_ratio = incre_sta_->adaptiveTopRatio();

  if (decision == EcoDecision::ACCEPT) {
    first_eco_entry_ = false;
    return current_ratio;
  }

  // Warmup revert is pre-ECO: don't touch the ratio (no halving, no init
  // ratio re-derivation — those only make sense once ECO has started).
  if (decision == EcoDecision::REVERT_WARMUP)
    return current_ratio;

  if (config_.strategy == EcoStrategy::NO_HALVE)
    return current_ratio;

  if (first_eco_entry_) {
    first_eco_entry_ = false;
    int change_count = incre_sta_->lastChangeCount();
    TaskArranger *ta = local_sta_->taskArranger();
    int total = static_cast<int>(ta->vertexCount());
    float init_ratio = (total > 0)
        ? static_cast<float>(change_count) * 1.5f / total
        : 0.3f;
    printf("ECO mode: init ratio=%.4f (changes=%d, total=%d)\n",
           init_ratio, change_count, total);
    return init_ratio;
  }

  if (config_.strategy == EcoStrategy::HALVE_ALWAYS) {
    float new_ratio = current_ratio * config_.halve_factor;
    printf("Halve ratio → %.4f\n", new_ratio);
    return new_ratio;
  }

  // HALVE_ON_CONSECUTIVE
  if (consecutive_reverts_ > 1) {
    float new_ratio = current_ratio * config_.halve_factor;
    printf("Consecutive revert → halve ratio to %.4f\n", new_ratio);
    return new_ratio;
  } else {
    printf("First revert after accept → keep ratio %.4f\n", current_ratio);
    return current_ratio;
  }
}

void
EcoController::executeAccept()
{
  odb::dbDatabase::endEco(block_);
  odb::dbDatabase::beginEco(block_);
}

void
EcoController::executeRevert()
{
  if (config_.lm_update_before_revert)
    incre_sta_->lmUpdate();

  odb::dbDatabase::endEco(block_);
  odb::dbDatabase::undoEco(block_);
  local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
  sta_->delaysInvalid();
  sta_->updateTiming(true);
  odb::dbDatabase::beginEco(block_);
}

bool
EcoController::execute(EcoDecision decision,
                       IterationHelper::Metrics &best,
                       const IterationHelper::Metrics &cur)
{
  bool first_revert = false;
  switch (decision) {
    case EcoDecision::ACCEPT:
      best = cur;
      executeAccept();
      break;
    case EcoDecision::REVERT:
      executeRevert();
      first_revert = (total_reverts_ == 1);
      break;
    case EcoDecision::REVERT_WARMUP:
      // Roll back the regression but don't touch `best`. Keeps the netlist
      // at the prior (better) checkpoint so the next warmup iter re-runs
      // the resize from the timing-best state.
      executeRevert();
      break;
    case EcoDecision::TERMINATE:
      executeRevert();
      break;
  }
  return first_revert;
}

EcoDecision
EcoController::runIteration(size_t iter,
                            IterationHelper::Metrics &best,
                            IterationHelper &helper,
                            float avg_delay, float avg_leakage,
                            float PT_tradeoff)
{
  // ① lmUpdate + findRequireds
  incre_sta_->lmUpdate();
  sta_->findRequireds();

  // ② Resize
  auto start = std::chrono::high_resolution_clock::now();

  bool use_precheck = in_eco_ && config_.use_precheck;
  if (use_precheck) {
    float ratio = incre_sta_->adaptiveTopRatio();
    printf("----- ECO Iteration %zu (precheck, ratio=%.4f) -----\n", iter+1, ratio);
    incre_sta_->parallelResizeByArrayWithPrecheck(
        resizer_, avg_delay, avg_leakage, PT_tradeoff, top_ratio_);
  } else {
    printf("----- %s Iteration %zu -----\n",
           in_eco_ ? "ECO (full)" : "Phase1", iter+1);
    incre_sta_->parallelResizeByArray(
        resizer_, avg_delay, avg_leakage, PT_tradeoff);
  }

  auto end = std::chrono::high_resolution_clock::now();
  double runtime = std::chrono::duration<double>(end - start).count();

  // ③ Sync parasitics + timing
  local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
  sta_->delaysInvalid();
  sta_->updateTiming(true);

  // ④ Snapshot metrics
  IterationHelper::Metrics cur = helper.snapshot(runtime);
  printf("WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
         cur.wns_ps, cur.tns_ps, cur.leakage * 1e10, runtime);

  // ⑤ Decide
  EcoDecision decision = decide(iter, cur, best);

  // ⑥ Update ratio
  float new_ratio = updateRatio(decision);
  incre_sta_->setAdaptiveTopRatio(new_ratio);

  // ⑦ Execute
  switch (decision) {
    case EcoDecision::ACCEPT:
      best = cur;
      executeAccept();
      break;
    case EcoDecision::REVERT:
    case EcoDecision::REVERT_WARMUP:
    case EcoDecision::TERMINATE:
      executeRevert();
      break;
  }

  // ⑧ Log
  helper.recordRow(iter+1, in_eco_ ? "eco" : "phase1", cur, best,
                   decisionStr(decision));
  printf("Decision: %s\n", decisionStr(decision));
  fflush(stdout);

  return decision;
}

const char *
EcoController::decisionStr(EcoDecision d) const
{
  switch (d) {
    case EcoDecision::ACCEPT:        return "accept";
    case EcoDecision::REVERT:        return "revert";
    case EcoDecision::REVERT_WARMUP: return "revert(warmup)";
    case EcoDecision::TERMINATE:     return "terminate";
  }
  return "unknown";
}

const char *
EcoController::strategyStr() const
{
  switch (config_.strategy) {
    case EcoStrategy::HALVE_ALWAYS:         return "halve_always";
    case EcoStrategy::HALVE_ON_CONSECUTIVE: return "halve_on_consecutive";
    case EcoStrategy::NO_HALVE:             return "no_halve";
  }
  return "unknown";
}

void
EcoController::printSummary() const
{
  printf("ECO summary: %zu accepts, %zu reverts (strategy=%s, halve=%.2f)\n",
         total_accepts_, total_reverts_, strategyStr(), config_.halve_factor);
}

}  // namespace lrf

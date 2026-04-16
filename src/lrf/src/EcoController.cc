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
  double cur_wns = cur.wns_ps / 1e12;
  double best_wns = best.wns_ps / 1e12;

  bool improved = (cur_wns > best_wns && cur_wns < 0)
      || (cur_wns >= 0.0 && (cur_wns > best_wns || cur.leakage < best.leakage));

  if (improved) {
    consecutive_reverts_ = 0;
    total_accepts_++;
    return EcoDecision::ACCEPT;
  }

  if (iter < config_.warmup_iters) {
    total_accepts_++;
    return EcoDecision::ACCEPT_WARMUP;
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

  if (decision == EcoDecision::ACCEPT || decision == EcoDecision::ACCEPT_WARMUP) {
    first_eco_entry_ = false;
    return current_ratio;
  }

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
    case EcoDecision::ACCEPT_WARMUP:
      best = cur;
      executeAccept();
      break;
    case EcoDecision::REVERT:
      executeRevert();
      first_revert = (total_reverts_ == 1);
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
    case EcoDecision::ACCEPT_WARMUP:
      best = cur;
      executeAccept();
      break;
    case EcoDecision::REVERT:
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
    case EcoDecision::ACCEPT_WARMUP: return "accept(warmup)";
    case EcoDecision::REVERT:        return "revert";
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

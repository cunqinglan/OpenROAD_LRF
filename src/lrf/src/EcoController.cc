#include "EcoController.hh"

#include <cmath>
#include <limits>

#include "LocalSta.hh"
#include "LrfUtil.hh"
#include "TaskArranger.hh"
#include "db_sta/dbSta.hh"
#include "est/EstimateParasitics.h"
#include "lrf/IncreSta.hh"
#include "rsz/Resizer.hh"

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
  if (best_wns >= 0.0)
    reached_positive_wns_ = true;

  // Accept criterion. The old test scored only WNS while timing was unmet (and
  // ignored TNS entirely), so a recovery iteration that held WNS but improved
  // TNS — or held timing and improved power — was reverted. Enrich only the
  // not-yet-met branch; the timing-met branch keeps its original power-recovery
  // behaviour (downsizing that trades WNS margin for leakage while staying
  // met). kMetricTolPs treats sub-0.1ps wobble as "flat" so a genuine
  // lower-priority gain is not masked by STA noise (a resize elsewhere in the
  // design routinely moves WNS by ~0.05-0.3ps).
  constexpr double kMetricTolPs = 0.1;
  const double dwns = cur.wns_ps - best.wns_ps;  // ps
  const double dtns = cur.tns_ps - best.tns_ps;  // ps
  const bool wns_flat = std::fabs(dwns) <= kMetricTolPs;
  const bool tns_flat = std::fabs(dtns) <= kMetricTolPs;
  bool improved =
      // Timing not yet met: WNS is primary; when WNS is flat a TNS gain still
      // counts; when both are flat a leakage gain at zero timing cost counts.
      (cur_wns < 0.0
       && ((dwns > kMetricTolPs) || (wns_flat && dtns > kMetricTolPs)
           || (wns_flat && tns_flat && cur.leakage < best.leakage)))
      // Timing met (TNS is 0 here): accept more WNS margin or any leakage
      // reduction while timing stays met — the original power-recovery rule.
      || (cur_wns >= 0.0 && (cur_wns > best_wns || cur.leakage < best.leakage));

  // A strict improvement always wins — never let plateau / closure-protection
  // discard an iteration that genuinely advanced the best metrics.
  if (improved) {
    consecutive_reverts_ = 0;
    total_accepts_++;
    return EcoDecision::ACCEPT;
  }

  // Closure protection: once timing met, any regression back to WNS<0 ends.
  if (reached_positive_wns_ && cur_wns < 0.0) {
    printf("ECO: WNS regressed below 0 (%.3f ps) after reaching timing closure, "
           "terminating at iter %zu.\n", cur.wns_ps, iter + 1);
    fflush(stdout);
    return EcoDecision::TERMINATE;
  }

  // Leakage plateau in power mode: terminate if avg per-iter reduction
  // over the last 3 power-mode iters falls below 1%. Only checked on
  // non-improving iters so a strict timing/leakage gain is never discarded.
  if (detectLeakagePlateau(iter, cur))
    return EcoDecision::TERMINATE;

  // Not improved.
  if (iter < config_.warmup_iters) {
    // Warmup: tolerate early-LM oscillation only when the iteration actually
    // advanced a metric. The only such case reaching here is a TNS gain paid
    // with WNS (a WNS gain is already accepted by `improved` above). The old
    // both-regressed revert rule was a loophole on designs with a pinned WNS
    // (an unfixable port path): wns_worse never fired, so warmup accepted
    // arbitrary TNS damage from the fresh-LM first passes into `best`.
    if (dtns > kMetricTolPs) {
      consecutive_reverts_ = 0;
      total_accepts_++;
      return EcoDecision::ACCEPT;
    }
    total_reverts_++;
    return EcoDecision::REVERT_WARMUP;
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

  if (config_.strategy == EcoStrategy::ADAPTIVE_FROM_CHANGE) {
    // ACCEPT or first REVERT → recompute from realized change count.
    // Consecutive REVERT → halve current ratio as safety net.
    int change = incre_sta_->lastChangeCount();
    int total  = static_cast<int>(local_sta_->taskArranger()->vertexCount());
    float r = (total > 0)
        ? std::max(static_cast<float>(change) * config_.adaptive_multiplier
                       / static_cast<float>(total),
                   config_.adaptive_floor)
        : current_ratio;
    printf("Adaptive ratio → %.4f (change=%d/%d × %.2f)\n",
           r, change, total, config_.adaptive_multiplier);
    return r;
  }

  // HALVE_ON_CONSECUTIVE
  if (consecutive_reverts_ > 1) {
    float new_ratio = current_ratio * config_.halve_factor;
    if (lrfVerbose()) printf("Consecutive revert → halve ratio to %.4f\n", new_ratio);
    return new_ratio;
  } else {
    if (lrfVerbose()) printf("First revert after accept → keep ratio %.4f\n", current_ratio);
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
  sta_->updateTiming(false);
  odb::dbDatabase::beginEco(block_);
}

void
EcoController::executeTerminate()
{
  if (config_.lm_update_before_revert)
    incre_sta_->lmUpdate();

  odb::dbDatabase::endEco(block_);
  odb::dbDatabase::undoEco(block_);
  local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
  sta_->delaysInvalid();
  sta_->updateTiming(false);
  local_sta_->taskArranger()->markDirty();
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
      // Same rollback as REVERT but does NOT reopen the ECO frame; callers
      // exit the loop and must not run a redundant final endEco/undoEco.
      executeTerminate();
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
    if (lrfVerbose()) {
      printf("----- ECO Iteration %zu (precheck, ratio=%.4f) -----\n", iter+1, ratio);
    }
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
  sta_->updateTiming(false);

  // ④ Snapshot metrics
  IterationHelper::Metrics cur = helper.snapshot(runtime);
  printf("WNS: %.3f ps, TNS: %.3f ps, Leakage: %.3f uW (%.1fs)\n",
         cur.wns_ps, cur.tns_ps, cur.leakage * 1e6, runtime);

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
      executeRevert();
      break;
    case EcoDecision::TERMINATE:
      executeTerminate();
      break;
  }

  // ⑧ Log
  helper.recordRow(iter+1, in_eco_ ? "eco" : "phase1", cur, best,
                   decisionStr(decision));
  if (lrfVerbose()) {
    printf("Decision: %s\n", decisionStr(decision));
  }
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
    case EcoStrategy::ADAPTIVE_FROM_CHANGE: return "adaptive_from_change";
  }
  return "unknown";
}

void
EcoController::printSummary() const
{
  printf("ECO summary: %zu accepts, %zu reverts (strategy=%s, halve=%.2f)\n",
         total_accepts_, total_reverts_, strategyStr(), config_.halve_factor);
}

// Queries IncreSta's global rolling history (single source of truth).
// History is populated externally via incre_sta_->recordMetrics() at
// snapshot time, so this controller is a pure consumer.
bool
EcoController::detectLeakagePlateau(size_t iter,
                                    const IterationHelper::Metrics &)
{
  if (incre_sta_ && incre_sta_->isLeakagePlateau()) {
    printf("ECO: global leakage plateau in power mode "
           "(3-iter avg reduction < 1%%), terminating at iter %zu.\n",
           iter + 1);
    fflush(stdout);
    return true;
  }
  return false;
}

// ─── BufferEcoController ─────────────────────────────────────────────
//
// Encapsulates the standard buffering-ECO config (NO_HALVE, no warmup,
// no lm-update-before-revert, effectively unlimited reverts) and forces
// any TERMINATE decision back to REVERT so a buffering pass never tears
// down the outer LR loop.

static EcoConfig
makeBufferEcoConfig()
{
  EcoConfig cfg = EcoConfig::make(EcoStrategy::NO_HALVE);
  cfg.lm_update_before_revert = false;
  cfg.warmup_iters = 0;
  cfg.max_eco_reverts = std::numeric_limits<size_t>::max();
  return cfg;
}

BufferEcoController::BufferEcoController(IncreSta *incre_sta,
                                         sta::dbSta *sta,
                                         odb::dbBlock *block,
                                         rsz::Resizer *resizer)
  : EcoController(makeBufferEcoConfig(), incre_sta, sta, block, resizer)
{
}

EcoDecision
BufferEcoController::decide(size_t iter,
                            const IterationHelper::Metrics &cur,
                            const IterationHelper::Metrics &best)
{
  EcoDecision d = EcoController::decide(iter, cur, best);
  return d == EcoDecision::TERMINATE ? EcoDecision::REVERT : d;
}

}  // namespace lrf

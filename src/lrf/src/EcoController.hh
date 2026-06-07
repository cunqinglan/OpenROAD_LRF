#pragma once

#include <cstddef>
#include <cstdio>
#include <chrono>
#include "lrf/LrConfig.hh"
#include "lrf/TestLrf.hh"

namespace odb { 
class dbBlock; 
}

namespace sta { 
class dbSta; 
}

namespace rsz { 
class Resizer; 
}

namespace lrf {

class IncreSta;
class LocalSta;
class TaskArranger;

enum class EcoDecision {
  ACCEPT,
  REVERT,          // ECO revert: undo regression, count toward ECO termination
  REVERT_WARMUP,   // Warmup revert: undo regression, stay in phase1, preserve best
  TERMINATE
};

// EcoController: owns ECO state machine and executes accept/revert operations.
// Holds pointers to IncreSta, dbSta, dbBlock — can perform lmUpdate, ECO
// checkpoint operations, parasitic sync, and ratio updates internally.
class EcoController {
public:
  EcoController(const EcoConfig &config,
                IncreSta *incre_sta,
                sta::dbSta *sta,
                odb::dbBlock *block,
                rsz::Resizer *resizer);
  virtual ~EcoController() = default;

  // ── Main entry: run one ECO iteration ──
  // Performs: lmUpdate → resize → evaluate → decide → execute.
  // Updates best in-place on accept. Returns the decision.
  EcoDecision runIteration(size_t iter,
                           IterationHelper::Metrics &best,
                           IterationHelper &helper,
                           float avg_delay, float avg_leakage,
                           float PT_tradeoff);

  // Execute a decision: accept (endEco+beginEco) or revert (lmUpdate+undoEco+sync).
  // Updates best on accept. Returns true if this is the first revert (for checkpoint).
  bool execute(EcoDecision decision, IterationHelper::Metrics &best,
               const IterationHelper::Metrics &cur);

  // ── State queries ──
  bool inEco() const { return in_eco_; }
  size_t consecutiveReverts() const { return consecutive_reverts_; }
  size_t totalAccepts() const { return total_accepts_; }
  size_t totalReverts() const { return total_reverts_; }
  const EcoConfig &config() const { return config_; }

  const char *decisionStr(EcoDecision d) const;
  const char *strategyStr() const;
  void printSummary() const;

  // ── Decision logic ──
  virtual EcoDecision decide(size_t iter,
                             const IterationHelper::Metrics &cur,
                             const IterationHelper::Metrics &best);

protected:
  // Track leakage history and signal termination if avg per-iter reduction
  // over the last 3 power-mode iters drops below 1%. Default: implements the
  // check. Override to a no-op for ECOs that don't optimize leakage
  // (e.g. buffering) — the override should not accumulate state either.
  virtual bool detectLeakagePlateau(size_t iter,
                                    const IterationHelper::Metrics &cur);

public:

  // ── Ratio management ──
  float updateRatio(EcoDecision decision);

  // ── ECO execution ──
  void executeAccept();
  void executeRevert();
  // Like executeRevert but does NOT reopen the ECO frame. Use when no further
  // iterations will run (TERMINATE), so the caller does not need a trailing
  // endEco/undoEco cleanup.
  void executeTerminate();

  // Whether ECO phase should use precheck (vs full resize)
  bool usePrecheck() const { return in_eco_ && config_.use_precheck; }

private:

  EcoConfig config_;
  IncreSta *incre_sta_;
  sta::dbSta *sta_;
  odb::dbBlock *block_;
  rsz::Resizer *resizer_;
  LocalSta *local_sta_;

  bool in_eco_ = false;
  bool first_eco_entry_ = false;
  bool reached_positive_wns_ = false;  // true once best WNS >= 0
  size_t consecutive_reverts_ = 0;
  size_t total_accepts_ = 0;
  size_t total_reverts_ = 0;
  float top_ratio_ = 0.3f;
  std::chrono::steady_clock::time_point wall_start_
      = std::chrono::steady_clock::now();
};

// Buffering-specific ECO. Encapsulates the standard buffer-ECO config
// (NO_HALVE strategy, no warmup, no lm-update-before-revert, effectively
// unlimited revert tolerance) and overrides decide() so the buffering
// pass never terminates the outer LR loop and never triggers the
// leakage-plateau check (buffering optimizes timing, not leakage).
class BufferEcoController : public EcoController {
public:
  BufferEcoController(IncreSta *incre_sta,
                      sta::dbSta *sta,
                      odb::dbBlock *block,
                      rsz::Resizer *resizer);

  EcoDecision decide(size_t iter,
                     const IterationHelper::Metrics &cur,
                     const IterationHelper::Metrics &best) override;

protected:
  // Buffering ECO doesn't optimize leakage; skip plateau check entirely
  // (no state accumulation either).
  bool detectLeakagePlateau(size_t,
                            const IterationHelper::Metrics &) override
  { return false; }
};

}  // namespace lrf

#pragma once

#include <cstddef>
#include <string>

namespace lrf {

// Modes for LR optimization — selects which operators and flow to use.
enum class LrMode {
  RESIZE,           // Pure resize (conflict-graph dispatch)
  RESIZE_BUFFER,    // Resize iterations + buffering phases
  PRECHECK,         // Precheck screening + resize
  PRECHECK_BUFFER,  // Precheck screening + resize + buffering
  COMBINED,         // Single-pass combined resize+buffer visitor
  RESIZE_RSZ_BUFFER // Resize iterations + RSZ-style rebuffering phases
};

// ── ECO Strategy ──────────────────────────────────────────
// Controls how the optimizer handles regression (WNS worse than best).
enum class EcoStrategy {
  HALVE_ALWAYS,          // Original: halve ratio on every revert
  HALVE_ON_CONSECUTIVE,  // Only halve on consecutive reverts; accept resets
  NO_HALVE               // Never halve, full resize every ECO iter
};

// ECO configuration — controls revert/accept/halve behavior.
struct EcoConfig {
  EcoStrategy strategy = EcoStrategy::HALVE_ON_CONSECUTIVE;
  float halve_factor = 0.5f;          // ratio *= halve_factor on revert
  size_t warmup_iters = 3;            // first N iters unconditionally accept
  size_t max_eco_reverts = 6;         // terminate after N consecutive reverts
  bool use_precheck = true;           // ECO phase uses precheck (vs full resize)
  bool lm_update_before_revert = true;// run lmUpdate on worse state before revert
  double max_runtime_seconds = 10800.0; // Hard wall-clock limit for LR loop (0=no limit, default 3h)

  // Preset configurations from experimental results (ECO_halve_effect.md).
  static EcoConfig make(EcoStrategy preset) {
    EcoConfig cfg;
    cfg.strategy = preset;
    switch (preset) {
      case EcoStrategy::HALVE_ON_CONSECUTIVE:
        // Best overall: halve=0.5, precheck, lmUpdate before revert.
        // Verified on ac97_top (2 ECO accepts) and fpu (1-2 ECO accepts).
        cfg.halve_factor = 0.5f;
        cfg.use_precheck = true;
        cfg.lm_update_before_revert = true;
        cfg.warmup_iters = 6;
        cfg.max_eco_reverts = 6;
        break;
      case EcoStrategy::HALVE_ALWAYS:
        // Original flow behavior (×0.25 on every revert).
        cfg.halve_factor = 0.25f;
        cfg.use_precheck = true;
        cfg.lm_update_before_revert = false;
        cfg.warmup_iters = 0;
        cfg.max_eco_reverts = 6;
        break;
      case EcoStrategy::NO_HALVE:
        // Full resize every ECO iter, no ratio change.
        // Experimentally: 0 ECO accepts after convergence.
        cfg.halve_factor = 1.0f;
        cfg.use_precheck = false;
        cfg.lm_update_before_revert = true;
        cfg.warmup_iters = 6;
        cfg.max_eco_reverts = 6;
        break;
    }
    return cfg;
  }
};

// All tunable parameters for an LR optimization run.
// Add new knobs here — no function signatures need to change.
struct LrConfig {
  // ── Mode ──
  LrMode mode = LrMode::RESIZE;

  // ── Iteration control ──
  size_t iterations = 12;
  size_t max_resize_num = 20000000;
  size_t num_no_improve_tolerance = 6;

  // ── Optimization weights ──
  float PT_tradeoff = 100.0f;
  float density_weight = 0.0f;      // 0 = disabled
  bool  ratcons = false;

  // ── LR solver ──
  std::string lr_helper_method = "RapidLRHelper";

  // ── Precheck ──
  float top_ratio = 0.3f;

  // ── Buffering ──
  // Fraction of total instances to select as buffering candidates (by
  // sensitivity). Used instead of a fixed absolute count so screening scales
  // with design size. 0.01 = top 1%.
  float buffer_top_ratio = 0.01f;
  float bakoglu_k = 2.5f;    // Bakoglu gate coefficient (lower = more nets eligible for buffering)
  size_t buffering_start_iter = 5;  // 1-indexed iter from which Buffering pass is allowed
                                    // (default 5 preserves original "i > 3" gating)

  // ── Legal check margin ──
  // Fraction of the library slew limit reserved as headroom in LocalSta's
  // legalCheck* (applied in getLegalSlewLimit() via limit * (1 - margin)).
  // Purpose: the optimizer picks cells that have enough slew slack to survive
  // the placement-RC → global_routing-RC change introduced by DPL+GRT, so we
  // don't ship post-GRT slew violations the contest eval would flag.
  // Default 0.10 = 10% headroom. Set to 0.0 to disable.
  float slew_margin = 0.10f;

  // ── LR Helper timing margin ──
  // Absolute slack headroom (seconds) subtracted from arc_slack inside
  // RapidLrHelper::getMultiplier *before* the critical/non-critical k
  // selection.  Paths with 0 < slack < timing_margin are treated as
  // violating (k=critical_arc_k_), making the optimizer fight harder to
  // keep headroom that survives post-GR degradation.
  // Default 0 = no headroom.  Typical: 20e-12 .. 50e-12 (20–50 ps).
  float timing_margin = 0.01f;

  // ── Initialization ──
  bool initialize = false;           // Run Sharma 3-step init before LR

  // ── Debug ──
  bool debug = false;             // Print detailed rebuffer/eval diagnostics

  // ── ECO ──
  EcoConfig eco;

  // ── Checkpoint ──
  // If non-empty, save ODB + LM at first regression point.
  std::string checkpoint_dir;
};

}  // namespace lrf

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
  COMBINED          // Single-pass combined resize+buffer visitor
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
        cfg.warmup_iters = 3;
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
        cfg.warmup_iters = 3;
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
  int buffer_top_n = 100;
  float bakoglu_k = 2.5f;    // Bakoglu gate coefficient (lower = more nets eligible for buffering)

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

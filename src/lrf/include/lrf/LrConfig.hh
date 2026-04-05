#pragma once

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

  // ── Initialization ──
  bool initialize = false;           // Run Sharma 3-step init before LR
};

}  // namespace lrf

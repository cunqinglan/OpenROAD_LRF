// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cstddef>

namespace rmp {

// Tunable parameters for PositionDrivenStrategy::remap().
// All fields carry defaults matching the previously hardcoded values.
struct RemapConfig {
  // Max successful remaps per endpoint before moving to the next endpoint.
  size_t max_vertices_per_endpoint = 1;

  // Gate-cloning threshold: if the extracted cut has more instances than this,
  // build a gate-clone cut instead of sending the oversized cut to ABC.
  size_t max_cut_instances = 5;

  // PI safety limit: skip a cut (even after gate cloning) when it has more
  // primary inputs than this, to avoid ABC enumeration hangs.
  size_t max_cut_pis = 10;

  // Number of mapping solutions ABC enumerates in the initial pass.
  int max_solutions = 150;

  // UCT (Upper Confidence Tree) search parameters.
  // UCT is disabled by default (uct_rounds=0): grid search across 4 designs
  // showed beam-only (ur=0) matches or beats UCT on 3/4 designs.
  // UCT helps ~2.5% on ac97_top; users can enable with -uct_rounds 3.
  int uct_batch_size = 20;
  int uct_rounds     = 0;
  double uct_c       = 1.414;

  // Use beam search instead of exhaustive backtracking for mapping enumeration.
  bool use_beam_search = false;

  // Beam search diversity weight. 0 = pure score (default), >0 encourages
  // structurally diverse solutions during beam pruning.
  // Grid search showed diversity has no consistent benefit after repair_timing.
  float beam_diversity = 0.0;

  // Candidate-vertex search cap in getWorstVertices / getWorstVerticesForEndpoint.
  size_t max_candidates = 100;

  // Number of top solutions from Phase 1 to re-evaluate with repairSetup
  // in Phase 2. Set to 0 to skip Phase 2 (raw slack ranking only).
  // Disabled by default: repairSetup in forked children crashes due to
  // stale STA pointers after networkChanged(). Enable when fixed.
  int top_k = 0;

  // Per-child timeout in seconds. If a forked child doesn't complete within
  // this limit, it is killed. 0 = no timeout (default).
  int child_timeout = 0;
};

}  // namespace rmp

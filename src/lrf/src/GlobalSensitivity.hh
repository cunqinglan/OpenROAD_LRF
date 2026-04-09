#pragma once

#include <vector>
#include <unordered_map>

#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/TimingArc.hh"
#include "sta/ArcDelayCalc.hh"
#include "lrf/LrfClass.hh"

namespace lrf {

// Computes per-arc Lambda-Delay Sensitivity (φ) on the full STA timing graph
// (Flach 2014 TCAD, Section VII-C, Eq.11).
//
// φ captures how a change in an arc's input slew propagates through downstream
// paths and affects the total lambda-delay.  Called once per LR iteration.
//
// Usage in cost function (Eq.5):
//   lambdaDelayCost(g) = Σ λ*d (driver/gate/sink arcs, computed locally)
//                      + Σ Δslew*φ (side arcs)
//                      + Σ Δslew_n * Σφ (drain nets)
//
// The side arc term is already implicitly handled by PtGraph's delayLmSum
// (sibling edges are in the local graph). The NEW contribution is the
// drain net term: for each net driven by a sink gate of the ref instance,
// φ captures the downstream timing impact beyond PtGraph's boundary.
class GlobalSensitivity {
public:
  GlobalSensitivity(::sta::Sta *sta);
  ~GlobalSensitivity() = default;

  // Compute φ for all arcs on the global graph.
  // Must be called after a full STA update (delays, slews, LMs current).
  void compute(::sta::ArcDelayCalc *arc_delay_calc,
               const ::sta::DcalcAnalysisPt *dcalc_ap);

  bool isReady() const { return ready_; }
  void invalidate() { ready_ = false; }

  // Look up drain-net φ sum for a global sta::Vertex (driver pin of a sink gate).
  // This is Σφ over all arcs driven by the net rooted at this driver,
  // i.e., the sum needed for Eq.13: ΔD^λ_n = Δslew_n * drainPhiSum(driver).
  float drainPhiSum(const ::sta::Vertex *drvr_vertex) const;

  // Look up φ for a specific arc on a specific edge (for debugging).
  float phi(const ::sta::Edge *edge, const ::sta::TimingArc *arc) const;

private:
  struct EdgeSensitivity {
    std::vector<ArcSensitivity> arcs;
  };

  const EdgeSensitivity *edgeSensitivity(const ::sta::Edge *edge) const;
  EdgeSensitivity &getOrCreateEdgeSensitivity(const ::sta::Edge *edge);

  ::sta::Sta *sta_;
  ::sta::DcalcAPIndex ap_index_ = 0;
  size_t ap_count_ = 0;
  bool ready_ = false;

  // Per-edge sensitivity, keyed by edge ObjectIdx.
  std::unordered_map<::sta::ObjectIdx, EdgeSensitivity> edge_sens_;

  // Per-vertex: max |φ| among incoming arcs (used in backward propagation).
  std::vector<float> vertex_dominant_phi_;

  // Per-vertex: sum of φ over all outgoing arcs from this driver vertex.
  // This is Σφ_{i→j} for all arcs i→j where i is this vertex.
  // Used for drain net cost: ΔD^λ_n = Δslew_n * drain_phi_sum_[vertex_id].
  std::vector<float> drain_phi_sum_;
};

}  // namespace lrf

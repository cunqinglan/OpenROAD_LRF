// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "parasitics/ConcreteParasiticsPvt.hh"
#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/ParasiticsClass.hh"

namespace lrf {

using sta::Parasitics;
using sta::Pin;
using sta::PinSet;
using sta::VertexId;

// Sentinel for "no parent" (root) and "no tree-node mapping".
constexpr uint32_t kInvalidTreeNodeIdx = std::numeric_limits<uint32_t>::max();

// One node of the full RC tree (mirrors a ParasiticNode at reduce time).
// Children are pushed AFTER their parent during the DFS, so children always
// have larger indices than their parent. Iterate tree_ in REVERSE for
// natural post-order processing (Algorithm 2's shape).
//
// `delay` and `impulse_sq` are slew-INVARIANT — pure functions of RC tree
// topology — and populated by precomputeMoments() at reduce time. They
// are the only Algorithm 1 outputs gateDelay actually queries at runtime:
//   - delay[n]      → wire_delay published per load (paper §IV-A step 5)
//   - impulse_sq[n] → fed to Eq.15 with the runtime slew_i to produce
//                     per-node slew[n] (used both for T_n in Algorithm 2
//                     and for per-load load_slew published)
// The intermediate Algorithm 1 quantities (load, ldelay, beta) live in
// PtElmoreCeff::algo1_*_scratch_ — mutable scratch vectors with reused
// capacity, kept OFF the node to keep PtRcNode small. Profiling showed
// putting them as fields (32B node) hurt cache behavior on wide RC trees
// even though it killed the malloc; separate scratch vectors with
// amortized 0-alloc reuse give the best of both.
struct PtRcNode {
  uint32_t parent_idx;   // kInvalidTreeNodeIdx for root
  float    branch_R;     // R of edge (parent → this); 0 for root
  float    local_cap;    // C at this node (gnd cap + coupling·factor + pin cap)
  float    delay;        // Elmore delay from root to this node (paper Algo.1)
  float    impulse_sq;   // 2·beta[n] - delay[n]²  (clamped at 0 for FP safety)
  // Algorithm 2 per-call accumulator. Initialized to local_cap at the start
  // of computeCeffAlgo2WithRefinedSlew, then receives Σ child contributions
  // as the reverse-order sweep walks up. Safe to live on the node because
  // ParallelVisitor allocates a fresh PtGraph per worker.visit() —
  // PtElmoreCeff is never touched by multiple workers concurrently, and
  // gateDelay calls within one worker are serial.
  float    ceff;
};

// Per-load record. tree_node_idx points back into PtElmoreCeff::tree_ so
// per-load wire delay / slew can use the cached node-level data without
// re-walking the tree.
struct PtRcLoad {
  VertexId    vertex_id;
  const Pin  *pin;            // nullptr for virtual loads
  uint32_t    tree_node_idx;  // kInvalidTreeNodeIdx if not mapped
  float       elmore;         // plain Elmore wire delay (B1 fallback path)
};

// PtGraph-local ElmoreCeff parasitic — sibling to PtPiElmore.
//
// Semantically a FULL RC TREE with Elmore-class moments — *not* a Pi-model
// 2-pole approximation. We therefore inherit only sta::ConcreteParasitic
// (the polymorphic abstract base needed to be a sta::Parasitic), and DO
// NOT inherit sta::ConcretePi. The default ConcreteParasitic returns false
// for isPiElmore()/isPiModel()/isPoleResidue()/..., which is honest:
// PtElmoreCeff is none of those.
//
// All inheritance does for us is: (a) be polymorphic so the OpenSTA
// gateDelay API can carry us as `const Parasitic *`, and (b) expose
// capacitance() / findElmore() through the standard interface in case
// any generic code queries them. Everything ElmoreCeff-specific (tree,
// loads, Ceff[root]) lives on this class directly.
class PtElmoreCeff : public sta::ConcreteParasitic
{
public:
  PtElmoreCeff() = default;

  // ConcreteParasitic interface ---------------------------------------------
  float capacitance() const override { return total_cap_; }
  void findElmore(const Pin *load_pin,
                  float &elmore, bool &exists) const override;
  // setElmore: ConcreteParasitic default is no-op; we don't need writeback.
  PinSet unannotatedLoads(const Pin *drvr_pin,
                          const Parasitics *parasitics) const override;
  // Intentionally NOT overriding isPiElmore()/isPiModel()/piModel()/
  // setPiModel(): default false / no-op, which is what we actually are.

  // VertexId-based Elmore lookup (for virtual loads without Pin*).
  float findElmoreByVertexId(VertexId vid, bool &exists) const;

  // Tree access -------------------------------------------------------------
  const std::vector<PtRcNode> &tree() const { return tree_; }
  std::vector<PtRcNode>       &tree()       { return tree_; }
  size_t treeNodeCount() const { return tree_.size(); }
  uint32_t rootIdx() const {
    return tree_.empty() ? kInvalidTreeNodeIdx : 0u;
  }

  // Per-load access ---------------------------------------------------------
  void addLoad(VertexId vid, const Pin *pin,
               uint32_t tree_node_idx, float elmore);
  const std::vector<PtRcLoad> &loads() const { return loads_; }
  size_t loadCount() const { return loads_.size(); }
  // Returns nullptr if pin isn't a load on this net. Gives callers
  // access to {elmore, tree_node_idx} so they can index back into tree_
  // for the cached delay / impulse_sq.
  const PtRcLoad *findLoadByPin(const Pin *load_pin) const;
  // Same as findLoadByPin but indexed by VertexId — used by the virtual-
  // load path in LocalSta::annotateLoadDelays where the load PtVertex
  // doesn't have a real Pin*.
  const PtRcLoad *findLoadByVertexId(VertexId vid) const;

  // Total cap (= C_total = Σ local_cap over the tree). Used as the initial
  // Cload seed by the gateDelay pipeline.
  float totalCap() const { return total_cap_; }
  void  setTotalCap(float v) { total_cap_ = v; }

  // Cached Ceff[root]. Populated by computeCeffAlgo2WithRefinedSlew() and
  // re-read by gateDelay. 0.0f sentinel means "not computed yet";
  // totalCap() is the safe fallback.
  float ceffRoot() const { return ceff_root_; }
  void  setCeffRoot(float v) { ceff_root_ = v; }

  // Paper Algorithm 1 (page-4 pseudocode). Four sequential O(N) array
  // sweeps over tree_ that fill in delay[n] and impulse_sq[n] for every
  // node. These are slew-INVARIANT (pure RC tree moments), so we compute
  // them once at reduce time and cache them on PtRcNode. gateDelay uses
  // Eq.15 at runtime to turn (slew_i, impulse_sq[n]) into per-node slew[n].
  //
  // No-op on empty tree.
  void precomputeMoments();

  // Paper Algorithm 2 (paper §III-B Eq.11). Single post-order traversal of
  // tree_ accumulating Ceff at each parent from its children's branches.
  //
  // Per-branch T_n derives from the parent's Eq.15-refined slew:
  //   slew[parent]² = slew_i² + slew_factor² · impulse_sq[parent]
  //   T_n           = sqrt(slew[parent]²) / 0.8     (ramp_factor=0.8)
  //
  // slew_i is the driver-output slew from the initial NLDM seed lookup
  // (paper §IV-A step 1, typically NLDM(in_slew, C_total)).
  // slew_factor is the corner-dependent Eq.15 factor (paper §III-C
  // heuristic: √(2π) for slow corners, 1.0 for fast corners).
  //
  // Returns Ceff[root]. Caches into ceff_root_. Safe with empty tree
  // (→ totalCap()), and degenerates gracefully on pathological RC values
  // (no-shielding fallback per branch).
  //
  // PRECONDITION: precomputeMoments() must have been called.
  // Uses the in-place `PtRcNode::ceff` accumulator — no scratch needed,
  // no allocation, no parameter. See PtRcNode::ceff for why this is
  // safe under LRF's per-worker PtGraph model.
  float computeCeffAlgo2WithRefinedSlew(float slew_i, float slew_factor);

  void clear();

private:
  std::vector<PtRcNode> tree_;
  std::vector<PtRcLoad> loads_;
  std::unordered_map<const Pin*, size_t> pin_index_;
  std::unordered_map<VertexId, size_t>   vid_index_;
  float total_cap_ = 0.0f;
  float ceff_root_ = 0.0f;

  // Algorithm 1 / Algorithm 2 scratch vectors are kept as static thread_local
  // inside precomputeMoments() and computeCeffAlgo2WithRefinedSlew() — see
  // .cc. Empirically, per-instance scratch (mutable members) caused a fpu LR
  // regression vs the original per-call malloc: on big designs each thread
  // visits thousands of PtElmoreCeff instances, and per-instance buffers
  // cold-fault into L1 on every visit. thread_local concentrates all calls
  // from one thread onto ONE buffer, keeping it cache-hot regardless of
  // which PtElmoreCeff is being processed.
};

} // namespace lrf

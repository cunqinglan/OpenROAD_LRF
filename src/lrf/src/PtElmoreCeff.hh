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
struct PtRcNode {
  uint32_t parent_idx;   // kInvalidTreeNodeIdx for root
  float    branch_R;     // R of edge (parent → this); 0 for root
  float    local_cap;    // C at this node (gnd cap + coupling·factor + pin cap)
  // Phase B1.3: float ceff;    // Algorithm 2 accumulator
  // Phase B2 : float delay;    // Algorithm 1 Elmore delay
  //            float ldelay;
  //            float beta;
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

  // Total cap (= C_total = Σ local_cap over the tree). Used as the initial
  // Cload seed by the gateDelay pipeline.
  float totalCap() const { return total_cap_; }
  void  setTotalCap(float v) { total_cap_ = v; }

  // Cached Ceff[root]. Populated by computeCeffAlgo2() (Phase B1.3) and
  // re-read by gateDelay. 0.0f sentinel means "not computed yet";
  // totalCap() is the safe fallback.
  float ceffRoot() const { return ceff_root_; }
  void  setCeffRoot(float v) { ceff_root_ = v; }

  // Paper Algorithm 2 (Eq.11): single post-order traversal of tree_ to
  // accumulate the closed-form effective capacitance at the driver root.
  // Ts is the input ramp duration (= drvr slew / ramp_factor, where the
  // paper uses ramp_factor=0.8 for 10–90% slew → 0–100% ramp).
  //
  // B1.3 supplies a UNIFORM Ts to every branch (taken from the driver's
  // slew estimate). B2 will replace it with per-node Ts derived from
  // Algorithm 1's Eq.15 refined slew so each subtree branch gets its
  // own ramp duration.
  //
  // Caches result into ceff_root_ and returns it. Safe with empty tree
  // (returns totalCap()), Ts<=0 (no shielding → C_total), or pathological
  // RC values (no-shielding fallback per branch).
  float computeCeffAlgo2(float Ts);

  void clear();

private:
  std::vector<PtRcNode> tree_;
  std::vector<PtRcLoad> loads_;
  std::unordered_map<const Pin*, size_t> pin_index_;
  std::unordered_map<VertexId, size_t>   vid_index_;
  float total_cap_ = 0.0f;
  float ceff_root_ = 0.0f;
};

} // namespace lrf

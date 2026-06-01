// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "PtElmoreCeff.hh"

#include <algorithm>
#include <cmath>

namespace lrf {

void
PtElmoreCeff::findElmore(const Pin *load_pin,
                         float &elmore, bool &exists) const
{
  exists = false;
  elmore = 0.0f;
  if (!load_pin) return;
  auto it = pin_index_.find(load_pin);
  if (it != pin_index_.end()) {
    elmore = loads_[it->second].elmore;
    exists = true;
  }
}

sta::PinSet
PtElmoreCeff::unannotatedLoads(const Pin *, const Parasitics *) const
{
  return sta::PinSet();
}

float
PtElmoreCeff::findElmoreByVertexId(VertexId vid, bool &exists) const
{
  auto it = vid_index_.find(vid);
  if (it != vid_index_.end()) {
    exists = true;
    return loads_[it->second].elmore;
  }
  exists = false;
  return 0.0f;
}

void
PtElmoreCeff::addLoad(VertexId vid, const Pin *pin,
                      uint32_t tree_node_idx, float elmore)
{
  const size_t idx = loads_.size();
  loads_.push_back({vid, pin, tree_node_idx, elmore});
  if (pin)
    pin_index_[pin] = idx;
  vid_index_[vid] = idx;
}

const PtRcLoad *
PtElmoreCeff::findLoadByPin(const Pin *load_pin) const
{
  if (!load_pin) return nullptr;
  auto it = pin_index_.find(load_pin);
  if (it == pin_index_.end()) return nullptr;
  return &loads_[it->second];
}

const PtRcLoad *
PtElmoreCeff::findLoadByVertexId(VertexId vid) const
{
  auto it = vid_index_.find(vid);
  if (it == vid_index_.end()) return nullptr;
  return &loads_[it->second];
}

void
PtElmoreCeff::precomputeMoments()
{
  const size_t n = tree_.size();
  if (n == 0) return;

  // Paper Algorithm 1 (page 4). Four sequential O(N) array sweeps.
  // tree_ is laid out in PRE-order (parent before children), so:
  //   - post-order = REVERSE iteration  (children before parent)
  //   - pre-order  = FORWARD iteration  (parent before children)
  //
  // Cannot fuse passes: pass 3 needs pass 2's delay; pass 4 needs pass 3's
  // ldelay. Strict dependency chain.
  //
  // Per-call vectors. reduce is called ~1000× less than gateDelay, so the
  // alloc cost is negligible against tcmalloc/jemalloc's heap fast-path.
  // A `thread_local` static was tested and turned out SLOWER overall
  // (+5.6% LR on fpu) — dynamic TLS access cost from this PIC lib exceeded
  // the alloc savings at this call rate.
  std::vector<float> load(n, 0.0f);
  std::vector<float> delay(n, 0.0f);
  std::vector<float> ldelay(n, 0.0f);
  std::vector<float> beta(n, 0.0f);

  // Pass 1 (post-order): load[n] = local_cap[n] + Σ load[children]
  for (size_t i = n; i-- > 0; ) {
    load[i] += tree_[i].local_cap;
    const uint32_t p = tree_[i].parent_idx;
    if (p != kInvalidTreeNodeIdx)
      load[p] += load[i];
  }

  // Pass 2 (pre-order): delay[n] = delay[parent] + R_branch · load[n]
  // Root (idx 0) keeps delay 0.
  for (size_t i = 1; i < n; ++i) {
    const PtRcNode &node = tree_[i];
    delay[i] = delay[node.parent_idx] + node.branch_R * load[i];
  }

  // Pass 3 (post-order): ldelay[n] = local_cap·delay + Σ ldelay[children]
  for (size_t i = n; i-- > 0; ) {
    ldelay[i] += tree_[i].local_cap * delay[i];
    const uint32_t p = tree_[i].parent_idx;
    if (p != kInvalidTreeNodeIdx)
      ldelay[p] += ldelay[i];
  }

  // Pass 4 (pre-order): beta[n] = beta[parent] + R_branch · ldelay[n].
  // impulse_sq[n] = 2·beta[n] - delay[n]², clamped at 0 for FP safety
  // (2β-δ² is non-negative analytically but can dip slightly below 0 for
  // tiny near-zero values). Write the slew-invariant outputs (delay and
  // impulse_sq) into tree_[i] for gateDelay to read.
  tree_[0].delay = 0.0f;
  tree_[0].impulse_sq = 0.0f;
  for (size_t i = 1; i < n; ++i) {
    const PtRcNode &node = tree_[i];
    beta[i] = beta[node.parent_idx] + node.branch_R * ldelay[i];
    tree_[i].delay = delay[i];
    tree_[i].impulse_sq = std::max(
        0.0f, 2.0f * beta[i] - delay[i] * delay[i]);
  }
}

float
PtElmoreCeff::computeCeffAlgo2WithRefinedSlew(float slew_i, float slew_factor)
{
  const size_t n = tree_.size();
  if (n == 0) {
    ceff_root_ = total_cap_;
    return ceff_root_;
  }

  constexpr float ramp_factor = 0.8f;
  const float slew_i_sq = slew_i * slew_i;
  const float factor_sq = slew_factor * slew_factor;

  // In-place accumulator on PtRcNode::ceff. Init from local_cap; each
  // non-root iteration below adds its child contribution to its parent's
  // ceff. No external scratch, no TLS, no parameter — every access is a
  // straight offset from &tree_[i]. Safe: see PtRcNode::ceff doc-comment.
  for (size_t i = 0; i < n; ++i)
    tree_[i].ceff = tree_[i].local_cap;

  // Reverse iteration ⇒ post-order. For each non-root branch (parent, i):
  //   slew[parent]² = slew_i² + slew_factor² · impulse_sq[parent]   (Eq.15)
  //   T_n           = sqrt(slew[parent]²) / ramp_factor
  //   ΔCeff[parent] = ceff[i] · T_n / (T_n + 2·R_n·ceff[i])         (Eq.11)
  for (size_t i = n - 1; i > 0; --i) {
    PtRcNode &node = tree_[i];
    const float parent_slew_sq =
        slew_i_sq + factor_sq * tree_[node.parent_idx].impulse_sq;
    const float Tn = std::sqrt(parent_slew_sq) / ramp_factor;

    const float denom = Tn + 2.0f * node.branch_R * node.ceff;
    float contribution = node.ceff;  // fallback: no shielding (Elmore-style sum)
    if (Tn > 0.0f && denom > 0.0f)
      contribution = node.ceff * Tn / denom;
    tree_[node.parent_idx].ceff += contribution;
  }

  ceff_root_ = tree_[0].ceff;
  return ceff_root_;
}

void
PtElmoreCeff::clear()
{
  tree_.clear();
  loads_.clear();
  pin_index_.clear();
  vid_index_.clear();
  total_cap_ = 0.0f;
  ceff_root_ = 0.0f;
}

} // namespace lrf

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "PtElmoreCeff.hh"

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

float
PtElmoreCeff::computeCeffAlgo2(float Ts)
{
  const size_t n = tree_.size();
  if (n == 0) {
    ceff_root_ = total_cap_;
    return ceff_root_;
  }

  // Initialize per-node Ceff to its local cap (Algorithm 2 line 3:
  // "C_eff[n] += C_n"). Children's branch contributions are added below.
  std::vector<float> ceff(n);
  for (size_t i = 0; i < n; ++i)
    ceff[i] = tree_[i].local_cap;

  // tree_ is in PRE-order (parent before children), so iterating in
  // REVERSE visits children before their parents — natural post-order
  // for Algorithm 2's accumulation. Root (idx 0) has no parent; skip.
  for (size_t i = n - 1; i > 0; --i) {
    const PtRcNode &node = tree_[i];
    const float denom = Ts + 2.0f * node.branch_R * ceff[i];
    // Default: no shielding (degenerates to Elmore-cap summation).
    // Applied when Ts<=0 (no ramp info) or denom is non-positive
    // (numerical edge case from pathological RC values).
    float contribution = ceff[i];
    if (Ts > 0.0f && denom > 0.0f)
      contribution = (ceff[i] * Ts) / denom;
    ceff[node.parent_idx] += contribution;
  }

  ceff_root_ = ceff[0];
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

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include "dcalc/LumpedCapDelayCalc.hh"

namespace lrf {

// ElmoreCeff delay calculator (paper: Liu/Guo/Wang/Lin, ISEDA 2026).
//
// Inherits sta::LumpedCapDelayCalc purely for plumbing access
// (pinPvt, thresholdAdjust, makeResult, default findParasitic/etc).
// gateDelay() is overridden completely with NO super call — the
// computation is entirely owned by this class. There is no DMP
// instance involved anywhere on the gate-delay path.
//
// Phase B1.1 (current): gateDelay does
//   1) ceff = C_total      (placeholder; B1.3 replaces with Algorithm 2)
//   2) one direct NLDM table lookup via arc->gateModel()
//   3) per-load wire_delay/load_slew via plain Elmore (PtPiElmore per-load)
// No iteration, no Newton, no root-find.
//
// Phase B2 (current): full paper pipeline (paper §IV-A):
//   1. NLDM(in_slew, C_total)  → slew_i₀  (seed driver output slew)
//   2. Algorithm 2 (Eq.11 per-branch T_n via Eq.15 + cached impulse_sq)
//                                → Ceff[root]
//   3. NLDM(in_slew, Ceff)     → final drvr_slew, gate_delay
//   4. per-load wire_delay = delay[load_n] (Elmore, cached)
//      per-load load_slew   = sqrt(drvr_slew² + factor²·impulse_sq[load_n])
//
// No Newton, no fixed-point iteration. Two NLDM table lookups + cached
// O(N) tree-array sweeps.
class LocalElmoreCeffDelayCalc : public sta::LumpedCapDelayCalc
{
public:
  explicit LocalElmoreCeffDelayCalc(sta::StaState *sta);

  sta::ArcDelayCalc *copy() override;
  std::string_view name() const override { return "lrf_elmore_ceff"; }

  sta::ArcDcalcResult gateDelay(const sta::Pin *drvr_pin,
                                const sta::TimingArc *arc,
                                const sta::Slew &in_slew,
                                float load_cap,
                                const sta::Parasitic *parasitic,
                                const sta::LoadPinIndexMap &load_pin_index_map,
                                const sta::Scene *scene, const sta::MinMax *min_max) override;

protected:
  explicit LocalElmoreCeffDelayCalc(const LocalElmoreCeffDelayCalc &other);
  // slew_factor lives on the dcalcSlewFactor() shared helper so that
  // gateDelay (here) and LocalSta::annotateLoadDelays' virtual-load
  // path apply the same Eq.15 factor.
};

sta::ArcDelayCalc *
makeLocalElmoreCeffDelayCalc(sta::StaState *sta);

} // namespace lrf

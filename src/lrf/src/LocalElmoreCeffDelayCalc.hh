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
// Phase B1.2: extends PtPiElmore to cache the full RC tree topology.
// Phase B1.3: replaces step (1)'s ceff = C_total with paper Algorithm 2
//             (post-order Eq.11) using T_n = in_slew/ramp_factor.
// Phase B2:   adds Algorithm 1 + Eq.15 refined per-node slew so T_n
//             becomes per-node and load_slew uses Eq.15.
class LocalElmoreCeffDelayCalc : public sta::LumpedCapDelayCalc
{
public:
  explicit LocalElmoreCeffDelayCalc(sta::StaState *sta);

  sta::ArcDelayCalc *copy() override;
  const char *name() const override { return "lrf_elmore_ceff"; }

  sta::ArcDcalcResult gateDelay(const sta::Pin *drvr_pin,
                                const sta::TimingArc *arc,
                                const sta::Slew &in_slew,
                                float load_cap,
                                const sta::Parasitic *parasitic,
                                const sta::LoadPinIndexMap &load_pin_index_map,
                                const sta::DcalcAnalysisPt *dcalc_ap) override;

protected:
  explicit LocalElmoreCeffDelayCalc(const LocalElmoreCeffDelayCalc &other);
};

sta::ArcDelayCalc *
makeLocalElmoreCeffDelayCalc(sta::StaState *sta);

} // namespace lrf

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, The OpenROAD Authors
//
// Pluggable arc delay calculator for the lrf local STA.
//
// Phase A (this file): a behavior-preserving wrapper that delegates the
// gate / input-port / load delay calculation to OpenSTA's dmp_ceff_elmore
// calculator. The point is purely to give the lrf flow a SINGLE, swappable
// delay-calc module (mirroring the OpenSTA ArcDelayCalc plug-in pattern)
// so that Phase B can replace the Dartu/Menezes/Pileggi iterative Ceff with
// the closed-form ElmoreCeff model without touching every call site.
//
// It also hosts the Elmore wire-delay / load-slew formula that lrf uses for
// virtual (pin-less) loads, relocated out of LocalSta::annotateLoadDelays
// (which carried an inline copy of DmpCeff::dspfWireDelaySlew).

#pragma once

#include "dcalc/LumpedCapDelayCalc.hh"

namespace sta {
class LibertyLibrary;
}

namespace lrf {

// Wraps an OpenSTA dmp_ceff_elmore calculator. Inherits LumpedCapDelayCalc so
// every ArcDelayCalc method that lrf does not exercise keeps its stock
// behavior; the public entry points lrf actually uses are forwarded to the
// wrapped DMP calculator so results stay bit-identical to the current flow.
class LocalDmpDelayCalc : public sta::LumpedCapDelayCalc
{
public:
  explicit LocalDmpDelayCalc(sta::StaState *sta);
  ~LocalDmpDelayCalc() override;

  sta::ArcDelayCalc *copy() override;
  const char *name() const override { return "lrf_local_dmp"; }
  void copyState(const sta::StaState *sta) override;

  // Forwarded to the wrapped DMP calculator (the paths lrf calls).
  sta::ArcDcalcResult gateDelay(const sta::Pin *drvr_pin,
                                const sta::TimingArc *arc,
                                const sta::Slew &in_slew,
                                float load_cap,
                                const sta::Parasitic *parasitic,
                                const sta::LoadPinIndexMap &load_pin_index_map,
                                const sta::DcalcAnalysisPt *dcalc_ap) override;
  sta::ArcDcalcResult inputPortDelay(const sta::Pin *port_pin,
                                     float in_slew,
                                     const sta::RiseFall *rf,
                                     const sta::Parasitic *parasitic,
                                     const sta::LoadPinIndexMap &load_pin_index_map,
                                     const sta::DcalcAnalysisPt *dcalc_ap) override;
  void finishDrvrPin() override;

  // Elmore wire delay + load slew for a virtual (pin-less) load, given the
  // load's per-vertex Elmore delay. Identical formula to the code previously
  // inlined in LocalSta::annotateLoadDelays. Static: it depends only on the
  // load library thresholds, the Elmore moment and the driver slew.
  static void elmoreWireDelaySlew(const sta::LibertyLibrary *load_lib,
                                  const sta::RiseFall *rf,
                                  float elmore,
                                  sta::Slew drvr_slew,
                                  // Return values.
                                  sta::ArcDelay &wire_delay,
                                  sta::Slew &load_slew);

private:
  // Owned OpenSTA dmp_ceff_elmore calculator that does the real work in Phase A.
  sta::ArcDelayCalc *dmp_;
};

} // namespace lrf

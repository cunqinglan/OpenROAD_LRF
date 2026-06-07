// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sta/ArcDelayCalc.hh"
#include "sta/Delay.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/NetworkClass.hh"
#include "sta/ParasiticsClass.hh"
#include "sta/StaState.hh"
#include "sta/TimingArc.hh"

namespace lrf {

// Phase-A behavior-preserving wrapper around sta::DmpCeffElmoreDelayCalc.
//
// Forwards every ArcDelayCalc virtual to an internally owned DMP instance
// so that wiring this in produces bit-identical numbers vs calling
// sta::ArcDelayCalc directly. Phase B will subclass / sibling-class this
// to swap in the closed-form ElmoreCeff model.
//
// Also hosts elmoreWireDelaySlew() — the plain-Elmore wire-delay/slew
// formula previously inlined in LocalSta::annotateLoadDelays for virtual
// loads. Centralising it here lets Phase B replace both the real-load
// (DMP) and virtual-load (plain Elmore) paths with the same Eq.15 slew.
class LocalDmpDelayCalc : public sta::ArcDelayCalc
{
public:
  explicit LocalDmpDelayCalc(sta::StaState *sta);
  ~LocalDmpDelayCalc() override;

  sta::ArcDelayCalc *copy() override;
  const char *name() const override { return "lrf_dmp_ceff_elmore"; }

  sta::Parasitic *findParasitic(const sta::Pin *drvr_pin,
                                const sta::RiseFall *rf,
                                const sta::DcalcAnalysisPt *dcalc_ap) override;
  bool reduceSupported() const override;
  sta::Parasitic *reduceParasitic(const sta::Parasitic *parasitic_network,
                                  const sta::Pin *drvr_pin,
                                  const sta::RiseFall *rf,
                                  const sta::DcalcAnalysisPt *dcalc_ap) override;
  void reduceParasitic(const sta::Parasitic *parasitic_network,
                       const sta::Net *net,
                       const sta::Corner *corner,
                       const sta::MinMaxAll *min_max) override;

  void setDcalcArgParasiticSlew(sta::ArcDcalcArg &gate,
                                const sta::DcalcAnalysisPt *dcalc_ap) override;
  void setDcalcArgParasiticSlew(sta::ArcDcalcArgSeq &gates,
                                const sta::DcalcAnalysisPt *dcalc_ap) override;

  sta::ArcDcalcResult inputPortDelay(const sta::Pin *port_pin,
                                     float in_slew,
                                     const sta::RiseFall *rf,
                                     const sta::Parasitic *parasitic,
                                     const sta::LoadPinIndexMap &load_pin_index_map,
                                     const sta::DcalcAnalysisPt *dcalc_ap) override;

  sta::ArcDcalcResult gateDelay(const sta::Pin *drvr_pin,
                                const sta::TimingArc *arc,
                                const sta::Slew &in_slew,
                                float load_cap,
                                const sta::Parasitic *parasitic,
                                const sta::LoadPinIndexMap &load_pin_index_map,
                                const sta::DcalcAnalysisPt *dcalc_ap) override;
  void gateDelay(const sta::TimingArc *arc,
                 const sta::Slew &in_slew,
                 float load_cap,
                 const sta::Parasitic *parasitic,
                 float related_out_cap,
                 const sta::Pvt *pvt,
                 const sta::DcalcAnalysisPt *dcalc_ap,
                 sta::ArcDelay &gate_delay,
                 sta::Slew &drvr_slew) override;

  sta::ArcDcalcResultSeq gateDelays(sta::ArcDcalcArgSeq &args,
                                    const sta::LoadPinIndexMap &load_pin_index_map,
                                    const sta::DcalcAnalysisPt *dcalc_ap) override;

  sta::ArcDelay checkDelay(const sta::Pin *check_pin,
                           const sta::TimingArc *arc,
                           const sta::Slew &from_slew,
                           const sta::Slew &to_slew,
                           float related_out_cap,
                           const sta::DcalcAnalysisPt *dcalc_ap) override;

  std::string reportGateDelay(const sta::Pin *drvr_pin,
                              const sta::TimingArc *arc,
                              const sta::Slew &in_slew,
                              float load_cap,
                              const sta::Parasitic *parasitic,
                              const sta::LoadPinIndexMap &load_pin_index_map,
                              const sta::DcalcAnalysisPt *dcalc_ap,
                              int digits) override;
  std::string reportCheckDelay(const sta::Pin *check_pin,
                               const sta::TimingArc *arc,
                               const sta::Slew &from_slew,
                               const char *from_slew_annotation,
                               const sta::Slew &to_slew,
                               float related_out_cap,
                               const sta::DcalcAnalysisPt *dcalc_ap,
                               int digits) override;

  void finishDrvrPin() override;

  // Plain-Elmore wire delay / load slew, relocated from
  // LocalSta::annotateLoadDelays for the virtual-load fallback path.
  // load_lib may be null (uses generic 0.5/0.2/0.8 thresholds + derate=1).
  static void elmoreWireDelaySlew(float elmore,
                                  const sta::Slew &drvr_slew,
                                  const sta::LibertyLibrary *load_lib,
                                  const sta::RiseFall *to_rf,
                                  sta::ArcDelay &wire_delay,
                                  sta::Slew &load_slew);

protected:
  explicit LocalDmpDelayCalc(const LocalDmpDelayCalc &other);

  std::unique_ptr<sta::ArcDelayCalc> inner_;
};

// Cached read of LRF_USE_ELMORECEFF env var (read once at first call).
// True selects the ElmoreCeff dcalc + ElmoreCeff parasitic build path.
// False keeps the existing DMP + PtPiElmore path.
bool useElmoreCeff();

// Cached read of LRF_DCALC_SLEW_FACTOR env var (read once at first call).
// Eq.15 (paper §III-C) factor — √(2π) ≈ 2.507 by default (paper's slow-
// corner heuristic). Shared by LocalElmoreCeffDelayCalc::gateDelay and
// LocalSta::annotateLoadDelays virtual-load branch so they apply the
// same Eq.15 formula to per-load slews.
float dcalcSlewFactor();

// Factory — owns the returned pointer; caller deletes.
// Returns LocalDmpDelayCalc by default; LRF_USE_ELMORECEFF=1 selects
// the (sibling) LocalElmoreCeffDelayCalc. Common base is sta::ArcDelayCalc.
sta::ArcDelayCalc *
makeLocalDelayCalc(sta::StaState *sta);

} // namespace lrf

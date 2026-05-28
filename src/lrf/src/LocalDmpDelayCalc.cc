// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025, The OpenROAD Authors

#include "LocalDmpDelayCalc.hh"

#include <cmath>

#include "dcalc/DmpDelayCalc.hh"   // makeDmpCeffElmoreDelayCalc
#include "sta/ArcDelayCalc.hh"
#include "sta/Liberty.hh"

namespace lrf {

using sta::ArcDcalcResult;
using sta::ArcDelay;
using sta::ArcDelayCalc;
using sta::DcalcAnalysisPt;
using sta::LibertyLibrary;
using sta::LoadPinIndexMap;
using sta::Parasitic;
using sta::Pin;
using sta::RiseFall;
using sta::Slew;
using sta::StaState;
using sta::TimingArc;

LocalDmpDelayCalc::LocalDmpDelayCalc(StaState *sta)
  : sta::LumpedCapDelayCalc(sta),
    // lrf never calls set_delay_calculator, so the configured calculator is
    // always dmp_ceff_elmore; wrap a fresh instance of it.
    dmp_(sta::makeDmpCeffElmoreDelayCalc(sta))
{
}

LocalDmpDelayCalc::~LocalDmpDelayCalc()
{
  delete dmp_;
}

ArcDelayCalc *
LocalDmpDelayCalc::copy()
{
  return new LocalDmpDelayCalc(this);
}

void
LocalDmpDelayCalc::copyState(const StaState *sta)
{
  sta::LumpedCapDelayCalc::copyState(sta);
  dmp_->copyState(sta);
}

ArcDcalcResult
LocalDmpDelayCalc::gateDelay(const Pin *drvr_pin,
                             const TimingArc *arc,
                             const Slew &in_slew,
                             float load_cap,
                             const Parasitic *parasitic,
                             const LoadPinIndexMap &load_pin_index_map,
                             const DcalcAnalysisPt *dcalc_ap)
{
  return dmp_->gateDelay(drvr_pin, arc, in_slew, load_cap, parasitic,
                         load_pin_index_map, dcalc_ap);
}

ArcDcalcResult
LocalDmpDelayCalc::inputPortDelay(const Pin *port_pin,
                                  float in_slew,
                                  const RiseFall *rf,
                                  const Parasitic *parasitic,
                                  const LoadPinIndexMap &load_pin_index_map,
                                  const DcalcAnalysisPt *dcalc_ap)
{
  return dmp_->inputPortDelay(port_pin, in_slew, rf, parasitic,
                              load_pin_index_map, dcalc_ap);
}

void
LocalDmpDelayCalc::finishDrvrPin()
{
  dmp_->finishDrvrPin();
}

void
LocalDmpDelayCalc::elmoreWireDelaySlew(const LibertyLibrary *load_lib,
                                       const RiseFall *rf,
                                       float elmore,
                                       Slew drvr_slew,
                                       // Return values.
                                       ArcDelay &wire_delay,
                                       Slew &load_slew)
{
  float vth = 0.5f, vl = 0.2f, vh = 0.8f, slew_derate = 1.0f;
  if (load_lib) {
    vth = load_lib->inputThreshold(rf);
    vl = load_lib->slewLowerThreshold(rf);
    vh = load_lib->slewUpperThreshold(rf);
    slew_derate = load_lib->slewDerateFromLibrary();
  }
  // Same formula as DmpCeff::dspfWireDelaySlew (previously inlined in
  // LocalSta::annotateLoadDelays for virtual loads).
  wire_delay = -elmore * std::log(1.0 - vth);
  load_slew = drvr_slew + elmore * std::log((1.0 - vl) / (1.0 - vh)) / slew_derate;
}

} // namespace lrf

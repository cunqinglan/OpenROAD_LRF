// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "LocalElmoreCeffDelayCalc.hh"

#include <atomic>
#include <chrono>
#include <cmath>  // isnan
#include <cstdio>
#include <cstdlib>
#include <mutex>

#include "LocalDmpDelayCalc.hh"  // elmoreWireDelaySlew helper
#include "PtElmoreCeff.hh"
#include "dcalc/LumpedCapDelayCalc.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"
#include "sta/Delay.hh"
#include "sta/Liberty.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingModel.hh"
#include "sta/Units.hh"
#include "sta/Variables.hh"

namespace lrf {

using sta::ArcDcalcResult;
using sta::ArcDelay;
using sta::ArcDelayCalc;
using sta::DcalcAnalysisPt;
using sta::delayAsFloat;
using sta::delay_zero;
using sta::GateTimingModel;
using sta::LibertyLibrary;
using sta::LoadPinIndexMap;
using sta::Parasitic;
using sta::Pin;
using sta::RiseFall;
using sta::Slew;
using sta::StaState;
using sta::TimingArc;

////////////////////////////////////////////////////////////////
// Performance instrumentation (mirrors LocalDmpDelayCalc).
// Gated by LRF_DCALC_STATS=1.
////////////////////////////////////////////////////////////////
namespace {
std::atomic<uint64_t> g_ec_gate_calls{0};
std::atomic<uint64_t> g_ec_gate_ns{0};

void printEcStats()
{
  const char *s = std::getenv("LRF_DCALC_STATS");
  if (!s || s[0] != '1') return;
  const uint64_t calls = g_ec_gate_calls.load(std::memory_order_relaxed);
  const uint64_t ns    = g_ec_gate_ns.load(std::memory_order_relaxed);
  const double  ms    = ns / 1.0e6;
  const double  us_pc = calls ? (ns / 1.0e3 / calls) : 0.0;
  std::fprintf(stderr,
      "[lrf_dcalc_stats] LocalElmoreCeffDelayCalc gateDelay  "
      "calls=%llu  wall_ms=%.3f  us_per_call=%.3f\n",
      static_cast<unsigned long long>(calls), ms, us_pc);
}

void registerEcAtExitOnce()
{
  static std::once_flag once;
  std::call_once(once, []() { std::atexit(&printEcStats); });
}
} // namespace

LocalElmoreCeffDelayCalc::LocalElmoreCeffDelayCalc(StaState *sta) :
  sta::LumpedCapDelayCalc(sta)
{
  registerEcAtExitOnce();
}

LocalElmoreCeffDelayCalc::LocalElmoreCeffDelayCalc(
    const LocalElmoreCeffDelayCalc &other) :
  sta::LumpedCapDelayCalc(other)
{
  registerEcAtExitOnce();
}

ArcDelayCalc *
LocalElmoreCeffDelayCalc::copy()
{
  return new LocalElmoreCeffDelayCalc(*this);
}

ArcDcalcResult
LocalElmoreCeffDelayCalc::gateDelay(const Pin *drvr_pin,
                                    const TimingArc *arc,
                                    const Slew &in_slew,
                                    float load_cap,
                                    const Parasitic *parasitic,
                                    const LoadPinIndexMap &load_pin_index_map,
                                    const DcalcAnalysisPt *dcalc_ap)
{
  const auto _t0 = std::chrono::steady_clock::now();
  struct ScopedTimer {
    std::chrono::steady_clock::time_point t0;
    ~ScopedTimer() {
      const auto t1 = std::chrono::steady_clock::now();
      g_ec_gate_calls.fetch_add(1, std::memory_order_relaxed);
      g_ec_gate_ns.fetch_add(
          std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count(),
          std::memory_order_relaxed);
    }
  } _timer{_t0};

  const RiseFall *rf = arc->toEdge()->asRiseFall();
  const LibertyLibrary *drvr_library = arc->to()->libertyLibrary();
  GateTimingModel *model = arc->gateModel(dcalc_ap);

  // Step 1: Algorithm 2 (paper §III-B Eq.11) on the cached RC tree.
  // sta::Parasitic is a non-polymorphic empty base; polymorphism lives on
  // sta::ConcreteParasitic. Cast through it before checking PtElmoreCeff.
  PtElmoreCeff *pt_ec = nullptr;
  if (parasitic) {
    const sta::ConcreteParasitic *cp =
        static_cast<const sta::ConcreteParasitic *>(parasitic);
    pt_ec = const_cast<PtElmoreCeff *>(dynamic_cast<const PtElmoreCeff *>(cp));
  }

  // Ts seed: paper §IV-A step 1 uses NLDM(C_total) to get drvr_slew_initial,
  // then Ts = drvr_slew/ramp_factor. B1.3 simplifies — use the input slew
  // directly as the ramp duration source (it's the only slew available at
  // gateDelay-time without an extra NLDM lookup). B2 will switch to the
  // proper per-node Eq.15 slew chain so each branch gets its own Ts.
  constexpr float kRampFactor = 0.8f;
  const float Ts = delayAsFloat(in_slew) / kRampFactor;
  const float ceff = pt_ec
      ? pt_ec->computeCeffAlgo2(Ts)
      : load_cap;

  // Step 2: one direct NLDM table lookup with Ceff. No iteration.
  ArcDcalcResult result(load_pin_index_map.size());
  if (model) {
    if (std::isnan(ceff) || std::isnan(delayAsFloat(in_slew)))
      report_->error(1351, "lrf::LocalElmoreCeffDelayCalc: NaN in gate-delay input");
    ArcDelay gate_delay;
    Slew drvr_slew;
    model->gateDelay(pinPvt(drvr_pin, dcalc_ap), delayAsFloat(in_slew), ceff,
                     variables_->pocvEnabled(),
                     gate_delay, drvr_slew);
    result.setGateDelay(gate_delay);
    result.setDrvrSlew(drvr_slew);
  }
  else {
    result.setGateDelay(delay_zero);
    result.setDrvrSlew(delay_zero);
  }

  // Step 3: per-load wire delay / load slew via plain Elmore (Phase B1).
  // B2 will replace this with Eq.15 refined slew.
  const Slew drvr_slew_final = result.drvrSlew();
  for (const auto [load_pin, load_idx] : load_pin_index_map) {
    ArcDelay wire_delay = 0.0;
    Slew load_slew = drvr_slew_final;
    if (pt_ec) {
      float elmore = 0.0f;
      bool exists = false;
      pt_ec->findElmore(load_pin, elmore, exists);
      if (exists && elmore > 0.0f) {
        LocalDmpDelayCalc::elmoreWireDelaySlew(
            elmore, drvr_slew_final, drvr_library, rf, wire_delay, load_slew);
      }
    }
    // thresholdAdjust handles input-pin threshold differences between
    // driver and load libraries; inherited from DelayCalcBase.
    thresholdAdjust(load_pin, drvr_library, rf, wire_delay, load_slew);
    result.setWireDelay(load_idx, wire_delay);
    result.setLoadSlew(load_idx, load_slew);
  }
  return result;
}

ArcDelayCalc *
makeLocalElmoreCeffDelayCalc(StaState *sta)
{
  return new LocalElmoreCeffDelayCalc(sta);
}

} // namespace lrf

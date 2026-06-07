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
                                    const sta::Scene *scene, const sta::MinMax *min_max)
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
  GateTimingModel *model = arc->gateModel(scene, min_max);
  const float in_slew_f = delayAsFloat(in_slew);
  ArcDcalcResult result(load_pin_index_map.size());

  // sta::Parasitic is a non-polymorphic empty base; polymorphism lives on
  // sta::ConcreteParasitic. Cast through it before checking PtElmoreCeff.
  PtElmoreCeff *pt_ec = nullptr;
  if (parasitic) {
    const sta::ConcreteParasitic *cp =
        static_cast<const sta::ConcreteParasitic *>(parasitic);
    pt_ec = const_cast<PtElmoreCeff *>(dynamic_cast<const PtElmoreCeff *>(cp));
  }

  // Fast path: no PtElmoreCeff parasitic → single NLDM with the caller's
  // load_cap. Mirrors LumpedCapDelayCalc behaviour, no Ceff math.
  if (!pt_ec) {
    if (model) {
      float gate_delay = 0.0f;
      float drvr_slew = 0.0f;
      model->gateDelay(pinPvt(drvr_pin, scene, min_max), in_slew_f, load_cap,
                       gate_delay, drvr_slew);
      result.setGateDelay(gate_delay);
      result.setDrvrSlew(drvr_slew);
    } else {
      result.setGateDelay(delay_zero);
      result.setDrvrSlew(delay_zero);
    }
    for (const auto [load_pin, load_idx] : load_pin_index_map) {
      double wire_delay = 0.0;
      double load_slew = delayAsFloat(result.drvrSlew());
      thresholdAdjust(load_pin, drvr_library, rf, wire_delay, load_slew);
      result.setWireDelay(load_idx, wire_delay);
      result.setLoadSlew(load_idx, load_slew);
    }
    return result;
  }

  if (!model) {
    result.setGateDelay(delay_zero);
    result.setDrvrSlew(delay_zero);
    return result;
  }

  if (std::isnan(in_slew_f))
    report_->error(1351, "lrf::LocalElmoreCeffDelayCalc: NaN in_slew");

  // --- Paper §IV-A 5-phase pipeline (B2 full implementation) ---

  // Phase 1: NLDM(in_slew, C_total) → slew_i₀ seed.
  // This is the only slew available before we know Ceff. Used to drive
  // Eq.15 inside Algorithm 2 (paper accepts the single-refinement gap
  // between this seed slew and the post-Ceff final slew).
  float gd_seed = 0.0f;
  float    ds_seed = 0.0f;
  model->gateDelay(pinPvt(drvr_pin, scene, min_max), in_slew_f, pt_ec->totalCap(),
                   gd_seed, ds_seed);
  const float slew_i_seed = ds_seed;

  // Phase 2: Algorithm 2 (Eq.11) with per-branch T_n derived from Eq.15
  // (impulse_sq cached on PtRcNode by precomputeMoments at reduce time).
  const float slew_factor = dcalcSlewFactor();
  const float ceff = pt_ec->computeCeffAlgo2WithRefinedSlew(
      slew_i_seed, slew_factor);

  // Phase 3: NLDM(in_slew, Ceff) → published gate_delay + final drvr_slew.
  float gate_delay = 0.0f;
  float    drvr_slew = 0.0f;
  model->gateDelay(pinPvt(drvr_pin, scene, min_max), in_slew_f, ceff,
                   gate_delay, drvr_slew);
  result.setGateDelay(gate_delay);
  result.setDrvrSlew(drvr_slew);

  // Phase 4: per-load annotation.
  //   wire_delay[n] = delay[n]                                    (Algo.1)
  //   load_slew[n]  = sqrt(drvr_slew² + factor² · impulse_sq[n])  (Eq.15
  //                  with FINAL drvr_slew — paper §IV-A step 5)
  // Falls back to plain Elmore for loads not mapped into tree_ (shouldn't
  // happen with a well-formed PtElmoreCeff but defensive).
  const float drvr_slew_f = drvr_slew;
  const float drvr_slew_sq = drvr_slew_f * drvr_slew_f;
  const float factor_sq = slew_factor * slew_factor;

  for (const auto [load_pin, load_idx] : load_pin_index_map) {
    ArcDelay wire_delay = 0.0;
    Slew load_slew = drvr_slew;

    if (const PtRcLoad *load = pt_ec->findLoadByPin(load_pin)) {
      if (load->tree_node_idx != kInvalidTreeNodeIdx) {
        const PtRcNode &n = pt_ec->tree()[load->tree_node_idx];
        wire_delay = n.delay;
        load_slew = std::sqrt(drvr_slew_sq + factor_sq * n.impulse_sq);
      } else if (load->elmore > 0.0f) {
        LocalDmpDelayCalc::elmoreWireDelaySlew(
            load->elmore, drvr_slew, drvr_library, rf, wire_delay, load_slew);
      }
    }
    double wd_adj = delayAsFloat(wire_delay);
    double ls_adj = delayAsFloat(load_slew);
    thresholdAdjust(load_pin, drvr_library, rf, wd_adj, ls_adj);
    result.setWireDelay(load_idx, wd_adj);
    result.setLoadSlew(load_idx, ls_adj);
  }
  return result;
}

ArcDelayCalc *
makeLocalElmoreCeffDelayCalc(StaState *sta)
{
  return new LocalElmoreCeffDelayCalc(sta);
}

} // namespace lrf

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "LocalDmpDelayCalc.hh"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mutex>

#include "LocalElmoreCeffDelayCalc.hh"
#include "dcalc/DmpDelayCalc.hh"
#include "sta/Liberty.hh"

namespace lrf {

using sta::ArcDcalcArg;
using sta::ArcDcalcArgSeq;
using sta::ArcDcalcResult;
using sta::ArcDcalcResultSeq;
using sta::ArcDelay;
using sta::ArcDelayCalc;
using sta::Scene;
using sta::LibertyLibrary;
using sta::LoadPinIndexMap;
using sta::MinMaxAll;
using sta::Net;
using sta::Parasitic;
using sta::Pin;
using sta::Pvt;
using sta::RiseFall;
using sta::Slew;
using sta::StaState;
using sta::TimingArc;

////////////////////////////////////////////////////////////////
// Performance instrumentation
//
// Static counters accumulate gateDelay call count + cumulative wall-time
// across ALL LocalDmpDelayCalc instances (LR's ParallelVisitor spawns one
// per worker thread). Gated by LRF_DCALC_STATS=1 to keep production
// silent; the atexit handler runs once at process exit (registered on
// the first construction via call_once).
////////////////////////////////////////////////////////////////
namespace {
std::atomic<uint64_t> g_dmp_gate_calls{0};
std::atomic<uint64_t> g_dmp_gate_ns{0};

void printDmpStats()
{
  const char *s = std::getenv("LRF_DCALC_STATS");
  if (!s || s[0] != '1') return;
  const uint64_t calls = g_dmp_gate_calls.load(std::memory_order_relaxed);
  const uint64_t ns    = g_dmp_gate_ns.load(std::memory_order_relaxed);
  const double  ms    = ns / 1.0e6;
  const double  us_pc = calls ? (ns / 1.0e3 / calls) : 0.0;
  std::fprintf(stderr,
      "[lrf_dcalc_stats] LocalDmpDelayCalc       gateDelay  "
      "calls=%llu  wall_ms=%.3f  us_per_call=%.3f\n",
      static_cast<unsigned long long>(calls), ms, us_pc);
}

void registerDmpAtExitOnce()
{
  static std::once_flag once;
  std::call_once(once, []() { std::atexit(&printDmpStats); });
}
} // namespace

LocalDmpDelayCalc::LocalDmpDelayCalc(StaState *sta) :
  ArcDelayCalc(sta),
  inner_(sta::makeDmpCeffElmoreDelayCalc(sta))
{
  registerDmpAtExitOnce();
}

LocalDmpDelayCalc::LocalDmpDelayCalc(const LocalDmpDelayCalc &other) :
  ArcDelayCalc(other),
  inner_(other.inner_->copy())
{
  registerDmpAtExitOnce();
}

LocalDmpDelayCalc::~LocalDmpDelayCalc() = default;

ArcDelayCalc *
LocalDmpDelayCalc::copy()
{
  return new LocalDmpDelayCalc(*this);
}

Parasitic *
LocalDmpDelayCalc::findParasitic(const Pin *drvr_pin,
                                 const RiseFall *rf,
                                 const sta::Scene *scene, const sta::MinMax *min_max)
{
  return inner_->findParasitic(drvr_pin, rf, scene, min_max);
}

bool
LocalDmpDelayCalc::reduceSupported() const
{
  return inner_->reduceSupported();
}

Parasitic *
LocalDmpDelayCalc::reduceParasitic(const Parasitic *parasitic_network,
                                   const Pin *drvr_pin,
                                   const RiseFall *rf,
                                   const sta::Scene *scene, const sta::MinMax *min_max)
{
  return inner_->reduceParasitic(parasitic_network, drvr_pin, rf, scene, min_max);
}

void
LocalDmpDelayCalc::reduceParasitic(const Parasitic *parasitic_network,
                                   const Net *net,
                                   const Scene *corner,
                                   const MinMaxAll *min_max)
{
  inner_->reduceParasitic(parasitic_network, net, corner, min_max);
}

void
LocalDmpDelayCalc::setDcalcArgParasiticSlew(ArcDcalcArg &gate,
                                            const sta::Scene *scene, const sta::MinMax *min_max)
{
  inner_->setDcalcArgParasiticSlew(gate, scene, min_max);
}

void
LocalDmpDelayCalc::setDcalcArgParasiticSlew(ArcDcalcArgSeq &gates,
                                            const sta::Scene *scene, const sta::MinMax *min_max)
{
  inner_->setDcalcArgParasiticSlew(gates, scene, min_max);
}

ArcDcalcResult
LocalDmpDelayCalc::inputPortDelay(const Pin *port_pin,
                                  float in_slew,
                                  const RiseFall *rf,
                                  const Parasitic *parasitic,
                                  const LoadPinIndexMap &load_pin_index_map,
                                  const sta::Scene *scene, const sta::MinMax *min_max)
{
  return inner_->inputPortDelay(port_pin, in_slew, rf, parasitic,
                                load_pin_index_map, scene, min_max);
}

ArcDcalcResult
LocalDmpDelayCalc::gateDelay(const Pin *drvr_pin,
                             const TimingArc *arc,
                             const Slew &in_slew,
                             float load_cap,
                             const Parasitic *parasitic,
                             const LoadPinIndexMap &load_pin_index_map,
                             const sta::Scene *scene, const sta::MinMax *min_max)
{
  const auto t0 = std::chrono::steady_clock::now();
  ArcDcalcResult r = inner_->gateDelay(drvr_pin, arc, in_slew, load_cap,
                                       parasitic, load_pin_index_map, scene, min_max);
  const auto t1 = std::chrono::steady_clock::now();
  g_dmp_gate_calls.fetch_add(1, std::memory_order_relaxed);
  g_dmp_gate_ns.fetch_add(
      std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count(),
      std::memory_order_relaxed);
  return r;
}

void
LocalDmpDelayCalc::gateDelay(const TimingArc *arc,
                             const Slew &in_slew,
                             float load_cap,
                             const Parasitic *parasitic,
                             float related_out_cap,
                             const Pvt *pvt,
                             const sta::Scene *scene, const sta::MinMax *min_max,
                             ArcDelay &gate_delay,
                             Slew &drvr_slew)
{
  inner_->gateDelay(arc, in_slew, load_cap, parasitic, related_out_cap,
                    pvt, scene, min_max, gate_delay, drvr_slew);
}

ArcDcalcResultSeq
LocalDmpDelayCalc::gateDelays(ArcDcalcArgSeq &args,
                              const LoadPinIndexMap &load_pin_index_map,
                              const sta::Scene *scene, const sta::MinMax *min_max)
{
  return inner_->gateDelays(args, load_pin_index_map, scene, min_max);
}

ArcDelay
LocalDmpDelayCalc::checkDelay(const Pin *check_pin,
                              const TimingArc *arc,
                              const Slew &from_slew,
                              const Slew &to_slew,
                              float related_out_cap,
                              const sta::Scene *scene, const sta::MinMax *min_max)
{
  return inner_->checkDelay(check_pin, arc, from_slew, to_slew,
                            related_out_cap, scene, min_max);
}

std::string
LocalDmpDelayCalc::reportGateDelay(const Pin *drvr_pin,
                                   const TimingArc *arc,
                                   const Slew &in_slew,
                                   float load_cap,
                                   const Parasitic *parasitic,
                                   const LoadPinIndexMap &load_pin_index_map,
                                   const sta::Scene *scene, const sta::MinMax *min_max,
                                   int digits)
{
  return inner_->reportGateDelay(drvr_pin, arc, in_slew, load_cap, parasitic,
                                 load_pin_index_map, scene, min_max, digits);
}

std::string
LocalDmpDelayCalc::reportCheckDelay(const Pin *check_pin,
                                    const TimingArc *arc,
                                    const Slew &from_slew,
                                    std::string_view from_slew_annotation,
                                    const Slew &to_slew,
                                    float related_out_cap,
                                    const sta::Scene *scene, const sta::MinMax *min_max,
                                    int digits)
{
  return inner_->reportCheckDelay(check_pin, arc, from_slew, from_slew_annotation,
                                  to_slew, related_out_cap, scene, min_max, digits);
}

void
LocalDmpDelayCalc::finishDrvrPin()
{
  inner_->finishDrvrPin();
}

void
LocalDmpDelayCalc::elmoreWireDelaySlew(float elmore,
                                       const Slew &drvr_slew,
                                       const LibertyLibrary *load_lib,
                                       const RiseFall *to_rf,
                                       ArcDelay &wire_delay,
                                       Slew &load_slew)
{
  float vth = 0.5f, vl = 0.2f, vh = 0.8f, slew_derate = 1.0f;
  if (load_lib) {
    vth = load_lib->inputThreshold(to_rf);
    vl = load_lib->slewLowerThreshold(to_rf);
    vh = load_lib->slewUpperThreshold(to_rf);
    slew_derate = load_lib->slewDerateFromLibrary();
  }
  wire_delay = -elmore * std::log(1.0 - vth);
  load_slew = drvr_slew + elmore * std::log((1.0 - vl) / (1.0 - vh)) / slew_derate;
}

bool
useElmoreCeff()
{
  // Cached read: env var is parsed on first call. Use a static local so
  // it's thread-safe (C++11 guarantees one-time init) and consistent
  // across the run.
  static const bool flag = []() {
    const char *e = std::getenv("LRF_USE_ELMORECEFF");
    return e && e[0] == '1';
  }();
  return flag;
}

float
dcalcSlewFactor()
{
  // Cached read. Default = √(2π) ≈ 2.5066 (paper §III-C slow-corner
  // heuristic). LRF_DCALC_SLEW_FACTOR=<float> overrides (e.g. 1.0 for
  // fast-corner sensitivity tests). Non-positive overrides ignored.
  static const float factor = []() {
    const char *e = std::getenv("LRF_DCALC_SLEW_FACTOR");
    if (!e) return 2.5066282746310002f;
    const float v = std::strtof(e, nullptr);
    return (v > 0.0f) ? v : 2.5066282746310002f;
  }();
  return factor;
}

sta::ArcDelayCalc *
makeLocalDelayCalc(StaState *sta)
{
  // Opt-in switch (design doc §4 "switch to fall back to LocalDmpDelayCalc
  // for A/B comparison and safety"). Defaults to DMP for behavior parity.
  if (useElmoreCeff())
    return makeLocalElmoreCeffDelayCalc(sta);
  return new LocalDmpDelayCalc(sta);
}

} // namespace lrf

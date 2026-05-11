
#include "sta/SearchPred.hh"
#include "sta/Scene.hh"
#include "sta/TimingRole.hh"
#include "sta/Clock.hh"
#include "sta/Sdc.hh"
#include "sta/PathExpanded.hh"
#include "LrHelper.hh"
#include "lrf/LrfClass.hh"
#include "sta/Scene.hh"
#include "sta/Network.hh"


namespace lrf {
///////////////////////////////////////////////////////////////////
// RapidLrHelper, idea is from Rapid Gate Sizing with Fewer 
// Iterations of Lagrangian Relaxation by Ankur
///////////////////////////////////////////////////////////////////

RapidLrHelper::RapidLrHelper(sta::dbSta *sta) : LRHelper(sta) 
{
  setTimingMode();
}

float
RapidLrHelper::getMultiplier(Slack arc_slack) {
  // Only consider the first clock now
  float clock_period = 0.0f;
  for (Clock *clock : *sdc_->clocks()) {
    float period = clock->period();
    if (period > clock_period) {
      clock_period = period;
      break;
    }
  }
  if (clock_period == 0) {
    printf("RapidLrHelper::getMultiplier: ERROR: found zero clock period\n");
    fflush(stdout);
    throw std::runtime_error("RapidLrHelper::updateArcLms: found zero clock period");
  }
  // Shift both the effective period and the slack by timing_margin_ ratio.
  //   T_eff     = T * (1 + m)
  //   slack_eff = arc_slack + T * m
  // This preserves the identity  T_eff - slack_eff == T - arc_slack
  // (= real path delay), so the scaling base is
  //   (T_eff - slack_eff) / T_eff = path_delay / T_eff
  // Semantics by sign of timing_margin_:
  //   m < 0  : T_eff shrinks  → scaling grows → higher LM on every arc
  //            (treat clock as tighter; adds headroom that survives
  //            post-GR RC degradation — recommended for GRT robustness).
  //   m > 0  : T_eff grows    → scaling shrinks → lower LM on every arc
  //            (treat clock as looser; gentler optimizer pressure).
  // k branch uses slack_eff so paths that are safe under real T but
  // critical under T_eff get the critical exponent.
  // Guard: slack_eff >= T_eff  ⇔  arc_slack >= T, so a path whose delay
  // already uses up the full nominal period returns 0.
  float T_eff = clock_period * (1.0f + timing_margin_);
  float slack_eff = arc_slack + timing_margin_ * clock_period;
  int k = slack_eff < 0.0 ? critical_arc_k_ : non_critical_arc_k_;
  if (slack_eff >= T_eff) {
    return 0.0f;
  }
  float scaling_factor = std::pow((T_eff - slack_eff) / T_eff, k);
  return scaling_factor;
}

void 
RapidLrHelper::updateEndPointArcLms(Edge *edge, TimingArc *arc, Sta *sta, 
                                    DcalcAnalysisPt const *dcalc_ap) 
{
  const size_t ap_index = dcalc_ap->index();
  const RiseFall *from_rf = arc->fromEdge()->asRiseFall();
  const RiseFall *to_rf = arc->toEdge()->asRiseFall();
  const MinMax *delay_minmax = dcalc_ap->delayMinMax();
  Delay delay = sta->arcDelay(edge, arc, dcalc_ap);
  size_t lm_idx = arc->index() * graph_->apCount() + ap_index;
  Vertex *from_vertex = edge->from(graph_);
  Vertex *to_vertex = edge->to(graph_);
  Arrival from_aat = sta->pinArrival(from_vertex->pin(), from_rf, delay_minmax);
  Required to_rat = sta->vertexRequired(to_vertex, to_rf, delay_minmax);
  LMValue *lms = edge->arcLms();

  // Disabled edge: unconstrained timing values.
  // max: to_rat == +INF (no setup RAT), from_aat == -INF (no arrival)
  // min: to_rat == -INF (no hold RAT),  from_aat == +INF (no min arrival)
  if (to_rat == INF || to_rat == -INF || from_aat == -INF || from_aat == INF) {
    lms[lm_idx] = 0.0;
    return;
  }

  from_aat = std::max(from_aat, 0.0f);
  to_rat = std::max(to_rat, 0.0f);
  Slack arc_slack = to_rat - (from_aat + delay);
  if (delay_minmax == MinMax::min()) {
    arc_slack = (from_aat + delay) - to_rat;
  }

  // Here we use path_delay / clock_period as the scaling factor
  float multiplier = getMultiplier(arc_slack);
  if (multiplier < 0.0) {
    printf("RapidLrHelper::updateEndPointArcLms: ERROR: edge %s AP corner %s delay min/max %s: computed non-positive multiplier %.6f with aat %.6f, rat %.6f, delay %.6f\n",
            edge->to_string(graph_).c_str(),
            dcalc_ap->corner()->name(),
            delay_minmax->to_string().c_str(),
            multiplier,
            from_aat * 1.0e12, to_rat * 1.0e12, delay * 1.0e12);
    fflush(stdout);
    throw std::runtime_error("RapidLrHelper::updateEndPointArcLms: computed non-positive multiplier");
  }

  lms[lm_idx] = lms[lm_idx] * multiplier;
  static constexpr LMValue LM_FLOOR = 1e-16;
  if (lms[lm_idx] < LM_FLOOR) {
    lms[lm_idx] = LM_FLOOR;
  }
}

void 
RapidLrHelper::updateArcLms(Edge *edge, TimingArc *arc, Sta *sta, 
                            DcalcAnalysisPt const *dcalc_ap) 
{
  size_t ap_index = dcalc_ap->index();
  size_t lm_idx = lmIndex(arc, ap_index, graph_->apCount());
  sta::Vertex *from_vertex = edge->from(graph_);
  sta::Vertex *to_vertex = edge->to(graph_);
  sta::RiseFall const *from_rf = arc->fromEdge()->asRiseFall();
  sta::RiseFall  const *to_rf = arc->toEdge()->asRiseFall();
  sta::MinMax const *delay_minmax = dcalc_ap->delayMinMax();
  sta::Arrival from_aat = sta->pinArrival(from_vertex->pin(), from_rf, delay_minmax);
  sta::Required to_rat = sta->vertexRequired(to_vertex, to_rf, delay_minmax);
  sta::Delay delay = sta->arcDelay(edge, arc, dcalc_ap);
  LMValue *lms = edge->arcLms();

  // Disabled edge: unconstrained timing values.
  if (to_rat == INF || to_rat == -INF || from_aat == -INF || from_aat == INF) {
    lms[lm_idx] = 0.0;
    return;
  }
  
  from_aat = std::max(from_aat, 0.0f);
  to_rat = std::max(to_rat, 0.0f);
  Slack arc_slack = to_rat - (from_aat + delay);
  if (delay_minmax == MinMax::min()) {
    arc_slack = (from_aat + delay) - to_rat;
  }

  // Only consider the first clock now
  float multiplier = getMultiplier(arc_slack);
  lms[lm_idx] = lms[lm_idx] * multiplier;
  // Floor: prevent LM from being irreversibly zeroed out.
  // Once LM=0, multiplicative updates (lm*=x) can never recover it.
  // A small floor keeps non-disabled arcs recoverable if paths shift.
  static constexpr LMValue LM_FLOOR = 1e-16;
  if (lms[lm_idx] < LM_FLOOR) {
    lms[lm_idx] = LM_FLOOR;
  }
}

bool
RapidLrHelper::updateCriticalPathLms(sta::Path *path_end)
{
  if (path_end->slack(sta_) >= 0.0f) {
    return true;
  }
  sta::PathExpanded expended_path(path_end, sta_);
  int path_length = expended_path.size();
  if (path_length <= 1) {
    return true;
  }
  const int start_index = expended_path.startIndex();
  const sta::DcalcAnalysisPt *dcalc_ap = path_end->dcalcAnalysisPt(sta_);
  const int lib_ap = dcalc_ap->libertyIndex();
  float min_delta_lm = 0.0f;
  
  // First pass: calculate min_delta_lm
  for (int i = start_index; i < path_length; ++i) {
    const sta::Path *path = expended_path.path(i);
    const sta::Pin *path_pin = path->pin(sta_);
    if (i > 0 && !network_->isTopLevelPort(path_pin)) {
      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      // Here we should consider if we should skip regs and
      // other special paths
      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      sta::Edge *edge = path->prevEdge(sta_);
      if (!edge) {
        printf("RapidLrHelper::updateCriticalPathLms: path at vertex %s has no prevEdge\n", 
              path->vertex(sta_)->to_string(graph_).c_str());
        fflush(stdout);
        continue;
      }
      sta::TimingArc *arc = path->prevArc(sta_);
      if (!arc) {
        printf("RapidLrHelper::updateCriticalPathLms: path at index %d has no prevArc\n", i);
        fflush(stdout);
        continue;
      }
      LMValue *lms = edge->arcLms();
      if (!lms) {
        printf("RapidLrHelper::updateCriticalPathLms: edge %s has no arcLms\n",
              edge->to_string(graph_).c_str());
        fflush(stdout);
        continue;
      }
      size_t lm_idx = lmIndex(arc, dcalc_ap->index(), graph_->apCount());
      Slack path_slack = path->slack(sta_);
      float multiplier = getMultiplier(path_slack);
      if (multiplier == 0.0f) {
        throw std::runtime_error("RapidLrHelper::updateCriticalPathLms: accounted a invalid path");
      }
      float delta_lm = lms[lm_idx] * (multiplier - 1.0f);
      if (delta_lm < min_delta_lm) 
        min_delta_lm = delta_lm;
    }
  }
  
  // Second pass: apply min_delta_lm
  for (int i = start_index; i < path_length; i++) {
    const sta::Path *path = expended_path.path(i);
    const sta::Pin *path_pin = path->pin(sta_);
    if (i > 0 && !network_->isTopLevelPort(path_pin)) {
      sta::Edge *edge = path->prevEdge(sta_);
      const sta::TimingArc *arc = path->prevArc(sta_);
      if (!edge || !arc) {
        continue;
      }
      LMValue *lms = edge->arcLms();
      if (!lms) {
        continue;
      }
      size_t lm_idx = lmIndex(arc, dcalc_ap->index(), graph_->apCount());
      lms[lm_idx] += min_delta_lm;
      static constexpr LMValue LM_FLOOR = 1e-16;
      if (lms[lm_idx] < LM_FLOOR) {
        lms[lm_idx] = LM_FLOOR;
      }
    }
  }
  return true;
}

void
RapidLrHelper::setMode(std::string mode) {
  if (mode == "power") {
    setPowerMode();
  } else if (mode == "timing") {
    setTimingMode();
  } else {
    printf("RapidLrHelper::setMode: unknown mode %s, defaulting to timing mode\n", mode.c_str());
    fflush(stdout);
    setTimingMode();
  }
}

void
RapidLrHelper::setPowerMode() {
  critical_arc_k_ = 1;
  non_critical_arc_k_ = 6;
  power_mode_ = true;
}

void
RapidLrHelper::setTimingMode() {
  critical_arc_k_ = 4;
  non_critical_arc_k_ = 1;
  power_mode_ = false;
}

} // namespace lrf
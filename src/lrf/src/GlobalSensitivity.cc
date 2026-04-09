#include "GlobalSensitivity.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sta/Sta.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/ArcDelayCalc.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "sta/Network.hh"
#include "sta/Liberty.hh"
#include "DcalcAnalysisPt.hh"
#include "Corner.hh"
#include "search/Levelize.hh"

namespace lrf {

using sta::ArcDelay;
using sta::DcalcAPIndex;
using sta::Edge;
using sta::EdgeId;
using sta::Graph;
using sta::Level;
using sta::MinMax;
using sta::RiseFall;
using sta::Slew;
using sta::TimingArc;
using sta::Vertex;
using sta::VertexId;

GlobalSensitivity::GlobalSensitivity(sta::Sta *sta)
    : sta_(sta)
{
}

void
GlobalSensitivity::compute(sta::ArcDelayCalc *arc_delay_calc,
                           const sta::DcalcAnalysisPt *dcalc_ap)
{
  ready_ = false;
  if (!arc_delay_calc || !dcalc_ap)
    return;

  Graph *graph = sta_->graph();
  if (!graph)
    return;

  ap_index_ = dcalc_ap->index();
  ap_count_ = sta_->corners()->dcalcAnalysisPtCount();

  sta_->ensureLevelized();
  int max_level = sta_->levelize()->maxLevel();

  VertexId vertex_count = graph->vertexCount();
  vertex_dominant_phi_.assign(vertex_count, 0.0f);
  drain_phi_sum_.assign(vertex_count, 0.0f);
  edge_sens_.clear();

  sta::GraphDelayCalc *gdc = sta_->graphDelayCalc();

  // ---- Phase 1: Local partial derivatives for all edges ----
  sta::VertexIterator vert_iter(graph);
  while (vert_iter.hasNext()) {
    Vertex *vertex = vert_iter.next();
    sta::VertexInEdgeIterator in_iter(vertex, graph);
    while (in_iter.hasNext()) {
      Edge *edge = in_iter.next();

      if (edge->isDisabledConstraint() || edge->isDisabledCond()
          || edge->isDisabledLoop())
        continue;

      sta::TimingArcSet *arc_set = edge->timingArcSet();
      if (!arc_set)
        continue;

      // Wire edges: slew passes through, no delay sensitivity to slew
      if (edge->isWire()) {
        EdgeSensitivity &es = getOrCreateEdgeSensitivity(edge);
        size_t arc_count = 0;
        for (TimingArc *arc : arc_set->arcs())
          arc_count++;
        es.arcs.resize(arc_count * ap_count_, ArcSensitivity{});
        for (TimingArc *arc : arc_set->arcs()) {
          size_t idx = lmIndex(arc, ap_index_, ap_count_);
          if (idx < es.arcs.size()) {
            es.arcs[idx].dd_dslew = 0.0f;
            es.arcs[idx].dslew_dslew = 1.0f;
          }
        }
        continue;
      }

      // Gate edge: finite-difference partial derivatives
      Vertex *from_vertex = edge->from(graph);
      Vertex *to_vertex = edge->to(graph);
      const sta::Pin *drvr_pin = to_vertex->pin();
      if (!drvr_pin)
        continue;

      EdgeSensitivity &es = getOrCreateEdgeSensitivity(edge);
      size_t arc_count = 0;
      for (TimingArc *arc : arc_set->arcs())
        arc_count++;
      es.arcs.resize(arc_count * ap_count_, ArcSensitivity{});

      sta::LoadPinIndexMap empty_map(sta_->network());

      for (TimingArc *arc : arc_set->arcs()) {
        const RiseFall *from_rf = arc->fromEdge()->asRiseFall();
        const RiseFall *to_rf = arc->toEdge()->asRiseFall();
        if (!from_rf || !to_rf)
          continue;

        size_t idx = lmIndex(arc, ap_index_, ap_count_);
        if (idx >= es.arcs.size())
          continue;

        const Slew &in_slew_val = graph->slew(from_vertex, from_rf, ap_index_);
        float in_slew = sta::delayAsFloat(in_slew_val);
        float load_cap = gdc->loadCap(drvr_pin, to_rf, dcalc_ap);
        float delta = std::max(std::abs(in_slew) * 0.02f, 1e-14f);

        sta::ArcDcalcResult base_result = arc_delay_calc->gateDelay(
            drvr_pin, arc, in_slew, load_cap, nullptr,
            empty_map, dcalc_ap);
        float base_d = sta::delayAsFloat(base_result.gateDelay());
        float base_s = sta::delayAsFloat(base_result.drvrSlew());
        arc_delay_calc->finishDrvrPin();

        sta::ArcDcalcResult pert_result = arc_delay_calc->gateDelay(
            drvr_pin, arc, in_slew + delta, load_cap, nullptr,
            empty_map, dcalc_ap);
        float pert_d = sta::delayAsFloat(pert_result.gateDelay());
        float pert_s = sta::delayAsFloat(pert_result.drvrSlew());
        arc_delay_calc->finishDrvrPin();

        es.arcs[idx].dd_dslew = (pert_d - base_d) / delta;
        es.arcs[idx].dslew_dslew = (pert_s - base_s) / delta;
      }
    }
  }

  // ---- Phase 2: Backward topological propagation of φ ----
  // Collect vertices per level for efficient traversal.
  std::vector<std::vector<Vertex*>> vertices_by_level(max_level + 1);
  {
    sta::VertexIterator vert_iter2(graph);
    while (vert_iter2.hasNext()) {
      Vertex *vertex = vert_iter2.next();
      int lvl = vertex->level();
      if (lvl >= 0 && lvl <= max_level)
        vertices_by_level[lvl].push_back(vertex);
    }
  }

  for (int level = max_level; level >= 0; level--) {
    for (Vertex *vertex : vertices_by_level[level]) {
      VertexId vid = vertex->objectIdx();

      // Step A: Find dominant downstream φ.
      // For each outgoing edge, the downstream φ is stored in
      // vertex_dominant_phi_ of the target vertex.
      float downstream_phi = 0.0f;
      sta::VertexOutEdgeIterator out_iter(vertex, graph);
      while (out_iter.hasNext()) {
        Edge *out_edge = out_iter.next();
        if (out_edge->isDisabledConstraint() || out_edge->isDisabledCond()
            || out_edge->isDisabledLoop())
          continue;
        Vertex *to_vertex = out_edge->to(graph);
        VertexId to_vid = to_vertex->objectIdx();
        if (to_vid < vertex_dominant_phi_.size())
          downstream_phi = std::max(downstream_phi, vertex_dominant_phi_[to_vid]);
      }

      // Step B: Compute φ for each incoming edge's arcs.
      // φ_{i→j} = λ_{i→j} * δd/δslew + δslew_out/δslew_in * φ_dominant
      sta::VertexInEdgeIterator in_iter(vertex, graph);
      while (in_iter.hasNext()) {
        Edge *in_edge = in_iter.next();
        if (in_edge->isDisabledConstraint() || in_edge->isDisabledCond()
            || in_edge->isDisabledLoop())
          continue;

        EdgeSensitivity *es = const_cast<EdgeSensitivity*>(edgeSensitivity(in_edge));
        if (!es)
          continue;

        const LMValue *lms = in_edge->arcLms();
        sta::TimingArcSet *arc_set = in_edge->timingArcSet();
        if (!arc_set)
          continue;

        for (TimingArc *arc : arc_set->arcs()) {
          size_t idx = lmIndex(arc, ap_index_, ap_count_);
          if (idx >= es->arcs.size())
            continue;

          float lam = (lms != nullptr) ? lms[idx] : 0.0f;
          float dd_ds = es->arcs[idx].dd_dslew;
          float ds_ds = es->arcs[idx].dslew_dslew;

          es->arcs[idx].phi = lam * dd_ds + ds_ds * downstream_phi;
        }
      }

      // Step C: Update vertex_dominant_phi_ = max |φ| among incoming arcs.
      float vertex_phi = 0.0f;
      sta::VertexInEdgeIterator in_iter2(vertex, graph);
      while (in_iter2.hasNext()) {
        Edge *in_edge = in_iter2.next();
        const EdgeSensitivity *es = edgeSensitivity(in_edge);
        if (!es)
          continue;
        sta::TimingArcSet *arc_set = in_edge->timingArcSet();
        if (!arc_set)
          continue;
        for (TimingArc *arc : arc_set->arcs()) {
          size_t idx = lmIndex(arc, ap_index_, ap_count_);
          if (idx < es->arcs.size())
            vertex_phi = std::max(vertex_phi, std::abs(es->arcs[idx].phi));
        }
      }

      if (vid < vertex_dominant_phi_.size())
        vertex_dominant_phi_[vid] = vertex_phi;
    }
  }

  // ---- Phase 3: Compute drain_phi_sum per driver vertex ----
  // For each driver vertex v, drain_phi_sum_[v] = Σ φ_{v→j} over all
  // outgoing arcs.  This is the Σφ term in Eq.13 for drain net cost.
  sta::VertexIterator vert_iter3(graph);
  while (vert_iter3.hasNext()) {
    Vertex *vertex = vert_iter3.next();
    if (!vertex->hasFanout())
      continue;
    VertexId vid = vertex->objectIdx();
    float phi_sum = 0.0f;

    sta::VertexOutEdgeIterator out_iter(vertex, graph);
    while (out_iter.hasNext()) {
      Edge *out_edge = out_iter.next();
      if (out_edge->isDisabledConstraint() || out_edge->isDisabledCond()
          || out_edge->isDisabledLoop())
        continue;

      const EdgeSensitivity *es = edgeSensitivity(out_edge);
      if (!es)
        continue;
      sta::TimingArcSet *arc_set = out_edge->timingArcSet();
      if (!arc_set)
        continue;
      for (TimingArc *arc : arc_set->arcs()) {
        size_t idx = lmIndex(arc, ap_index_, ap_count_);
        if (idx < es->arcs.size())
          phi_sum += es->arcs[idx].phi;
      }
    }

    if (vid < drain_phi_sum_.size())
      drain_phi_sum_[vid] = phi_sum;
  }

  ready_ = true;
}

float
GlobalSensitivity::drainPhiSum(const sta::Vertex *drvr_vertex) const
{
  if (!ready_ || !drvr_vertex)
    return 0.0f;
  VertexId vid = drvr_vertex->objectIdx();
  if (vid < drain_phi_sum_.size())
    return drain_phi_sum_[vid];
  return 0.0f;
}

float
GlobalSensitivity::phi(const sta::Edge *edge, const sta::TimingArc *arc) const
{
  if (!ready_)
    return 0.0f;
  const EdgeSensitivity *es = edgeSensitivity(edge);
  if (!es)
    return 0.0f;
  size_t idx = lmIndex(arc, ap_index_, ap_count_);
  if (idx >= es->arcs.size())
    return 0.0f;
  return es->arcs[idx].phi;
}

const GlobalSensitivity::EdgeSensitivity *
GlobalSensitivity::edgeSensitivity(const sta::Edge *edge) const
{
  auto it = edge_sens_.find(edge->objectIdx());
  if (it == edge_sens_.end())
    return nullptr;
  return &it->second;
}

GlobalSensitivity::EdgeSensitivity &
GlobalSensitivity::getOrCreateEdgeSensitivity(const sta::Edge *edge)
{
  return edge_sens_[edge->objectIdx()];
}

}  // namespace lrf

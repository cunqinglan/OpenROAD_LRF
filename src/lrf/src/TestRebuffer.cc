#include <set>
#include <cstdlib>
#include <cstring>
#include "TestRebuffer.hh"
#include "LocalSta.hh"
#include "PtGraph.hh"
#include "NetlistTransformation.hh"
#include "rsz/Resizer.hh"
#include "odb/db.h"
#include "db_sta/dbNetwork.hh"
#include "sta/Search.hh"
#include "search/Tag.hh"
#include "search/TagGroup.hh"
#include "sta/PathAnalysisPt.hh"

namespace lrf {

// ────────────────────────────────────────────────────────────────────────
// rebufferPinVG: Verified probe for one pin with one method.
//   method: 0=RSZ (slack-based), 1=LRF-worst, 2=LRF-sum
//
// Local phase: fresh PtGraph → generate bnet → virtual buffer → local timing
//   → print local worst/sum slack delta
// Global phase: ECO → apply buffer to DB → estimate_parasitics → full STA
//   → print global worst_sink/sum_sink/WNS/TNS delta → undoEco
//
// Uses separate PtGraph for local vs global to avoid state contamination.
// For LRF methods, use_sum_threshold switches the evaluateOption threshold
// between localWorstSlackOnSinks (worst) and localSlackOnSinks (sum).
// ────────────────────────────────────────────────────────────────────────
void
TestRebuffer::rebufferPinVG(const sta::Pin *drvr_pin, sta::Instance *inst,
                             odb::dbBlock *block, int method,
                             const GlobalBaseline &baseline)
{
  const char *names[] = {"RSZ", "LRF-worst", "LRF-sum"};
  const char *label = names[method];

  if (network_->isTopLevelPort(drvr_pin)) return;
  sta::Vertex *drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
  if (!drvr_vertex) return;
  int fo = Rebuffer::fanout(drvr_vertex);
  sta::Net *net = network_->net(drvr_pin);
  drvr_port_ = network_->libertyPort(drvr_pin);
  if (!net || !drvr_port_ || hasTopLevelOutputPort(net)) return;

  // ── Fresh PtGraph for local evaluation ──
  PtGraph *pg = local_sta_->makePtGraph(inst, true);
  if (!pg) { printf("    [%s] PtGraph failed\n", label); return; }

  PtVertex *drvr_pv = nullptr;
  for (size_t i = 0; i < pg->vertexCount(); i++) {
    PtVertex &pv = pg->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput
        && pv.vertex()->pin() == drvr_pin) {
      drvr_pv = &pv;
      break;
    }
  }
  if (!drvr_pv) { printf("    [%s] no driver PtVertex\n", label); return; }

  eval_ctx_->pt_graph = pg;
  VertexId vid = drvr_pv->objectIdx();

  // Local baseline
  local_sta_->increAndGetLocalTimingCost(pg, arc_delay_calc_, nullptr);
  float orig_w = local_sta_->localWorstSlackOnSinks(pg);
  float orig_s = local_sta_->localSlackOnSinks(pg);

  // ── Generate bnet based on method ──
  rsz::BufferedNetPtr bnet = nullptr;
  int local_bufs = 0;
  float local_w_after = orig_w, local_s_after = orig_s;

  if (method == 0) {
    // RSZ: slack-based bufferForTiming
    setPin(const_cast<sta::Pin*>(drvr_pin));
    drvr_pin_ = drvr_pin;
    bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
    if (bnet) {
      sta_->findRequireds();
      annotateLoadSlacks(bnet, drvr_vertex);
      for (int i = 0; i < 3; i++) {
        bnet = Rebuffer::bufferForTiming(bnet, true);
        if (!bnet) break;
      }
    }
    if (bnet && bufferNum(bnet) > 0) {
      local_bufs = bufferNum(bnet);
      VirtualBufferInfo vinfo = buildVirtualBuffer(vid, bnet);
      if (!vinfo.failed) {
        pg->topoSortVertices();
        buildSyntheticParasitics(vid, bnet, vinfo);
        local_sta_->increAndGetLocalTimingCost(pg, arc_delay_calc_, nullptr);
        local_w_after = local_sta_->localWorstSlackOnSinks(pg);
        local_s_after = local_sta_->localSlackOnSinks(pg);
        removeVirtualBuffer(vinfo);
      }
    }
  } else {
    // LRF: rebufferPin with worst or sum threshold
    bool saved = eval_ctx_->use_sum_threshold;
    eval_ctx_->use_sum_threshold = (method == 2);

    best_bnet_ = nullptr;
    best_cost_ = std::numeric_limits<float>::max();
    best_vinfo_ = VirtualBufferInfo{};

    rebufferPin(drvr_pin, *drvr_pv);

    if (best_bnet_) {
      local_bufs = bufferNum(best_bnet_);
      local_w_after = local_sta_->localWorstSlackOnSinks(pg);
      local_s_after = local_sta_->localSlackOnSinks(pg);
      cleanupVirtualBuffer();
    }
    eval_ctx_->use_sum_threshold = saved;
  }

  // Print local results
  if (local_bufs > 0) {
    printf("    [%s] bufs=%d | local_worst: %+.1f (%.1f->%.1f) | local_sum: %+.1f (%.1f->%.1f)\n",
           label, local_bufs,
           (local_w_after - orig_w) * 1e12, orig_w * 1e12, local_w_after * 1e12,
           (local_s_after - orig_s) * 1e12, orig_s * 1e12, local_s_after * 1e12);
  } else {
    printf("    [%s] no buffer | local: worst=%.1f sum=%.1f\n",
           label, orig_w * 1e12, orig_s * 1e12);
  }

  // ── Global verification: apply to DB, measure, revert ──
  if (local_bufs > 0 || method == 0) {
    odb::dbDatabase::beginEco(block);
    int global_bufs = 0;

    if (method == 0) {
      int before = block->getInsts().size();
      resizer_->rebufferNet(drvr_pin);
      global_bufs = block->getInsts().size() - before;
    } else {
      // Re-run LRF rebufferPin on fresh PtGraph for DB application
      PtGraph *pg2 = local_sta_->makePtGraph(inst, true);
      if (pg2) {
        PtVertex *pv2 = nullptr;
        for (size_t i = 0; i < pg2->vertexCount(); i++) {
          PtVertex &pv = pg2->ptVertex(i);
          if (pv.vertex() && pv.type() == PtVertexType::RefOutput
              && pv.vertex()->pin() == drvr_pin) {
            pv2 = &pv; break;
          }
        }
        if (pv2) {
          eval_ctx_->pt_graph = pg2;
          bool saved = eval_ctx_->use_sum_threshold;
          eval_ctx_->use_sum_threshold = (method == 2);
          best_bnet_ = nullptr;
          best_cost_ = std::numeric_limits<float>::max();
          best_vinfo_ = VirtualBufferInfo{};
          rebufferPin(drvr_pin, *pv2);
          if (best_bnet_) {
            int before = block->getInsts().size();
            applyBufferingToDb();
            global_bufs = block->getInsts().size() - before;
          }
          cleanupVirtualBuffer();
          eval_ctx_->use_sum_threshold = saved;
        }
      }
    }

    if (global_bufs > 0) {
      local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
      sta_->delaysInvalid();
      sta_->updateTiming(true);
      sta_->findRequireds();

      double wns = sta_->worstSlack(sta::MinMax::max());
      double tns = sta_->totalNegativeSlack(sta::MinMax::max());

      // Use pre-captured original sink vertices (not current net which is split by buffer)
      double g_ws = 1e30, g_ss = 0;
      for (sta::Vertex *v : baseline.orig_sink_vertices) {
        float s = sta_->vertexSlack(v, sta::MinMax::max());
        if (s < g_ws) g_ws = s;
        g_ss += s;
      }

      printf("           global: bufs=%d dWorstSink=%+.1f dSumSink=%+.1f dWNS=%+.1f dTNS=%+.1f\n",
             global_bufs,
             (g_ws - baseline.worst_sink) * 1e12,
             (g_ss - baseline.sum_sink) * 1e12,
             (wns - baseline.wns) * 1e12,
             (tns - baseline.tns) * 1e12);
    }

    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
    sta_->delaysInvalid();
    sta_->updateTiming(true);
    sta_->findRequireds();
  }
}

void
TestRebuffer::probeRszBnetWithLocalEval(const sta::Pin *drvr_pin,
                                         PtVertex &drvr_pt_vertex)
{
  if (network_->isTopLevelPort(drvr_pin))
    return;

  sta::Vertex *drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
  int fo = Rebuffer::fanout(drvr_vertex);
  if (fo <= 1 || fo >= 20)
    return;
  if (!resizer_->okToBufferNet(drvr_pin))
    return;

  sta::Net *net = network_->net(drvr_pin);
  drvr_port_ = network_->libertyPort(drvr_pin);
  if (!net || !drvr_port_ || hasTopLevelOutputPort(net))
    return;

  // ── Step 1: RSZ bnet generation (slack-based) ──
  setPin(const_cast<sta::Pin*>(drvr_pin));
  rsz::BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet) return;

  sta_->findRequireds();
  annotateLoadSlacks(bnet, drvr_vertex);

  for (int i = 0; i < 3; i++) {
    bnet = Rebuffer::bufferForTiming(bnet, true);
    if (!bnet) return;
  }

  int buf_count = bufferNum(bnet);
  if (buf_count == 0) return;

  std::optional<rsz::FixedDelay> rsz_slack_opt = Rebuffer::evaluateOption(bnet, 0);
  float rsz_slack = rsz_slack_opt ? rsz_slack_opt->toSeconds() : -1e30f;

  // ── Step 2: LRF local timing evaluation on the RSZ bnet ──
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  VertexId drvr_vid = drvr_pt_vertex.objectIdx();

  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float orig_worst_sinks = local_sta_->localWorstSlackOnSinks(pt_graph);
  float orig_sum_sinks   = local_sta_->localSlackOnSinks(pt_graph);
  float orig_around_ref  = local_sta_->localSlackAroundRef(pt_graph);

  rsz::BufferedNetPtr bnet_lm = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (bnet_lm) annotateLoadLMs(drvr_pt_vertex, bnet_lm);

  VirtualBufferInfo vinfo = buildVirtualBuffer(drvr_vid, bnet);
  if (vinfo.failed) {
    removeVirtualBuffer(vinfo);
    printf("[PROBE] pin=%s buffers=%d rsz_slack=%.3e VBUF_FAILED\n",
           network_->name(drvr_pin), buf_count, rsz_slack);
    return;
  }

  pt_graph->topoSortVertices();
  buildSyntheticParasitics(drvr_vid, bnet, vinfo);

  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float after_worst_sinks = local_sta_->localWorstSlackOnSinks(pt_graph);
  float after_sum_sinks   = local_sta_->localSlackOnSinks(pt_graph);
  float after_around_ref  = local_sta_->localSlackAroundRef(pt_graph);

  printf("[PROBE] pin=%s buffers=%d rsz_slack=%.3e "
         "worst_orig=%.3e worst_after=%.3e worst_delta=%.3e worst_pass=%d | "
         "sum_orig=%.3e sum_after=%.3e sum_delta=%.3e sum_pass=%d | "
         "ref_orig=%.3e ref_after=%.3e ref_delta=%.3e ref_pass=%d\n",
         network_->name(drvr_pin), buf_count, rsz_slack,
         orig_worst_sinks, after_worst_sinks,
         after_worst_sinks - orig_worst_sinks,
         after_worst_sinks >= orig_worst_sinks,
         orig_sum_sinks, after_sum_sinks,
         after_sum_sinks - orig_sum_sinks,
         after_sum_sinks >= orig_sum_sinks,
         orig_around_ref, after_around_ref,
         after_around_ref - orig_around_ref,
         after_around_ref >= orig_around_ref);

  removeVirtualBuffer(vinfo);
  local_sta_->recomputeSinglePtParasitic(pt_graph, drvr_vid);

  // ── Step 3: LRF rebufferPin on the same pin for comparison ──
  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float lrf_orig_worst = local_sta_->localWorstSlackOnSinks(pt_graph);

  rebufferPin(drvr_pin, pt_graph->ptVertex(drvr_vid));

  int lrf_buf_count = 0;
  float lrf_worst_after = lrf_orig_worst;
  if (bestBnet()) {
    lrf_buf_count = bufferNum(bestBnet());
    lrf_worst_after = local_sta_->localWorstSlackOnSinks(pt_graph);
  }
  float lrf_cost = bestCost();

  printf("[PROBE-LRF] pin=%s lrf_bufs=%d lrf_cost=%.3e "
         "lrf_worst_orig=%.3e lrf_worst_after=%.3e lrf_worst_delta=%.3e lrf_pass=%d\n",
         network_->name(drvr_pin), lrf_buf_count, lrf_cost,
         lrf_orig_worst, lrf_worst_after,
         lrf_worst_after - lrf_orig_worst,
         lrf_worst_after >= lrf_orig_worst);

  cleanupVirtualBuffer();
}

// ────────────────────────────────────────────────────────────────────────
// probeAllOptions: enumerate ALL bnet candidates for one driver pin,
// evaluate each with both local (virtual buffer on PtGraph) and global
// (actual DB insertion + full STA + revert) metrics.
//
// For each candidate bnet (RSZ + all LRF options):
//   Local:  buildVirtualBuffer → buildSyntheticParasitics → increAndGetLocalTimingCost
//           → localWorstSlackOnSinks / localSlackOnSinks
//   Global: beginEco → exportBufferTree → estimate_parasitics → updateTiming
//           → per-sink vertexSlack → worstSlack / totalNegativeSlack → undoEco
//
// Purpose: identify discrepancy between local PtGraph slack prediction and
// actual global STA result after buffer insertion, per option.
// ────────────────────────────────────────────────────────────────────────
void
TestRebuffer::probeAllOptions(const sta::Pin *drvr_pin, sta::Instance *inst,
                               odb::dbBlock *block,
                               const GlobalBaseline &baseline)
{
  if (network_->isTopLevelPort(drvr_pin)) return;
  sta::Vertex *drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
  if (!drvr_vertex) return;
  sta::Net *net = network_->net(drvr_pin);
  drvr_port_ = network_->libertyPort(drvr_pin);
  if (!net || !drvr_port_ || hasTopLevelOutputPort(net)) return;

  printf("\n  ══ probeAllOptions: %s (fanout=%d) ══\n",
         network_->pathName(inst), Rebuffer::fanout(drvr_vertex));

  // ── Step 1: Generate bnet candidates ──
  // LRF: makeBufferedNet → annotateLoadLMs → bufferForTiming ×3
  //   → last_top_opts_ = all Pareto-optimal bnet options from final iteration
  // RSZ: makeBufferedNet → annotateLoadSlacks → Rebuffer::bufferForTiming ×3
  //   → single best bnet (slack-based pruning)
  PtGraph *pg = local_sta_->makePtGraph(inst, true);
  if (!pg) { printf("    PtGraph failed\n"); return; }

  PtVertex *drvr_pv = nullptr;
  for (size_t i = 0; i < pg->vertexCount(); i++) {
    PtVertex &pv = pg->ptVertex(i);
    if (pv.vertex() && pv.type() == PtVertexType::RefOutput
        && pv.vertex()->pin() == drvr_pin) {
      drvr_pv = &pv; break;
    }
  }
  if (!drvr_pv) { printf("    no driver PtVertex\n"); return; }

  eval_ctx_->pt_graph = pg;
  VertexId vid = drvr_pv->objectIdx();

  // Local baseline
  local_sta_->increAndGetLocalTimingCost(pg, arc_delay_calc_, nullptr);
  float orig_w = local_sta_->localWorstSlackOnSinks(pg);
  float orig_s = local_sta_->localSlackOnSinks(pg);
  printf("    local_baseline: worst=%.1f ps  sum=%.1f ps\n",
         orig_w * 1e12, orig_s * 1e12);

  // Pre-capture original sink vertices BEFORE any buffer insertion.
  // Walk wire edges from driver, skip through existing buffers to find
  // final non-buffer load pins. These are the "true sinks" whose slack
  // we compare before/after buffer insertion.
  std::vector<sta::Vertex*> orig_sinks;
  {
    std::vector<sta::Vertex*> stack = {drvr_vertex};
    std::set<sta::Vertex*> visited;
    while (!stack.empty()) {
      sta::Vertex *v = stack.back(); stack.pop_back();
      if (!visited.insert(v).second) continue;
      sta::VertexOutEdgeIterator oi(v, graph_);
      while (oi.hasNext()) {
        sta::Edge *e = oi.next();
        if (!e->isWire()) continue;
        sta::Vertex *to = e->to(graph_);
        const sta::Pin *to_pin = to->pin();
        sta::LibertyPort *port = network_->libertyPort(to_pin);
        bool is_buf = port && port->libertyCell()
                      && port->libertyCell()->isBuffer();
        if (is_buf && network_->isLoad(to_pin)) {
          // This is an existing buffer input — skip through to its output
          sta::Instance *buf_inst = network_->instance(to_pin);
          sta::InstancePinIterator *ipi = network_->pinIterator(buf_inst);
          while (ipi->hasNext()) {
            sta::Pin *bp = ipi->next();
            if (network_->isDriver(bp)) {
              sta::Vertex *bv = graph_->pinDrvrVertex(bp);
              if (bv) stack.push_back(bv);
            }
          }
          delete ipi;
        } else {
          // Non-buffer load = true sink
          orig_sinks.push_back(to);
        }
      }
    }
  }

  // Print per-sink GLOBAL baseline slack (original sinks only)
  double bl_worst = 1e30, bl_sum = 0;
  printf("    per-sink GLOBAL baseline (%zu original sinks, ps):\n", orig_sinks.size());
  for (sta::Vertex *lv : orig_sinks) {
    float s = sta_->vertexSlack(lv, sta::MinMax::max());
    printf("      %-50s %.1f\n", network_->pathName(lv->pin()), s * 1e12);
    if (s < bl_worst) bl_worst = s;
    bl_sum += s;
  }
  // Update baseline to use original sinks (not the 1-layer net iterator)
  GlobalBaseline sink_bl = baseline;
  sink_bl.worst_sink = bl_worst;
  sink_bl.sum_sink = bl_sum;

  // Generate LRF bnet options
  setPin(const_cast<sta::Pin*>(drvr_pin));
  drvr_pin_ = drvr_pin;
  last_top_opts_.clear();

  rsz::BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet) { printf("    makeBufferedNet failed\n"); return; }

  local_sta_->increAndGetLocalTimingCost(pg, arc_delay_calc_, nullptr);
  annotateLoadLMs(*drvr_pv, bnet);

  // Enable pruning trace for targeted diagnostic pins (env var PRUNE_DEBUG_PIN
  // acts as a substring filter; empty string disables).
  bool want_prune_debug = false;
  if (const char *pat = std::getenv("PRUNE_DEBUG_PIN")) {
    if (pat[0] != '\0'
        && strstr(network_->pathName(drvr_pin), pat) != nullptr) {
      want_prune_debug = true;
    }
  }
  prune_debug_ = want_prune_debug;
  if (prune_debug_) {
    printf("[DBG-PRUNE] enabled for pin %s\n", network_->pathName(drvr_pin));
  }

  // Experiment: use worst-slack gate (instead of sum-slack gate) in
  // evaluateOption, to align with global WNS. Sum-slack gate rejects RSZ-
  // style "sacrifice non-critical sinks for worst-sink" multi-buffer
  // topologies.
  bool saved_sum_thresh = eval_ctx_->use_sum_threshold;
  eval_ctx_->use_sum_threshold = false;

  // Run 3 iterations; last_top_opts_ saved on final iteration
  for (int i = 0; i < 3; i++) {
    bnet = bufferForTiming(vid, bnet, true, /*last_iteration=*/(i == 2));
    if (!bnet) break;
  }

  eval_ctx_->use_sum_threshold = saved_sum_thresh;
  prune_debug_ = false;
  cleanupVirtualBuffer();

  if (last_top_opts_.empty()) {
    printf("    no bnet options generated\n");
    return;
  }

  // Also generate RSZ bnet for comparison
  rsz::BufferedNetPtr rsz_bnet = nullptr;
  {
    PtGraph *pg_rsz = local_sta_->makePtGraph(inst, true);
    if (pg_rsz) {
      eval_ctx_->pt_graph = pg_rsz;
      setPin(const_cast<sta::Pin*>(drvr_pin));
      rsz::BufferedNetPtr rb = resizer_->makeBufferedNet(drvr_pin, corner_);
      if (rb) {
        sta_->findRequireds();
        annotateLoadSlacks(rb, drvr_vertex);
        for (int i = 0; i < 3; i++) {
          rb = Rebuffer::bufferForTiming(rb, true);
          if (!rb) break;
        }
      }
      rsz_bnet = rb;
    }
  }

  // ── Step 2: Evaluate each bnet option (RSZ + all LRF candidates) ──
  // For each option we measure:
  //   Local:  lcl_dWorst = localWorstSlackOnSinks(after) - localWorstSlackOnSinks(before)
  //           lcl_dSum   = localSlackOnSinks(after) - localSlackOnSinks(before)
  //           (uses virtual buffer + synthetic Pi parasitics on PtGraph)
  //   Global: gbl_dWSink = worst_sink_slack(after) - worst_sink_slack(baseline)
  //           gbl_dSSink = sum_sink_slack(after) - sum_sink_slack(baseline)
  //           gbl_dWNS   = WNS(after) - WNS(baseline)
  //           gbl_dTNS   = TNS(after) - TNS(baseline)
  //           (uses exportBufferTree + estimate_parasitics + full STA update)
  // Per-sink global slack is printed to identify which sinks improve/degrade.
  printf("\n    %-4s %4s | %10s %12s | %10s %12s %8s %10s | %11s %11s %11s %11s\n",
         "#", "bufs", "lcl_dWorst", "lcl_dSum", "gbl_dWSink", "gbl_dSSink", "gbl_dWNS", "gbl_dTNS",
         "bnetCost", "dlyLmSum", "leakage", "swapCost");

  auto evalOneOption = [&](const char *label, const rsz::BufferedNetPtr &opt) {
    int bufs = bufferNum(opt);

    // ── Local eval (fresh PtGraph + virtual buffer) ──
    PtGraph *pg_l = local_sta_->makePtGraph(inst, true);
    float lw = orig_w, ls = orig_s;
    float dlm_nobuf = 0.0f;
    float dlm_after = 0.0f;
    float swap_nobuf = 0.0f;
    float swap_after = 0.0f;
    if (pg_l) {  // process buf==0 too for baseline verification
      eval_ctx_->pt_graph = pg_l;
      PtVertex *pv = nullptr;
      for (size_t j = 0; j < pg_l->vertexCount(); j++) {
        PtVertex &p = pg_l->ptVertex(j);
        if (p.vertex() && p.type() == PtVertexType::RefOutput
            && p.vertex()->pin() == drvr_pin) { pv = &p; break; }
      }
      if (pv) {
        VertexId v = pv->objectIdx();
        auto res_nobuf = local_sta_->increAndGetLocalTimingCost(pg_l, arc_delay_calc_, nullptr);
        dlm_nobuf = res_nobuf.delay_lm_sum;
        swap_nobuf = eval_ctx_->swapCost(dlm_nobuf, 0.0f);

        // Arrival BEFORE buffer (local vs global baseline)
        char lbl_before[64];
        snprintf(lbl_before, sizeof(lbl_before), "%s-BEFORE", label);
        local_sta_->printPerSinkArrivals(pg_l, lbl_before);

        VirtualBufferInfo vi = buildVirtualBuffer(v, opt);
        if (!vi.failed) {
          pg_l->topoSortVertices();
          printf("      ── [%s] bufs=%d bnet_root_cap=%.4f fF bnetCost=%.3e leakage=%.3e ──\n",
                 label, bufs, opt->cap() * 1e15,
                 opt->bufferCost(), opt->leakage());
          bool saved_debug = eval_ctx_->debug;
          eval_ctx_->debug = true;
          buildSyntheticParasitics(v, opt, vi);
          eval_ctx_->debug = saved_debug;
          auto res_after = local_sta_->increAndGetLocalTimingCost(pg_l, arc_delay_calc_, nullptr);
          dlm_after = res_after.delay_lm_sum;
          swap_after = eval_ctx_->swapCost(dlm_after, opt->leakage());
          lw = local_sta_->localWorstSlackOnSinks(pg_l);
          ls = local_sta_->localSlackOnSinks(pg_l);
          printf("      [%s] LR cost:  dlyLmSum nobuf=%.3e → after=%.3e (Δ=%+.3e) | "
                 "swapCost nobuf=%.3e → after=%.3e (Δ=%+.3e) | bnetCost=%.3e\n",
                 label, dlm_nobuf, dlm_after, dlm_after - dlm_nobuf,
                 swap_nobuf, swap_after, swap_after - swap_nobuf,
                 opt->bufferCost());

          // Collect synthetic Pi for all RefOutput vertices (driver + virtual buffer outputs)
          // and print per-sink local arrival after buffer
          struct SynPi { VertexId vid; const sta::Pin *pin; float c2, rpi, c1; };
          std::vector<SynPi> syn_pis;
          sta::DcalcAnalysisPt *dap = corners_->findCorner("default")
              ->findDcalcAnalysisPt(sta::MinMax::max());
          for (size_t si = 0; si < pg_l->vertexCount(); si++) {
            PtVertex &spv = pg_l->ptVertex(si);
            if (spv.type() == PtVertexType::RefOutput
                || (spv.vertex() && network_->isDriver(spv.vertex()->pin()))) {
              PtPiElmore *pi = pg_l->findPtParasitic(
                  spv.objectIdx(), sta::RiseFall::rise(), dap->index());
              if (pi) {
                float pc2, prpi, pc1;
                pi->piModel(pc2, prpi, pc1);
                syn_pis.push_back({spv.objectIdx(),
                                   spv.vertex() ? spv.vertex()->pin() : nullptr,
                                   pc2, prpi, pc1});
              }
            }
          }

          // Print synthetic Pi summary
          printf("      [%s] synthetic Pi (rise/max, fF):\n", label);
          for (auto &sp : syn_pis) {
            printf("        vid=%-4u %-35s C2=%.4f Rpi=%.1f C1=%.4f total=%.4f\n",
                   (unsigned)sp.vid,
                   sp.pin ? network_->pathName(sp.pin) : "(virtual)",
                   sp.c2 * 1e15, sp.rpi, sp.c1 * 1e15,
                   (sp.c2 + sp.c1) * 1e15);
          }

          // Print per-sink local arrival after buffer
          char lbl_after[64];
          snprintf(lbl_after, sizeof(lbl_after), "%s-AFTER", label);
          local_sta_->printPerSinkArrivals(pg_l, lbl_after);
          removeVirtualBuffer(vi);
        }
      }
    }

    // ── Global eval (apply to DB + revert) ──
    double g_dws = 0, g_dss = 0, g_dwns = 0, g_dtns = 0;
    int g_bufs = 0;
    if (bufs > 0) {
      PtGraph *pg_g = local_sta_->makePtGraph(inst, true);
      if (pg_g) {
        PtVertex *pv2 = nullptr;
        for (size_t j = 0; j < pg_g->vertexCount(); j++) {
          PtVertex &p = pg_g->ptVertex(j);
          if (p.vertex() && p.type() == PtVertexType::RefOutput
              && p.vertex()->pin() == drvr_pin) { pv2 = &p; break; }
        }
        if (pv2) {
          eval_ctx_->pt_graph = pg_g;
          VertexId v2 = pv2->objectIdx();

          // Rebuild: rebufferPin won't give us the exact same bnet, so we
          // directly build virtual buffer + export to DB
          odb::dbDatabase::beginEco(block);

          VirtualBufferInfo vi2 = buildVirtualBuffer(v2, opt);
          if (!vi2.failed) {
            int before = block->getInsts().size();
            // exportBufferTree applies the bnet to the physical DB
            odb::dbNet *db_net = db_network_->flatNet(drvr_pin);
            exportBufferTree(opt, db_network_->dbToSta(db_net), 1, nullptr, "probe");
            g_bufs = block->getInsts().size() - before;
            removeVirtualBuffer(vi2);
          }

          if (g_bufs > 0) {
            local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
            sta_->delaysInvalid();
            sta_->updateTiming(true);
            sta_->findRequireds();

            double wns = sta_->worstSlack(sta::MinMax::max());
            double tns = sta_->totalNegativeSlack(sta::MinMax::max());
            double ws = 1e30, ss = 0;

            // Read slack on the SAME original sink vertices (pre-captured).
            // Also print per-sink global arrival+slack after buffer for comparison
            // with local arrival from printPerSinkArrivals.
            printf("      [%s-GLOBAL] per-sink after buffer (ps):\n", label);
            printf("        %-40s %10s %10s\n", "sink", "gbl_arr", "gbl_slack");
            for (sta::Vertex *lv : orig_sinks) {
              float s = sta_->vertexSlack(lv, sta::MinMax::max());
              // Get max/rise arrival from vertex paths
              float arr = sta::INF;
              sta::Path *paths = lv->paths();
              if (paths) {
                sta::TagGroup *tg = search_->tagGroup(lv);
                if (tg) {
                  for (size_t pi = 0; pi < tg->pathCount(); pi++) {
                    sta::Tag *tag = paths[pi].tag(sta_);
                    if (tag && tag->rfIndex() == sta::RiseFall::riseIndex()
                        && tag->pathAnalysisPt(sta_)->pathMinMax() == sta::MinMax::max()) {
                      arr = paths[pi].arrival();
                      break;
                    }
                  }
                }
              }
              printf("        %-40s %+10.1f %+10.1f\n",
                     network_->name(lv->pin()),
                     sta::delayInf(arr) ? 0.0 : arr * 1e12,
                     s * 1e12);
              if (s < ws) ws = s;
              ss += s;
            }
            g_dws = (ws - sink_bl.worst_sink) * 1e12;
            g_dss = (ss - sink_bl.sum_sink) * 1e12;
            g_dwns = (wns - sink_bl.wns) * 1e12;
            g_dtns = (tns - sink_bl.tns) * 1e12;

            // Compare global Pi with synthetic Pi:
            // Read Pi model from global STA for the original driver pin
            // and any newly inserted buffer output pins.
            printf("      [%s-GLOBAL] Pi model comparison (rise/max, fF):\n", label);
            auto printGlobalPi = [&](const sta::Pin *pin, const char *desc) {
              float gc2, grpi, gc1;
              bool exists;
              sta_->findPiElmore(const_cast<sta::Pin*>(pin),
                                 sta::RiseFall::rise(), sta::MinMax::max(),
                                 gc2, grpi, gc1, exists);
              if (exists) {
                printf("        %-35s C2=%.4f Rpi=%.1f C1=%.4f total=%.4f  (%s)\n",
                       network_->pathName(pin),
                       gc2 * 1e15, grpi, gc1 * 1e15, (gc2 + gc1) * 1e15, desc);
              }
            };
            printGlobalPi(drvr_pin, "orig driver");
            // Find buffer output pins from ECO instances
            for (auto *db_inst : block->getInsts()) {
              std::string iname = db_inst->getName();
              if (iname.find("probe") != std::string::npos) {
                for (auto *iterm : db_inst->getITerms()) {
                  if (iterm->isOutputSignal() && iterm->getNet()) {
                    sta::Pin *buf_pin = db_network_->dbToSta(iterm);
                    if (buf_pin) printGlobalPi(buf_pin, "buffer out");
                  }
                }
              }
            }
          }

          odb::dbDatabase::endEco(block);
          odb::dbDatabase::undoEco(block);
          local_sta_->updateGlobalParasiticsAndSync(resizer_->getEstimateParasitics());
          sta_->delaysInvalid();
          sta_->updateTiming(true);
          sta_->findRequireds();
        }
      }
    }

    printf("    %-4s %4d | %+10.1f %+12.1f | %+10.1f %+12.1f %+8.1f %+10.1f | %+11.3e %+11.3e %+11.3e %+11.3e\n",
           label, bufs,
           (lw - orig_w) * 1e12, (ls - orig_s) * 1e12,
           g_dws, g_dss, g_dwns, g_dtns,
           opt->bufferCost(),
           dlm_after - dlm_nobuf,
           opt->leakage(),
           swap_after - swap_nobuf);
    fflush(stdout);
  };

  // Evaluate RSZ bnet
  if (rsz_bnet && bufferNum(rsz_bnet) > 0) {
    evalOneOption("RSZ", rsz_bnet);
  } else {
    printf("    RSZ    0 |        -            - |        -            -        -          -\n");
  }

  // Evaluate all LRF options
  for (size_t i = 0; i < last_top_opts_.size(); i++) {
    char label[16];
    snprintf(label, sizeof(label), "L%zu", i);
    evalOneOption(label, last_top_opts_[i]);
  }
}

}  // namespace lrf

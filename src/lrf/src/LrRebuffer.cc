



#include "LrRebuffer.hh"
#include "NetlistTransformation.hh"
#include "PtGraph.hh"
#include <chrono>
#include "odb/db.h"
#include "PtPiElmore.hh"
#include "PtElmoreCeff.hh"
#include "rsz/Resizer.hh"
#include "LocalSta.hh"
#include "LocalSearch.hh"
#include "LocalReduceParasitic.hh"
#include "LocalDmpDelayCalc.hh"  // useElmoreCeff()
#include "sta/FuncExpr.hh"
#include "sta/Fuzzy.hh"
#include "sta/TimingRole.hh"
#include "sta/PortDirection.hh"
#include "search/TagGroup.hh"
#include "sta/Search.hh"
#include "parasitics/ConcreteParasiticsPvt.hh"


namespace {

static float bufferCin(const sta::LibertyCell *cell)
{
  sta::LibertyPort *a, *y;
  cell->bufferPorts(a, y);
  return a->capacitance();
}

// ── Static helpers re-implemented from Rebuffer.cc (kept file-static there) ──
// Used by LrRebuffer's slack-DP variant; cleaner than friending Rebuffer.

static rsz::BufferedNetPtr stripWireOnBnet(rsz::BufferedNetPtr ptr)
{
  while (ptr->type() == rsz::BufferedNetType::wire
         || ptr->type() == rsz::BufferedNetType::via) {
    ptr = ptr->ref();
  }
  return ptr;
}

static rsz::BufferedNetPtr stripWiresAndBuffersOnBnet(rsz::BufferedNetPtr ptr)
{
  while (ptr->type() == rsz::BufferedNetType::wire
         || ptr->type() == rsz::BufferedNetType::buffer
         || ptr->type() == rsz::BufferedNetType::via) {
    ptr = ptr->ref();
  }
  return ptr;
}

static const sta::RiseFallBoth*
combinedTransition(const sta::RiseFallBoth* a, const sta::RiseFallBoth* b)
{
  if (a == b) return a;
  if (a == nullptr) return b;
  if (b == nullptr) return a;
  return sta::RiseFallBoth::riseFall();
}

static rsz::BufferedNetPtr createBnetJunctionLrf(
    rsz::Resizer* resizer,
    const rsz::BufferedNetPtr& p,
    const rsz::BufferedNetPtr& q,
    odb::Point location)
{
  rsz::BufferedNetPtr junc = std::make_shared<rsz::BufferedNet>(
      rsz::BufferedNetType::junction, location, p, q, resizer);
  junc->setSlackTransition(
      combinedTransition(p->slackTransition(), q->slackTransition()));
  junc->setSlack(std::min(p->slack(), q->slack()));
  return junc;
}

static std::optional<int> findWireLayerLrf(rsz::BufferedNetPtr node)
{
  while (node->type() != rsz::BufferedNetType::wire
         && node->type() != rsz::BufferedNetType::load
         && node->type() != rsz::BufferedNetType::junction) {
    node = node->ref();
  }
  if (node->type() == rsz::BufferedNetType::wire) {
    return {node->layer()};
  }
  return {};
}

} // namespace

namespace lrf {

using rsz::BufferedNetType;
using rsz::BufferedNetSeq;
using rsz::BufferedNetPtr;
using rsz::BufferedNet;
using rsz::FixedDelay;
using BnetType = BufferedNetType;
using BnetSeq = BufferedNetSeq;
using BnetPtr = BufferedNetPtr;
using BnetMetrics = BufferedNet::Metrics;

int 
LrRebuffer::bufferNum(const BnetPtr& tree)
{
  int num_buffers = 0;
  visitTree(
    [&](auto& recurse, int level, const BnetPtr& node) -> int {
      switch (node->type()) {
        case BnetType::buffer: num_buffers++; return recurse(node->ref());
        case BnetType::junction: return recurse(node->ref()) + recurse(node->ref2());
        case BnetType::wire: case BnetType::via: return recurse(node->ref());
        case BnetType::load: return 1;
        default: return 0;
      }
    }, tree);
  return num_buffers;
}

LrRebuffer::LrRebuffer(rsz::Resizer *resizer, LocalSta *local_sta,
                           EvalContext* eval_ctx) :
    Rebuffer(resizer),
    local_sta_(local_sta),
    eval_ctx_(eval_ctx)
{
  arc_delay_calc_ = eval_ctx_->arc_delay_calc;
}

void
LrRebuffer::annotateLoadSlacksSlackDp(BnetPtr& tree, sta::VertexId drvr_vid)
{
  // Step 1: cache driver arrival per rise/fall for the target dcalc_ap.
  // Replaces Rebuffer::arrival_paths_ (single-thread, shared-STA based) with
  // thread-local state derived from local_sta_ + the known PtGraph dap.
  sta::DcalcAnalysisPt *target_dcalc_ap = eval_ctx_->pt_graph->dcalcAnalysisPt();
  sta::Arrival drvr_arrival[sta::RiseFall::index_count] = {0, 0};
  PtVertex &drvr_pv = eval_ctx_->pt_graph->ptVertex(drvr_vid);
  PtVertexPathIterator drvr_iter(drvr_pv, local_sta_, eval_ctx_->pt_graph);
  while (drvr_iter.hasNext()) {
    sta::Path *p = drvr_iter.next();
    if (p->dcalcAnalysisPt(local_sta_) == target_dcalc_ap) {
      int rf_idx = p->transition(local_sta_)->index();
      drvr_arrival[rf_idx] = p->arrival();
    }
  }

  // Step 2: per-sink slack via LOCAL worst-slack path lookup. No shared-STA
  // calls (no sta_->vertexSlack / findRequired) — safe in parallel visit.
  visitTree(
    [&](auto& recurse, int level, const BnetPtr& node) -> int {
      switch (node->type()) {
        case BnetType::wire:
        case BnetType::via:
        case BnetType::buffer:
          return recurse(node->ref());
        case BnetType::junction:
          return recurse(node->ref()) + recurse(node->ref2());
        case BnetType::load: {
          const sta::Pin *load_pin = node->loadPin();
          sta::Vertex *sv = graph_->pinLoadVertex(load_pin);
          PtVertex *load_pv = sv ? eval_ctx_->pt_graph->ptVertex(sv) : nullptr;
          sta::Path *req_path = load_pv
              ? local_sta_->ptVertexWorstSlackPath(*load_pv, target_dcalc_ap)
              : nullptr;
          if (req_path == nullptr) {
            node->setSlackTransition(nullptr);
            node->setSlack(FixedDelay::INF);
          } else {
            const sta::RiseFall *rf = req_path->transition(local_sta_);
            sta::Delay d = req_path->required() - drvr_arrival[rf->index()];
            node->setSlack(FixedDelay(d, resizer_));
            node->setSlackTransition(rf->asRiseFallBoth());
          }
          return 1;
        }
        default:
          return 0;
      }
    },
    tree);
}

// ============================================================================
// Slack-based DP variant — self-contained copy of Rebuffer::bufferForTiming /
// insertBufferOptions / attemptTopologyRewrite. Replaces every bufferDelay()
// call with computeBufferGateDelay() to avoid arrival_paths_ dependency.
// Slack values come from annotateLoadSlacksFast.
// ============================================================================

BnetPtr
LrRebuffer::attemptTopologyRewriteSlackDp(const BnetPtr& node,
                                            const BnetPtr& left,
                                            const BnetPtr& right,
                                            float best_cap)
{
  FixedDelay junc_slack = std::min(left->slack(), right->slack());

  BnetPtr crit1, aux1;
  if (left->slack() < right->slack()) {
    crit1 = stripWireOnBnet(left);
    aux1 = stripWireOnBnet(right);
  } else {
    crit1 = stripWireOnBnet(right);
    aux1 = stripWireOnBnet(left);
  }
  if (crit1->type() == BnetType::junction) {
    BnetPtr crit2 = crit1->ref(), aux2 = crit1->ref2();
    if (crit2->slack() > aux2->slack()) {
      std::swap(crit2, aux2);
    }
    aux2 = stripWireOnBnet(aux2);
    if (aux1->type() == BnetType::buffer || aux2->type() == BnetType::buffer) {
      aux1 = stripWiresAndBuffersOnBnet(aux1);
      aux2 = stripWiresAndBuffersOnBnet(aux2);
      crit2 = stripWiresAndBuffersOnBnet(crit2);

      const BnetPtr in1 = addWire(aux1, node->location(), -1);
      const BnetPtr in2 = addWire(aux2, node->location(), -1);
      // Create inner junction, merge LMs from its two children, then wrap
      // in a wire. Without merging, junc1->lms() stays empty and the buffer
      // created below (with propagateLmsThroughBuffer(buf, junc1)) would
      // inherit empty LMs → downstream virtual edges warn "no lm values".
      BnetPtr junc_inner
          = createBnetJunctionLrf(resizer_, in1, in2, node->location());
      {
        auto merged_lms = mergeLmVectors(in1->lms(), in2->lms());
        if (!merged_lms.empty())
          junc_inner->setLms(std::move(merged_lms));
      }
      const BnetPtr junc1
          = addWire(junc_inner, node->location(), -1);
      const BnetPtr in3 = addWire(crit2, node->location(), -1);

      for (rsz::Rebuffer::BufferSize size : buffer_sizes_) {
        sta::LibertyPort *in, *out;
        size.cell->bufferPorts(in, out);

        if (fuzzyGreaterEqual(in->capacitance() + in3->cap(), best_cap)
            || fuzzyGreaterEqual(in->capacitance() + in3->cap(),
                                 left->cap() + right->cap())
            || junc1->slack() - size.intrinsic_delay < junc_slack) {
          break;
        }

        const FixedDelay buffer_delay = computeBufferGateDelay(
            size.cell, junc1->cap() + out->capacitance());
        const FixedDelay buffer_slack = junc1->slack() - buffer_delay;

        if (buffer_slack >= junc_slack && bufferSizeCanDriveLoad(size, junc1)) {
          BnetPtr buffer = std::make_shared<rsz::BufferedNet>(
              BnetType::buffer, node->location(), size.cell, junc1,
              corner_, resizer_, estimate_parasitics_);
          buffer->setSlack(buffer_slack);
          buffer->setSlackTransition(junc1->slackTransition());
          buffer->setDelay(buffer_delay);
          propagateLmsThroughBuffer(buffer, junc1);
          return createBnetJunctionLrf(resizer_, buffer, in3, node->location());
        }
      }
    }
  }
  return {};
}

void
LrRebuffer::insertBufferOptionsSlackDp(BnetSeq& opts,
                                        int level,
                                        int next_segment_wl,
                                        bool lrcost_oriented,
                                        FixedDelay slack_threshold,
                                        rsz::BufferedNet* exemplar)
{
  if (opts.empty()) return;

  rsz::Rebuffer::BufferSize& strong_driver = buffer_sizes_.back();

  BnetMetrics assured_envelope
      = lrcost_oriented ? exemplar->metrics().withSlack(slack_threshold)
                        : BnetMetrics{};
  bool assured_satisfied = !lrcost_oriented;

  float best_lrcost = INF;
  FixedDelay best_slack = -FixedDelay::INF;

  // both `opts` and `buffer_sizes_` are ordered by ascending input cap
  BnetSeq new_opts;
  new_opts.reserve(opts.size() * 2);
  auto opts_iter = opts.begin();

  auto pass_through = [&](float threshold_cap) {
    for (; opts_iter != opts.end() && (*opts_iter)->cap() <= threshold_cap;
         opts_iter++) {
      BnetPtr& opt = *opts_iter;
      bool keep = lrcost_oriented
                      ? (fuzzyLess(opt->bufferCost(), best_lrcost)
                         && opt->slack() >= slack_threshold)
                      : (opt->slack() > best_slack);
      if (!bufferSizeCanDriveLoad(strong_driver, opt, next_segment_wl)) {
        keep = false;
      }
      if (keep) {
        new_opts.push_back(opt);
        if (!assured_satisfied && opt->fitsEnvelope(assured_envelope)) {
          assured_satisfied = true;
        }
        best_slack = opt->slack();
        best_lrcost = opt->bufferCost();
      }
    }
  };

  for (rsz::Rebuffer::BufferSize buffer_size : buffer_sizes_) {
    sta::LibertyCell* buffer_cell = buffer_size.cell;
    sta::LibertyPort *in, *out;
    buffer_cell->bufferPorts(in, out);
    pass_through(in->capacitance());

    BnetPtr load_opt;
    FixedDelay load_opt_buffer_delay = FixedDelay::ZERO;
    float load_opt_buf_added_cost = 0.0f;
    auto it = (new_opts.empty() && opts_iter == opts.end()
               && opts_iter > opts.begin())
                  ? (opts_iter - 1)
                  : opts_iter;
    for (; it != opts.end(); it++) {
      BnetPtr& opt = *it;

      // Estimate buffer cost contribution using intrinsic delay (cheap
      // pre-filter — same pattern as base class area_oriented).
      float buf_leak = local_sta_->cellAvgLeakage(buffer_cell);
      float estim_added = computeBufferAddedCost(
          buffer_size.intrinsic_delay.toSeconds(), buf_leak, opt);
      bool initial_pass = lrcost_oriented
          ? (opt->slack() - buffer_size.intrinsic_delay >= slack_threshold
             && fuzzyLess(opt->bufferCost() + estim_added, best_lrcost))
          : ((opt->slack() - buffer_size.intrinsic_delay) > best_slack);

      if (initial_pass && bufferSizeCanDriveLoad(buffer_size, opt)) {
        // Precise delay calculation
        const FixedDelay buffer_delay = computeBufferGateDelay(
            buffer_cell, opt->cap() + out->capacitance());
        const FixedDelay slack = opt->slack() - buffer_delay;
        float precise_added = computeBufferAddedCost(
            buffer_delay.toSeconds(), buf_leak, opt);
        float precise_total = opt->bufferCost() + precise_added;

        bool precise_pass = lrcost_oriented
            ? (slack >= slack_threshold
               && fuzzyLess(precise_total, best_lrcost))
            : (slack > best_slack);

        if (precise_pass) {
          load_opt = opt;
          load_opt_buffer_delay = buffer_delay;
          load_opt_buf_added_cost = precise_added;
          best_slack = slack;
          best_lrcost = precise_total;
        }
      }
    }

    if (load_opt) {
      BnetPtr z = std::make_shared<rsz::BufferedNet>(
          BnetType::buffer, load_opt->location(), buffer_cell, load_opt,
          corner_, resizer_, estimate_parasitics_);
      z->setSlack(best_slack);
      z->setSlackTransition(load_opt->slackTransition());
      z->setDelay(load_opt_buffer_delay);
      // Annotate bufferCost so downstream Pareto pruning + selection works.
      z->setBufferCost(load_opt->bufferCost() + load_opt_buf_added_cost);
      z->setLeakage(load_opt->leakage()
                    + local_sta_->cellAvgLeakage(buffer_cell));
      // Propagate LMs through buffer so downstream virtual buffer edges
      // have arc_lms_ populated (delayLmSum would otherwise warn
      // "pt_edge N has no lm values" and skip this edge).
      propagateLmsThroughBuffer(z, load_opt);
      if (!assured_satisfied && z->fitsEnvelope(assured_envelope)) {
        assured_satisfied = true;
      }
      new_opts.push_back(std::move(z));
    }
  }
  pass_through(INF);

  // Assured-envelope fallback: mirrors base class behaviour. If no kept
  // option fits the exemplar's envelope, try every original option with
  // the exemplar's cell, then any option that fits envelope as last resort.
  if (!assured_satisfied) {
    if (exemplar && exemplar->type() == BnetType::buffer) {
      sta::LibertyCell* buffer_cell = exemplar->bufferCell();
      sta::LibertyPort *in, *out;
      buffer_cell->bufferPorts(in, out);

      float best_lrcost_local = INF;
      BnetPtr best_option;
      float best_added_cost = 0.0f;
      FixedDelay best_buf_delay = FixedDelay::ZERO;
      for (const BnetPtr& load_opt : opts) {
        if (load_opt->bufferCost() >= best_lrcost_local) continue;
        const FixedDelay buffer_delay = computeBufferGateDelay(
            buffer_cell, load_opt->cap() + out->capacitance());
        float buf_leak = local_sta_->cellAvgLeakage(buffer_cell);
        float added = computeBufferAddedCost(
            buffer_delay.toSeconds(), buf_leak, load_opt);

        if (bufferSizeCanDriveLoad(*buffer_sizes_index_.at(buffer_cell),
                                   load_opt)
            && load_opt->slack() - buffer_delay >= slack_threshold) {
          BnetPtr z = std::make_shared<rsz::BufferedNet>(
              BnetType::buffer, load_opt->location(), buffer_cell,
              load_opt, corner_, resizer_, estimate_parasitics_);
          z->setSlack(load_opt->slack() - buffer_delay);
          z->setSlackTransition(load_opt->slackTransition());
          z->setDelay(buffer_delay);
          z->setBufferCost(load_opt->bufferCost() + added);
          z->setLeakage(load_opt->leakage() + buf_leak);
          propagateLmsThroughBuffer(z, load_opt);
          if (z->fitsEnvelope(assured_envelope)) {
            best_lrcost_local = load_opt->bufferCost();
            best_option = z;
            best_added_cost = added;
            best_buf_delay = buffer_delay;
          }
        }
      }
      if (best_option) {
        insertAssuredOption(new_opts, best_option, level);
        assured_satisfied = true;
      }
    } else {
      for (const BnetPtr& opt : opts) {
        if (opt->fitsEnvelope(assured_envelope)) {
          insertAssuredOption(new_opts, opt, level);
          assured_satisfied = true;
          break;
        }
      }
    }
    if (!assured_satisfied) {
      printf("LrRebuffer::insertBufferOptionsSlackDp: pin %s assured fallback "
             "failed (lr-cost recovery cannot reproduce solution at "
             "slack_threshold=%.3e)\n",
             network_->name(pin_), slack_threshold.toSeconds());
    }
  }

  new_opts.swap(opts);
}

BnetPtr
LrRebuffer::bufferForTimingSlackDp(VertexId drvr_vertex_id,
                                    const BnetPtr& tree,
                                    bool allow_topology_rewrite)
{
  BnetSeq top_opts = visitTree(
    [&](auto& recurse, int level, const BnetPtr& node) -> BnetSeq {
      switch (node->type()) {
        case BnetType::via:
        case BnetType::buffer:
        case BnetType::wire: {
          int layer = -1;
          if (auto wire_layer = findWireLayerLrf(node)) {
            layer = wire_layer.value();
          }
          BnetSeq opts = recurse(stripWiresAndBuffersOnBnet(node->ref()));
          odb::Point location
              = stripWiresAndBuffersOnBnet(node->ref())->location();

          const int full_wl
              = odb::Point::manhattanDistance(node->location(), location);
          if (full_wl > wire_length_step_ / 2) {
            insertBufferOptionsSlackDp(
                opts, level, std::min(full_wl, wire_length_step_));
          } else {
            BnetSeq opts1 = opts;
            for (BnetPtr& opt : opts1) {
              opt = addWire(opt, node->location(), layer, level);
            }
            insertBufferOptionsSlackDp(opts1, level, 0);
            if (opts1.empty()) {
              opts1 = opts;
              insertBufferOptionsSlackDp(opts1, level, full_wl);
              for (BnetPtr& opt : opts1) {
                opt = addWire(opt, node->location(), layer, level);
              }
              insertBufferOptionsSlackDp(opts1, level, 0);
            }
            if (opts1.empty()) {
              printf("LrRebuffer::bufferForTimingSlackDp: pin %s wire step "
                     "options empty\n", network_->name(pin_));
            }
            return opts1;
          }

          int round = 0;
          while (location != node->location()) {
            const int step = wire_length_step_;
            int dx = node->location().x() - location.x();
            int dy = node->location().y() - location.y();

            if (abs(dx) + abs(dy) >= step) {
              const float ratio
                  = (float) abs(dx) / (float) (abs(dx) + abs(dy));
              const int dx_abs = std::min((int)(ratio * step), step);
              const int dy_abs = step - dx_abs;
              dx = dx > 0 ? dx_abs : -dx_abs;
              dy = dy > 0 ? dy_abs : -dy_abs;
            }
            location.addX(dx);
            location.addY(dy);

            const int remaining_wl
                = odb::Point::manhattanDistance(node->location(), location);

            for (BnetPtr& opt : opts) {
              opt = addWire(opt, location, layer, level);
            }
            insertBufferOptionsSlackDp(
                opts, level, std::min(remaining_wl, step));

            if (opts.empty()) {
              printf("LrRebuffer::bufferForTimingSlackDp: pin %s wire step "
                     "options empty round %d\n", network_->name(pin_), round);
            }
            round++;
          }
          return opts;
        }

        case BnetType::junction: {
          const BnetSeq& opts_left = recurse(node->ref());
          const BnetSeq& opts_right = recurse(node->ref2());

          BnetSeq opts;
          opts.reserve(std::max(opts_left.size(), opts_right.size()));
          float best_cap = INF;

          auto li = opts_left.rbegin(), lend = opts_left.rend();
          auto ri = opts_right.rbegin(), rend = opts_right.rend();

          while (li != lend && ri != rend) {
            while (li + 1 != lend
                   && (*(li + 1))->slack() >= (*ri)->slack()) li++;
            while (ri + 1 != rend
                   && (*(ri + 1))->slack() >= (*li)->slack()) ri++;

            bool rewrote = false;
            BnetPtr junc;
            if (allow_topology_rewrite) {
              junc = attemptTopologyRewriteSlackDp(node, *li, *ri, best_cap);
              if (junc) {
                rewrote = true;
                // Topology-rewrite returns a junction wrapping (buffer, in3).
                // Merge LMs from the two original children so downstream
                // virtual edges inherit non-empty arc_lms_.
                auto merged_lms = mergeLmVectors((*li)->lms(), (*ri)->lms());
                if (!merged_lms.empty())
                  junc->setLms(std::move(merged_lms));
              }
            }
            if (!rewrote) {
              junc = createBnetJunctionLrf(resizer_, *li, *ri, node->location());
              // Propagate LMs through junction (merge of children) so that
              // downstream insertBufferOptionsSlackDp can transitively pass
              // them to newly-created buffer nodes (otherwise delayLmSum
              // warns "pt_edge has no lm values" for virtual edges).
              auto merged_lms = mergeLmVectors((*li)->lms(), (*ri)->lms());
              if (!merged_lms.empty())
                junc->setLms(std::move(merged_lms));
            }

            if (junc->fanout() <= fanout_limit_) {
              best_cap = junc->cap();
              opts.push_back(std::move(junc));
            }

            while (true) {
              FixedDelay next_li_slack = (li + 1 != lend)
                                             ? (*(li + 1))->slack()
                                             : -FixedDelay::INF;
              FixedDelay next_ri_slack = (ri + 1 != rend)
                                             ? (*(ri + 1))->slack()
                                             : -FixedDelay::INF;
              if (next_li_slack > next_ri_slack) li++;
              else                               ri++;

              if (li == lend || ri == rend
                  || (*li)->cap() + (*ri)->cap() < best_cap) {
                break;
              }
            }
          }
          std::ranges::reverse(opts);
          return opts;
        }

        case BnetType::load:
          return {node};

        default:
          printf("LrRebuffer::bufferForTimingSlackDp: unhandled BnetType\n");
          return {};
      }
    },
    tree);

  if (top_opts.empty()) {
    printf("LrRebuffer::bufferForTimingSlackDp: pin %s no options produced\n",
           network_->name(pin_));
    return nullptr;
  }

  // Final selection via LrRebuffer::evaluateOption — runs precise local STA
  // (virtual buffer + synthetic Pi + increAndGetLocalTimingCost), enforces
  // ERC (slew + drvr cap high water mark) and slack guard, returns swapCost
  // in normalized units. Same machinery used by the cost-DP path's last
  // iteration, so behaviour and gating thresholds are identical.
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float original_slack = eval_ctx_->use_sum_threshold
      ? local_sta_->localSlackOnSinks(pt_graph)
      : local_sta_->localWorstSlackOnSinks(pt_graph);

  float best_cost = INF;
  BnetPtr best_option = nullptr;
  for (const BnetPtr& p : top_opts) {
    LMValue cost = evaluateOption(drvr_vertex_id, p, original_slack);
    if (cost < best_cost) {
      best_cost = cost;
      best_option = p;
    }
  }
  // Fallback: if every option was rejected by ERC / slack gate, return the
  // max-slack candidate so the caller still has something to apply (or
  // explicitly skip this pin via best_option == nullptr check).
  if (best_option == nullptr && !top_opts.empty()) {
    FixedDelay best_slack = -FixedDelay::INF;
    for (const BnetPtr& p : top_opts) {
      if (p->slackTransition() == nullptr || p->slack() > best_slack) {
        best_slack = p->slack();
        best_option = p;
      }
    }
  }
  return best_option;
}

// ============================================================================
// Recover LR cost — full port of Rebuffer::recoverArea.
//
// Identical structure to recoverArea: drvr pin slack correction, top-down
// arrival delay spread, bottom-up DP enumeration with assured-envelope
// guard, final selection by objective among slack-satisfying options.
//
// ONE substitution: the optimization objective is bufferCost (delay×LM +
// leakage, normalized to swapCost units via computeBufferAddedCost) instead
// of cell area. Every other piece of the recoverArea machinery is preserved:
//   - alpha-blended slack threshold via FixedDelay::lerp
//   - assured envelope tracking + insertAssuredOption fallback
//   - junction full N×M cross product + Pareto pruning
//   - wire/buffer step via insertBufferOptionsSlackDp(lrcost_oriented=true)
// ============================================================================

// Bottom-up walk that populates bufferCost on every node of the tree.
// Mirrors the per-node cost accumulation done during cost-DP enumeration:
//   load     : bufferCost = 0
//   wire/via : child.bufferCost + wire_delay × Σ LM (this node carries below)
//   buffer   : child.bufferCost + buf_delay × Σ LM + leakage
//   junction : left.bufferCost + right.bufferCost
// Caller must have annotated load LMs (annotateLoadLMs) on the bnet first.
void
LrRebuffer::computeAndAnnotateBufferCost(const BnetPtr& root)
{
  visitTree(
    [&](auto& recurse, int level, const BnetPtr& node) -> int {
      switch (node->type()) {
        case BnetType::load:
          node->setBufferCost(0.0f);
          return 1;
        case BnetType::via: {
          recurse(node->ref());
          node->setBufferCost(node->ref()->bufferCost());
          node->setLeakage(node->ref()->leakage());
          return 1;
        }
        case BnetType::wire: {
          recurse(node->ref());
          double layer_res = 0, layer_cap = 0;
          node->wireRC(corner_, resizer_, estimate_parasitics_,
                       layer_res, layer_cap);
          double wl = resizer_->dbuToMeters(node->length());
          double wd = (wl * layer_res)
                      * (wl * layer_cap / 2 + node->ref()->cap());
          float wire_delta = computeBufferAddedCost(wd, 0.0f, node->ref());
          node->setBufferCost(node->ref()->bufferCost() + wire_delta);
          node->setLeakage(node->ref()->leakage());
          return 1;
        }
        case BnetType::buffer: {
          recurse(node->ref());
          float buf_leak = local_sta_->cellAvgLeakage(node->bufferCell());
          FixedDelay buf_dly = computeBufferGateDelay(
              node->bufferCell(), node->ref()->cap());
          float buf_delta = computeBufferAddedCost(
              buf_dly.toSeconds(), buf_leak, node->ref());
          node->setBufferCost(node->ref()->bufferCost() + buf_delta);
          node->setLeakage(node->ref()->leakage() + buf_leak);
          return 1;
        }
        case BnetType::junction: {
          recurse(node->ref());
          recurse(node->ref2());
          node->setBufferCost(node->ref()->bufferCost()
                              + node->ref2()->bufferCost());
          node->setLeakage(node->ref()->leakage()
                           + node->ref2()->leakage());
          return 1;
        }
      }
      return 0;
    }, root);
}

BnetPtr
LrRebuffer::recoverLrCost(VertexId drvr_vertex_id,
                           const BnetPtr& root, FixedDelay slack_target,
                           float alpha)
{
  // ── Step 0: ensure bufferCost annotated on every node ──
  computeAndAnnotateBufferCost(root);

  // ── Step 1: drvr pin slack correction (LRF context — no arrival_paths_) ──
  // The original recoverArea calls drvrPinTiming which reads arrival_paths_.
  // In LRF context that's not populated; we use 0 as the correction. The
  // correction is a constant additive term to bnet->slack() across all
  // options and so does not affect ranking — only shifts the absolute slack
  // value used in slack_target comparisons.
  sta::Delay slack_correction = 0;

  if (!root->slackTransition()) {
    slack_correction = 0;
    slack_target = -FixedDelay::INF;
  }

  // ── Step 2: top-down spread of arrival delay through tree ──
  visitTree(
    [](auto& recurse, int level, const BnetPtr& node, FixedDelay arrival)
        -> int {
      node->setArrivalDelay(arrival);
      switch (node->type()) {
        case BnetType::via:
        case BnetType::wire:
        case BnetType::buffer:
          recurse(node->ref(), arrival + node->delay());
          break;
        case BnetType::junction:
          recurse(node->ref(), arrival);
          recurse(node->ref2(), arrival);
          break;
        case BnetType::load:
          break;
      }
      return 0;
    },
    root,
    -FixedDelay(slack_correction, resizer_));

  // ── Step 3: bottom-up DP enumeration (LR-cost-oriented) ──
  BnetSeq top_opts = visitTree(
    [&](auto& recurse, int level, const BnetPtr& node, int upstream_wl)
        -> BnetSeq {
      switch (node->type()) {
        case BnetType::buffer:
        case BnetType::wire: {
          const BnetPtr& inner
              = (node->type() == BnetType::buffer) ? node->ref() : node;

          BnetSeq opts;
          if (inner->type() == BnetType::wire) {
            opts = recurse(inner->ref(), inner->length());
            for (BnetPtr& opt : opts) {
              opt = addWire(opt, inner->location(), inner->layer(), level);
            }
          } else {
            opts = recurse(inner, 0);
          }

          FixedDelay threshold = FixedDelay::lerp(
              node->slack(), slack_target + node->arrivalDelay(), alpha);
          insertBufferOptionsSlackDp(opts,
                                     level,
                                     /*next_segment_wl=*/upstream_wl,
                                     /*lrcost_oriented=*/true,
                                     threshold,
                                     /*exemplar=*/node.get());
          return opts;
        }
        case BnetType::junction: {
          const BnetSeq& left_opts = recurse(node->ref(), upstream_wl);
          const BnetSeq& right_opts = recurse(node->ref2(), upstream_wl);

          FixedDelay threshold = FixedDelay::lerp(
              node->slack(), slack_target + node->arrivalDelay(), alpha);
          BnetMetrics assured_envelope = node->metrics().withSlack(threshold);
          BnetPtr assured_fallback;

          BnetSeq opts;
          opts.reserve(left_opts.size() * right_opts.size());
          for (const BnetPtr& left : left_opts) {
            for (const BnetPtr& right : right_opts) {
              BnetPtr junc = createBnetJunctionLrf(
                  resizer_, left, right, node->location());
              // Annotate junction's bufferCost (sum of children).
              junc->setBufferCost(left->bufferCost() + right->bufferCost());
              junc->setLeakage(left->leakage() + right->leakage());
              // Propagate merged LMs (non-SDP path does this via mergeLmVectors;
              // without it, downstream virtual buffer edges get empty arc_lms_).
              auto merged_lms = mergeLmVectors(left->lms(), right->lms());
              if (!merged_lms.empty())
                junc->setLms(std::move(merged_lms));
              if (!assured_fallback && junc->fitsEnvelope(assured_envelope)) {
                assured_fallback = junc;
              }
              if (junc->fanout() <= fanout_limit_) {
                opts.push_back(std::move(junc));
              }
            }
          }

          // Pareto pruning on (bufferCost, cap) — variant of
          // pruneCapVsAreaOptions with bufferCost replacing area.
          std::ranges::sort(
              opts,
              [](const BnetPtr& a, const BnetPtr& b) {
                return std::make_tuple(a->bufferCost(), a->cap())
                       < std::make_tuple(b->bufferCost(), b->cap());
              });
          if (!opts.empty()) {
            float lowest_cap_seen = opts[0]->cap();
            size_t si = 1;
            for (size_t pi = si; pi < opts.size(); pi++) {
              const BnetPtr& p = opts[pi];
              float cap = p->cap();
              if (fuzzyLess(cap, lowest_cap_seen)) {
                opts[si++] = p;
                lowest_cap_seen = cap;
              }
            }
            opts.resize(si);
            std::ranges::reverse(opts);
          }

          // Ensure assured-envelope fallback exists.
          bool assured_found = false;
          for (const BnetPtr& opt : opts) {
            if (opt->fitsEnvelope(assured_envelope)) {
              assured_found = true;
              break;
            }
          }
          if (!assured_found && assured_fallback) {
            insertAssuredOption(opts, assured_fallback, level);
          }
          return opts;
        }
        case BnetType::load: {
          return {node};
        }
        default:
          return {};
      }
    },
    root,
    0);

  // ── Step 4: precise selection via LrRebuffer::evaluateOption ──
  // evaluateOption runs ERC + slack guard + virtual-buffer local STA, and
  // returns swapCost in normalized units. Slack baseline is measured directly
  // from PtGraph (pre-recovery state, including any buffering done by an
  // earlier slack-DP pass) — same source as bufferForTimingSlackDp uses, no
  // dependency on the user-supplied slack_target as a gate value.
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float baseline_slack = eval_ctx_->use_sum_threshold
      ? local_sta_->localSlackOnSinks(pt_graph)
      : local_sta_->localWorstSlackOnSinks(pt_graph);

  float best_cost = INF;
  BnetPtr best_eval_option = nullptr;
  for (const BnetPtr& p : top_opts) {
    LMValue cost = evaluateOption(drvr_vertex_id, p, baseline_slack);
    if (cost < best_cost) {
      best_cost = cost;
      best_eval_option = p;
    }
  }
  if (best_eval_option) return best_eval_option;

  // Fallback when ERC / slack gate rejects everything: pick min bufferCost
  // among analytically slack-meeting options; if none meet slack, max slack.
  FixedDelay best_slack = -FixedDelay::INF;
  float best_lrcost = std::numeric_limits<float>::max();
  BnetPtr best_lrcost_option = nullptr, best_slack_option = nullptr;
  for (const BnetPtr& p : top_opts) {
    FixedDelay slack = p->slack() + FixedDelay(slack_correction, resizer_);
    if (best_slack_option == nullptr
        || p->slackTransition() == nullptr
        || slack > best_slack) {
      best_slack = slack;
      best_slack_option = p;
    }
    if ((slack >= slack_target || p->slackTransition() == nullptr)
        && (best_lrcost_option == nullptr
            || fuzzyLess(p->bufferCost(), best_lrcost))) {
      best_lrcost = p->bufferCost();
      best_lrcost_option = p;
    }
  }
  if (best_lrcost_option) return best_lrcost_option;
  if (best_slack_option) return best_slack_option;
  return nullptr;
}

void
LrRebuffer::initGlobalPreamble(sta::dbSta *sta, rsz::Resizer *resizer)
{
  sta->checkCapacitanceLimitPreamble();
  sta->checkSlewLimitPreamble();
  sta->checkFanoutLimitPreamble();
  resizer->resizePreamble();
}

void
LrRebuffer::init()
{
  // Per-instance init; global preamble must have been called once in serial
  // via LrRebuffer::initGlobalPreamble() before this runs.
  logger_ = resizer_->logger_;
  dbStaState::init(resizer_->sta_);
  db_network_ = resizer_->db_network_;
  estimate_parasitics_ = resizer_->estimate_parasitics_;
  resizer_max_wire_length_
      = resizer_->metersToDbu(resizer_->findMaxWireLength());

  buffer_sizes_.clear();
  for (auto cell : resizer_->buffer_fast_sizes_) {
    sta::LibertyPort *in, *out;
    cell->bufferPorts(in, out);
    buffer_sizes_.push_back(BufferSize{
        cell,
        FixedDelay(out->intrinsicDelay(sta_), resizer_),
        /*margined_max_cap=*/0.0f,
        out->driveResistance(),
    });
  }
  std::ranges::sort(buffer_sizes_, [=](BufferSize a, BufferSize b) {
    return bufferCin(a.cell) < bufferCin(b.cell);
  });
  buffer_sizes_index_.clear();
  for (auto& size : buffer_sizes_) {
    buffer_sizes_index_[size.cell] = &size;
  }

  arc_delay_calc_ = eval_ctx_->arc_delay_calc;

  sta::Corner *corner = corners_->findCorner("default");
  if (corner) {
    initOnCorner(corner);
  } else {
    printf("LrRebuffer::init: Warning: 'default' corner not found\n");
  }
}

void
LrRebuffer::annotateLoadLMs(PtVertex &drvr_pt_vertex, const BnetPtr& tree)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  sta::Vertex *root_vertex = drvr_pt_vertex.vertex();
  // Map from load pin to its LM vector
  std::map<const sta::Pin*, std::vector<float>> load_pin_lm_map;
  
  // First pass: collect LM vectors from all wire edges from driver to loads.
  // Use STA graph edges (not PtGraph) to cover ALL load pins, including those
  // filtered out by searchThru/searchFrom during PtGraph construction.
  sta::Vertex *sta_drvr = drvr_pt_vertex.vertex();
  if (!sta_drvr) return;
  sta::VertexOutEdgeIterator out_edge_iter(sta_drvr, graph_);
  while (out_edge_iter.hasNext()) {
    sta::Edge *edge = out_edge_iter.next();
    if (!edge->isWire()) continue;

    sta::Vertex *to_vertex = edge->to(graph_);
    const sta::Pin *load_pin = to_vertex->pin();

    int lmVecSize = sta::TimingArcSet::wireArcCount() * graph_->apCount();

    LMValue *load_lms = edge->arcLms();
    if (load_lms == nullptr)
      continue;

    std::vector<float> lmVec(load_lms, load_lms + lmVecSize);
    load_pin_lm_map[load_pin] = std::move(lmVec);
  }

  // Second pass: traverse the buffered net tree and annotate LMs to load nodes
  visitTree(
      [&](auto& recurse, int level, const BnetPtr& node) -> int {
        switch (node->type()) {
          case BnetType::via:
          case BnetType::wire:
          case BnetType::buffer:
            return recurse(node->ref());
          case BnetType::junction:
            return recurse(node->ref()) + recurse(node->ref2());
          case BnetType::load: {
            // Annotate the LMs of the load directly to the BufferedNet node
            const sta::Pin* load_pin = node->loadPin();
            auto it = load_pin_lm_map.find(load_pin);
            if (it != load_pin_lm_map.end()) {
              node->setLms(it->second);  // Copy LM vector to the node
              if (prune_debug_) {
                float lm_sum = 0.0f;
                for (float v : it->second) lm_sum += v;
                printf("[DBG-LM-LEAF] load_pin=%s lm_sum=%.4e\n",
                       network_->pathName(load_pin), lm_sum);
              }
            } else {
              printf("LrRebuffer::annotateLoadLMs: Warning: no LM found for load pin %s\n",
                     network_->pathName(load_pin));
              // Set zero LM vector as fallback
              int lmVecSize = sta::TimingArcSet::wireArcCount() * graph_->apCount();
              std::vector<float> zero_lm(lmVecSize, 0.0f);
              node->setLms(std::move(zero_lm));
            }
            return 1;
          }
          default:
            throw std::runtime_error(
                "annotateLoadLMs: unhandled BufferedNet type");
        }
      },
      tree);
}

// ---------------------------------------------------------------------------
// Sensitivity-based precheck for buffer insertion.
//
// S(v,e) = Λ_node(v)·[R_up(v)·(C_down(e) - C_buf_in)]
//        - Λ_edge(e)·[D_buf_int + R_buf·C_down(e)]  - γ·ΔP
//
// Design:
//   - Junction Λ_node = SUM of children Λ  (all paths benefit)
//   - Wire discretization: uses wire_length_step_ from bufferForTiming
//   - Bakoglu gate: 1{D_current > D_opt}, D_current from PtPiElmore
//   - R_drv: LibertyPort::driveResistance()  (TODO: differential R_eff)
// ---------------------------------------------------------------------------
float
LrRebuffer::computeNetSensitivity(const sta::Pin *drvr_pin,
                                   PtVertex &drvr_pt_vertex,
                                   float /*avg_delay*/, float /*avg_leakage*/)
{
  // Reference buffer: pick middle-sized from buffer_sizes_ (sorted by cin asc)
  if (buffer_sizes_.empty())
    return -std::numeric_limits<float>::infinity();
  const BufferSize &ref_buf = buffer_sizes_[buffer_sizes_.size() / 2];
  const float c_buf_in = bufferCin(ref_buf.cell);
  const float r_buf = ref_buf.driver_resistance;
  const float d_buf_int = ref_buf.intrinsic_delay.toSeconds();
  const float buf_leakage = local_sta_->cellAvgLeakage(ref_buf.cell);

  // Driver resistance (TODO: differential R_eff = Δdelay/ΔC)
  sta::LibertyPort *drvr_port = network_->libertyPort(drvr_pin);
  if (!drvr_port)
    return -std::numeric_limits<float>::infinity();
  const float r_drv = drvr_port->driveResistance();

  PtGraph *pt_graph = eval_ctx_->pt_graph;
  const sta::DcalcAPIndex ap_index = pt_graph->dcalcAnalysisPt()->index();
  const sta::DcalcAPIndex ap_count = graph_->apCount();

  // ---- Bakoglu gate: 1{D_current > D_opt} via PtPiElmore ----
  // env=off main reduce builds PtPiElmore for every net. env=on builds
  // only PtElmoreCeff; Bakoglu's formula still wants Pi parameters
  // (rpi / c1+c2 / per-load Elmore), so we JIT-build Pi for just this
  // driver via ensurePtPiElmore. Result is cached in pt_graph until
  // the next clearPtParasitics, so multiple rf accesses or repeated
  // Bakoglu hits on the same net are free.
  VertexId drvr_vid = drvr_pt_vertex.objectIdx();
  local_sta_->recomputeSinglePtParasitic(pt_graph, drvr_vid);

  float d_current = 0.0f;
  float r_eq = 0.0f;
  float c_eq = 0.0f;
  for (const sta::RiseFall *rf : sta::RiseFall::range()) {
    PtPiElmore *pt_pi = local_sta_->localParasitics()->ensurePtPiElmore(
        pt_graph, drvr_vid, rf, ap_index);
    if (!pt_pi)
      continue;
    for (const auto &load : pt_pi->loads())
      d_current = std::max(d_current, load.elmore);
    float c2, rpi, c1;
    pt_pi->piModel(c2, rpi, c1);
    r_eq = std::max(r_eq, rpi);
    c_eq = std::max(c_eq, c1 + c2);
  }
  d_current += r_drv * c_eq;

  // Bakoglu gate: skip nets whose delay is too small to benefit from buffering.
  // d_opt = k * sqrt(r_buf * c_buf_in * r_eq * c_eq) where k=2.5 is ideal.
  // Lower k allows more nets through; sensitivity score further ranks them.
  float d_opt = eval_ctx_->bakoglu_k * std::sqrt(r_buf * c_buf_in * r_eq * c_eq);
  if (d_current <= d_opt)
    return -std::numeric_limits<float>::infinity();

  // ---- Build BufferedNet tree and annotate LMs ----
  BnetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet)
    return -std::numeric_limits<float>::infinity();
  annotateLoadLMs(drvr_pt_vertex, bnet);

  // ---- Bottom-up: compute Λ_edge per subtree ----
  std::unordered_map<BufferedNet*, float> node_lambda;
  rsz::visitTree(
      [&](auto &recurse, int level, const BnetPtr &node) -> float {
        float lambda = 0.0f;
        switch (node->type()) {
          case BnetType::load: {
            const auto &lms = node->lms();
            if (!lms.empty()) {
              for (const sta::RiseFall *rf : sta::RiseFall::range()) {
                int idx = rf->index() * ap_count + ap_index;
                if (idx < static_cast<int>(lms.size()))
                  lambda += lms[idx];
              }
            }
            break;
          }
          case BnetType::wire:
          case BnetType::via:
            lambda = recurse(node->ref());
            break;
          case BnetType::junction:
            lambda = recurse(node->ref()) + recurse(node->ref2());
            break;
          default:
            break;
        }
        node_lambda[node.get()] = lambda;
        return lambda;
      },
      bnet);

  // ---- Top-down: accumulate R_up, evaluate S(v,e) ----
  float max_sensitivity = -std::numeric_limits<float>::infinity();
  const float power_penalty = eval_ctx_->swapCost(0.0f, buf_leakage);

  std::function<void(const BnetPtr&, float)> topDown =
      [&](const BnetPtr &node, float r_up) {
        switch (node->type()) {
          case BnetType::wire: {
            double wire_res_per_m, wire_cap_per_m;
            node->wireRC(corner_, resizer_, estimate_parasitics_,
                         wire_res_per_m, wire_cap_per_m);
            int wire_len = node->length();
            float l_meters = resizer_->dbuToMeters(wire_len);
            float seg_res_total = wire_res_per_m * l_meters;
            float seg_cap_total = wire_cap_per_m * l_meters;
            float child_cap = node->ref()->cap();
            float lambda_edge = node_lambda[node->ref().get()];

            // Discretize using wire_length_step_ (same as bufferForTiming)
            int n_steps = std::max(1, wire_len / wire_length_step_);
            for (int k = 0; k < n_steps; k++) {
              float frac = static_cast<float>(k) / n_steps;
              float r_up_k = r_up + seg_res_total * frac;
              float c_down_k = child_cap + seg_cap_total * (1.0f - frac);

              float decoupling = lambda_edge * r_up_k * (c_down_k - c_buf_in);
              float penalty_t = lambda_edge * (d_buf_int + r_buf * c_down_k);
              float s = eval_ctx_->swapCost(decoupling - penalty_t, 0.0f)
                        - power_penalty;
              max_sensitivity = std::max(max_sensitivity, s);
            }
            topDown(node->ref(), r_up + seg_res_total);
            break;
          }
          case BnetType::via: {
            double via_res = node->viaResistance(
                corner_, resizer_, estimate_parasitics_);
            topDown(node->ref(), r_up + via_res);
            break;
          }
          case BnetType::junction: {
            float lambda_left = node_lambda[node->ref().get()];
            float lambda_right = node_lambda[node->ref2().get()];
            float lambda_node = lambda_left + lambda_right;

            {
              float c_down = node->ref()->cap();
              float decoupling = lambda_node * r_up * (c_down - c_buf_in);
              float penalty_t = lambda_left * (d_buf_int + r_buf * c_down);
              float s = eval_ctx_->swapCost(decoupling - penalty_t, 0.0f)
                        - power_penalty;
              max_sensitivity = std::max(max_sensitivity, s);
            }
            {
              float c_down = node->ref2()->cap();
              float decoupling = lambda_node * r_up * (c_down - c_buf_in);
              float penalty_t = lambda_right * (d_buf_int + r_buf * c_down);
              float s = eval_ctx_->swapCost(decoupling - penalty_t, 0.0f)
                        - power_penalty;
              max_sensitivity = std::max(max_sensitivity, s);
            }
            topDown(node->ref(), r_up);
            topDown(node->ref2(), r_up);
            break;
          }
          case BnetType::load:
          default:
            break;
        }
      };

  topDown(bnet, r_drv);

  if (eval_ctx_->debug) {
    float root_lambda = node_lambda.count(bnet.get()) ? node_lambda[bnet.get()] : 0.0f;
    printf("[DBG-SENS] pin=%s sens=%.3e root_lm=%.3e r_drv=%.3e c_eq=%.3e "
           "d_buf_int=%.3e r_buf=%.3e c_buf_in=%.3e power_pen=%.3e\n",
           network_->name(drvr_pin), max_sensitivity, root_lambda,
           r_drv, c_eq, d_buf_int, r_buf, c_buf_in, power_penalty);
  }

  return max_sensitivity;
}

int
LrRebuffer::applyBufferingToDb()
{
  odb::dbNet* const db_net = db_network_->flatNet(drvr_pin_);
  int count = exportBufferTree(best_bnet_, db_network_->dbToSta(db_net), 1, nullptr, "rebuffer");
  if (count > 0) {
    persistBufferParasitics();
    writeLmsToGraph();
    writeTimingToGraph();
  }
  // Clean up virtual buffer vertices/edges left by bufferForTiming.
  if (!best_vinfo_.vertex_ids.empty()) {
    removeVirtualBuffer(best_vinfo_);
    best_vinfo_ = VirtualBufferInfo{};
  }
  return count;
}

void
LrRebuffer::persistBufferParasitics()
{
  if (!best_bnet_ || !drvr_pin_)
    return;

  auto persistNet = [&](const sta::Net *net) {
    if (!net) return;
    estimate_parasitics_->estimateWireParasiticNoDeleteNetwork(net);
  };

  using BnetType = rsz::BufferedNetType;
  std::function<void(const BufferedNetPtr&)> walkTree;
  walkTree = [&](const BufferedNetPtr& node) {
    if (!node) return;
    switch (node->type()) {
      case BnetType::buffer: {
        sta::Instance *buf_inst = node->bufInst();
        if (buf_inst) {
          sta::LibertyPort *in_port, *out_port;
          node->bufferCell()->bufferPorts(in_port, out_port);
          const sta::Pin *out_pin = network_->findPin(buf_inst, out_port);
          if (out_pin)
            persistNet(network_->net(out_pin));
        }
        walkTree(node->ref());
        break;
      }
      case BnetType::junction:
        walkTree(node->ref());
        walkTree(node->ref2());
        break;
      case BnetType::wire: case BnetType::via:
        walkTree(node->ref());
        break;
      default: break;
    }
  };
  walkTree(best_bnet_);

  // Also rebuild original driver's net (topology changed by buffer insertion).
  persistNet(network_->net(drvr_pin_));
}

void
LrRebuffer::rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex)
{
  best_bnet_ = nullptr;
  best_cost_ = std::numeric_limits<float>::max();
  best_vinfo_ = VirtualBufferInfo{};
  if (network_->isTopLevelPort(drvr_pin)) {
    printf("LrRebuffer::rebufferPin: Warning: rebuffering does not support top port as the driver pin: %s\n",
           network_->name(drvr_pin));
    return;
  }

  PtGraph *pt_graph = eval_ctx_->pt_graph;
  sta::LibertyCell *cur_lib_cell = pt_graph->refGate();
  drvr_port_ = cur_lib_cell->findLibertyPort(network_->portName(drvr_pin));
  drvr_pin_ = drvr_pin;
  sta::Net *net = network_->net(drvr_pin);
  odb::dbNet* const db_net = db_network_->flatNet(drvr_pin);
  if (net && drvr_port_ &&
      // Verilog connects by net name, so there is no way to distinguish the
      // net from the port.
      !hasTopLevelOutputPort(net)) {
    auto t_total_start = std::chrono::steady_clock::now();

    setPin(const_cast<sta::Pin*>(drvr_pin));
    auto t_setup_start = std::chrono::steady_clock::now();
    BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);

    if (!bnet) {
      printf("LrRebuffer::rebufferPin: Warning: unable to create buffered net for pin %s\n",
             network_->name(drvr_pin));
      return;
    }

    // Compute RAT and AAT of the local graph
    local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
    // localAnnotateLoadSlacks removed: slack annotation on BnetPtr nodes
    // is only used for debug prints, not cost computation.
    annotateLoadLMs(drvr_pt_vertex, bnet);
    auto t_setup_end = std::chrono::steady_clock::now();

    // Save VertexId — PtVertex references may be invalidated by vector
    // reallocation inside buildVirtualBuffer during evaluateOption.
    VertexId drvr_vid = drvr_pt_vertex.objectIdx();
    const bool allow_topology_rewrite = true;
    // Skip coarse iterations, go directly to precise evaluation
    // to debug whether buffer solutions can win with accurate local STA cost.
    auto t_iter_start = std::chrono::steady_clock::now();
    for (int i = 0; i < 3; ++i) { // Example loop, replace with actual iteration logic
      bool last_iteration = (i == 2);
      bnet = bufferForTimingLrf(drvr_vid, bnet, allow_topology_rewrite, /*last_iteration=*/last_iteration);
    }
    auto t_iter_end = std::chrono::steady_clock::now();
    (*eval_ctx_->runtime_map)["rebuffer_precise"]
        += std::chrono::duration<double>(t_iter_end - t_iter_start).count();
    if (!bnet && eval_ctx_->debug) {
      printf("LrRebuffer::rebufferPin: Warning: bufferForTiming failed for pin %s\n",
             network_->name(drvr_pin));
    }

    (*eval_ctx_->runtime_map)["rebuffer_setup"]
        += std::chrono::duration<double>(t_setup_end - t_setup_start).count();
    (*eval_ctx_->runtime_map)["rebuffer_total"]
        += std::chrono::duration<double>(std::chrono::steady_clock::now() - t_total_start).count();
    (*eval_ctx_->runtime_map)["rebuffer_pin_count"] += 1.0;

    if (!bnet) {
      return;
    }

    best_bnet_ = bnet;
  }
}

rsz::BufferedNetPtr
LrRebuffer::prepareBufferOptions(const sta::Pin *drvr_pin,
                                   PtVertex &drvr_pt_vertex)
{
  // Reset state — only remove virtual buffer if it belongs to the current
  // PtGraph.  Each instance gets a fresh PtGraph, so stale best_vinfo_ from
  // a previous instance must not be applied to the new graph.
  best_bnet_ = nullptr;
  best_cost_ = std::numeric_limits<float>::max();
  best_vinfo_ = VirtualBufferInfo{};

  if (network_->isTopLevelPort(drvr_pin))
    return nullptr;

  PtGraph *pt_graph = eval_ctx_->pt_graph;
  sta::LibertyCell *cur_lib_cell = pt_graph->refGate();
  drvr_port_ = cur_lib_cell->findLibertyPort(network_->portName(drvr_pin));
  drvr_pin_ = drvr_pin;
  sta::Net *net = network_->net(drvr_pin);
  if (!net || !drvr_port_ || hasTopLevelOutputPort(net))
    return nullptr;

  setPin(const_cast<sta::Pin*>(drvr_pin));

  // Step 1: Build Steiner tree (cell-independent)
  BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet)
    return nullptr;

  // Step 2: Annotate LMs on tree nodes (cell-independent)
  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  annotateLoadLMs(drvr_pt_vertex, bnet);

  // Return raw buffer tree with LM annotations.
  // No coarse filtering — let evaluateBufferOnCandidate use precise evaluation
  // to pick the best option (coarse was incorrectly biased against buffer solutions).
  return bnet;
}

void
LrRebuffer::evaluateBufferOnCandidate(sta::VertexId drvr_vid,
                                         const rsz::BufferedNetPtr &prepared_bnet)
{
  // Run 1 round of precise bufferForTiming on the current PtGraph state.
  // PtGraph should already reflect the resize candidate via
  // increAndGetLocalTimingCost before calling this.
  const bool allow_topology_rewrite = true;
  BufferedNetPtr result = bufferForTimingLrf(drvr_vid, prepared_bnet,
                                             allow_topology_rewrite,
                                             /*last_iteration=*/true);
  if (result) {
    best_bnet_ = result;
    // best_cost_ is set inside bufferForTiming → evaluateOption
  }
}

void
LrRebuffer::cleanupVirtualBuffer()
{
  if (!best_vinfo_.vertex_ids.empty()) {
    removeVirtualBuffer(best_vinfo_);
  }
  best_vinfo_ = VirtualBufferInfo{};
  best_bnet_ = nullptr;
  best_cost_ = std::numeric_limits<float>::max();
}


static std::optional<int> findWireLayer(BnetPtr node)
{
  while (node->type() != BnetType::wire && node->type() != BnetType::load
         && node->type() != BnetType::junction) {
    node = node->ref();
  }
  if (node->type() == BnetType::wire) {
    return {node->layer()};
  }
  return {};
}

static BnetPtr stripWireOnBnet(BnetPtr ptr)
{
  while (ptr->type() == BnetType::wire || ptr->type() == BnetType::via) {
    ptr = ptr->ref();
  }
  return ptr;
}

static BnetPtr stripWiresAndBuffersOnBnet(BnetPtr ptr)
{
  while (ptr->type() == BnetType::wire || ptr->type() == BnetType::buffer
         || ptr->type() == BnetType::via) {
    ptr = ptr->ref();
  }
  return ptr;
}

static const sta::RiseFallBoth* combinedTransition(const sta::RiseFallBoth* a,
                                                   const sta::RiseFallBoth* b)
{
  if (a == b) {
    return a;
  }
  if (a == nullptr) {
    return b;
  }
  if (b == nullptr) {
    return a;
  }
  return sta::RiseFallBoth::riseFall();
}

static BufferedNetPtr createBnetJunction(rsz::Resizer* resizer,
                                         const BufferedNetPtr& p,
                                         const BufferedNetPtr& q,
                                         odb::Point location)
{
  BufferedNetPtr junc = std::make_shared<BufferedNet>(
      BufferedNetType::junction, location, p, q, resizer);
  junc->setSlackTransition(
      combinedTransition(p->slackTransition(), q->slackTransition()));
  junc->setSlack(std::min(p->slack(), q->slack()));
  return junc;
}

// Find buffering choices with best delay LM sum and leakage
BnetPtr
LrRebuffer::bufferForTimingLrf(VertexId drvr_vertex_id,
                            const BnetPtr &tree,
                            bool allow_topology_rewrite,
                            bool last_iteration)
{
  sta::LibertyPort *strong_driver;
  {
    sta::LibertyPort* dummy;
    buffer_sizes_.back().cell->bufferPorts(dummy, strong_driver);
  }

  BnetSeq top_opts = visitTree(
    [&](auto& recurse, int level, const BnetPtr& node) -> BnetSeq {
        switch (node->type()) {
          case BnetType::via:
          case BnetType::buffer:
          case BnetType::wire: {
            int layer = -1;
            if (auto wire_layer = findWireLayer(node)) {
              layer = wire_layer.value();
            }
            BnetSeq opts = recurse(stripWiresAndBuffersOnBnet(node->ref()));
            odb::Point location
                = stripWiresAndBuffersOnBnet(node->ref())->location();

            const int full_wl
                = odb::Point::manhattanDistance(node->location(), location);
            if (full_wl > wire_length_step_ / 2) {
              // This is a long wire, allow for insertion of buffers at the
              // farther end
              insertBufferOptions(
                  opts, level, std::min(full_wl, wire_length_step_));
            } else {
              BnetSeq opts1 = opts;
              for (BnetPtr& opt : opts1) {
                opt = addWire(opt, node->location(), layer, level);
              }
              insertBufferOptions(opts1, level, 0);
              if (opts1.empty()) {
                // if generated options empty, start again but allow for
                // insertion of buffers at the farther end
                opts1 = opts;
                insertBufferOptions(opts1, level, full_wl);
                for (BnetPtr& opt : opts1) {
                  opt = addWire(opt, node->location(), layer, level);
                }
                insertBufferOptions(opts1, level, 0);
              }
              if (opts1.empty()) {
                // if generated options still empty, this is an internal error
                // of the algorithm (wire_length_step_ should have been chosen
                // to always allow a minimal size buffer to drive itself without
                // ERC)
                printf("LrRebuffer::bufferForTiming: Warning: Buffer pin %s: wire step options empty\n",
                       network_->name(pin_));
              }
              return opts1;
            }
          
            int round = 0;
            while (location != node->location()) {
              const int step = wire_length_step_;

              // move `location` towards `node->location()` by `step`
              int dx = node->location().x() - location.x();
              int dy = node->location().y() - location.y();

              if (abs(dx) + abs(dy) >= step) {
                const float ratio
                    = (float) abs(dx) / (float) (abs(dx) + abs(dy));
                const int dx_abs = std::min((int) (ratio * step), step);
                const int dy_abs = step - dx_abs;
                dx = dx > 0 ? dx_abs : -dx_abs;
                dy = dy > 0 ? dy_abs : -dy_abs;
              }

              location.addX(dx);
              location.addY(dy);

              const int remaining_wl
                  = odb::Point::manhattanDistance(node->location(), location);

              for (BnetPtr& opt : opts) {
                opt = addWire(opt, location, layer, level);
              }
              insertBufferOptions(opts, level, std::min(remaining_wl, step));

              if (opts.empty()) {
                printf("LrRebuffer::bufferForTiming: Warning: Buffer pin %s: wire step options empty at round %d\n",
                       network_->name(pin_), round);
              }
              round++;
            }
            return opts;
          }

          case BnetType::junction: {
            const BnetSeq& opts_left = recurse(node->ref());
            const BnetSeq& opts_right = recurse(node->ref2());

            // Full O(N×M) enumeration: for cost-sum DP, every (L_i, R_j)
            // contributes a candidate (cap = L.cap + R.cap, cost = L.cost +
            // R.cost). The original slack-style diagonal walk only visits
            // ~N+M pairs and misses non-diagonal (L_i, R_j) combinations
            // that may dominate on the (cap, cost) Pareto front.
            BnetSeq raw;
            raw.reserve(opts_left.size() * opts_right.size());

            if (prune_debug_) {
              printf("[DBG-PRUNE-JNC] lvl=%d node_loc=(%d,%d) left_n=%zu right_n=%zu (full N×M=%zu)\n",
                     level, node->location().x(), node->location().y(),
                     opts_left.size(), opts_right.size(),
                     opts_left.size() * opts_right.size());
              for (size_t j = 0; j < opts_left.size(); j++) {
                printf("[DBG-PRUNE-JNC]   L[%zu] bufs=%d cap=%.3f cost=%.3e\n",
                       j, bufferNum(opts_left[j]),
                       opts_left[j]->cap() * 1e15,
                       opts_left[j]->bufferCost());
              }
              for (size_t j = 0; j < opts_right.size(); j++) {
                printf("[DBG-PRUNE-JNC]   R[%zu] bufs=%d cap=%.3f cost=%.3e\n",
                       j, bufferNum(opts_right[j]),
                       opts_right[j]->cap() * 1e15,
                       opts_right[j]->bufferCost());
              }
            }

            for (size_t i = 0; i < opts_left.size(); i++) {
              for (size_t j = 0; j < opts_right.size(); j++) {
                const BnetPtr& l = opts_left[i];
                const BnetPtr& r = opts_right[j];

                bool rewrote = false;
                BnetPtr junc;

                if (allow_topology_rewrite) {
                  junc = attemptTopologyRewrite(node, l, r, INF);
                  if (junc) {
                    rewrote = true;
                    float junc_cost = junc->ref()->bufferCost()
                                    + junc->ref2()->bufferCost();
                    junc->setBufferCost(junc_cost);
                    junc->setLeakage(junc->ref()->leakage()
                                     + junc->ref2()->leakage());
                    auto merged_lms = mergeLmVectors(junc->ref()->lms(),
                                                     junc->ref2()->lms());
                    junc->setLms(std::move(merged_lms));
                  }
                }

                if (!rewrote) {
                  junc = createBnetJunction(resizer_, l, r, node->location());
                  float junc_cost = l->bufferCost() + r->bufferCost();
                  junc->setBufferCost(junc_cost);
                  junc->setLeakage(l->leakage() + r->leakage());
                  auto merged_lms = mergeLmVectors(l->lms(), r->lms());
                  junc->setLms(std::move(merged_lms));
                }

                bool within_fanout = (junc->fanout() <= fanout_limit_);
                if (prune_debug_) {
                  printf("[DBG-PRUNE-JNC]   PAIR L[%zu]+R[%zu] → junc[bufs=%d "
                         "cap=%.3f cost=%.3e fanout=%.0f] %s%s\n",
                         i, j, bufferNum(junc), junc->cap() * 1e15,
                         junc->bufferCost(), junc->fanout(),
                         rewrote ? "REWROTE " : "",
                         within_fanout ? "KEEP" : "PRUNE(fanout>limit)");
                }
                if (within_fanout) {
                  raw.push_back(std::move(junc));
                }
              }
            }

            // Pareto pruning on (cap, cost): both minimized.
            // Sort by ascending cap; an option survives iff its cost is
            // strictly less than the min cost among lower-cap options.
            // Result: ascending cap, descending cost.
            std::sort(raw.begin(), raw.end(),
                      [](const BnetPtr& a, const BnetPtr& b) {
                        if (a->cap() != b->cap())
                          return a->cap() < b->cap();
                        return a->bufferCost() < b->bufferCost();
                      });
            BnetSeq opts;
            opts.reserve(raw.size());
            // Sweep from smallest cap to largest cap, keeping options whose
            // cost is strictly lower than running min. Iterating low→high
            // cap, options with lower cap are seen first; subsequent (higher
            // cap) options survive only if they offer a strictly better
            // cost — i.e. they're not dominated by any lower-cap option.
            float min_cost = INF;
            for (const BnetPtr& opt : raw) {
              if (opt->bufferCost() < min_cost) {
                min_cost = opt->bufferCost();
                opts.push_back(opt);
              }
            }
            if (prune_debug_) {
              printf("[DBG-PRUNE-JNC]   Pareto: raw=%zu kept=%zu\n",
                     raw.size(), opts.size());
              for (size_t k = 0; k < opts.size(); k++) {
                printf("[DBG-PRUNE-JNC]     P[%zu] bufs=%d cap=%.3f cost=%.3e\n",
                       k, bufferNum(opts[k]), opts[k]->cap() * 1e15,
                       opts[k]->bufferCost());
              }
            }
            return opts;
          }

          case BnetType::load: {
            // Load node: initialize buffer cost to 0 (starting point)
            node->setBufferCost(0.0f);
            return {node};
          }
          
          default:
            printf("LrRebuffer::bufferForTiming: Error: unhandled BufferedNet type\n");
            return {};
        }
      },
      tree);

  if (top_opts.empty()) {
    printf("LrRebuffer::bufferForTiming: Warning: no buffering options generated for pin %s\n",
           network_->name(pin_));
  }

  // Select best option based on buffer cost
  float best_cost = INF;
  BnetPtr best_option = nullptr;
  int best_index = 0;
  int i = 1;

  PtGraph *pt_graph = eval_ctx_->pt_graph;
  float origial_slack = 0.0f;
  if (last_iteration) {
    local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
    origial_slack = eval_ctx_->use_sum_threshold
        ? local_sta_->localSlackOnSinks(pt_graph)
        : local_sta_->localWorstSlackOnSinks(pt_graph);
    if (eval_ctx_->debug)
      printf("[DBG-BFT] pin=%s top_opts=%zu origial_slack(%s)=%.3e last_iter=%d\n",
             network_->name(pin_), top_opts.size(),
             eval_ctx_->use_sum_threshold ? "sum" : "worst",
             origial_slack, last_iteration);
  }
  // Two-pass: first find no-buffer baseline delay_lm_sum, then compare
  float nobuf_delay_lm_sum = INF;
  float nobuf_cost = INF;
  float nobuf_leakage = 0.0f;
  int nobuf_opts_count = 0;
  int buf_opts_count = 0;
  if (last_iteration) {
    for (const BnetPtr& p : top_opts) {
      if (p->bufferCount() == 0) nobuf_opts_count++;
      else buf_opts_count++;
    }
    // Unified pass: evaluate all options (both nobuf and buf) under the
    // same metric (swapCost from evaluateOption). The slack gate inside
    // evaluateOption already rejects options that degrade local slack
    // (returns INF in that case), so a simple min-cost selection suffices.
    // This replaces the former pass-1 (nobuf by cost) + pass-2 (buf by
    // slack) split, which used inconsistent selection metrics.
    for (const BnetPtr& p : top_opts) {
      LMValue cost = evaluateOption(drvr_vertex_id, p, origial_slack);
      if (p->bufferCount() == 0
          && last_delay_lm_sum_ < nobuf_delay_lm_sum) {
        nobuf_delay_lm_sum = last_delay_lm_sum_;
        nobuf_cost = cost;
        nobuf_leakage = p->leakage();
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_option = p;
        best_index = i;
      }
      i++;
    }

    bool do_print = eval_ctx_->debug;
    if (do_print) {
      float nobuf_delay_part = eval_ctx_->PT_tradeoff * nobuf_delay_lm_sum / eval_ctx_->average_delay;
      float nobuf_leak_part = nobuf_leakage / eval_ctx_->average_leakage;
      printf("[DBG-TWOPASS] pin=%s top_opts=%zu nobuf=%d buf=%d "
             "nobuf_delay_lm=%.3e nobuf_leak=%.3e nobuf_delay_part=%.3e nobuf_leak_part=%.3e "
             "nobuf_ratio=%.2f nobuf_cost=%.3e orig_slack=%.3e\n",
             network_->name(pin_), top_opts.size(), nobuf_opts_count, buf_opts_count,
             nobuf_delay_lm_sum, nobuf_leakage,
             nobuf_delay_part, nobuf_leak_part,
             nobuf_leak_part > 0 ? nobuf_delay_part / nobuf_leak_part : 0.0f,
             nobuf_cost, origial_slack);
    }

  } else {
    // Non-last iteration: coarse evaluation (analytical cost only, no
    // virtual buffer rebuild).
    for (const BnetPtr& p : top_opts) {
      LMValue cost = evaluateOptionCoarse(drvr_vertex_id, p);
      if (cost < best_cost) {
        best_cost = cost;
        best_option = p;
        best_index = i;
      }
      i++;
    }
  }

  if (best_option) {
    // Count buffers in best option tree
    size_t buf_count = 0;
    visitTree(
      [&](auto& recurse, int level, const BnetPtr& node) -> int {
        switch (node->type()) {
          case BnetType::buffer: buf_count++; return recurse(node->ref());
          case BnetType::junction: return recurse(node->ref()) + recurse(node->ref2());
          case BnetType::wire: case BnetType::via: return recurse(node->ref());
          default: return 0;
        }
      }, best_option);
    if (eval_ctx_->debug) {
      printf("best option: %d cost=%.3e, slack=%.3e, cap=%.3e, fanout=%.0f, buffers=%zu\n",
             best_index, best_cost, best_option->slack().toSeconds(),
             best_option->cap(), best_option->fanout(), buf_count);
      fflush(stdout);
    }

    // Persist best cost for external callers (e.g. CombinedVisitor).
    if (last_iteration)
      best_cost_ = best_cost;

    if (last_iteration) {
      // Rebuild virtual buffer for best option and run local timing so that
      // PtGraph contains complete timing data (slew, arrival, required, arc
      // delay) on all virtual vertices/edges.  Keep the VirtualBufferInfo in
      // best_vinfo_ — it will be consumed by writeTimingToGraph after physical
      // insertion and cleaned up by removeVirtualBuffer in applyBufferingToDb.
      best_vinfo_ = buildVirtualBuffer(drvr_vertex_id, best_option);
      if (!best_vinfo_.failed) {
        pt_graph->topoSortVertices();
        buildSyntheticParasitics(drvr_vertex_id, best_option, best_vinfo_);
        local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
      }
    }
  }

  // Save all options for external analysis (TestRebuffer::probeAllOptions)
  if (last_iteration)
    last_top_opts_ = top_opts;

  return best_option;
}

LMValue
LrRebuffer::evaluateOptionCoarse(VertexId pt_vertex_id, const BnetPtr& option)
{
  float max_slew = 0.0f;
  float cell_delay_lm_sum = cellDelayLmSum(pt_vertex_id, option, max_slew);
  if (hasViolation(option, max_slew)) {
    return INF;
  }
  // bufferCost is now stored in normalized swapCost-compatible units
  // (PT × delay_lm / avg_delay + leakage / avg_leakage). Add the driver
  // cell's delay contribution with the same normalization. option->leakage()
  // is already folded into bufferCost by computeBufferAddedCost.
  float cell_normalized = eval_ctx_->PT_tradeoff * cell_delay_lm_sum
                              / eval_ctx_->average_delay;
  float cost = option->bufferCost() + cell_normalized;
  if (eval_ctx_->debug) {
    printf("[DBG-COARSE] pin=%s buffers=%d bufferCost=%.3e cellDelayLmSum=%.3e "
           "cell_norm=%.3e cost=%.3e avg_delay=%.3e avg_leak=%.3e\n",
           network_->name(pin_), option->bufferCount(), option->bufferCost(),
           cell_delay_lm_sum, cell_normalized, cost,
           eval_ctx_->average_delay, eval_ctx_->average_leakage);
  }
  return cost;
}

LMValue
LrRebuffer::evaluateOption(VertexId pt_vertex_id, const BnetPtr& option,
                           float original_slack)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  float total_cost = INF;
  float max_slew = 0.0f;
  float cell_delay_lm_sum = cellDelayLmSum(pt_vertex_id, option, max_slew);
  if (hasViolation(option, max_slew)) {
    return INF;  // Fast fail if option has any violation
  }

  // Virtual slack checkZ: build virtual sub-graph and run full local timing
  VirtualBufferInfo vinfo = buildVirtualBuffer(pt_vertex_id, option);
  if (vinfo.failed) {
    removeVirtualBuffer(vinfo);
    return INF;  // Fall back to analytical cost only
  }

  pt_graph->topoSortVertices();
  buildSyntheticParasitics(pt_vertex_id, option, vinfo);
  auto result = local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float delay_lm_sum = result.delay_lm_sum;
  last_delay_lm_sum_ = delay_lm_sum;
  float slack_after = eval_ctx_->use_sum_threshold
      ? local_sta_->localSlackOnSinks(pt_graph)
      : local_sta_->localWorstSlackOnSinks(pt_graph);
  last_slack_after_ = slack_after;

  // Multiplicative slack margin (same semantics as ParallelVisitor's
  // slack_margin_): for negative slack, margin > 1.0 relaxes the gate by
  // (margin-1)*|slack|. Default 1.0 = strict equality with original behavior.
  float thresh = original_slack * eval_ctx_->slack_margin;
  if (slack_after >= thresh) {
    total_cost = eval_ctx_->swapCost(delay_lm_sum, option->leakage());
  } else if (option->bufferCount() > 0 && eval_ctx_->debug) {
    float worst_after = local_sta_->localWorstSlackOnSinks(pt_graph);
    float sum_after_val = local_sta_->localSlackOnSinks(pt_graph);
    printf("[BUF-REJECT] pin=%s bufs=%d mode=%s gap=%.1fps | "
           "local_worst=%.1fps local_sum=%.1fps\n",
           network_->name(pin_), option->bufferCount(),
           eval_ctx_->use_sum_threshold ? "sum" : "worst",
           (slack_after - thresh) * 1e12,
           worst_after * 1e12, sum_after_val * 1e12);
  }

  if (eval_ctx_->debug) {
    if (option->bufferCount() == 0) {
      printf("[DBG-NOBUF-EVAL] pin=%s delay_lm=%.3e cost=%.3e "
             "slack_after=%.3e orig_slack=%.3e delta_slack=%.3e pass=%d\n",
             network_->name(pin_),
             delay_lm_sum, total_cost,
             slack_after, original_slack,
             slack_after - original_slack,
             slack_after >= thresh);
    }
    float delay_part = eval_ctx_->PT_tradeoff * delay_lm_sum / eval_ctx_->average_delay;
    float leak_part = option->leakage() / eval_ctx_->average_leakage;
    printf("[DBG-PRECISE] pin=%s buffers=%d delay_lm=%.3e leak=%.3e "
           "delay_part=%.3e leak_part=%.3e ratio=%.2f "
           "cost=%.3e slack_after=%.3e orig_slack=%.3e pass=%d\n",
           network_->name(pin_), option->bufferCount(),
           delay_lm_sum, option->leakage(),
           delay_part, leak_part,
           leak_part > 0 ? delay_part / leak_part : 0.0f,
           total_cost, slack_after, original_slack,
           slack_after >= thresh);
  }

  removeVirtualBuffer(vinfo);
  local_sta_->recomputeSinglePtParasitic(pt_graph, pt_vertex_id);
  return total_cost;
}

bool
LrRebuffer::hasViolation(const BnetPtr& option, sta::Slew slew)
{
  if (!loadSlewSatisfactory(drvr_port_, option)) return true;
  if (slew > drvr_pin_max_slew_ && option->cap() > drvr_load_high_water_mark_) {
    return true;
  }
  return false;
}

void 
LrRebuffer::insertBufferOptions(BnetSeq& opts,
                                int level,
                                int next_segment_wl)
{
  if (opts.empty()) {
    return;
  }

  BufferSize& strong_driver = buffer_sizes_.back();

  float best_area = INF;
  float best_cost = INF;  // Use buffer cost instead of slack

  // both `opts` and `buffer_sizes_` are ordered by ascending input capacitance
  BnetSeq new_opts;
  new_opts.reserve(opts.size() * 2);
  auto opts_iter = opts.begin();

  auto pass_through = [&](float threshold_cap) {
    // pass through non-redundant options with cap below `threshold_cap`
    for (; opts_iter != opts.end() && (*opts_iter)->cap() <= threshold_cap;
         opts_iter++) {
  BnetPtr& opt = *opts_iter;

      // Use the already computed buffer cost from recursive call
      float opt_cost = opt->bufferCost();

      // Keep option if it has better (smaller) buffer cost
      bool keep = (opt_cost < best_cost);
      bool can_drive = bufferSizeCanDriveLoad(strong_driver, opt, next_segment_wl);

      if (!can_drive) {
        keep = false;
      }

      if (prune_debug_) {
        const char *reason;
        if (!can_drive) reason = "PRUNE(no_drive)";
        else if (!keep)  reason = "PRUNE(cost>=best)";
        else             reason = "KEEP";
        printf("[DBG-PRUNE-PT] lvl=%d thr_cap=%.3f opt[bufs=%d cap=%.3f cost=%.3e] "
               "best_cost=%.3e → %s%s\n",
               level, threshold_cap * 1e15, bufferNum(opt),
               opt->cap() * 1e15, opt_cost, best_cost, reason,
               keep ? " new_best" : "");
      }

      if (keep) {
        new_opts.push_back(opt);
        best_cost = opt_cost;
        best_area = opt->area();
      }
    }
  };

  for (BufferSize buffer_size : buffer_sizes_) {
    sta::LibertyCell* buffer_cell = buffer_size.cell;
    sta::LibertyPort *in, *out;
    buffer_cell->bufferPorts(in, out);
    pass_through(in->capacitance());

  BnetPtr load_opt;
    FixedDelay load_opt_buffer_delay = FixedDelay::ZERO;
    float load_opt_total_cost = INF;

    auto it = (new_opts.empty() && opts_iter == opts.end()
               && opts_iter > opts.begin())
                  ? (opts_iter - 1)
                  : opts_iter;

    for (; it != opts.end(); it++) {
      BnetPtr& opt = *it;

      // Get the already computed buffer cost from the load option
      float opt_cost = opt->bufferCost();

      // Step 1: Fast filtering using intrinsic_delay (cheap approximation)
      float buffer_leakage = local_sta_->cellAvgLeakage(buffer_cell);

      // Use intrinsic_delay for quick filtering
      float intrinsic_delay_seconds = buffer_size.intrinsic_delay.toSeconds();
      float estimated_delta_cost = computeBufferAddedCost(intrinsic_delay_seconds,
                                                           buffer_leakage,
                                                           opt);
      float estimated_total_cost = opt_cost + estimated_delta_cost;

      bool estim_pass = (estimated_total_cost < best_cost);
      bool can_drive = bufferSizeCanDriveLoad(buffer_size, opt);

      // Only compute precise delay if this passes initial filter
      if (estim_pass && can_drive) {
        // Step 2: Precise delay calculation (expensive, only for candidates)
        sta::LibertyPort *in, *out;
        buffer_cell->bufferPorts(in, out);
        const float load_cap = opt->cap() + out->capacitance();

        // LRF does not track slackTransition on bnet nodes (annotateLoadSlacks
        // is removed) and therefore cannot use Rebuffer::bufferDelay which
        // relies on arrival_paths_ populated by annotateLoadSlacks. Compute
        // the gate delay directly via the PtGraph's dcalcAnalysisPt, taking
        // max(rise, fall) as the buffer_delay upper bound.
        const FixedDelay buffer_delay = computeBufferGateDelay(buffer_cell, load_cap);
        const float buffer_delay_seconds = buffer_delay.toSeconds();

        // Recalculate cost with precise delay
        float precise_delta_cost = computeBufferAddedCost(buffer_delay_seconds,
                                                           buffer_leakage,
                                                           opt);
        float precise_total_cost = opt_cost + precise_delta_cost;

        bool precise_pass = (precise_total_cost < best_cost);

        if (prune_debug_) {
          printf("[DBG-PRUNE-BV] lvl=%d cell=%s load[bufs=%d cap=%.3f cost=%.3e] "
                 "estim=%.3e precise=%.3e best_cost=%.3e → %s\n",
                 level, buffer_cell->name(), bufferNum(opt),
                 opt->cap() * 1e15, opt_cost,
                 estimated_total_cost, precise_total_cost, best_cost,
                 precise_pass ? "ACCEPT new_best" : "REJECT(precise>=best)");
        }

        // Final check with precise cost
        if (precise_pass) {
          load_opt = opt;
          load_opt_buffer_delay = buffer_delay;
          load_opt_total_cost = precise_total_cost;
          best_cost = precise_total_cost;
          best_area = opt->area() + buffer_cell->area();
        }
      } else if (prune_debug_) {
        const char *reason = !can_drive ? "REJECT(no_drive)"
                                        : "REJECT(estim>=best)";
        printf("[DBG-PRUNE-BV] lvl=%d cell=%s load[bufs=%d cap=%.3f cost=%.3e] "
               "estim=%.3e best_cost=%.3e → %s\n",
               level, buffer_cell->name(), bufferNum(opt),
               opt->cap() * 1e15, opt_cost,
               estimated_total_cost, best_cost, reason);
      }
    }

    if (load_opt) {
      BnetPtr z = std::make_shared<rsz::BufferedNet>(BnetType::buffer,
                                                     load_opt->location(),
                                                     buffer_cell,
                                                     load_opt,
                                                     corner_,
                                                     resizer_,
                                                     estimate_parasitics_);
      z->setSlack(load_opt->slack() - load_opt_buffer_delay);  // Still maintain slack for debugging
      z->setSlackTransition(load_opt->slackTransition());
      z->setDelay(load_opt_buffer_delay);
      
      // Set the total buffer cost for this buffer option
      z->setBufferCost(load_opt_total_cost);
      z->setLeakage(load_opt->leakage() + local_sta_->cellAvgLeakage(buffer_cell));

      // Propagate LMs through buffer
      propagateLmsThroughBuffer(z, load_opt);

      new_opts.push_back(std::move(z));
    }
  }
  pass_through(INF);

  new_opts.swap(opts);
}

float
LrRebuffer::cellDelayLmSum(VertexId pt_vertex_id,
                           const BnetPtr& load_opt,
                           float &max_slew)
{
  max_slew = -INF;
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  float delay_lm_sum = 0.0f;
  sta::DcalcAnalysisPt *dcalc_pt = pt_graph->dcalcAnalysisPt();
  sta::DcalcAPIndex ap_index = dcalc_pt->index();
  float output_cap = load_opt->cap();
  PtVertexInEdgeIterator in_edge_iter(pt_vertex_id, pt_graph);
  while (in_edge_iter.hasNext()) {
    PtEdge &pt_edge = in_edge_iter.next();
    sta::Edge *edge = pt_edge.edge();
    auto *lms = edge->arcLms();
    if (lms == nullptr) {
      PtVertex &v = pt_graph->ptVertex(pt_vertex_id);
      printf("LrRebuffer::cellDelayLmSum: Warning: edge from vertex %s has no LM values\n",
             v.vertex() ? v.vertex()->to_string(graph_).c_str() : "virtual");
      continue;
    }
    PtVertex &pt_from_vertex = pt_graph->ptVertex(pt_edge.ptFromId());
    for (sta::TimingArc *arc : edge->timingArcSet()->arcs()) {
      const sta::RiseFall *rf = arc->fromEdge()->asRiseFall();
      sta::Slew in_slew = pt_graph->slew(pt_from_vertex, rf, ap_index);
      sta::LoadPinIndexMap load_pin_index_map(network_);
      auto dcalc_result = arc_delay_calc_->gateDelay(nullptr,
                                                    arc,
                                                    in_slew,
                                                    output_cap,
                                                    nullptr,
                                                    load_pin_index_map,
                                                    dcalc_pt);
      sta::Delay arc_delay = dcalc_result.gateDelay();
      int lm_index = lmIndex(arc, ap_index, graph_->apCount());
      LMValue lm = lms[lm_index];
      delay_lm_sum += arc_delay * lm;
      if (dcalc_result.drvrSlew() > max_slew) {
        max_slew = dcalc_result.drvrSlew();
      }
    }
  }
  return delay_lm_sum;
}

// LRF-local replacement for Rebuffer::bufferDelay.
// The base class implementation uses arrival_paths_[rf->index()]->dcalcAnalysisPt,
// but LRF doesn't call annotateLoadSlacks (which populates arrival_paths_),
// so passing any rf there crashes. Instead, pull the dcalcAnalysisPt from the
// PtGraph (the local timing graph) and compute gate_delays via resizer_.
// Returns max(rise_delay, fall_delay) as a nonzero upper bound.
rsz::FixedDelay
LrRebuffer::computeBufferGateDelay(sta::LibertyCell *buffer_cell,
                                   float load_cap)
{
  sta::DcalcAnalysisPt *dcalc_ap = eval_ctx_->pt_graph->dcalcAnalysisPt();
  sta::LibertyPort *input, *output;
  buffer_cell->bufferPorts(input, output);
  sta::ArcDelay gate_delays[sta::RiseFall::index_count];
  sta::Slew slews[sta::RiseFall::index_count];
  resizer_->gateDelays(output, load_cap, dcalc_ap, gate_delays, slews);
  rsz::FixedDelay delay = rsz::FixedDelay::ZERO;
  for (auto rf : sta::RiseFall::range()) {
    delay = std::max<rsz::FixedDelay>(
        delay, rsz::FixedDelay(gate_delays[rf->index()], resizer_));
  }
  return delay;
}

// Compute the delta (added) cost when inserting a buffer
// Cost = delay_LM_sum + leakage
float
LrRebuffer::computeBufferAddedCost(float buffer_delay_seconds,
                                    float buffer_leakage,
                                    const BnetPtr& load_opt)
{
  sta::DcalcAPIndex ap_index = eval_ctx_->pt_graph->dcalcAnalysisPt()->index();
  // Part 1: Calculate buffer delay × LM contribution
  float buffer_delta_delay_lm = 0.0f;
  const auto& load_lms = load_opt->lms();

  float rise_lm = 0.0f, fall_lm = 0.0f;
  if (!load_lms.empty()) {
    // Buffer delay × output LM (assume output LM ≈ load LM)
    for (const sta::RiseFall *rf : sta::RiseFall::range()) {
      int lm_index = rf->index() * graph_->apCount() + ap_index;
      buffer_delta_delay_lm += buffer_delay_seconds * load_lms[lm_index];
      if (rf->index() == 0) rise_lm = load_lms[lm_index];
      else fall_lm = load_lms[lm_index];
    }
  }
  // Part 2: Normalized LR cost — matches swapCost() semantics so that
  // bufferCost (used in Pareto pruning) and swapCost (used in final
  // evaluation) share the same units. Without this normalization,
  // raw leakage (~1e-8 W for large buffers) overwhelms raw delay×LM
  // (~3e-10 s·LM), biasing DP toward small-cell / low-leakage options
  // that have large delay — which matches the v79 probe finding that
  // RSZ's large-buffer bnet scored 20× worse in analytical bufferCost
  // than LRF's small-buffer options, but 16% better in real swapCost.
  float total_cost = eval_ctx_->PT_tradeoff * buffer_delta_delay_lm
                         / eval_ctx_->average_delay
                   + buffer_leakage / eval_ctx_->average_leakage;

  if (prune_debug_) {
    float vec_sum = 0.0f;
    for (float v : load_lms) vec_sum += v;
    printf("[DBG-LM-COST] buf_delay=%.2eps rise_lm=%.4e fall_lm=%.4e "
           "used_sum(rise+fall)=%.4e vec_sum=%.4e → delta_dly_lm=%.3e + "
           "leak=%.3e = total_delta=%.3e\n",
           buffer_delay_seconds * 1e12, rise_lm, fall_lm,
           rise_lm + fall_lm, vec_sum,
           buffer_delta_delay_lm, buffer_leakage, total_cost);
  }

  if (eval_ctx_->debug) {
    float norm_delay = eval_ctx_->PT_tradeoff * buffer_delta_delay_lm / eval_ctx_->average_delay;
    float norm_leak = buffer_leakage / eval_ctx_->average_leakage;
    printf("[DBG-ADDCOST] buf_delay_s=%.3e delta_delay_lm=%.3e leak=%.3e "
           "raw_total=%.3e | norm_delay=%.3e norm_leak=%.3e norm_ratio=%.2f\n",
           buffer_delay_seconds, buffer_delta_delay_lm, buffer_leakage,
           total_cost, norm_delay, norm_leak,
           norm_leak > 0 ? norm_delay / norm_leak : 0.0f);
  }

  return total_cost;
}

// Add wire segment and update delay LM sum
BnetPtr
LrRebuffer::addWire(const BnetPtr& p,
                    odb::Point wire_end,
                    int wire_layer,
                    int level)
{
  // Create wire node
  BnetPtr z = std::make_shared<rsz::BufferedNet>(BnetType::wire,
                                                 wire_end,
                                                 wire_layer,
                                                 p,
                                                 corner_,
                                                 resizer_,
                                                 estimate_parasitics_);

  // Calculate wire resistance and capacitance
  double layer_res, layer_cap;
  z->wireRC(corner_, resizer_, estimate_parasitics_, layer_res, layer_cap);
  double wire_length = resizer_->dbuToMeters(z->length());
  double wire_res = wire_length * layer_res;
  double wire_cap = wire_length * layer_cap;
  
  // Calculate wire delay using Elmore delay model
  FixedDelay wire_delay = FixedDelay(wire_res * (wire_cap / 2 + p->cap()), resizer_);

  // Set delay for the wire segment
  z->setDelay(wire_delay);
  
  // Update slack (maintain for debugging/compatibility)
  z->setSlack(p->slack() - wire_delay);
  z->setSlackTransition(p->slackTransition());

  // Wire passes through LMs from the load side
  z->setLms(p->lms());

  // Calculate wire's contribution to buffer cost (delay × LM)
  const auto& lms = p->lms();
  float wire_delta_cost = computeBufferAddedCost(wire_delay.toSeconds(), 0.0f, p);  // Leakage is 0 for wire.

  // Update total buffer cost: previous cost + wire's delta
  float total_cost = p->bufferCost() + wire_delta_cost;
  z->setBufferCost(total_cost);
  z->setLeakage(p->leakage());

  if (prune_debug_) {
    float lm_sum = 0.0f;
    for (float v : lms) lm_sum += v;
    printf("[DBG-LM-WIRE] lvl=%d len_um=%d wire_delay=%.2eps p_cap=%.3f "
           "lm_sum=%.4e wire_delta=%.3e z_cost=%.3e\n",
           level, z->length(), wire_delay.toSeconds() * 1e12,
           p->cap() * 1e15, lm_sum, wire_delta_cost, total_cost);
  }

  return z;
}

// Propagate LMs through a buffer node
void
LrRebuffer::propagateLmsThroughBuffer(BnetPtr& buffer_node,
                                      const BnetPtr& load_opt)
{
  // For now, simply copy the load's LM to the buffer output
  // In a more sophisticated version, you'd compute the buffer's input LM
  // based on the output LM and the buffer's timing characteristics

  const auto& load_lms = load_opt->lms();
  if (!load_lms.empty()) {
    buffer_node->setLms(load_lms);
  }
  if (prune_debug_) {
    float lm_sum = 0.0f;
    for (float v : load_lms) lm_sum += v;
    printf("[DBG-LM-BUF] propagate through buffer: load_lm_sum=%.4e "
           "(buffer output LM = load LM, unchanged)\n", lm_sum);
  }
}

// Merge two LM vectors (for junction nodes)
std::vector<float>
LrRebuffer::mergeLmVectors(const std::vector<float>& lm1, 
                           const std::vector<float>& lm2)
{
  size_t size = std::max(lm1.size(), lm2.size());
  if (lm1.size() != lm2.size()) {
    printf("LrRebuffer::mergeLmVectors: Warning: LM vector size mismatch (%zu vs %zu), merging with zero-padding\n",
           lm1.size(), lm2.size());
    fflush(stdout);
  }
  std::vector<float> merged(size, 0.0f);

  for (size_t i = 0; i < lm1.size(); i++) {
    merged[i] += lm1[i];
  }
  for (size_t i = 0; i < lm2.size(); i++) {
    merged[i] += lm2[i];
  }

  if (prune_debug_) {
    float s1 = 0.0f, s2 = 0.0f, sm = 0.0f;
    for (float v : lm1) s1 += v;
    for (float v : lm2) s2 += v;
    for (float v : merged) sm += v;
    printf("[DBG-LM-MRG] lm1_sum=%.4e lm2_sum=%.4e → merged_sum=%.4e\n",
           s1, s2, sm);
  }

  return merged;
}

// After exportBufferTree physically inserts buffers into the real graph,
// walk the BnetPtr tree in lockstep with the real graph and write LMs from
// each BnetPtr node to the corresponding real edge, so that KKT conditions
// are satisfied without an extra lmUpdate pass.
//
// Mapping (BnetPtr → real graph edge → LM source):
//   load   → drvr_vertex → loadPin wire edge     : load->lms()
//   junction → no new edge; children share same drvr_vertex
//   buffer → drvr_vertex → buf_input wire edge    : buffer->lms()
//            buf_input → buf_output gate edge      : buffer->lms()
//            buf_output becomes new drvr_vertex for subtree
//   wire/via → transparent, follow through to ref()
void
LrRebuffer::writeLmsToGraph()
{
  if (!best_bnet_ || !drvr_pin_)
    return;

  sta::Graph *graph = graph_;
  const size_t ap_count = graph->apCount();
  const size_t wire_lm_size = sta::TimingArcSet::wireArcCount() * ap_count;

  // Helper: copy LM vector into a real edge's arcLms array.
  auto writeLms = [](sta::Edge *edge, const std::vector<float>& src,
                     size_t max_size) {
    LMValue *dst = edge->arcLms();
    if (!dst || src.empty())
      return;
    size_t n = std::min(src.size(), max_size);
    for (size_t i = 0; i < n; i++)
      dst[i] = src[i];
  };

  std::function<void(sta::Vertex*, const BnetPtr&)> writeSubtree;
  using BnetType = rsz::BufferedNetType;

  writeSubtree = [&](sta::Vertex *drvr_vertex, const BnetPtr& tree) {
    if (!drvr_vertex || !tree)
      return;

    switch (tree->type()) {
      case BnetType::wire:
      case BnetType::via:
        writeSubtree(drvr_vertex, tree->ref());
        break;

      case BnetType::load: {
        const sta::Pin *load_pin = tree->loadPin();
        if (!load_pin || tree->lms().empty())
          break;
        sta::VertexOutEdgeIterator out_iter(drvr_vertex, graph);
        while (out_iter.hasNext()) {
          sta::Edge *edge = out_iter.next();
          if (!edge->role()->isWire())
            continue;
          if (edge->to(graph)->pin() == load_pin) {
            writeLms(edge, tree->lms(), wire_lm_size);
            break;
          }
        }
        break;
      }

      case BnetType::junction:
        writeSubtree(drvr_vertex, tree->ref());
        writeSubtree(drvr_vertex, tree->ref2());
        break;

      case BnetType::buffer: {
        // Use the instance recorded by exportBufferTree for exact matching.
        sta::Instance *buf_inst = tree->bufInst();
        if (!buf_inst)
          break;

        sta::LibertyCell *buf_cell = tree->bufferCell();
        sta::LibertyPort *input_port, *output_port;
        buf_cell->bufferPorts(input_port, output_port);
        const sta::Pin *buf_in_pin = nullptr;
        const sta::Pin *buf_out_pin = nullptr;
        sta::InstancePinIterator *pin_iter = network_->pinIterator(buf_inst);
        while (pin_iter->hasNext()) {
          const sta::Pin *pin = pin_iter->next();
          if (network_->direction(pin)->isInput())
            buf_in_pin = pin;
          else if (network_->direction(pin)->isOutput())
            buf_out_pin = pin;
        }
        delete pin_iter;

        if (!buf_in_pin || !buf_out_pin)
          break;

        // 1. Wire edge: drvr_vertex → buf_input — write merged downstream LMs.
        sta::VertexOutEdgeIterator out_iter(drvr_vertex, graph);
        while (out_iter.hasNext()) {
          sta::Edge *edge = out_iter.next();
          if (!edge->role()->isWire())
            continue;
          if (edge->to(graph)->pin() == buf_in_pin) {
            writeLms(edge, tree->lms(), wire_lm_size);
            break;
          }
        }

        // 2. Gate edge: buf_input → buf_output — same LMs (positive-unate
        //    buffer arcs have same index layout as wire rise/fall).
        sta::Vertex *buf_out_vertex = graph->pinDrvrVertex(buf_out_pin);
        if (buf_out_vertex && !tree->lms().empty()) {
          sta::VertexInEdgeIterator in_iter(buf_out_vertex, graph);
          while (in_iter.hasNext()) {
            sta::Edge *gate_edge = in_iter.next();
            if (gate_edge->role()->isWire())
              continue;
            size_t gate_lm_size
                = gate_edge->timingArcSet()->arcCount() * ap_count;
            writeLms(gate_edge, tree->lms(), gate_lm_size);
          }
        }

        // 3. Recurse into subtree with buf_output as new driver.
        if (buf_out_vertex) {
          writeSubtree(buf_out_vertex, tree->ref());
        }
        break;
      }
    }
  };

  sta::Vertex *drvr_vertex = graph->pinDrvrVertex(drvr_pin_);
  if (drvr_vertex) {
    writeSubtree(drvr_vertex, best_bnet_);
  }
}

// Write timing data (slew, arrival/required paths) from PtGraph virtual
// buffer vertices to the real graph.  This mirrors the tree walk order of
// buildVirtualBuffer so that best_vinfo_.vertex_ids[vi] and
// best_vinfo_.edge_ids[ei] correspond to the same BnetPtr nodes.
// Edge delays are NOT written — downstream LocalSta recomputes them from
// slew and load cap.
//
// For each buffer in the BnetPtr tree:
//   vertex_ids consumed: buf_in, buf_out  (2 per buffer)
//   edge_ids consumed:   wire_in, gate    (2 per buffer, consumed but not used)
// For each load:
//   edge_ids consumed:   wire_to_load     (1 per load, consumed but not used)

// Initialize a newly-created STA vertex (from buffer insertion) with timing
// data from the corresponding PtGraph virtual vertex.  New STA vertices have
// no tag group and no paths; this copies tag_index, arrival, required from
// the PtVertex paths so that subsequent PtGraph construction on adjacent nets
// sees valid timing data instead of garbage.
// Find the upstream (driver-side) sta::Vertex's global TagGroup.  Used for
// Option-B writeback of buffer-inserted vertices whose pt_vertex carries a
// local-encoded TG index — we cannot safely register that local TG into the
// global tag_group_set_ (findTagGroup writes the unlocked-read set, racing
// against concurrent parallel readers), so we borrow whatever global TG the
// upstream vertex already has and tag-match copy the local arrivals into the
// matching slots.  Returns nullptr if no in-edge or no upstream TG exists.
static sta::TagGroup *
findUpstreamGlobalTagGroupImpl(sta::Vertex *sta_vertex,
                               sta::Graph *graph,
                               sta::Search *search)
{
  sta::VertexInEdgeIterator edge_iter(sta_vertex, graph);
  while (edge_iter.hasNext()) {
    sta::Edge *edge = edge_iter.next();
    sta::Vertex *from_v = edge->from(graph);
    if (!from_v)
      continue;
    sta::TagGroup *tg = search->tagGroup(from_v);
    if (tg)
      return tg;
  }
  return nullptr;
}

void
LrRebuffer::initNewStaVertexPaths(const PtVertex &pt_vertex,
                                  sta::Vertex *sta_vertex)
{
  sta::Path *pt_paths = pt_vertex.paths();
  if (!pt_paths)
    return;
  uint32_t encoded = static_cast<uint32_t>(pt_vertex.tagGroupIndex());

  // Local-encoded tag groups (minted by LocalArrivalVisitor C1b for virtual
  // buffer vertices): borrow the upstream sta::Vertex's existing global TG
  // as the writeback layout and tag-match copy local arrivals into matching
  // slots.  Tags present only in the local TG (typically CRPR variants
  // introduced by local-no-derate) are dropped — acceptable for buffer cells
  // which do not change clock domains.  No findTagGroup write, so no race
  // against concurrent unlocked readers in parallel arrival workers.
  if (PtGraph::isLocalTagGroupIndex(encoded)) {
    sta::TagGroup *layout =
        findUpstreamGlobalTagGroupImpl(sta_vertex, graph_, search_);
    if (!layout)
      return;
    PtGraph *pt_graph_local = eval_ctx_->pt_graph;
    if (!pt_graph_local)
      return;
    sta::TagGroup *local_tg =
        pt_graph_local->resolveTagGroup(static_cast<int>(encoded));
    if (!local_tg)
      return;

    size_t path_count = layout->pathCount();
    sta::Path *sta_paths = graph_->makePaths(sta_vertex, path_count);

    // Init each slot per the global layout — every slot gets a valid Tag*
    // and a sentinel arrival.  Matching slots below overwrite arrival/required.
    sta::Arrival init_arr = sta::delayInitValue(sta::MinMax::max());
    for (auto const& entry : *layout->pathIndexMap()) {
      sta::Tag *tag = entry.first;
      size_t slot = entry.second;
      sta_paths[slot].init(sta_vertex, tag, init_arr, this);
    }

    // Tag-match copy from local_tg paths into the global layout's slots.
    for (auto const& entry : *local_tg->pathIndexMap()) {
      sta::Tag *tag = entry.first;
      size_t local_slot = entry.second;
      size_t global_slot;
      bool exists;
      layout->pathIndex(tag, global_slot, exists);
      if (exists) {
        sta_paths[global_slot].setArrival(pt_paths[local_slot].arrival());
        sta_paths[global_slot].setRequired(pt_paths[local_slot].required());
      }
    }
    sta_vertex->setTagGroupIndex(layout->index());
    layout->incrRefCount();
    return;
  }

  PtGraph *pt_graph = eval_ctx_->pt_graph;
  sta::TagGroup *pt_tg = pt_graph ? pt_graph->tagGroup(pt_vertex)
                                  : search_->tagGroup(pt_vertex.tagGroupIndex());
  if (!pt_tg)
    return;

  // If the STA vertex already has matching paths, just update arrival/required.
  sta::TagGroup *sta_tg = search_->tagGroup(sta_vertex);
  if (sta_tg && sta_tg->index() == pt_tg->index()) {
    sta::Path *sta_paths = sta_vertex->paths();
    if (sta_paths) {
      size_t count = pt_tg->pathCount();
      for (size_t i = 0; i < count; i++) {
        sta_paths[i].setArrival(pt_paths[i].arrival());
        sta_paths[i].setRequired(pt_paths[i].required());
      }
      return;
    }
  }

  // New vertex: allocate paths and initialize from PtVertex data.
  size_t path_count = pt_tg->pathCount();
  sta::Path *sta_paths = graph_->makePaths(sta_vertex, path_count);
  for (size_t i = 0; i < path_count; i++) {
    sta::Tag *tag = search_->tag(pt_paths[i].tagIndex(this));
    sta_paths[i].init(sta_vertex, tag, pt_paths[i].arrival(), this);
    // Path::init sets required = 0.0.  Required on buffer vertices is not
    // needed: sink required is unchanged by upstream buffer insertion, and
    // the next global findRequireds() will compute correct values.
  }
  sta_vertex->setTagGroupIndex(pt_tg->index());
  pt_tg->incrRefCount();
}

void
LrRebuffer::writeTimingToGraph()
{
  if (!best_bnet_ || !drvr_pin_ || best_vinfo_.failed)
    return;

  PtGraph *pt_graph = eval_ctx_->pt_graph;
  sta::Graph *sta_graph = graph_;

  // Indices into best_vinfo_ arrays, consumed in tree-walk order.
  size_t vi = 0;  // vertex index
  size_t ei = 0;  // edge index

  using BnetType = rsz::BufferedNetType;
  std::function<void(sta::Vertex*, const BnetPtr&)> walkTree;

  walkTree = [&](sta::Vertex *real_drvr, const BnetPtr &tree) {
    if (!real_drvr || !tree)
      return;

    switch (tree->type()) {
      case BnetType::wire:
      case BnetType::via:
        walkTree(real_drvr, tree->ref());
        break;

      case BnetType::load: {
        // Consume the wire edge ID (keep indices in sync with buildVirtualBuffer)
        if (ei >= best_vinfo_.edge_ids.size()) {
          printf("Warning: writeTimingToGraph: edge index desync at load "
                 "(ei=%zu >= edge_ids.size()=%zu); aborting tree walk\n",
                 ei, best_vinfo_.edge_ids.size());
          fflush(stdout);
          break;
        }
        EdgeId pt_wire_eid = best_vinfo_.edge_ids[ei++];

        const sta::Pin *load_pin = tree->loadPin();
        if (!load_pin)
          break;

        // Update load vertex slew + paths
        sta::Vertex *load_vertex = sta_graph->pinLoadVertex(load_pin);
        if (load_vertex) {
          VertexId pt_load_vid = pt_graph->edge(pt_wire_eid).ptToId();
          const PtVertex &pt_load = pt_graph->ptVertex(pt_load_vid);
          pt_graph->writeSlewToGraph(pt_load, load_vertex);
          pt_graph->writePathsToGraph(pt_load, load_vertex);
        }
        break;
      }

      case BnetType::junction:
        walkTree(real_drvr, tree->ref());
        walkTree(real_drvr, tree->ref2());
        break;

      case BnetType::buffer: {
        if (vi + 1 >= best_vinfo_.vertex_ids.size()
            || ei + 1 >= best_vinfo_.edge_ids.size()) {
          printf("Warning: writeTimingToGraph: index desync at buffer "
                 "(vi=%zu vertex_ids.size()=%zu, ei=%zu edge_ids.size()=%zu); "
                 "aborting tree walk\n",
                 vi, best_vinfo_.vertex_ids.size(),
                 ei, best_vinfo_.edge_ids.size());
          fflush(stdout);
          break;
        }

        // Consume PtGraph virtual IDs (same order as buildVirtualBuffer)
        VertexId pt_buf_in_vid = best_vinfo_.vertex_ids[vi++];
        VertexId pt_buf_out_vid = best_vinfo_.vertex_ids[vi++];
        ei += 2;  // skip wire_in and gate edge IDs

        const PtVertex &pt_buf_in = pt_graph->ptVertex(pt_buf_in_vid);
        const PtVertex &pt_buf_out = pt_graph->ptVertex(pt_buf_out_vid);

        // Find real buffer instance recorded during exportBufferTree
        sta::Instance *buf_inst = tree->bufInst();
        if (!buf_inst) {
          // Indices already advanced above, but the downstream subtree is
          // skipped here -> any sibling branch will read desynced vi/ei.
          printf("Warning: writeTimingToGraph: buffer node has no bufInst; "
                 "subtree timing skipped, sibling indices may desync\n");
          fflush(stdout);
          break;
        }
        const sta::Pin *buf_in_pin = nullptr;
        const sta::Pin *buf_out_pin = nullptr;
        sta::InstancePinIterator *pin_iter = network_->pinIterator(buf_inst);
        while (pin_iter->hasNext()) {
          const sta::Pin *pin = pin_iter->next();
          if (network_->direction(pin)->isInput())
            buf_in_pin = pin;
          else if (network_->direction(pin)->isOutput())
            buf_out_pin = pin;
        }
        delete pin_iter;

        if (!buf_in_pin || !buf_out_pin) {
          printf("Warning: writeTimingToGraph: buffer instance %s missing "
                 "in/out pin; subtree timing skipped, sibling indices may "
                 "desync\n",
                 network_->name(buf_inst));
          fflush(stdout);
          break;
        }

        sta::Vertex *real_buf_in = sta_graph->pinLoadVertex(buf_in_pin);
        sta::Vertex *real_buf_out = sta_graph->pinDrvrVertex(buf_out_pin);

        // Buffer input vertex: slew + paths
        if (real_buf_in) {
          pt_graph->writeSlewToGraph(pt_buf_in, real_buf_in);
          initNewStaVertexPaths(pt_buf_in, real_buf_in);
        }

        // Buffer output vertex: slew + paths
        if (real_buf_out) {
          pt_graph->writeSlewToGraph(pt_buf_out, real_buf_out);
          initNewStaVertexPaths(pt_buf_out, real_buf_out);
        }

        // Recurse into subtree with buf_output as new driver.
        // If real_buf_out is null the recursion no-ops at the entry guard,
        // leaving the subtree's vi/ei unconsumed -> sibling branches desync.
        if (!real_buf_out) {
          printf("Warning: writeTimingToGraph: buffer instance %s has no "
                 "output driver vertex; subtree indices not consumed, sibling "
                 "indices may desync\n",
                 network_->name(buf_inst));
          fflush(stdout);
        }
        walkTree(real_buf_out, tree->ref());
        break;
      }
    }
  };

  sta::Vertex *real_drvr = sta_graph->pinDrvrVertex(drvr_pin_);
  if (real_drvr) {
    // Update ref instance RefOutput vertices — slew changed because the
    // output load cap decreased after buffer insertion.
    for (VertexId vid : pt_graph->sortedVertexIds()) {
      PtVertex &pv = pt_graph->ptVertex(vid);
      if (pv.type() == PtVertexType::RefOutput && pv.vertex()) {
        pt_graph->writeSlewToGraph(pv, pv.vertex());
        pt_graph->writePathsToGraph(pv, pv.vertex());
      }
    }
    // Walk the BnetPtr tree to write timing for buffer and load vertices.
    walkTree(real_drvr, best_bnet_);
  }
}

// LrRebuffer version of attemptTopologyRewrite:
// Uses bufferCost (lower = better) instead of slack (higher = better)
// to decide which branch is "costly" (the one to shortcut) vs "cheap".
//
// Original topology:
//     +-----[buffer*]- a
//     |
// ----+
//     |   +-[buffer*]- b
//     |   |
//     +---+
//         |
//         +---------- (costly branch)
//
// Rewritten topology:
//     +-------------- (costly branch, gets direct path)
//     |
// ----+
//     |           +-- b
//     |           |
//     +-[buffer]--+
//                 |
//                 +-- a
//
BnetPtr
LrRebuffer::attemptTopologyRewrite(const BnetPtr& node,
                                   const BnetPtr& left,
                                   const BnetPtr& right,
                                   float best_cap)
{
  // The branch with HIGHER bufferCost is the "costly" one
  // (analogous to the "critical" branch in the slack-based version).
  // We want to give it a direct path (fewer buffers in the way).
  BnetPtr costly1, aux1;
  if (left->bufferCost() > right->bufferCost()) {
    costly1 = stripWireOnBnet(left);
    aux1 = stripWireOnBnet(right);
  } else {
    costly1 = stripWireOnBnet(right);
    aux1 = stripWireOnBnet(left);
  }

  if (prune_debug_) {
    printf("[DBG-PRUNE-RW] node_loc=(%d,%d) left[bufs=%d cost=%.3e] "
           "right[bufs=%d cost=%.3e] best_cap=%.3f\n",
           node->location().x(), node->location().y(),
           bufferNum(left), left->bufferCost(),
           bufferNum(right), right->bufferCost(),
           best_cap * 1e15);
  }

  if (costly1->type() != BnetType::junction) {
    if (prune_debug_) {
      printf("[DBG-PRUNE-RW]   REJECT early: costly1 not junction (type=%d)\n",
             (int)costly1->type());
    }
    return {};
  }

  // costly1 is a junction with two sub-branches
  BnetPtr costly2 = costly1->ref(), aux2 = costly1->ref2();
  // Pick the sub-branch with higher cost as the one to promote
  if (costly2->bufferCost() < aux2->bufferCost()) {
    std::swap(costly2, aux2);
  }
  aux2 = stripWireOnBnet(aux2);

  // Only rewrite if at least one of the auxiliary branches has a buffer
  if (aux1->type() != BnetType::buffer && aux2->type() != BnetType::buffer) {
    if (prune_debug_) {
      printf("[DBG-PRUNE-RW]   REJECT: neither aux branch has a buffer "
             "(aux1.type=%d aux2.type=%d)\n",
             (int)aux1->type(), (int)aux2->type());
    }
    return {};
  }

  // Strip wires and buffers to get to the raw loads/junctions
  aux1 = stripWiresAndBuffersOnBnet(aux1);
  aux2 = stripWiresAndBuffersOnBnet(aux2);
  costly2 = stripWiresAndBuffersOnBnet(costly2);

  // Build the new topology:
  //   aux1 and aux2 are merged into a junction, buffered, then joined with costly2
  const BnetPtr in1 = addWire(aux1, node->location(), -1);
  const BnetPtr in2 = addWire(aux2, node->location(), -1);
  BnetPtr junc1_raw = createBnetJunction(resizer_, in1, in2, node->location());
  // Merge LMs from both branches onto the junction before wrapping with wire
  auto junc1_lms = mergeLmVectors(in1->lms(), in2->lms());
  junc1_raw->setLms(std::move(junc1_lms));
  junc1_raw->setBufferCost(in1->bufferCost() + in2->bufferCost());
  junc1_raw->setLeakage(in1->leakage() + in2->leakage());
  const BnetPtr junc1 = addWire(junc1_raw, node->location(), -1);
  const BnetPtr in3 = addWire(costly2, node->location(), -1);

  // The cost of the merged auxiliary junction (before buffer)
  float junc1_cost = junc1->bufferCost();

  // Find a buffer size such that the total cost after buffering
  // the auxiliary junction is still better than not rewriting.
  // buffer_sizes_ is sorted by ascending input capacitance (smallest first).
  float original_cost = left->bufferCost() + right->bufferCost();

  if (prune_debug_) {
    printf("[DBG-PRUNE-RW]   original_cost=%.3e junc1_cost=%.3e "
           "in3_cost=%.3e in3_cap=%.3f\n",
           original_cost, junc1_cost, in3->bufferCost(), in3->cap() * 1e15);
  }

  for (BufferSize size : buffer_sizes_) {
    sta::LibertyPort *in, *out;
    size.cell->bufferPorts(in, out);

    // Cap check: rewritten topology must not exceed original cap or best_cap
    if (fuzzyGreaterEqual(in->capacitance() + in3->cap(), best_cap)
        || fuzzyGreaterEqual(in->capacitance() + in3->cap(),
                             left->cap() + right->cap())) {
      if (prune_debug_) {
        printf("[DBG-PRUNE-RW]   cell=%s BREAK: new_cap=%.3f >= best_cap=%.3f "
               "or orig_cap=%.3f\n",
               size.cell->name(),
               (in->capacitance() + in3->cap()) * 1e15,
               best_cap * 1e15, (left->cap() + right->cap()) * 1e15);
      }
      break;
    }

    // ERC check
    if (!bufferSizeCanDriveLoad(size, junc1)) {
      if (prune_debug_) {
        printf("[DBG-PRUNE-RW]   cell=%s SKIP: cannot drive junc1 (cap=%.3f)\n",
               size.cell->name(), junc1->cap() * 1e15);
      }
      continue;
    }

    // See note in insertBufferOptions: use the LRF-local helper that queries
    // PtGraph's dcalcAnalysisPt instead of arrival_paths_ (which is unset
    // because LRF doesn't call annotateLoadSlacks).
    const FixedDelay buffer_delay
        = computeBufferGateDelay(size.cell, junc1->cap() + out->capacitance());

    // Compute the buffer's added cost (delay×LM + leakage)
    float buffer_leakage = local_sta_->cellAvgLeakage(size.cell);
    float buffer_delta_cost = computeBufferAddedCost(
        buffer_delay.toSeconds(), buffer_leakage, junc1);

    // Total cost of the rewritten topology
    float rewrite_cost = junc1_cost + buffer_delta_cost + in3->bufferCost();

    bool rewrite_ok = (rewrite_cost < original_cost);
    if (prune_debug_) {
      printf("[DBG-PRUNE-RW]   cell=%s buf_delay=%.3e buf_delta_cost=%.3e "
             "rewrite_cost=%.3e orig=%.3e → %s\n",
             size.cell->name(), buffer_delay.toSeconds(),
             buffer_delta_cost, rewrite_cost, original_cost,
             rewrite_ok ? "ACCEPT" : "REJECT(rewrite>=orig)");
    }

    // Only commit to the rewrite if it produces lower total cost
    if (rewrite_ok) {
      BnetPtr buffer = std::make_shared<rsz::BufferedNet>(
          BnetType::buffer,
          node->location(),
          size.cell,
          junc1,
          corner_,
          resizer_,
          estimate_parasitics_);
      buffer->setSlack(junc1->slack() - buffer_delay);
      buffer->setSlackTransition(junc1->slackTransition());
      buffer->setDelay(buffer_delay);
      buffer->setBufferCost(junc1_cost + buffer_delta_cost);
      buffer->setLeakage(junc1->leakage() + buffer_leakage);

      // Propagate LMs through the buffer
      propagateLmsThroughBuffer(buffer, junc1);

      BnetPtr result = createBnetJunction(resizer_, buffer, in3, node->location());
      // The caller will set the final bufferCost and LMs on the returned junction
      return result;
    }
  }
  if (prune_debug_) {
    printf("[DBG-PRUNE-RW]   all sizes failed → no rewrite\n");
  }
  return {};
}

VirtualBufferInfo
LrRebuffer::buildVirtualBuffer(VertexId drvr_vertex_id,
                                const BnetPtr &option)
{
  VirtualBufferInfo info;
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  sta::Vertex *drvr_vertex = pt_graph->ptVertex(drvr_vertex_id).vertex();

  // Pre-count buffers and loads to reserve vector space,
  // avoiding reallocation that would invalidate references.
  size_t num_buffers = 0, num_loads = 0;
  visitTree(
    [&](auto& recurse, int level, const BnetPtr& node) -> int {
      switch (node->type()) {
        case BnetType::buffer: num_buffers++; return recurse(node->ref());
        case BnetType::junction: return recurse(node->ref()) + recurse(node->ref2());
        case BnetType::wire: case BnetType::via: return recurse(node->ref());
        case BnetType::load: num_loads++; return 1;
        default: return 0;
      }
    }, option);
  // Each buffer creates 2 vertices + 3 edges (wire_in, gate, wire_out implicit)
  // Each load creates 1 wire edge
  // Over-reserve to avoid reallocation during walk
  size_t v_extra = num_buffers * 4 + num_loads + 16;
  size_t e_extra = num_buffers * 4 + num_loads * 2 + 16;
  pt_graph->reserveVertices(pt_graph->vertexCount() + v_extra);
  pt_graph->reserveEdges(pt_graph->edgeCount() + e_extra);

  // 1. Collect and delete original wire edges from driver to loads
  {
    std::vector<EdgeId> orig_wire_eids;
    PtVertexOutEdgeIterator out_iter(drvr_vertex_id, pt_graph);
    while (out_iter.hasNext()) {
      PtEdge &pt_edge = out_iter.next();
      if (pt_edge.isWire())
        orig_wire_eids.push_back(pt_edge.objectIdx());
    }
    for (EdgeId eid : orig_wire_eids) {
      pt_graph->deleteEdge(eid);
    }
    info.orig_wire_edge_ids = std::move(orig_wire_eids);
    // Tag driver: original parasitic is invalid, use virtual load cap instead
    pt_graph->ptVertex(drvr_vertex_id).setHasVirtualBuffer(true);
  }

  // Annotate a virtual wire edge with a pre-computed RC delay.
  // Wire edges store delays indexed by rf->index() * ap_count + ap_index,
  // so use setWireArcDelay (not setArcDelay) to write the correct slots.
  auto setVirtualWireDelay = [&](EdgeId eid, float delay_sec) {
    PtEdge &e = pt_graph->edge(eid);
    sta::ArcDelay d(delay_sec);
    for (const sta::RiseFall *rf : sta::RiseFall::range()) {
      for (sta::DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
        pt_graph->setWireArcDelay(e, rf, dcalc_ap->index(), d);
      }
    }
  };

  // 2. Walk BnetPtr tree, build virtual sub-graph.
  // acc_wire_delay accumulates Elmore RC wire delays (seconds) from the
  // current driver through all wire/via segments to the next buffer or load.
  // Gate delays are left at 0 here and computed by updateLocalTiming later.
  using BnetWalker = std::function<void(const BnetPtr&, VertexId, float)>;
  BnetWalker walk = [&](const BnetPtr& node, VertexId current_drvr_id, float acc_wire_delay) {
    if (info.failed) return;
    switch (node->type()) {
      case BnetType::wire:
      case BnetType::via:
        // Accumulate first-order RC delay from BnetPtr node
        walk(node->ref(), current_drvr_id, acc_wire_delay + node->delay().toSeconds());
        break;

      case BnetType::buffer: {
        sta::LibertyCell *buf_cell = node->bufferCell();
        sta::LibertyPort *in_port, *out_port;
        buf_cell->bufferPorts(in_port, out_port);

        VertexId buf_in_id = pt_graph->makeVirtualVertex(
            buf_cell, in_port, false, true, PtVertexType::VirtualInput);
        info.vertex_ids.push_back(buf_in_id);

        VertexId buf_out_id = pt_graph->makeVirtualVertex(
            buf_cell, out_port, true, false, PtVertexType::VirtualOutput);
        info.vertex_ids.push_back(buf_out_id);

        // Levels: set by fixupVirtualLevels after the full tree walk.

        // Proxy vertex for tag_bldr init
        pt_graph->ptVertex(buf_in_id).setProxyVertex(drvr_vertex);
        pt_graph->ptVertex(buf_out_id).setProxyVertex(drvr_vertex);

        // Init paths from driver's tag group
        pt_graph->initVirtualPaths(pt_graph->ptVertex(buf_in_id),
                                   pt_graph->ptVertex(drvr_vertex_id));
        pt_graph->initVirtualPaths(pt_graph->ptVertex(buf_out_id),
                                   pt_graph->ptVertex(drvr_vertex_id));

        // Wire edge: current_drvr → buf_in
        // Delay = accumulated RC wire delay from current driver to buffer insertion point
        EdgeId wire_in_eid = pt_graph->makeVirtualEdge(
            current_drvr_id, buf_in_id,
            sta::TimingArcSet::wireTimingArcSet(), true);
        info.edge_ids.push_back(wire_in_eid);
        setVirtualWireDelay(wire_in_eid, acc_wire_delay);
        // Set LMs on wire-to-buffer edge (same as gate: sum of downstream load LMs)
        {
          const auto &wire_lms = node->lms();
          if (!wire_lms.empty()) {
            pt_graph->setVirtualEdgeLms(pt_graph->edge(wire_in_eid), wire_lms);
          }
        }

        // Gate edge: buf_in → buf_out (delay left 0; computed by updateLocalTiming)
        sta::TimingArcSet *arc_set = nullptr;
        if (in_port && out_port) {
          for (auto *as : buf_cell->timingArcSets(in_port, out_port)) {
            arc_set = as;
            break;
          }
        }
        if (!arc_set) {
          // No timing arc found — try all arc sets for this cell
          for (auto *as : buf_cell->timingArcSets()) {
            if (as->from() == in_port && as->to() == out_port) {
              arc_set = as;
              break;
            }
          }
        }
        if (!arc_set) {
          printf("Warning: buildVirtualBuffer: no timing arc set for %s (%s -> %s)\n",
                 buf_cell->name(),
                 in_port ? in_port->name() : "null",
                 out_port ? out_port->name() : "null");
          fflush(stdout);
          // Failed — caller will handle cleanup via removeVirtualBuffer
          info.failed = true;
          return;
        }
        EdgeId gate_eid = pt_graph->makeVirtualEdge(
            buf_in_id, buf_out_id, arc_set, false);
        info.edge_ids.push_back(gate_eid);

        const auto &lms = node->lms();
        if (!lms.empty()) {
          pt_graph->setVirtualEdgeLms(pt_graph->edge(gate_eid), lms);
        }

        // Recurse with fresh accumulator after the buffer
        walk(node->ref(), buf_out_id, 0.0f);
        break;
      }

      case BnetType::junction:
        walk(node->ref(), current_drvr_id, acc_wire_delay);
        walk(node->ref2(), current_drvr_id, acc_wire_delay);
        break;

      case BnetType::load: {
        const sta::Pin *load_pin = node->loadPin();
        sta::Vertex *load_vertex = graph_->pinLoadVertex(load_pin);
        PtVertex *load_pt_vertex = pt_graph->ptVertex(load_vertex);
        if (load_pt_vertex) {
          EdgeId wire_eid = pt_graph->makeVirtualEdge(
              current_drvr_id, load_pt_vertex->objectIdx(),
              sta::TimingArcSet::wireTimingArcSet(), true);
          info.edge_ids.push_back(wire_eid);
          // Accumulated RC wire delay from current driver to this load
          setVirtualWireDelay(wire_eid, acc_wire_delay);

          const auto &lms = node->lms();
          if (!lms.empty()) {
            pt_graph->setVirtualEdgeLms(pt_graph->edge(wire_eid), lms);
          }
        }
        break;
      }

      default:
        break;
    }
  };

  walk(option, drvr_vertex_id, 0.0f);

  // Bottom-up pass: set virtual vertex levels so topo sort is correct.
  if (!info.failed && !info.vertex_ids.empty()) {
    float drvr_level = pt_graph->ptVertex(drvr_vertex_id).level();
    size_t vi = 0;
    fixupVirtualLevels(option, drvr_level, info, vi);
  }

  return info;
}

float
LrRebuffer::fixupVirtualLevels(const rsz::BufferedNetPtr& node,
                                float drvr_level,
                                VirtualBufferInfo &info,
                                size_t &vi)
{
  using BnetType = rsz::BufferedNetType;
  PtGraph *pt_graph = eval_ctx_->pt_graph;

  switch (node->type()) {
    case BnetType::load: {
      const sta::Pin *load_pin = node->loadPin();
      sta::Vertex *load_vertex = graph_->pinLoadVertex(load_pin);
      if (load_vertex)
        return static_cast<float>(load_vertex->level());
      return drvr_level + 1.0f;  // fallback
    }

    case BnetType::wire:
    case BnetType::via:
      return fixupVirtualLevels(node->ref(), drvr_level, info, vi);

    case BnetType::junction: {
      float l1 = fixupVirtualLevels(node->ref(), drvr_level, info, vi);
      float l2 = fixupVirtualLevels(node->ref2(), drvr_level, info, vi);
      return std::min(l1, l2);
    }

    case BnetType::buffer: {
      // Consume the two vertex IDs (same order as walk: buf_in, buf_out)
      if (vi + 1 >= info.vertex_ids.size())
        return drvr_level + 1.0f;
      VertexId buf_in_id  = info.vertex_ids[vi++];
      VertexId buf_out_id = info.vertex_ids[vi++];

      // Recurse first to get downstream level
      float downstream_level = fixupVirtualLevels(node->ref(), drvr_level, info, vi);

      // Interpolate: buf_in at 1/4, buf_out at 3/4 between drvr and downstream
      float buf_in_level  = drvr_level * 0.75f + downstream_level * 0.25f;
      float buf_out_level = drvr_level * 0.25f + downstream_level * 0.75f;
      pt_graph->ptVertex(buf_in_id).setLevel(buf_in_level);
      pt_graph->ptVertex(buf_out_id).setLevel(buf_out_level);

      // Return buf_in level (closest to driver) for upstream computation
      return buf_in_level;
    }

    default:
      return drvr_level + 1.0f;
  }
}

// Info collected per leaf node when building a synthetic parasitic network.
struct SyntheticLoadInfo {
  VertexId vertex_id;
  const sta::Pin *pin;    // nullptr for virtual loads (buffer input)
  unsigned node_id;       // ConcreteParasiticNode id in the synthetic network
};

// Build a synthetic ConcreteParasiticNetwork from BnetPtr wire/via RC segments
// for a single driver's sub-tree (from driver to its immediate buffer/load leaves).
// Caller must delete the returned network.
static sta::ConcreteParasiticNetwork*
buildSyntheticRCNetwork(const BufferedNetPtr& bnet,
                        PtGraph *pt_graph,
                        VertexId drvr_vid,
                        const sta::Net *fallback_net,
                        const sta::Corner *corner,
                        const sta::RiseFall *rf,
                        const sta::MinMax *min_max,
                        rsz::Resizer *resizer,
                        est::EstimateParasitics *estimate_parasitics,
                        sta::Graph *sta_graph,
                        const sta::Network *network,
                        const std::vector<VertexId> &vinfo_vids,
                        int &vi_ref,
                        sta::ConcreteParasiticNode *&out_drvr_node,
                        std::vector<SyntheticLoadInfo> &out_loads)
{
  const sta::Pin *drvr_pin = pt_graph->ptVertex(drvr_vid).pin();
  const sta::Net *net = drvr_pin ? network->net(drvr_pin) : fallback_net;

  auto *syn_net = new sta::ConcreteParasiticNetwork(net, false, network);
  int node_id = 0;
  size_t res_id = 0;
  out_drvr_node = syn_net->ensureParasiticNode(net, node_id++, network);

  using RCWalker = std::function<void(const BufferedNetPtr&,
                                      sta::ConcreteParasiticNode*)>;
  RCWalker rc_walk = [&](const BufferedNetPtr& node,
                         sta::ConcreteParasiticNode* cur_node) {
    switch (node->type()) {
      case BufferedNetType::wire: {
        double unit_res, unit_cap;
        const_cast<BufferedNet*>(node.get())->wireRC(
            corner, resizer, estimate_parasitics, unit_res, unit_cap);
        double wire_length = resizer->dbuToMeters(node->length());
        double wire_res = wire_length * unit_res;
        double wire_cap = wire_length * unit_cap;
        auto *next = syn_net->ensureParasiticNode(net, node_id++, network);
        syn_net->addResistor(
            new sta::ConcreteParasiticResistor(res_id++, wire_res, cur_node, next));
        // Split cap: half on each end (pi-model, same as EstimateParasitics)
        cur_node->incrCapacitance(wire_cap / 2.0);
        next->incrCapacitance(wire_cap / 2.0);
        rc_walk(node->ref(), next);
        break;
      }
      case BufferedNetType::via: {
        double via_res = const_cast<BufferedNet*>(node.get())->viaResistance(
            corner, resizer, estimate_parasitics);
        auto *next = syn_net->ensureParasiticNode(net, node_id++, network);
        syn_net->addResistor(
            new sta::ConcreteParasiticResistor(res_id++, via_res, cur_node, next));
        rc_walk(node->ref(), next);
        break;
      }
      case BufferedNetType::junction:
        rc_walk(node->ref(), cur_node);
        rc_walk(node->ref2(), cur_node);
        break;
      case BufferedNetType::buffer: {
        if (vi_ref + 1 >= (int)vinfo_vids.size()) break;
        VertexId buf_in_id = vinfo_vids[vi_ref];  // peek, don't advance
        sta::LibertyCell *buf_cell = node->bufferCell();
        sta::LibertyPort *in_port, *out_port;
        buf_cell->bufferPorts(in_port, out_port);
        float buf_in_cap = in_port ? in_port->capacitance(rf, min_max) : 0.0f;
        auto *leaf = syn_net->ensureParasiticNode(net, node_id++, network);
        leaf->incrCapacitance(buf_in_cap);
        // Connect leaf to cur_node so reducePiDfs can reach it
        syn_net->addResistor(
            new sta::ConcreteParasiticResistor(res_id++, 0.0f, cur_node, leaf));
        out_loads.push_back({buf_in_id, nullptr, leaf->id()});
        break;
      }
      case BufferedNetType::load: {
        const sta::Pin *load_pin = node->loadPin();
        sta::LibertyPort *lp = load_pin ? network->libertyPort(load_pin) : nullptr;
        float pin_cap = lp ? lp->capacitance(rf, min_max) : 0.0f;
        auto *leaf = syn_net->ensureParasiticNode(net, node_id++, network);
        leaf->incrCapacitance(pin_cap);
        // Connect leaf to cur_node so reducePiDfs can reach it
        syn_net->addResistor(
            new sta::ConcreteParasiticResistor(res_id++, 0.0f, cur_node, leaf));
        sta::Vertex *lv = sta_graph->pinLoadVertex(load_pin);
        PtVertex *pt_lv = lv ? pt_graph->ptVertex(lv) : nullptr;
        VertexId load_vid = pt_lv ? pt_lv->objectIdx() : pt_vertex_id_null;
        out_loads.push_back({load_vid, load_pin, leaf->id()});
        break;
      }
      default:
        break;
    }
  };
  rc_walk(bnet, out_drvr_node);
  return syn_net;
}

void
LrRebuffer::buildSyntheticParasitics(VertexId drvr_vertex_id,
                                      const BufferedNetPtr& option,
                                      const VirtualBufferInfo &vinfo)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  int vi = 0;

  // Get the original driver's net as fallback for virtual drivers
  const sta::Pin *orig_drvr_pin = pt_graph->ptVertex(drvr_vertex_id).pin();
  const sta::Net *orig_net = orig_drvr_pin ? network_->net(orig_drvr_pin) : nullptr;

  using Walker = std::function<void(const BufferedNetPtr&, VertexId)>;
  Walker walk = [&](const BufferedNetPtr& bnet, VertexId current_drvr_id) {
    float dbg_orig_cap_rise = -999.0f, dbg_orig_cap_fall = -999.0f;
    if (eval_ctx_->debug) {
      sta::DcalcAnalysisPt *dap = corners_->findCorner("default")->findDcalcAnalysisPt(sta::MinMax::max());
      PtPiElmore *pi_r = pt_graph->findPtParasitic(current_drvr_id, sta::RiseFall::rise(), dap->index());
      PtPiElmore *pi_f = pt_graph->findPtParasitic(current_drvr_id, sta::RiseFall::fall(), dap->index());
      if (pi_r) dbg_orig_cap_rise = pi_r->capacitance();
      if (pi_f) dbg_orig_cap_fall = pi_f->capacitance();
    }

    pt_graph->clearPtParasitics(current_drvr_id);

    for (sta::DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
      const sta::Corner *corner = dcalc_ap->corner();
      const sta::MinMax *min_max = dcalc_ap->constraintMinMax();
      const sta::ParasiticAnalysisPt *ap = dcalc_ap->parasiticAnalysisPt();
      float coupling_cap_factor = ap->couplingCapFactor();

      for (const sta::RiseFall *rf : sta::RiseFall::range()) {
        int saved_vi = vi;
        sta::ConcreteParasiticNode *drvr_node = nullptr;
        std::vector<SyntheticLoadInfo> loads;

        auto *syn_net = buildSyntheticRCNetwork(
            bnet, pt_graph, current_drvr_id, orig_net,
            corner, rf, min_max, resizer_, estimate_parasitics_,
            graph_, network_, vinfo.vertex_ids, saved_vi,
            drvr_node, loads);
        vi = saved_vi;

        if (!syn_net || !drvr_node) {
          delete syn_net;
          continue;
        }

        // Reduce Pi model
        LocalReduceToPiElmore reducer(this, pt_graph);
        float c2, rpi, c1;
        reducer.reduceToPi(syn_net, nullptr, drvr_node, coupling_cap_factor,
                           rf, corner, min_max, ap, c2, rpi, c1);

        PtPiElmore &pt_pi = pt_graph->makePtParasitic(
            current_drvr_id, rf, dcalc_ap->index());
        pt_pi.setPiModel(c2, rpi, c1);

        if (eval_ctx_->debug) {
          float orig_cap = rf == sta::RiseFall::rise() ? dbg_orig_cap_rise : dbg_orig_cap_fall;
          float new_cap = c2 + c1;
          printf("[DBG-SYNPI] drvr_vid=%u rf=%s C2=%.4f Rpi=%.1f C1=%.4f total=%.4f (orig=%.4f delta=%.1f%%)\n",
                 (unsigned)current_drvr_id, rf->name(),
                 c2 * 1e15, rpi, c1 * 1e15, new_cap * 1e15,
                 orig_cap > 0 ? orig_cap * 1e15 : -1.0,
                 orig_cap > 0 ? (new_cap - orig_cap) / orig_cap * 100.0 : 0.0);
        }

        // Elmore DFS using downstream caps from reduceToPi
        auto resistor_map = parasitics_->parasiticNodeResistorMap(syn_net);
        std::set<sta::ParasiticNode*> visited;
        std::unordered_map<unsigned, float> node_elmore;

        std::function<void(sta::ParasiticNode*, sta::ParasiticResistor*, double)>
        elmoreDfs = [&](sta::ParasiticNode *node,
                        sta::ParasiticResistor *from_res,
                        double elmore) {
          visited.insert(node);
          auto *cn = static_cast<sta::ConcreteParasiticNode*>(node);
          node_elmore[cn->id()] = elmore;
          auto it = resistor_map.find(node);
          if (it != resistor_map.end()) {
            for (sta::ParasiticResistor *res : it->second) {
              sta::ParasiticNode *onode = parasitics_->otherNode(res, node);
              if (res != from_res && visited.find(onode) == visited.end()) {
                float r = parasitics_->value(res);
                double dwn_cap = reducer.downstreamCap(onode);
                elmoreDfs(onode, res, elmore + r * dwn_cap);
              }
            }
          }
        };
        elmoreDfs(drvr_node, nullptr, 0.0);

        for (const auto &load : loads) {
          auto it = node_elmore.find(load.node_id);
          float elmore = 0.0f;
          if (it != node_elmore.end()) {
            elmore = it->second;
          } else {
            printf("Warning: buildSyntheticParasitics: Elmore DFS did not reach "
                   "load node %u for drvr vertex %u, defaulting to 0\n",
                   load.node_id, (unsigned)current_drvr_id);
          }
          if (eval_ctx_->debug && rf == sta::RiseFall::rise()) {
            printf("[DBG-ELMORE] drvr_vid=%u load_vid=%u pin=%s elmore=%.1f ps\n",
                   (unsigned)current_drvr_id, (unsigned)load.vertex_id,
                   load.pin ? network_->name(load.pin) : "(virtual)",
                   elmore * 1e12);
          }
          pt_pi.addLoad(load.vertex_id, load.pin, elmore);
        }

        // Under env=on, also produce a PtElmoreCeff for the synthetic net
        // so the virtual buffer / virtual load gateDelay path uses
        // Algorithm 2 + Eq.15 (consistent with the main reduce path). The
        // Pi above stays in place for any Bakoglu re-gate on the virtual
        // driver (also needed for env=off rebuffering of buffered nets).
        if (useElmoreCeff()) {
          PtElmoreCeff &pt_ec = pt_graph->makePtElmoreCeff(
              current_drvr_id, rf, dcalc_ap->index());
          LocalReduceToPiElmore ec_reducer(this, pt_graph);
          ec_reducer.makePtElmoreCeffOnly(syn_net, nullptr, drvr_node,
                                          coupling_cap_factor, rf,
                                          corner, min_max, ap, pt_ec);
        }

        delete syn_net;
      }
    }

    // Advance vi and recurse into buffer output downstream sub-trees
    std::function<void(const BufferedNetPtr&)> advanceAndRecurse;
    advanceAndRecurse = [&](const BufferedNetPtr& node) {
      switch (node->type()) {
        case BufferedNetType::wire:
        case BufferedNetType::via:
          advanceAndRecurse(node->ref());
          break;
        case BufferedNetType::junction:
          advanceAndRecurse(node->ref());
          advanceAndRecurse(node->ref2());
          break;
        case BufferedNetType::buffer: {
          if (vi + 1 >= (int)vinfo.vertex_ids.size()) break;
          vi++;  // skip buf_in_id
          VertexId buf_out_id = vinfo.vertex_ids[vi++];
          walk(node->ref(), buf_out_id);
          break;
        }
        case BufferedNetType::load:
        default:
          break;
      }
    };
    advanceAndRecurse(bnet);
  };

  walk(option, drvr_vertex_id);
}

void
LrRebuffer::removeVirtualBuffer(VirtualBufferInfo &info)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;

  // 0. Clean up virtual parasitics
  for (VertexId vid : info.vertex_ids) {
    pt_graph->clearPtParasitics(vid);
  }
  if (!info.orig_wire_edge_ids.empty()) {
    VertexId drvr_id = pt_graph->edge(info.orig_wire_edge_ids[0]).ptFromId();
    pt_graph->clearPtParasitics(drvr_id);
  }

  // 1. Delete all virtual vertices (also deletes their edges via adjacency).
  for (VertexId vid : info.vertex_ids) {
    pt_graph->deleteVertex(vid);
  }
  for (EdgeId eid : info.edge_ids) {
    if (pt_graph->edge(eid).type() != PtEdgeType::Sentinel)
      pt_graph->deleteEdge(eid);
  }

  // 2. Clear virtual buffer tag on driver vertex
  if (!info.orig_wire_edge_ids.empty()) {
    VertexId drvr_id = pt_graph->edge(info.orig_wire_edge_ids[0]).ptFromId();
    pt_graph->ptVertex(drvr_id).setHasVirtualBuffer(false);
  }

  // 3. Re-link original wire edges
  for (EdgeId eid : info.orig_wire_edge_ids) {
    PtEdge &pt_edge = pt_graph->edge(eid);
    VertexId from_id = pt_edge.ptFromId();
    VertexId to_id = pt_edge.ptToId();

    pt_edge.setType(PtEdgeType::None);

    EdgeId old_head = pt_graph->ptVertex(from_id).out_edges_;
    pt_edge.vertex_out_next_ = old_head;
    pt_edge.vertex_out_prev_ = pt_edge_id_null;
    if (old_head != pt_edge_id_null)
      pt_graph->edge(old_head).vertex_out_prev_ = eid;
    pt_graph->ptVertex(from_id).out_edges_ = eid;

    pt_edge.vertex_in_link_ = pt_graph->ptVertex(to_id).in_edges_;
    pt_graph->ptVertex(to_id).in_edges_ = eid;
  }

  // 4. Shrink vectors to prevent unbounded growth from repeated cycles.
  pt_graph->popSentinelTail();
}

float
LrRebuffer::computeVirtualSlack(const VirtualBufferInfo &info)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  return local_sta_->localSlackAroundRef(pt_graph);
}

// ═══════════════════════════════════════════════════════════
// repairSlew — standalone slew repair via buffer insertion
// ═══════════════════════════════════════════════════════════

static float computeSlewRCFactor(const sta::Network *network)
{
  const sta::LibertyLibrary *library = network->defaultLibertyLibrary();
  if (!library) return 1.0f;
  float factor = 0.0f;
  for (auto rf : sta::RiseFall::range()) {
    float th_low, th_high;
    if (rf == sta::RiseFall::rise()) {
      th_low = 1.0f - library->slewUpperThreshold(rf);
      th_high = 1.0f - library->slewLowerThreshold(rf);
    } else {
      th_low = library->slewLowerThreshold(rf);
      th_high = library->slewUpperThreshold(rf);
    }
    float t_high = -log(th_high);
    float t_low = -log(th_low);
    float rf_factor = (t_low - t_high) / library->slewDerateFromLibrary();
    factor = std::max(factor, rf_factor);
  }
  const float pessimism = 0.10f;
  return factor * (1.0f + pessimism);
}

int
LrRebuffer::repairSlew(const sta::Pin *drvr_pin, rsz::Resizer *resizer)
{
  if (network_->isTopLevelPort(drvr_pin))
    return 0;

  sta::LibertyPort *drvr_port = network_->libertyPort(drvr_pin);
  if (!drvr_port)
    return 0;

  const sta::Corner *corner = sta_->cmdCorner();
  est::EstimateParasitics *est = resizer->getEstimateParasitics();

  // Build BufferedNet (Steiner tree)
  BufferedNetPtr bnet = resizer->makeBufferedNet(drvr_pin, corner);
  if (!bnet)
    bnet = resizer->makeBufferedNetSteiner(drvr_pin, corner);
  if (!bnet) {
    printf("repairSlew: cannot build BufferedNet for %s\n",
           network_->name(drvr_pin));
    return 0;
  }

  float slew_rc_factor = computeSlewRCFactor(network_);
  float r_drvr = drvr_port->driveResistance();
  // Clip to strongest buffer resistance (same as RepairDesign)
  for (auto &bs : buffer_sizes_) {
    float r = resizer->bufferDriveResistance(bs.cell);
    r_drvr = std::max(r_drvr, r);
  }

  // Phase 1: Bottom-up walk to propagate maxLoadSlew and compute cap/slew
  struct NodeInfo {
    float cap = 0;
    float max_load_slew = sta::INF;
    int wire_length = 0;
    sta::PinSeq load_pins;
  };

  using Walker = std::function<NodeInfo(const BufferedNetPtr&)>;
  Walker walk = [&](const BufferedNetPtr& node) -> NodeInfo {
    switch (node->type()) {
      case BnetType::load: {
        const sta::Pin *load_pin = node->loadPin();
        sta::LibertyPort *lp = network_->libertyPort(load_pin);
        float cap = lp ? lp->capacitance() : 0.0f;
        float limit = local_sta_->getPortMaxSlewLimit(lp);
        NodeInfo info;
        info.cap = cap;
        info.max_load_slew = limit;
        info.load_pins.push_back(load_pin);
        return info;
      }
      case BnetType::wire:
      case BnetType::via: {
        NodeInfo child = walk(node->ref());
        if (node->type() == BnetType::via) {
          float r_via = const_cast<BufferedNet*>(node.get())->viaResistance(
              corner, resizer, est);
          child.max_load_slew -= r_via * child.cap * slew_rc_factor;
          return child;
        }
        // wire
        double wire_res, wire_cap;
        const_cast<BufferedNet*>(node.get())->wireRC(
            corner, resizer, est, wire_res, wire_cap);
        int length = node->length();
        double length_m = resizer->dbuToMeters(length);
        double r_wire = length_m * wire_res;
        double c_wire = length_m * wire_cap;
        child.cap += c_wire;
        child.wire_length += length;
        child.max_load_slew -= r_wire * (c_wire / 2.0 + child.cap - c_wire)
                               * slew_rc_factor;
        return child;
      }
      case BnetType::junction: {
        NodeInfo left = walk(node->ref());
        NodeInfo right = walk(node->ref2());
        NodeInfo info;
        info.cap = left.cap + right.cap;
        info.max_load_slew = std::min(left.max_load_slew,
                                      right.max_load_slew);
        info.wire_length = std::max(left.wire_length, right.wire_length);
        for (auto *p : left.load_pins) info.load_pins.push_back(p);
        for (auto *p : right.load_pins) info.load_pins.push_back(p);
        return info;
      }
      default: {
        return NodeInfo{};
      }
    }
  };

  NodeInfo root = walk(bnet);
  float load_slew = r_drvr * root.cap * slew_rc_factor;

  // Check driver output slew limit
  float drvr_slew_limit = local_sta_->getPortMaxSlewLimit(drvr_port);
  float effective_limit = std::min(drvr_slew_limit, root.max_load_slew);

  printf("repairSlew: pin=%s r_drvr=%.3e cap=%.3e slew_est=%.3eps "
         "limit=%.3eps (drvr=%.3eps load=%.3eps) loads=%zu\n",
         network_->name(drvr_pin), r_drvr, root.cap,
         load_slew * 1e12, effective_limit * 1e12,
         drvr_slew_limit * 1e12, root.max_load_slew * 1e12,
         root.load_pins.size());

  if (load_slew <= effective_limit) {
    printf("repairSlew: no violation, skip\n");
    return 0;
  }

  // Phase 2: Insert buffer to split loads
  float max_cap = effective_limit / (r_drvr * slew_rc_factor);
  printf("repairSlew: need to reduce cap from %.3e to %.3e\n",
         root.cap, max_cap);

  struct LoadInfo {
    const sta::Pin *pin;
    float cap;
  };
  std::vector<LoadInfo> loads;
  for (const sta::Pin *lp : root.load_pins) {
    sta::LibertyPort *port = network_->libertyPort(lp);
    float c = port ? port->capacitance() : 0.0f;
    loads.push_back({lp, c});
  }
  std::sort(loads.begin(), loads.end(),
            [](const LoadInfo &a, const LoadInfo &b) { return a.cap > b.cap; });

  int inserted = 0;
  float remaining_cap = root.cap;

  while (remaining_cap > max_cap && loads.size() > 1) {
    sta::PinSeq buf_loads;
    float buf_group_cap = 0.0f;
    float target_split = remaining_cap - max_cap;

    auto it = loads.begin();
    while (it != loads.end() && buf_group_cap < target_split) {
      buf_loads.push_back(const_cast<sta::Pin*>(it->pin));
      buf_group_cap += it->cap;
      it = loads.erase(it);
    }

    if (buf_loads.empty())
      break;

    sta::LibertyCell *buf_cell = resizer->findTargetCell(
        resizer->buffer_lowest_drive_, buf_group_cap, false);

    int sum_x = 0, sum_y = 0;
    sta::dbNetwork *db_network = resizer->getDbNetwork();
    for (const sta::Pin *p : buf_loads) {
      odb::Point loc = db_network->location(p);
      sum_x += loc.x();
      sum_y += loc.y();
    }
    odb::Point buf_loc(sum_x / (int)buf_loads.size(),
                       sum_y / (int)buf_loads.size());

    printf("repairSlew: inserting buffer %s for %zu loads (cap=%.3e), "
           "remaining=%zu loads (cap=%.3e)\n",
           buf_cell->name(), buf_loads.size(), buf_group_cap,
           loads.size(), remaining_cap - buf_group_cap);

    sta::Instance *buf_inst = resizer->insertBufferBeforeLoads(
        nullptr, &buf_loads, buf_cell, &buf_loc, "slew_repair");
    if (!buf_inst) {
      printf("repairSlew: insertBufferBeforeLoads failed\n");
      break;
    }
    inserted++;

    sta::LibertyPort *buf_in, *buf_out;
    buf_cell->bufferPorts(buf_in, buf_out);
    sta::Pin *buf_out_pin = network_->findPin(buf_inst, buf_out);
    if (buf_out_pin)
      resizer->resizeToTargetSlew(buf_out_pin);

    sta::LibertyCell *final_buf_cell = network_->libertyCell(buf_inst);
    final_buf_cell->bufferPorts(buf_in, buf_out);
    float buf_in_cap = buf_in ? buf_in->capacitance() : 0.0f;
    remaining_cap = remaining_cap - buf_group_cap + buf_in_cap;

    printf("repairSlew: after insert, remaining_cap=%.3e, "
           "new_slew_est=%.3eps, limit=%.3eps\n",
           remaining_cap, remaining_cap * r_drvr * slew_rc_factor * 1e12,
           effective_limit * 1e12);
  }

  if (inserted > 0) {
    est->updateParasitics();
    printf("repairSlew: inserted %d buffers for pin %s\n",
           inserted, network_->name(drvr_pin));
  }

  return inserted;
}

// Same as the static criticalPathDelay in Rebuffer.cc — computes the
// worst-slack-to-root slack difference used for area recovery relaxation.
static FixedDelay rszCriticalPathDelay(const BufferedNetPtr &root)
{
  FixedDelay worst_load_slack = FixedDelay::INF;
  rsz::visitTree(
      [&](auto &recurse, int level, const BnetPtr &node) -> int {
        switch (node->type()) {
          case BnetType::wire:
          case BnetType::buffer:
            return recurse(node->ref());
          case BnetType::junction:
            return recurse(node->ref()) + recurse(node->ref2());
          case BnetType::load:
            if (node->slack() < worst_load_slack) {
              worst_load_slack = node->slack();
            }
            return 1;
          default:
            printf("rszCriticalPathDelay: unhandled BufferedNet type\n");
            return 0;
        }
      },
      root);
  return worst_load_slack - root->slack();
}

bool
LrRebuffer::prepareRszBnet(const sta::Pin *drvr_pin,
                            sta::VertexId drvr_vid,
                            int bft_iter)
{
  best_bnet_ = nullptr;

  // Mirror rsz::BufferMove::doMove fanout checks.
  static constexpr int rebuffer_max_fanout = 20;

  if (network_->isTopLevelPort(drvr_pin))
    return false;

  sta::Vertex *drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
  int fo = Rebuffer::fanout(drvr_vertex);
  if (fo <= 1)
    return false;
  if (fo >= rebuffer_max_fanout)
    return false;
  if (!resizer_->okToBufferNet(drvr_pin))
    return false;

  sta::Net *net = network_->net(drvr_pin);
  drvr_port_ = network_->libertyPort(drvr_pin);
  if (!net || !drvr_port_ || hasTopLevelOutputPort(net))
    return false;

  setPin(const_cast<sta::Pin*>(drvr_pin));
  drvr_pin_ = drvr_pin;

  BnetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet) {
    printf("prepareRszBnet: Warning: unable to create buffered net for pin %s\n",
           network_->name(drvr_pin));
    return false;
  }

  // Parallel-safe slack + LM annotation (no arrival_paths_ dependency).
  annotateLoadSlacksSlackDp(bnet, drvr_vid);
  annotateLoadLMs(eval_ctx_->pt_graph->ptVertex(drvr_vid), bnet);

  const bool allow_topology_rewrite
      = (estimate_parasitics_->getParasiticsSrc()
         == est::ParasiticsSrc::placement);

  // Phase 1: parallel-safe slack-DP (replaces Rebuffer::bufferForTiming which
  // reads arrival_paths_ / bufferDelay from shared STA).
  for (int i = 0; i < bft_iter; i++) {
    bnet = bufferForTimingSlackDp(drvr_vid, bnet, allow_topology_rewrite);
    if (!bnet) {
      printf("prepareRszBnet: Warning: bufferForTimingSlackDp failed for pin %s "
             "after %d rounds\n", network_->name(drvr_pin), i + 1);
      return false;
    }
  }

  // No Phase 2 recovery — pure slack-DP result (RSZ ablation baseline).
  // recoverArea is not parallel-safe (reads arrival_paths_).

  best_bnet_ = bnet;

  // Rebuild virtual buffer and refresh local timing (same as prepareSlackDpBnet).
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  best_vinfo_ = buildVirtualBuffer(drvr_vid, best_bnet_);
  if (!best_vinfo_.failed) {
    pt_graph->topoSortVertices();
    buildSyntheticParasitics(drvr_vid, best_bnet_, best_vinfo_);
    local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  }
  return true;
}

bool
LrRebuffer::prepareSlackDpBnet(const sta::Pin *drvr_pin,
                                sta::VertexId drvr_vid,
                                int bft_iter,
                                int recover_iter)
{
  best_bnet_ = nullptr;
  static constexpr int rebuffer_max_fanout = 20;

  if (network_->isTopLevelPort(drvr_pin)) return false;
  sta::Vertex *drvr_vertex = graph_->pinDrvrVertex(drvr_pin);
  int fo = Rebuffer::fanout(drvr_vertex);
  if (fo <= 1 || fo >= rebuffer_max_fanout) return false;
  if (!resizer_->okToBufferNet(drvr_pin)) return false;

  sta::Net *net = network_->net(drvr_pin);
  drvr_port_ = network_->libertyPort(drvr_pin);
  if (!net || !drvr_port_ || hasTopLevelOutputPort(net)) return false;

  setPin(const_cast<sta::Pin*>(drvr_pin));
  drvr_pin_ = drvr_pin;

  BnetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet) {
    printf("prepareSlackDpBnet: makeBufferedNet failed for pin %s\n",
           network_->name(drvr_pin));
    return false;
  }

  // ── Slack-DP path: annotate sink slacks (no arrival_paths_) ──
  annotateLoadSlacksSlackDp(bnet, drvr_vid);

  // Annotate LMs so recoverLrCost / evaluateOption can compute LR cost.
  // Re-look up PtVertex each time (O(1) index) — pt_graph may have been
  // mutated by any prior step; vid stays valid.
  annotateLoadLMs(eval_ctx_->pt_graph->ptVertex(drvr_vid), bnet);

  const bool allow_topology_rewrite
      = (estimate_parasitics_->getParasiticsSrc()
         == est::ParasiticsSrc::placement);

  // Phase 1: bufferForTimingSlackDp iterations.
  for (int i = 0; i < bft_iter; i++) {
    bnet = bufferForTimingSlackDp(drvr_vid, bnet, allow_topology_rewrite);
    if (!bnet) {
      printf("prepareSlackDpBnet: bufferForTimingSlackDp failed for pin %s "
             "after %d rounds\n", network_->name(drvr_pin), i + 1);
      return false;
    }
  }

  // Phase 2: recoverLrCost iterations with alpha-blended slack threshold.
  // slack_target = current slack (don't regress vs slack-DP achievement).
  rsz::FixedDelay slack_target = bnet->slack();
  for (int i = 0; i < recover_iter && bnet; i++) {
    bnet = recoverLrCost(drvr_vid, bnet, slack_target,
                         (float)(1 + i) / recover_iter);
  }

  if (!bnet) {
    printf("prepareSlackDpBnet: recoverLrCost failed for pin %s\n",
           network_->name(drvr_pin));
    return false;
  }

  best_bnet_ = bnet;

  // Rebuild virtual buffer for best_bnet_ and refresh local timing so that
  // (a) best_vinfo_ matches the tree writeTimingToGraph will walk, and
  // (b) findLocalArrivals allocates paths_ on each virtual vertex.
  // Without this, writeTimingToGraph's walkTree hits the vi/ei-bounds break
  // and never calls initNewStaVertexPaths on the inserted rebuffer sta::Vertex.
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  best_vinfo_ = buildVirtualBuffer(drvr_vid, best_bnet_);
  if (!best_vinfo_.failed) {
    pt_graph->topoSortVertices();
    buildSyntheticParasitics(drvr_vid, best_bnet_, best_vinfo_);
    local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  }
  return true;
}

void
LrRebuffer::probeRszBnetWithLocalEval(const sta::Pin *drvr_pin,
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
  BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);
  if (!bnet) return;

  sta_->findRequireds();
  annotateLoadSlacks(bnet, drvr_vertex);

  const bool allow_topology_rewrite = true;
  for (int i = 0; i < 3; i++) {
    bnet = Rebuffer::bufferForTiming(bnet, allow_topology_rewrite);
    if (!bnet) return;
  }

  int buf_count = bufferNum(bnet);
  if (buf_count == 0) return;  // RSZ chose no-buffer option

  // RSZ slack evaluation
  std::optional<rsz::FixedDelay> rsz_slack_opt = Rebuffer::evaluateOption(bnet, 0);
  float rsz_slack = rsz_slack_opt ? rsz_slack_opt->toSeconds() : -1e30f;

  // ── Step 2: LRF local timing evaluation on the same bnet ──
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  VertexId drvr_vid = drvr_pt_vertex.objectIdx();

  // Compute original local slack (before buffer) — all methods
  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float orig_worst_sinks = local_sta_->localWorstSlackOnSinks(pt_graph);
  float orig_sum_sinks   = local_sta_->localSlackOnSinks(pt_graph);
  float orig_around_ref  = local_sta_->localSlackAroundRef(pt_graph);

  // Build virtual buffer sub-graph from RSZ's bnet
  BufferedNetPtr bnet_lm = resizer_->makeBufferedNet(drvr_pin, corner_);
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

  if (eval_ctx_->debug) {
    sta::DcalcAnalysisPt *dap = corners_->findCorner("default")->findDcalcAnalysisPt(sta::MinMax::max());
    for (const sta::RiseFall *rf : sta::RiseFall::range()) {
      PtPiElmore *pi = pt_graph->findPtParasitic(drvr_vid, rf, dap->index());
      printf("[DBG-PRE-INCRE] drvr_vid=%u rf=%s cap=%.4e (before increAndGetLocalTimingCost)\n",
             (unsigned)drvr_vid, rf->name(),
             pi ? pi->capacitance() : -1.0f);
    }
  }

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
  // Re-compute original slack (PtGraph is restored)
  local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
  float lrf_orig_worst = local_sta_->localWorstSlackOnSinks(pt_graph);

  rebufferPin(drvr_pin, pt_graph->ptVertex(drvr_vid));

  int lrf_buf_count = 0;
  float lrf_worst_after = lrf_orig_worst;
  if (bestBnet()) {
    lrf_buf_count = bufferNum(bestBnet());
    // bestBnet already built virtual buffer + synthetic parasitics + timing
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
// ═══════════════════════════════════════════════════════════
// repairCap — standalone cap repair via buffer insertion
//
// Walks the Steiner tree bottom-up. At each junction where
// cap_left + cap_right > max_cap, inserts a buffer to isolate
// the larger branch. Along wire segments, accumulates wire cap
// and inserts buffers when the running total exceeds max_cap.
// ═══════════════════════════════════════════════════════════
int
LrRebuffer::repairCap(const sta::Pin *drvr_pin, float max_cap,
                       sta::dbSta *sta, rsz::Resizer *resizer)
{
  sta::Network *network = sta->network();
  if (network->isTopLevelPort(drvr_pin))
    return 0;

  const sta::Corner *corner = sta->cmdCorner();
  est::EstimateParasitics *est = resizer->getEstimateParasitics();
  sta::dbNetwork *db_network = resizer->getDbNetwork();

  // Build BufferedNet (Steiner tree).
  BufferedNetPtr bnet = resizer->makeBufferedNet(drvr_pin, corner);
  if (!bnet)
    bnet = resizer->makeBufferedNetSteiner(drvr_pin, corner);
  if (!bnet) {
    printf("repairCap: no BufferedNet for %s\n", network->pathName(drvr_pin));
    return 0;
  }

  int inserted = 0;

  // Bottom-up recursive walk.
  // Returns: {cap at this node, load pins below this node}.
  struct NodeInfo {
    float cap = 0;
    sta::PinSeq load_pins;
  };

  std::function<NodeInfo(const BufferedNetPtr&)> walk
      = [&](const BufferedNetPtr& node) -> NodeInfo {
    switch (node->type()) {
      case BnetType::load: {
        const sta::Pin *load_pin = node->loadPin();
        sta::LibertyPort *lp = network->libertyPort(load_pin);
        NodeInfo info;
        info.cap = lp ? resizer->portCapacitance(lp, corner) : 0.0f;
        info.load_pins.push_back(load_pin);
        return info;
      }
      case BnetType::wire: {
        NodeInfo child = walk(node->ref());
        double wire_res, wire_cap_per_m;
        const_cast<BufferedNet*>(node.get())->wireRC(
            corner, resizer, est, wire_res, wire_cap_per_m);
        double length_m = resizer->dbuToMeters(node->length());
        double c_wire = length_m * wire_cap_per_m;
        float load_cap = child.cap + c_wire;

        // Same as RepairDesign::repairNetWire: if load_cap exceeds
        // max_cap, insert buffer(s) along this wire segment.
        while (load_cap > max_cap && !child.load_pins.empty()) {
          // Slew-aware buffer selection.
          sta::LibertyCell *buf_cell
              = findBufferUnderSlew(resizer, /*max_slew=*/
                  std::numeric_limits<float>::max(), load_cap);
          if (!buf_cell) break;

          // Compute split point along the wire.
          odb::Point from_loc = node->location();
          odb::Point to_loc = node->ref()->location();
          double split_m = (wire_cap_per_m > 0)
              ? (max_cap - child.cap) / wire_cap_per_m
              : length_m;
          split_m = std::max(0.0, std::min(split_m, length_m));
          double ratio = (length_m > 0) ? split_m / length_m : 0.5;
          int buf_x = to_loc.getX()
              + ratio * (from_loc.getX() - to_loc.getX());
          int buf_y = to_loc.getY()
              + ratio * (from_loc.getY() - to_loc.getY());
          odb::Point buf_loc(buf_x, buf_y);

          // Insert + resize + update pins/cap via makeRepeater.
          float repeater_cap = 0.0f;
          if (!makeRepeater(resizer, corner, buf_loc,
                            buf_cell, child.load_pins, repeater_cap))
            break;
          inserted++;

          // Recalculate: remaining wire from split point to driver.
          double remaining_wire_m = length_m - split_m;
          child.cap = repeater_cap;
          load_cap = child.cap + remaining_wire_m * wire_cap_per_m;
          length_m = remaining_wire_m;
        }

        child.cap = load_cap;
        return child;
      }
      case BnetType::via: {
        return walk(node->ref());
      }
      case BnetType::junction: {
        NodeInfo left = walk(node->ref());
        NodeInfo right = walk(node->ref2());

        // Same strategy as RepairDesign::repairNetJunc:
        // if combined cap exceeds limit, buffer one or both branches.
        float total_cap = left.cap + right.cap;
        bool repeater_left = false;
        bool repeater_right = false;

        if (total_cap > max_cap) {
          // Buffer the larger branch first.
          if (left.cap > right.cap)
            repeater_left = true;
          else
            repeater_right = true;
          // If one branch alone exceeds max_cap, buffer both.
          // Notice: Should this be protected by downstream buffering?
          // if (left.cap > max_cap) repeater_left = true;
          // if (right.cap > max_cap) repeater_right = true;
        }

        // Insert repeater on violating branch(es).
        // Use Steiner junction location + slew-aware buffer selection
        // (mirrors RepairDesign::repairNetJunc → makeRepeater).
        odb::Point junc_loc = node->location();

        auto insertBranchRepeater = [&](sta::PinSeq &pins, float &cap) {
          if (pins.empty()) return;
          // Determine max input slew from loads.
          float max_slew = std::numeric_limits<float>::max();
          for (const sta::Pin *p : pins) {
            sta::LibertyPort *lp = network->libertyPort(p);
            if (lp) {
              float s = resizer->maxInputSlew(lp, corner);
              if (s > 0 && s < max_slew) max_slew = s;
            }
          }
          // Slew-aware buffer selection.
          sta::LibertyCell *buf_cell
              = findBufferUnderSlew(resizer, max_slew, cap);
          if (!buf_cell) return;
          // Insert + resize + update pins/cap.
          if (makeRepeater(resizer, corner, junc_loc,
                           buf_cell, pins, cap))
            inserted++;
        };

        if (repeater_left)
          insertBranchRepeater(left.load_pins, left.cap);
        if (repeater_right)
          insertBranchRepeater(right.load_pins, right.cap);

        // Merge.
        NodeInfo info;
        info.cap = left.cap + right.cap;
        for (auto *p : left.load_pins) info.load_pins.push_back(p);
        for (auto *p : right.load_pins) info.load_pins.push_back(p);
        return info;
      }
      default:
        return NodeInfo{};
    }
  };

  NodeInfo root = walk(bnet);

  if (inserted == 0) {
    printf("repairCap: pin %s — tree_cap=%.2f fF, max_cap=%.2f fF, "
           "loads=%zu, no buffer inserted\n",
           network->pathName(drvr_pin), root.cap * 1e15, max_cap * 1e15,
           root.load_pins.size());
  } else {
    est->updateParasitics();
  }

  return inserted;
}

// ═══════════════════════════════════════════════════════════
// findBufferUnderSlew — slew-aware buffer cell selection
//
// Mirrors RepairDesign::findBufferUnderSlew.
// Iterates through swappable buffer cells sorted by drive resistance
// (weakest → strongest) and returns the first whose output slew
// stays under max_slew when driving load_cap.
// Falls back to the buffer with minimum achievable slew.
// ═══════════════════════════════════════════════════════════
sta::LibertyCell *
LrRebuffer::findBufferUnderSlew(rsz::Resizer *resizer,
                                float max_slew,
                                float load_cap)
{
  sta::LibertyCell *min_slew_buffer = resizer->buffer_lowest_drive_;
  float min_slew = std::numeric_limits<float>::max();

  sta::LibertyCellSeq swappable
      = resizer->getSwappableCells(resizer->buffer_lowest_drive_);
  if (swappable.empty())
    return min_slew_buffer;

  // Sort by drive resistance descending (weakest / smallest first).
  std::sort(swappable.begin(), swappable.end(),
      [&](const sta::LibertyCell *a, const sta::LibertyCell *b) {
        return resizer->bufferDriveResistance(a)
             > resizer->bufferDriveResistance(b);
      });

  for (sta::LibertyCell *buffer : swappable) {
    float slew = resizer->bufferSlew(
        buffer, load_cap, resizer->tgt_slew_dcalc_ap_);
    if (slew < max_slew)
      return buffer;
    if (slew < min_slew) {
      min_slew = slew;
      min_slew_buffer = buffer;
    }
  }
  // No buffer meets max_slew — return the one with minimum slew.
  return min_slew_buffer;
}

// ═══════════════════════════════════════════════════════════
// makeRepeater — insert buffer, resize, update cap & pins
//
// Mirrors RepairDesign::makeRepeater.
// 1. Inserts buffer_cell before load_pins at loc.
// 2. Resizes the buffer to target slew.
// 3. Updates load_pins to {buffer_input_pin} and repeater_cap
//    to the corner-aware input capacitance.
// Returns true on success, false if insertion failed.
// ═══════════════════════════════════════════════════════════
bool
LrRebuffer::makeRepeater(rsz::Resizer *resizer,
                          const sta::Corner *corner,
                          const odb::Point &loc,
                          sta::LibertyCell *buffer_cell,
                          sta::PinSeq &load_pins,
                          float &repeater_cap)
{
  sta::Network *network = resizer->getDbNetwork();

  // Insert buffer before loads.
  odb::Point mutable_loc(loc);
  sta::Instance *buffer = resizer->insertBufferBeforeLoads(
      nullptr, &load_pins, buffer_cell, &mutable_loc, "cap_repair");
  if (!buffer)
    return false;

  sta::LibertyPort *buf_input, *buf_output;
  buffer_cell->bufferPorts(buf_input, buf_output);

  // Resize repeater to target slew.
  sta::Pin *buf_out_pin = network->findPin(buffer, buf_output);
  if (buf_out_pin)
    resizer->resizeToTargetSlew(buf_out_pin);

  // Re-read cell/ports after resize (cell may have changed).
  sta::LibertyCell *final_cell = network->libertyCell(buffer);
  final_cell->bufferPorts(buf_input, buf_output);

  // Update: load_pins becomes {buffer_input_pin}.
  load_pins.clear();
  sta::Pin *buf_in_pin = network->findPin(buffer, buf_input);
  if (buf_in_pin)
    load_pins.push_back(buf_in_pin);

  // Corner-aware input capacitance.
  repeater_cap = buf_input
      ? resizer->portCapacitance(buf_input, corner) : 0.0f;

  return true;
}

}

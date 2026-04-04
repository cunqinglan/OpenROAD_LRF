



#include "LrRebufferV2.hh"
#include "NetlistTransformation.hh"
#include "LrRebuffer.hh"
#include "PtGraph.hh"
#include <chrono>
#include "PtPiElmore.hh"
#include "rsz/Resizer.hh"
#include "LocalSta.hh"
#include "LocalReduceParasitic.hh"
// ParallelVisitor.hh removed — V2 uses EvalContext, not ParallelLrVisitor
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
LrRebufferV2::bufferNum(const BnetPtr& tree)
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

LrRebufferV2::LrRebufferV2(rsz::Resizer *resizer, LocalSta *local_sta,
                           EvalContext* eval_ctx) :
    Rebuffer(resizer),
    local_sta_(local_sta),
    eval_ctx_(eval_ctx)
{
  arc_delay_calc_ = eval_ctx_->arc_delay_calc;
}

void
LrRebufferV2::initGlobalPreamble(sta::dbSta *sta, rsz::Resizer *resizer)
{
  sta->checkCapacitanceLimitPreamble();
  sta->checkSlewLimitPreamble();
  sta->checkFanoutLimitPreamble();
  resizer->resizePreamble();
}

void
LrRebufferV2::init()
{
  // Per-instance init; global preamble must have been called once in serial
  // via LrRebufferV2::initGlobalPreamble() before this runs.
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
    printf("LrRebufferV2::init: Warning: 'default' corner not found\n");
  }
}

void
LrRebufferV2::annotateLoadLMs(PtVertex &drvr_pt_vertex, const BnetPtr& tree)
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
            } else {
              printf("LrRebufferV2::annotateLoadLMs: Warning: no LM found for load pin %s\n",
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
LrRebufferV2::computeNetSensitivity(const sta::Pin *drvr_pin,
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
  VertexId drvr_vid = drvr_pt_vertex.objectIdx();
  local_sta_->recomputeSinglePtParasitic(pt_graph, drvr_vid);

  float d_current = 0.0f;
  float r_eq = 0.0f;
  float c_eq = 0.0f;
  for (const sta::RiseFall *rf : sta::RiseFall::range()) {
    PtPiElmore *pt_pi = pt_graph->findPtParasitic(drvr_vid, rf, ap_index);
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

  float d_opt = 2.5f * std::sqrt(r_buf * c_buf_in * r_eq * c_eq);
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
  return max_sensitivity;
}

int
LrRebufferV2::applyBufferingToDb()
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
LrRebufferV2::persistBufferParasitics()
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
LrRebufferV2::rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex)
{
  best_bnet_ = nullptr;
  best_cost_ = std::numeric_limits<float>::max();
  best_vinfo_ = VirtualBufferInfo{};
  if (network_->isTopLevelPort(drvr_pin)) {
    printf("LrRebufferV2::rebufferPin: Warning: rebuffering does not support top port as the driver pin: %s\n",
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
      printf("LrRebufferV2::rebufferPin: Warning: unable to create buffered net for pin %s\n",
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
    bnet = bufferForTiming(drvr_vid, bnet, allow_topology_rewrite, /*last_iteration=*/true);
    auto t_iter_end = std::chrono::steady_clock::now();
    (*eval_ctx_->runtime_map)["rebuffer_precise"]
        += std::chrono::duration<double>(t_iter_end - t_iter_start).count();
    if (!bnet) {
      printf("LrRebufferV2::rebufferPin: Warning: bufferForTiming failed for pin %s\n",
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
LrRebufferV2::prepareBufferOptions(const sta::Pin *drvr_pin,
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
LrRebufferV2::evaluateBufferOnCandidate(sta::VertexId drvr_vid,
                                         const rsz::BufferedNetPtr &prepared_bnet)
{
  // Run 1 round of precise bufferForTiming on the current PtGraph state.
  // PtGraph should already reflect the resize candidate via
  // increAndGetLocalTimingCost before calling this.
  const bool allow_topology_rewrite = true;
  BufferedNetPtr result = bufferForTiming(drvr_vid, prepared_bnet,
                                         allow_topology_rewrite,
                                         /*last_iteration=*/true);
  if (result) {
    best_bnet_ = result;
    // best_cost_ is set inside bufferForTiming → evaluateOption
  }
}

void
LrRebufferV2::cleanupVirtualBuffer()
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
LrRebufferV2::bufferForTiming(VertexId drvr_vertex_id,
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
                printf("LrRebufferV2::bufferForTiming: Warning: Buffer pin %s: wire step options empty\n",
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
                printf("LrRebufferV2::bufferForTiming: Warning: Buffer pin %s: wire step options empty at round %d\n",
                       network_->name(pin_), round);
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
              // Use buffer cost instead of slack for comparison
              // Smaller buffer cost is better
              while (li + 1 != lend && (*(li + 1))->bufferCost() <= (*ri)->bufferCost()) {
                li++;
              }
              while (ri + 1 != rend && (*(ri + 1))->bufferCost() <= (*li)->bufferCost()) {
                ri++;
              }

              bool rewrote = false;
              BnetPtr junc;

              if (allow_topology_rewrite) {
                junc = attemptTopologyRewrite(node, *li, *ri, best_cap);
                if (junc) {
                  rewrote = true;
                  // The rewritten junction's sub-nodes already have correct
                  // bufferCost set by attemptTopologyRewrite. Compute
                  // junction cost from its actual children (buffer + direct).
                  float junc_cost = junc->ref()->bufferCost()
                                  + junc->ref2()->bufferCost();
                  junc->setBufferCost(junc_cost);
                  junc->setLeakage(junc->ref()->leakage() + junc->ref2()->leakage());
                  // Merge LMs from the rewritten children
                  auto merged_lms = mergeLmVectors(junc->ref()->lms(),
                                                   junc->ref2()->lms());
                  junc->setLms(std::move(merged_lms));
                }
              }

              if (!rewrote) {
                junc = createBnetJunction(resizer_, *li, *ri, node->location());
                
                // Calculate junction's buffer cost = sum of both branches
                float junc_cost = (*li)->bufferCost() + (*ri)->bufferCost();
                junc->setBufferCost(junc_cost);
                junc->setLeakage((*li)->leakage() + (*ri)->leakage());

                // Merge LMs from both branches
                const auto& left_lms = (*li)->lms();
                const auto& right_lms = (*ri)->lms();
                auto merged_lms = mergeLmVectors(left_lms, right_lms);
                junc->setLms(std::move(merged_lms));
              }

              if (junc->fanout() <= fanout_limit_) {
                // printf("junction fanout %zu within limit %f\n", junc->fanout(), fanout_limit_);
                best_cap = junc->cap();
                opts.push_back(std::move(junc));
              }

              while (true) {
                // Increment either li or ri, whichever leads to smaller buffer cost increase
                // Smaller buffer cost is better, so we want the next one with smaller cost
                float next_li_cost = (li + 1 != lend)
                                           ? (*(li + 1))->bufferCost()
                                           : INF;
                float next_ri_cost = (ri + 1 != rend)
                                           ? (*(ri + 1))->bufferCost()
                                           : INF;

                if (next_li_cost < next_ri_cost) {
                  li++;
                } else {
                  ri++;
                }

                if (li == lend || ri == rend
                    || (*li)->cap() + (*ri)->cap() < best_cap) {
                  break;
                }
              }
            }
            std::ranges::reverse(opts);
            return opts;
          }

          case BnetType::load: {
            // Load node: initialize buffer cost to 0 (starting point)
            node->setBufferCost(0.0f);
            return {node};
          }
          
          default:
            printf("LrRebufferV2::bufferForTiming: Error: unhandled BufferedNet type\n");
            return {};
        }
      },
      tree);

  if (top_opts.empty()) {
    printf("LrRebufferV2::bufferForTiming: Warning: no buffering options generated for pin %s\n",
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
    origial_slack = local_sta_->localSlackOnSinks(pt_graph);
    static int dbg_bft_count = 0;
    if (++dbg_bft_count <= 10)
      printf("[DBG-BFT] pin=%s top_opts=%zu origial_slack=%.3e last_iter=%d\n",
             network_->name(pin_), top_opts.size(), origial_slack, last_iteration);
  }
  // Two-pass: first find no-buffer baseline delay_lm_sum, then compare
  float nobuf_delay_lm_sum = INF;
  float nobuf_cost = INF;
  if (last_iteration) {
    for (const BnetPtr& p : top_opts) {
      if (p->bufferCount() == 0) {
        LMValue cost = evaluateOption(drvr_vertex_id, p, origial_slack);
        if (last_delay_lm_sum_ < nobuf_delay_lm_sum) {
          nobuf_delay_lm_sum = last_delay_lm_sum_;
          nobuf_cost = cost;
        }
        if (cost < best_cost) {
          best_cost = cost;
          best_option = p;
          best_index = i;
        }
        i++;
      }
    }
    i = 1;
    for (const BnetPtr& p : top_opts) {
      if (p->bufferCount() > 0) {
        LMValue cost = evaluateOption(drvr_vertex_id, p, origial_slack);
        if (last_delay_lm_sum_ < nobuf_delay_lm_sum) {
          printf("[DBG-BUF-WIN] pin=%s buffers=%d buf_delay_lm=%.3e nobuf_delay_lm=%.3e "
                 "delta=%.3e%% leakage=%.3e buf_cost=%.3e nobuf_cost=%.3e\n",
                 network_->name(pin_), p->bufferCount(),
                 last_delay_lm_sum_, nobuf_delay_lm_sum,
                 (last_delay_lm_sum_ - nobuf_delay_lm_sum) / nobuf_delay_lm_sum * 100.0,
                 p->leakage(), cost, nobuf_cost);
        }
        if (cost < best_cost) {
          best_cost = cost;
          best_option = p;
          best_index = i;
        }
      }
      i++;
    }
  } else {
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
    if (verbose_) {
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

  return best_option;
}

LMValue
LrRebufferV2::evaluateOptionCoarse(VertexId pt_vertex_id, const BnetPtr& option)
{
  float max_slew = 0.0f;
  float cell_delay_lm_sum = cellDelayLmSum(pt_vertex_id, option, max_slew);
  if (hasViolation(option, max_slew)) {
    return INF;
  }
  float delay_part = option->bufferCost() + cell_delay_lm_sum;
  float leakage_part = option->leakage();
  float cost = eval_ctx_->swapCost(delay_part, leakage_part);
  static int dbg_coarse_count = 0;
  if (++dbg_coarse_count <= 40) {
    printf("[DBG-COARSE] pin=%s buffers=%d bufferCost=%.3e cellDelayLmSum=%.3e "
           "delay_part=%.3e leakage=%.3e cost=%.3e avg_delay=%.3e avg_leak=%.3e\n",
           network_->name(pin_), option->bufferCount(), option->bufferCost(),
           cell_delay_lm_sum, delay_part, leakage_part, cost,
           eval_ctx_->average_delay, eval_ctx_->average_leakage);
  }
  return cost;
}

LMValue
LrRebufferV2::evaluateOption(VertexId pt_vertex_id, const BnetPtr& option,
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
  float slack_after = local_sta_->localSlackOnSinks(pt_graph);

  float thresh = original_slack;
  if (slack_after >= thresh) {
    total_cost = eval_ctx_->swapCost(delay_lm_sum, option->leakage());
  }

  static int dbg_eval_count = 0;
  if (++dbg_eval_count <= 50) {
    printf("[DBG-PRECISE] pin=%s buffers=%d delay_lm_sum=%.3e leakage=%.3e "
           "cost=%.3e slack_after=%.3e orig_slack=%.3e pass=%d vinfo_failed=%d\n",
           network_->name(pin_), option->bufferCount(),
           delay_lm_sum, option->leakage(), total_cost,
           slack_after, original_slack,
           slack_after >= thresh, vinfo.failed);
  }

  removeVirtualBuffer(vinfo);
  local_sta_->recomputeSinglePtParasitic(pt_graph, pt_vertex_id);
  return total_cost;
}

bool
LrRebufferV2::hasViolation(const BnetPtr& option, sta::Slew slew)
{
  if (!loadSlewSatisfactory(drvr_port_, option)) return true;
  if (slew > drvr_pin_max_slew_ && option->cap() > drvr_load_high_water_mark_) {
    return true;
  }
  return false;
}

void 
LrRebufferV2::insertBufferOptions(BnetSeq& opts,
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

      if (!bufferSizeCanDriveLoad(strong_driver, opt, next_segment_wl)) {
        keep = false;
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
      // Calculate buffer leakage once (shared for both filtering and final cost)
      float buffer_leakage = local_sta_->cellAvgLeakage(buffer_cell);
      
      // Use intrinsic_delay for quick filtering
      float intrinsic_delay_seconds = buffer_size.intrinsic_delay.toSeconds();
      float estimated_delta_cost = computeBufferAddedCost(intrinsic_delay_seconds, 
                                                           buffer_leakage, 
                                                           opt);
      float estimated_total_cost = opt_cost + estimated_delta_cost;

      // Only compute precise delay if this passes initial filter
      if (estimated_total_cost < best_cost && bufferSizeCanDriveLoad(buffer_size, opt)) {
        // Step 2: Precise delay calculation (expensive, only for candidates)
        sta::LibertyPort *in, *out;
        buffer_cell->bufferPorts(in, out);
        const float load_cap = opt->cap() + out->capacitance();
        
        const FixedDelay buffer_delay
            = bufferDelay(buffer_cell, opt->slackTransition(), load_cap);
        const float buffer_delay_seconds = buffer_delay.toSeconds();
        
        // Recalculate cost with precise delay
        float precise_delta_cost = computeBufferAddedCost(buffer_delay_seconds, 
                                                           buffer_leakage, 
                                                           opt);
        float precise_total_cost = opt_cost + precise_delta_cost;

        // Final check with precise cost
        if (precise_total_cost < best_cost) {
          load_opt = opt;
          load_opt_buffer_delay = buffer_delay;
          load_opt_total_cost = precise_total_cost;
          best_cost = precise_total_cost;
          best_area = opt->area() + buffer_cell->area();
        }
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
LrRebufferV2::cellDelayLmSum(VertexId pt_vertex_id,
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
      printf("LrRebufferV2::cellDelayLmSum: Warning: edge from vertex %s has no LM values\n",
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

// Compute the delta (added) cost when inserting a buffer
// Cost = delay_LM_sum + leakage
float
LrRebufferV2::computeBufferAddedCost(float buffer_delay_seconds,
                                    float buffer_leakage,
                                    const BnetPtr& load_opt)
{
  sta::DcalcAPIndex ap_index = eval_ctx_->pt_graph->dcalcAnalysisPt()->index();
  // Part 1: Calculate buffer delay × LM contribution
  float buffer_delta_delay_lm = 0.0f;
  const auto& load_lms = load_opt->lms();
  
  if (!load_lms.empty()) {
    // Buffer delay × output LM (assume output LM ≈ load LM)
    for (const sta::RiseFall *rf : sta::RiseFall::range()) {
      int lm_index = rf->index() * graph_->apCount() + ap_index;
      buffer_delta_delay_lm += buffer_delay_seconds * load_lms[lm_index];
    }
  }
  // Part 2: Combine delay_LM_sum and leakage as total cost
  // Simple sum: cost = delay_LM_sum + leakage
  float total_cost = buffer_delta_delay_lm + buffer_leakage;
  
  return total_cost;
}

// Add wire segment and update delay LM sum
BnetPtr
LrRebufferV2::addWire(const BnetPtr& p,
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

  if (level != -1) {
    // printf("  %*sAdded wire: length=%d um, %s s, delta_cost=%.3e, total_cost=%.3e\n",
    //        level * 2, "", z->length(), z->to_string(resizer_).c_str(),
    //        wire_delta_cost, total_cost);
  }

  return z;
}

// Propagate LMs through a buffer node
void
LrRebufferV2::propagateLmsThroughBuffer(BnetPtr& buffer_node,
                                      const BnetPtr& load_opt)
{
  // For now, simply copy the load's LM to the buffer output
  // In a more sophisticated version, you'd compute the buffer's input LM
  // based on the output LM and the buffer's timing characteristics
  
  const auto& load_lms = load_opt->lms();
  if (!load_lms.empty()) {
    buffer_node->setLms(load_lms);
  }
}

// Merge two LM vectors (for junction nodes)
std::vector<float>
LrRebufferV2::mergeLmVectors(const std::vector<float>& lm1, 
                           const std::vector<float>& lm2)
{
  size_t size = std::max(lm1.size(), lm2.size());
  if (lm1.size() != lm2.size()) {
    printf("LrRebufferV2::mergeLmVectors: Warning: LM vector size mismatch (%zu vs %zu), merging with zero-padding\n",
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
LrRebufferV2::writeLmsToGraph()
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
void
LrRebufferV2::initNewStaVertexPaths(const PtVertex &pt_vertex,
                                  sta::Vertex *sta_vertex)
{
  sta::Path *pt_paths = pt_vertex.paths();
  if (!pt_paths)
    return;
  sta::TagGroup *pt_tg = search_->tagGroup(pt_vertex.tagGroupIndex());
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
LrRebufferV2::writeTimingToGraph()
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
        if (ei >= best_vinfo_.edge_ids.size())
          break;
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
            || ei + 1 >= best_vinfo_.edge_ids.size())
          break;

        // Consume PtGraph virtual IDs (same order as buildVirtualBuffer)
        VertexId pt_buf_in_vid = best_vinfo_.vertex_ids[vi++];
        VertexId pt_buf_out_vid = best_vinfo_.vertex_ids[vi++];
        ei += 2;  // skip wire_in and gate edge IDs

        const PtVertex &pt_buf_in = pt_graph->ptVertex(pt_buf_in_vid);
        const PtVertex &pt_buf_out = pt_graph->ptVertex(pt_buf_out_vid);

        // Find real buffer instance recorded during exportBufferTree
        sta::Instance *buf_inst = tree->bufInst();
        if (!buf_inst)
          break;
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

        // Recurse into subtree with buf_output as new driver
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
LrRebufferV2::attemptTopologyRewrite(const BnetPtr& node,
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

  if (costly1->type() != BnetType::junction) {
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

  for (BufferSize size : buffer_sizes_) {
    sta::LibertyPort *in, *out;
    size.cell->bufferPorts(in, out);

    // Cap check: rewritten topology must not exceed original cap or best_cap
    if (fuzzyGreaterEqual(in->capacitance() + in3->cap(), best_cap)
        || fuzzyGreaterEqual(in->capacitance() + in3->cap(),
                             left->cap() + right->cap())) {
      break;
    }

    // ERC check
    if (!bufferSizeCanDriveLoad(size, junc1)) {
      continue;
    }

    const FixedDelay buffer_delay
        = bufferDelay(size.cell,
                      junc1->slackTransition(),
                      junc1->cap() + out->capacitance());

    // Compute the buffer's added cost (delay×LM + leakage)
    float buffer_leakage = local_sta_->cellAvgLeakage(size.cell);
    float buffer_delta_cost = computeBufferAddedCost(
        buffer_delay.toSeconds(), buffer_leakage, junc1);

    // Total cost of the rewritten topology
    float rewrite_cost = junc1_cost + buffer_delta_cost + in3->bufferCost();

    // Only commit to the rewrite if it produces lower total cost
    if (rewrite_cost < original_cost) {
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
  return {};
}

VirtualBufferInfo
LrRebufferV2::buildVirtualBuffer(VertexId drvr_vertex_id,
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

        // Levels: midpoint between driver and downstream
        float drvr_level = pt_graph->ptVertex(current_drvr_id).level();
        pt_graph->ptVertex(buf_in_id).setLevel(drvr_level + 0.25f);
        pt_graph->ptVertex(buf_out_id).setLevel(drvr_level + 0.5f);

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
  return info;
}

void
LrRebufferV2::buildVirtualParasitics(VertexId drvr_vertex_id,
                                    const BufferedNetPtr& option,
                                    const VirtualBufferInfo &vinfo)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  int vi = 0;  // index into vinfo.vertex_ids (pairs: buf_in, buf_out)

  using Walker = std::function<void(const BufferedNetPtr&, VertexId, float)>;
  Walker walk = [&](const BufferedNetPtr& node, VertexId current_drvr_id,
                    float acc_wire_delay) {
    switch (node->type()) {
      case BufferedNetType::wire:
      case BufferedNetType::via:
        walk(node->ref(), current_drvr_id,
             acc_wire_delay + node->delay().toSeconds());
        break;

      case BufferedNetType::buffer: {
        if (vi + 1 >= (int)vinfo.vertex_ids.size()) break;
        VertexId buf_in_id = vinfo.vertex_ids[vi++];
        VertexId buf_out_id = vinfo.vertex_ids[vi++];

        // Add virtual buffer input Elmore to current driver's PtPiElmore
        for (const sta::RiseFall *rf : sta::RiseFall::range()) {
          for (sta::DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
            PtPiElmore *drvr_pi = pt_graph->findPtParasitic(
                current_drvr_id, rf, dcalc_ap->index());
            if (drvr_pi) {
              drvr_pi->addLoad(buf_in_id, nullptr, acc_wire_delay);
            }
          }
        }

        // Create empty PtPiElmore for virtual buffer output
        for (const sta::RiseFall *rf : sta::RiseFall::range()) {
          for (sta::DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
            pt_graph->makePtParasitic(buf_out_id, rf, dcalc_ap->index());
          }
        }

        // Recurse: downstream loads will addLoad to buf_out_id's PtPiElmore
        walk(node->ref(), buf_out_id, 0.0f);

        // Set Pi model for buf_out: c2=0, rpi=0, c1=sum of downstream load caps
        for (const sta::RiseFall *rf : sta::RiseFall::range()) {
          for (sta::DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
            PtPiElmore *buf_pi = pt_graph->findPtParasitic(
                buf_out_id, rf, dcalc_ap->index());
            if (buf_pi) {
              float total_cap = 0.0f;
              for (const auto &load : buf_pi->loads()) {
                PtVertex &lv = pt_graph->ptVertex(load.vertex_id);
                sta::LibertyPort *lp = lv.hasBase()
                    ? network_->libertyPort(lv.pin()) : lv.libertyPort();
                if (lp) {
                  total_cap += lp->capacitance(rf, dcalc_ap->constraintMinMax());
                }
              }
              buf_pi->setPiModel(0.0f, 0.0f, total_cap);
            }
          }
        }
        break;
      }

      case BufferedNetType::junction:
        walk(node->ref(), current_drvr_id, acc_wire_delay);
        walk(node->ref2(), current_drvr_id, acc_wire_delay);
        break;

      case BufferedNetType::load: {
        const sta::Pin *load_pin = node->loadPin();
        sta::Vertex *load_vertex = graph_->pinLoadVertex(load_pin);
        PtVertex *load_pt_vertex = pt_graph->ptVertex(load_vertex);
        if (load_pt_vertex) {
          VertexId load_vid = load_pt_vertex->objectIdx();
          for (const sta::RiseFall *rf : sta::RiseFall::range()) {
            for (sta::DcalcAnalysisPt *dcalc_ap : corners_->dcalcAnalysisPts()) {
              PtPiElmore &drvr_pi = pt_graph->makePtParasitic(
                  current_drvr_id, rf, dcalc_ap->index());
              drvr_pi.addLoad(load_vid, load_pin, acc_wire_delay);
            }
          }
        }
        break;
      }

      default:
        break;
    }
  };

  walk(option, drvr_vertex_id, 0.0f);
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
LrRebufferV2::buildSyntheticParasitics(VertexId drvr_vertex_id,
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
    pt_graph->clearPtParasitics(current_drvr_id);

    for (auto *corner : *corners_) {
      sta::DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(sta::MinMax::max());
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
          pt_pi.addLoad(load.vertex_id, load.pin, elmore);
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
LrRebufferV2::removeVirtualBuffer(VirtualBufferInfo &info)
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
LrRebufferV2::computeVirtualSlack(const VirtualBufferInfo &info)
{
  PtGraph *pt_graph = eval_ctx_->pt_graph;
  return local_sta_->localSlackAroundRef(pt_graph);
}

}




#include "LrRebuffer.hh"
#include "rsz/Resizer.hh"
#include "LocalSta.hh"
#include "ParallelVisitor.hh"
#include "sta/Fuzzy.hh"


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

LrRebuffer::LrRebuffer(rsz::Resizer *resizer, ParallelLrVisitor* visitor) :
    Rebuffer(resizer),
    local_sta_(visitor->localSta()),
    visitor_(visitor)
{
  arc_delay_calc_ = visitor->arcDelayCalc();
}

void
LrRebuffer::initGlobalPreamble(sta::dbSta *sta, rsz::Resizer *resizer)
{
  sta->checkCapacitanceLimitPreamble();
  sta->checkSlewLimitPreamble();
  sta->checkFanoutLimitPreamble();
  resizer->findFastBuffers();
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

  arc_delay_calc_ = visitor_->arcDelayCalc();

  sta::Corner *corner = corners_->findCorner("default");
  if (corner) {
    initOnCorner(corner);
  } else {
    // printf("LrRebuffer::init: Warning: 'default' corner not found\n");
  }
}

void
LrRebuffer::annotateLoadLMs(PtVertex &drvr_pt_vertex, const BnetPtr& tree)
{
  PtGraph *pt_graph = visitor_->ptGraph();
  sta::Vertex *root_vertex = drvr_pt_vertex.vertex();
  // Map from load pin to its LM vector
  std::map<const sta::Pin*, std::vector<float>> load_pin_lm_map;
  
  // First pass: collect LM vectors from all wire edges from driver to loads
  PtVertexOutEdgeIterator out_edge_iter(drvr_pt_vertex, pt_graph);
  while (out_edge_iter.hasNext()) {
    PtEdge &pt_edge = out_edge_iter.next();
    sta::Edge *edge = pt_edge.edge();
    if (!edge->isWire()) continue;
    
    // Get the load pin at the end of this wire edge
    sta::Vertex *to_vertex = edge->to(graph_);
    const sta::Pin *load_pin = to_vertex->pin();
    
    int lmVecSize = sta::TimingArcSet::wireArcCount() * graph_->apCount();
    if (edge->timingArcSet()->arcCount() > 2) {
      // printf("LrRebuffer::annotateLoadLMs: Warning: more than 2 timing arcs on edge from driver to load, only first 2 will be considered for LM annotation\n");
    }
    
    LMValue *load_lms = edge->arcLms();
    if (load_lms == nullptr) {
      // printf("LrRebuffer::annotateLoadLMs: Warning: edge to pin %s has no LM values\n",
             // network_->pathName(load_pin));
      continue;
    }
    
    // Store LM vector in map using std::vector (automatic memory management)
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
              // printf("LrRebuffer::annotateLoadLMs: Warning: no LM found for load pin %s\n",
                     // network_->pathName(load_pin));
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

int 
LrRebuffer::applyBufferingToDb()
{
  odb::dbNet* const db_net = db_network_->flatNet(drvr_pin_);
  return exportBufferTree(best_bnet_, db_network_->dbToSta(db_net), 1, nullptr, "rebuffer");
}

void
LrRebuffer::rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex)
{
  best_bnet_ = nullptr;
  if (network_->isTopLevelPort(drvr_pin)) {
    // printf("LrRebuffer::rebufferPin: Warning: rebuffering does not support top port as the driver pin: %s\n",
           // network_->name(drvr_pin));
    return;
  }

  PtGraph *pt_graph = visitor_->ptGraph();
  sta::LibertyCell *cur_lib_cell = pt_graph->refGate();
  drvr_port_ = cur_lib_cell->findLibertyPort(network_->portName(drvr_pin));
  drvr_pin_ = drvr_pin;
  sta::Net *net = network_->net(drvr_pin);
  odb::dbNet* const db_net = db_network_->flatNet(drvr_pin);
  if (net && drvr_port_ &&
      // Verilog connects by net name, so there is no way to distinguish the
      // net from the port.
      !hasTopLevelOutputPort(net)) {
    setPin(const_cast<sta::Pin*>(drvr_pin));
    BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);

    if (!bnet) {
      // printf("LrRebuffer::rebufferPin: Warning: unable to create buffered net for pin %s\n",
             // network_->name(drvr_pin));
      return;
    }

    // Compute RAT and AAT of the local graph
    local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
    localAnnotateLoadSlacks(bnet, drvr_pt_vertex);
    annotateLoadLMs(drvr_pt_vertex, bnet);

    const bool allow_topology_rewrite = true;
    for (int i = 0; i < 3; i++) {
      bnet = bufferForTiming(drvr_pt_vertex, bnet, allow_topology_rewrite);
      if (!bnet) {
        // printf("LrRebuffer::rebufferPin: Warning: bufferForTiming failed for pin %s at iteration %d\n",
               // network_->name(drvr_pin), i);
        break;
      }
    }

    if (!bnet) {
      return;
    }

    best_bnet_ = bnet;
  }
}

void
LrRebuffer::localAnnotateLoadSlacks(const BnetPtr& tree, PtVertex &drvr_pt_vertex)
{
  for (auto rf_index : sta::RiseFall::rangeIndex()) {
    arrival_paths_[rf_index] = nullptr;
  }
  PtGraph *pt_graph = visitor_->ptGraph();

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
            const sta::Pin* load_pin = node->loadPin();
            sta::Vertex* vertex = graph_->pinLoadVertex(load_pin);
            PtVertex *pt_vertex = pt_graph->ptVertex(vertex);
            sta::Path* req_path
                = local_sta_->ptVertexWorstSlackPath(*pt_vertex, sta::MinMax::max());
            sta::Path* arrival_path = req_path;

            while (req_path && arrival_path->vertex(sta_) != drvr_pt_vertex.vertex()) {
              arrival_path = arrival_path->prevPath();
              if (!arrival_path) {
                // printf("LrRebuffer::annotateLoadSlacks: no arrival path from root to load %s\n",
                       // network_->pathName(load_pin));
                break;
              }
            }

            if (!arrival_path) {
              node->setSlackTransition(nullptr);
              node->setSlack(FixedDelay::INF);
            } else {
              const sta::RiseFall* rf = req_path->transition(sta_);
              node->setSlackTransition(rf->asRiseFallBoth());
              node->setSlack(FixedDelay(
                  req_path->required() - arrival_path->arrival(), resizer_));

              if (arrival_paths_[rf->index()] == nullptr) {
                arrival_paths_[rf->index()] = arrival_path;
              } else {
                // If there are multiple loads, we use the critial
                // path among them to do driver delay calculation.
                if (arrival_path->slack(this) < arrival_paths_[rf->index()]->slack(this)) {
                  arrival_paths_[rf->index()] = arrival_path;
                }
              }
            }
            return 1;
          }
          default:
            // printf("LrRebuffer::annotateLoadSlacks: Warning: unhandled BufferedNet type %d\n",
                   // static_cast<int>(node->type()));
            return 0;
        }
      },
      tree);
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
LrRebuffer::bufferForTiming(PtVertex &pt_drvr_vertex, 
                            const BnetPtr &tree, 
                            bool allow_topology_rewrite)
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
                // printf("LrRebuffer::bufferForTiming: Warning: Buffer pin %s: wire step options empty\n",
                       // network_->name(pin_));
              }
              return opts1;
            }
          
            int round = 0;
            while (location != node->location()) {
              // printf("LrRebuffer::bufferForTiming: round %d, location (%d, %d), target (%d, %d), options %zu\n",
                     // round, location.x(), location.y(), node->location().x(), node->location().y(), opts.size());

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
                // printf("LrRebuffer::bufferForTiming: Warning: Buffer pin %s: wire step options empty at round %d\n",
                       // network_->name(pin_), round);
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
            // printf("LrRebuffer::bufferForTiming: Error: unhandled BufferedNet type\n");
            return {};
        }
      },
      tree);

  if (top_opts.empty()) {
    // printf("LrRebuffer::bufferForTiming: Warning: no buffering options generated for pin %s\n",
           // network_->name(pin_));
  }

  // Select best option based on buffer cost
  float best_cost = INF;
  BnetPtr best_option = nullptr;
  int best_index = 0;
  int i = 1;
  
  for (const BnetPtr& p : top_opts) {
    LMValue cost = evaluateOption(pt_drvr_vertex, p);
    
    // printf("option %d: cost = %.3e, slack = %.3e, cap = %.3e, fanout = %0.f\n",
    //        i, cost, p->slack().toSeconds(), p->cap(), p->fanout());

    if (cost < best_cost) {
      best_cost = cost;
      best_option = p;
      best_index = i;
    }
    i++;
  }

  if (best_option) {
    // printf("best option: %d cost=%.3e, slack=%.3e, cap=%.3e, fanout=%.0f\n",
           // best_index, best_cost, best_option->slack().toSeconds(),
           // best_option->cap(), best_option->fanout());
    fflush(stdout);
  }

  return best_option;
}

LMValue
LrRebuffer::evaluateOption(PtVertex &pt_vertex, const BnetPtr& option)
{
  float driver_leakage = local_sta_->cellAvgLeakage(drvr_port_->libertyCell());
  sta::Slew max_slew;
  float cell_delay_lm_sum = cellDelayLmSum(pt_vertex, option, max_slew);
  float total_cost = option->bufferCost() + cell_delay_lm_sum + driver_leakage;
  if (hasViolation(option, max_slew)) {
    return INF;  // Assign infinite cost to options with violations
  }
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
      
      // Propagate LMs through buffer
      propagateLmsThroughBuffer(z, load_opt);

      new_opts.push_back(std::move(z));
    }
  }
  pass_through(INF);

  new_opts.swap(opts);
}

float
LrRebuffer::cellDelayLmSum(PtVertex &pt_drvr_vertex,
                           const BnetPtr& load_opt,
                           float &max_slew)
{
  max_slew = -INF;
  PtGraph *pt_graph = visitor_->ptGraph();
  float delay_lm_sum = 0.0f;
  sta::DcalcAnalysisPt *dcalc_pt = pt_graph->dcalcAnalysisPt();
  sta::DcalcAPIndex ap_index = dcalc_pt->index();
  float output_cap = load_opt->cap();
  PtVertexInEdgeIterator in_edge_iter(pt_drvr_vertex.objectIdx(), pt_graph);
  while (in_edge_iter.hasNext()) {
    PtEdge &pt_edge = in_edge_iter.next();
    sta::Edge *edge = pt_edge.edge();
    auto *lms = edge->arcLms();
    if (lms == nullptr) {
      // printf("LrRebuffer::cellDelayLmSum: Warning: edge from vertex %s has no LM values\n",
             // pt_drvr_vertex.vertex()->to_string(graph_).c_str());
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
LrRebuffer::computeBufferAddedCost(float buffer_delay_seconds,
                                    float buffer_leakage,
                                    const BnetPtr& load_opt)
{
  sta::DcalcAPIndex ap_index = visitor_->ptGraph()->dcalcAnalysisPt()->index();
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

  if (level != -1) {
    // printf("  %*sAdded wire: length=%d um, %s s, delta_cost=%.3e, total_cost=%.3e\n",
    //        level * 2, "", z->length(), z->to_string(resizer_).c_str(),
    //        wire_delta_cost, total_cost);
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
}

// Merge two LM vectors (for junction nodes)
std::vector<float>
LrRebuffer::mergeLmVectors(const std::vector<float>& lm1, 
                           const std::vector<float>& lm2)
{
  size_t size = std::max(lm1.size(), lm2.size());
  if (lm1.size() != lm2.size()) {
    // printf("LrRebuffer::mergeLmVectors: Warning: LM vector size mismatch (%zu vs %zu), merging with zero-padding\n",
           // lm1.size(), lm2.size());
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
  const BnetPtr junc1
      = addWire(createBnetJunction(resizer_, in1, in2, node->location()),
                node->location(),
                -1);
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

      // Propagate LMs through the buffer
      propagateLmsThroughBuffer(buffer, junc1);

      BnetPtr result = createBnetJunction(resizer_, buffer, in3, node->location());
      // The caller will set the final bufferCost and LMs on the returned junction
      return result;
    }
  }

  return {};
}


}
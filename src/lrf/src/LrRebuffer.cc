



#include "LrRebuffer.cc"
#include "rsz/Resizer.hh"
#include "LocalSta.hh"


namespace lrf {

LrRebuffer::LrRebuffer(rsz::Resizer *resizer, ParallelLrVisitor* visitor, sta::Corner *corner) :
    Rebuffer(resizer),
    local_sta_(visitor->localSta()),
    visitor_(visitor)
{
  arc_delay_calc_ = visitor->arcDelayCalc();
  // This has to be set since we require corner while
  // evaluating delay lm sum
  initOnCorner(corner);
}

void Rebuffer::init()
{
  logger_ = resizer_->logger_;
  dbStaState::init(resizer_->sta_);
  db_network_ = resizer_->db_network_;
  estimate_parasitics_ = resizer_->estimate_parasitics_;
  resizer_max_wire_length_
      = resizer_->metersToDbu(resizer_->findMaxWireLength());
  sta_->checkCapacitanceLimitPreamble();
  sta_->checkSlewLimitPreamble();
  sta_->checkFanoutLimitPreamble();

  resizer_->findFastBuffers();
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
}

void
LrRebuffer::annotateLoadLMs(PtVertex &drvr_pt_vertex, PtGraph *pt_graph, sta::Vertex *root_vertex, const rsz::BnetPtr& tree)
{
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
      printf("LrRebuffer::annotateLoadLMs: Warning: more than 2 timing arcs on edge from driver to load, only first 2 will be considered for LM annotation\n");
    }
    
    LMValue *load_lms = edge->arcLms();
    if (load_lms == nullptr) {
      printf("LrRebuffer::annotateLoadLMs: Warning: edge to pin %s has no LM values\n",
             network_->pathName(load_pin));
      continue;
    }
    
    // Store LM vector in map using std::vector (automatic memory management)
    std::vector<float> lmVec(load_lms, load_lms + lmVecSize);
    load_pin_lm_map[load_pin] = std::move(lmVec);
  }

  // Second pass: traverse the buffered net tree and annotate LMs to load nodes
  visitTree(
      [&](auto& recurse, int level, const rsz::BnetPtr& node) -> int {
        switch (node->type()) {
          case rsz::BnetType::via:
          case rsz::BnetType::wire:
          case rsz::BnetType::buffer:
            return recurse(node->ref());
          case rsz::BnetType::junction:
            return recurse(node->ref()) + recurse(node->ref2());
          case rsz::BnetType::load: {
            // Annotate the LMs of the load directly to the BufferedNet node
            const sta::Pin* load_pin = node->loadPin();
            auto it = load_pin_lm_map.find(load_pin);
            if (it != load_pin_lm_map.end()) {
              node->setLms(it->second);  // Copy LM vector to the node
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

int
LrRebuffer::rebufferPin(const sta::Pin *drvr_pin, PtVertex *drvr_pt_vertex,
   PtGraph *pt_graph)
{
  if (network_->isTopLevelPort(drvr_pin)) {
    printf("LrRebuffer::rebufferPin: Warning: rebuffering does not support top port as the driver pin: %s\n",
           network_->name(drvr_pin).c_str());
    return 0;
  }

  sta::LibertyCell *cur_lib_cell = pt_graph->refGate();
  drvr_port_ = cur_lib_cell->findLibertyPort(network_->portName(drvr_pin));
  sta::Net *net = network_->net(drvr_pin);
  odb::dbNet* const db_net = db_network_->flatNet(drvr_pin);
  if (net && drvr_port_ &&
      // Verilog connects by net name, so there is no way to distinguish the
      // net from the port.
      !hasTopLevelOutputPort(net)) {
    setPin(const_cast<sta::Pin*>(drvr_pin));
    BufferedNetPtr bnet = resizer_->makeBufferedNet(drvr_pin, corner_);

    if (!bnet) {
      printf("LrRebuffer::rebufferPin: Warning: unable to create buffered net for pin %s\n",
             network_->name(drvr_pin).c_str());
      return 0;
    }

    // Compute RAT and AAT of the local graph
    local_sta_->increAndGetLocalTimingCost(pt_graph, arc_delay_calc_, nullptr);
    localAnnotateLoadSlacks(bnet, drvr_pt_vertex, pt_graph);

    const bool allow_topology_rewrite = true;
    for (int i = 0; i < 3; i++) {
      bnet = bufferForTiming(bnet, allow_topology_rewrite);
      if (!bnet) {
        printf("LrRebuffer::rebufferPin: Warning: bufferForTiming failed for pin %s at iteration %d\n",
               network_->name(drvr_pin).c_str(), i);
        return 0;
      }
    }

    if (!bnet) {
      return 0;
    }

    // Evaluate the buffer solution.
    sta::Delay drvr_gate_delay;
    std::tie(drvr_gate_delay, std::ignore, std::ignore) = drvrPinTiming(bnet);
    sta::Delay relaxation = (std::max(drvr_gate_delay, 0.0f)
                             + criticalPathDelay(logger_, bnet).toSeconds())
                            * relaxation_factor_;
    float leakage = bNetPower(bnet);
    for (int i = 0; i < 5 && bnet; i++) {
      bnet = recoverArea(bnet, target, ((float) (1 + i)) / 5);
    }

    if (!bnet) {
      printf("LrRebuffer::rebufferPin: Warning: recoverArea failed for pin %s\n",
              network_->name(drvr_pin).c_str());
      return 0;
    }

    sta::Instance* parent
        = db_network_->getOwningInstanceParent(const_cast<sta::Pin*>(drvr_pin));
    int inserted_count;
    inserted_count = exportBufferTree(
        bnet, db_network_->dbToSta(db_net), 1, parent, "rebuffer");

    if (inserted_count > 0) {
      resizer_->level_drvr_vertices_valid_ = false;
    }
    
    return inserted_count;
  }

  return 0;
}

void
LrRebuffer::localAnnotateLoadSlacks(const rsz::BnetPtr& tree, PtVertex *drvr_pt_vertex, PtGraph *pt_graph)
{
  for (auto rf_index : sta::RiseFall::rangeIndex()) {
    arrival_paths_[rf_index] = nullptr;
  }

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
            PtVertex *pt_vertex = pt_graph->pinVertex(vertex);
            sta::Path* req_path
                = local_sta_->ptVertexWorstSlackPath(pt_vertex, sta::MinMax::max());
            sta::Path* arrival_path = req_path;

            while (req_path && arrival_path->vertex(sta_) != drvr_pt_vertex->vertex()) {
              arrival_path = arrival_path->prevPath();
              if (!arrival_path) {
                printf("LrRebuffer::annotateLoadSlacks: no arrival path from root to load %s\n",
                       network_->pathName(load_pin).c_str());
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
            logger_->critical(RSZ, 1001, "unhandled BufferedNet type");
        }
      },
      tree);
}

// Find buffering choices with best delay LM sum and leakage
rsz::BnetPtr
LrRebuffer::bufferForTiming(const rsz::BnetPtr &tree, 
                            bool allow_topology_rewrite)
{
  sta::LibertyPort *strong_driver;
  {
    sta::LibertyPort* dummy;
    buffer_sizes_.back().cell->bufferPorts(dummy, strong_driver);
  }

  rsz::BnetSeq top_opts = visitTree(
      [&](auto& recurse, int level, const rsz::BnetPtr& node) -> rsz::BnetSeq {
        switch (node->type()) {
          case rsz::BnetType::via:
          case rsz::BnetType::buffer:
          case rsz::BnetType::wire: {
            int layer = -1;
            if (auto wire_layer = findWireLayer(node)) {
              layer = wire_layer.value();
            }
            rsz::BnetSeq opts = recurse(stripWiresAndBuffersOnBnet(node->ref()));
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
              rsz::BnetSeq opts1 = opts;
              for (rsz::BnetPtr& opt : opts1) {
                opt = addWire(opt, node->location(), layer, level);
              }
              insertBufferOptions(opts1, level, 0);
              if (opts1.empty()) {
                // if generated options empty, start again but allow for
                // insertion of buffers at the farther end
                opts1 = opts;
                insertBufferOptions(opts1, level, full_wl);
                for (rsz::BnetPtr& opt : opts1) {
                  opt = addWire(opt, node->location(), layer, level);
                }
                insertBufferOptions(opts1, level, 0);
              }
              if (opts1.empty()) {
                // if generated options still empty, this is an internal error
                // of the algorithm (wire_length_step_ should have been chosen
                // to always allow a minimal size buffer to drive itself without
                // ERC)
                logger_->critical(RSZ,
                                  2008,
                                  "buffering pin {}: wire step options empty",
                                  network_->name(pin_));
              }
              return opts1;
            }
            
            // Long wire handling with stepping
            utl::DebugScopedTimer timer(long_wire_stepping_runtime_);
            int round = 0;
            while (location != node->location()) {
              debugPrint(logger_,
                         RSZ,
                         "rebuffer",
                         4,
                         "{:{}s}round {} no of options {}",
                         "",
                         level,
                         round,
                         opts.size());

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

              for (rsz::BnetPtr& opt : opts) {
                opt = addWire(opt, location, layer, level);
              }
              insertBufferOptions(opts, level, std::min(remaining_wl, step));

              if (opts.empty()) {
                logger_->critical(RSZ,
                                  2007,
                                  "buffering pin {}: wire step options empty",
                                  network_->name(pin_));
              }
              round++;
            }
            return opts;
          }

          case rsz::BnetType::junction: {
            const rsz::BnetSeq& opts_left = recurse(node->ref());
            const rsz::BnetSeq& opts_right = recurse(node->ref2());

            rsz::BnetSeq opts;
            opts.reserve(std::max(opts_left.size(), opts_right.size()));
            float best_cap = INF;

            auto li = opts_left.rbegin(), lend = opts_left.rend();
            auto ri = opts_right.rbegin(), rend = opts_right.rend();

            while (li != lend && ri != rend) {
              // Use delay LM sum instead of slack for comparison
              // Smaller delay LM sum is better
              while (li + 1 != lend && (*(li + 1))->delayLmSum() <= (*ri)->delayLmSum()) {
                li++;
              }
              while (ri + 1 != rend && (*(ri + 1))->delayLmSum() <= (*li)->delayLmSum()) {
                ri++;
              }

              bool rewrote = false;
              rsz::BnetPtr junc;

              if (allow_topology_rewrite) {
                junc = attemptTopologyRewrite(node, *li, *ri, best_cap);
                if (junc) {
                  rewrote = true;
                  // Update LM sum for rewritten junction
                  float junc_lmsum = (*li)->delayLmSum() + (*ri)->delayLmSum();
                  junc->setDelayLmSum(junc_lmsum);
                }
              }

              if (!rewrote) {
                junc = createBnetJunction(resizer_, *li, *ri, node->location());
                
                // Calculate junction's delay LM sum = sum of both branches
                float junc_lmsum = (*li)->delayLmSum() + (*ri)->delayLmSum();
                junc->setDelayLmSum(junc_lmsum);
                
                // Merge LMs from both branches
                const auto& left_lms = (*li)->lms();
                const auto& right_lms = (*ri)->lms();
                auto merged_lms = mergeLmVectors(left_lms, right_lms);
                junc->setLms(std::move(merged_lms));
              }

              if (junc->fanout() <= fanout_limit_) {
                printf("junction fanout %zu within limit %zu\n", junc->fanout(), fanout_limit_);
                best_cap = junc->cap();
                opts.push_back(std::move(junc));
              }

              while (true) {
                // Increment either li or ri, whichever leads to smaller delay LM sum increase
                // Smaller delay LM sum is better, so we want the next one with smaller LM sum
                float next_li_lmsum = (li + 1 != lend)
                                           ? (*(li + 1))->delayLmSum()
                                           : INF;
                float next_ri_lmsum = (ri + 1 != rend)
                                           ? (*(ri + 1))->delayLmSum()
                                           : INF;

                if (next_li_lmsum < next_ri_lmsum) {
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

          case rsz::BnetType::load: {
            // Load node: initialize delay LM sum to 0 (starting point)
            node->setDelayLmSum(0.0f);
            return {node};
          }
          
          default:
            logger_->error(RSZ, 1004, "unhandled BufferedNet type");
            return {};
        }
      },
      tree);

  if (top_opts.empty()) {
    logger_->critical(RSZ, 2009, "buffering pin {}: no options produced", 
                      network_->name(pin_));
  }

  // Select best option based on delay LM sum
  float best_lmsum = INF;
  rsz::BnetPtr best_option = nullptr;
  int best_index = 0;
  int i = 1;
  
  debugPrint(logger_, RSZ, "rebuffer", 2, "LM-sum-optimized options");
  for (const rsz::BnetPtr& p : top_opts) {
    float lmsum = p->delayLmSum();
    
    printf("option %d: LM sum = %.3e, slack = %.3e, cap = %.3f, fanout = %zu\n",
           i, lmsum, p->slack().toSeconds(), p->cap(), p->fanout());

    if (lmsum < best_lmsum) {
      best_lmsum = lmsum;
      best_option = p;
      best_index = i;
    }
    i++;
  }

  printf("best option: %d lmsum=%.3e\n", best_index, best_lmsum);

  return best_option;
}

void 
LrRebuffer::insertBufferOptions(rsz::BnetSeq& opts,
                                int level,
                                int next_segment_wl)
{
  if (opts.empty()) {
    return;
  }

  rsz::BufferSize& strong_driver = buffer_sizes_.back();

  float best_area = INF;
  float best_lmsum = INF;  // Use delay LM sum instead of slack

  // both `opts` and `buffer_sizes_` are ordered by ascending input capacitance
  rsz::BnetSeq new_opts;
  new_opts.reserve(opts.size() * 2);
  auto opts_iter = opts.begin();

  auto pass_through = [&](float threshold_cap) {
    // pass through non-redundant options with cap below `threshold_cap`
    for (; opts_iter != opts.end() && (*opts_iter)->cap() <= threshold_cap;
         opts_iter++) {
      rsz::BnetPtr& opt = *opts_iter;

      // Use the already computed delay LM sum from recursive call
      float opt_lmsum = opt->delayLmSum();

      // Keep option if it has better (smaller) delay LM sum
      bool keep = (opt_lmsum < best_lmsum);

      if (!bufferSizeCanDriveLoad(strong_driver, opt, next_segment_wl)) {
        keep = false;
      }

      if (keep) {
        new_opts.push_back(opt);
        best_lmsum = opt_lmsum;
        best_area = opt->area();
      }
    }
  };

  for (rsz::BufferSize buffer_size : buffer_sizes_) {
    sta::LibertyCell* buffer_cell = buffer_size.cell;
    sta::LibertyPort *in, *out;
    buffer_cell->bufferPorts(in, out);
    pass_through(in->capacitance());

    rsz::BnetPtr load_opt;
    FixedDelay load_opt_buffer_delay = FixedDelay::ZERO;
    float load_opt_total_lmsum = INF;
    
    auto it = (new_opts.empty() && opts_iter == opts.end()
               && opts_iter > opts.begin())
                  ? (opts_iter - 1)
                  : opts_iter;
    
    for (; it != opts.end(); it++) {
      rsz::BnetPtr& opt = *it;

      // Get the already computed delay LM sum from the load option
      float opt_lmsum = opt->delayLmSum();

      // Calculate buffer delay
      const FixedDelay buffer_delay
          = bufferDelay(buffer_cell,
                        opt->slackTransition(),
                        opt->cap() + out->capacitance());
      
      // Calculate the delta (added) delay LM sum from this buffer
      float buffer_delta_lmsum = computeBufferAddedLmSum(buffer_cell, opt, buffer_delay);
      
      // Total LM sum = load's LM sum + buffer's delta
      float total_lmsum = opt_lmsum + buffer_delta_lmsum;

      // Keep if total LM sum is better (smaller) and can drive the load
      if (total_lmsum < best_lmsum && bufferSizeCanDriveLoad(buffer_size, opt)) {
        load_opt = opt;
        load_opt_buffer_delay = buffer_delay;
        load_opt_total_lmsum = total_lmsum;
        best_lmsum = total_lmsum;
        best_area = opt->area() + buffer_cell->area();
      }
    }

    if (load_opt) {
      rsz::BnetPtr z = make_shared<rsz::BufferedNet>(rsz::BnetType::buffer,
                                                     load_opt->location(),
                                                     buffer_cell,
                                                     load_opt,
                                                     corner_,
                                                     resizer_,
                                                     estimate_parasitics_);
      z->setSlack(load_opt->slack() - load_opt_buffer_delay);  // Still maintain slack for debugging
      z->setSlackTransition(load_opt->slackTransition());
      z->setDelay(load_opt_buffer_delay);
      
      // Set the total delay LM sum for this buffer option
      z->setDelayLmSum(load_opt_total_lmsum);
      
      // Propagate LMs through buffer
      propagateLmsThroughBuffer(z, buffer_cell, load_opt);

      new_opts.push_back(std::move(z));
    }
  }
  pass_through(INF);

  new_opts.swap(opts);
}

// Compute the delta (added) delay LM sum when inserting a buffer
float
LrRebuffer::computeBufferAddedLmSum(sta::LibertyCell* buffer_cell,
                                    const rsz::BnetPtr& load_opt,
                                    const FixedDelay& buffer_delay)
{
  // The buffer adds: buffer_delay × LM_output
  // The output LM depends on the load's LM
  
  const auto& load_lms = load_opt->lms();
  if (load_lms.empty()) {
    return 0.0f;
  }
  
  // For now, assume buffer output LM ≈ load LM (simplified)
  // In a more sophisticated version, you'd back-propagate through the buffer's timing arcs
  float buffer_delta_lmsum = 0.0f;
  float delay_seconds = buffer_delay.toSeconds();
  
  for (size_t i = 0; i < load_lms.size(); i++) {
    buffer_delta_lmsum += delay_seconds * load_lms[i];
  }
  
  return buffer_delta_lmsum;
}

// Add wire segment and update delay LM sum
rsz::BnetPtr
LrRebuffer::addWire(const rsz::BnetPtr& p,
                    odb::Point wire_end,
                    int wire_layer,
                    int level)
{
  // Create wire node
  rsz::BnetPtr z = make_shared<rsz::BufferedNet>(rsz::BnetType::wire,
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

  // Calculate wire's contribution to delay LM sum
  const auto& lms = p->lms();
  float wire_delta_lmsum = 0.0f;
  
  if (!lms.empty()) {
    float delay_seconds = wire_delay.toSeconds();
    for (size_t i = 0; i < lms.size(); i++) {
      wire_delta_lmsum += delay_seconds * lms[i];
    }
  }

  // Update total delay LM sum: previous sum + wire's delta
  float total_lmsum = p->delayLmSum() + wire_delta_lmsum;
  z->setDelayLmSum(total_lmsum);

  if (level != -1) {
    debugPrint(logger_,
               RSZ,
               "rebuffer",
               3,
               "{:{}s}wire wl {} lmsum={:.3e} {}",
               "",
               level,
               z->length(),
               total_lmsum,
               z->to_string(resizer_));
  }

  return z;
}

// Propagate LMs through a buffer node
void
LrRebuffer::propagateLmsThroughBuffer(rsz::BnetPtr& buffer_node,
                                      sta::LibertyCell* buffer_cell,
                                      const rsz::BnetPtr& load_opt)
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
  std::vector<float> merged(size, 0.0f);
  
  for (size_t i = 0; i < lm1.size(); i++) {
    merged[i] += lm1[i];
  }
  for (size_t i = 0; i < lm2.size(); i++) {
    merged[i] += lm2[i];
  }
  
  return merged;
}


}
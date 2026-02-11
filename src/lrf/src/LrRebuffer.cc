



#include "LrRebuffer.cc"
#include "rsz/Resizer.hh"
#include "LocalSta.hh"


namespace lrf {

LrRebuffer::LrRebuffer(rsz::Resizer *resizer, ParallelLrVisitor* visitor) :
    Rebuffer(resizer),
    local_sta_(visitor->localSta()),
    visitor_(visitor)
{
  arc_delay_calc_ = visitor->arcDelayCalc();
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
LrRebuffer::clear()
{
  for (auto& pair : bnet_lm_map_) {
    delete[] pair.second;
  }
  bnet_lm_map_.clear();
}

void
LrRebuffer::annotateLoadLMs(PtVertex &drvr_pt_vertex, PtGraph *pt_graph, sta::Vertex *root_vertex, const rsz::BnetPtr& tree)
{
  std::map<const sta::Pin*, LMValue*> load_pin_lm_map;
  PtVertexOutEdgeIterator out_edge_iter(drvr_pt_vertex, pt_graph);
  while (out_edge_iter.hasNext()) {
    PtEdge &pt_edge = out_edge_iter.next();
    sta::Edge *edge = pt_edge->edge();
    if (!edge->isWire()) continue;
    int lmVecSize = sta::TimingArcSet::wireArcCount() * graph_->apCount();
    if (edge->timingArcSet()->arcCount() > 2) {
      printf("LrRebuffer::annotateLoadLMs: Warning: more than 2 timing arcs on edge from driver to load, only first 2 will be considered for LM annotation\n");
    }
    LMValue *load_lms = edge->arcLms();
    LMValue *lmValues = new LMValue[lmVecSize];
    load_pin_lm_map[load_pin] = lmValues;
    for (int i = 0; i < lmVecSize; i++) {
      lmValues[i] = load_lms[i];
    }
  }

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
            // Annotate the LMs of the load, later we will propagate 
            // the LMs through the tree and annotate the LM of the 
            // driver pin at the end
            const sta::Pin* load_pin = node->loadPin();
            bnet_lm_map_[node] = load_pin_lm_map[load_pin];
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

LMValue *
LrRebuffer::mergeWireLMVec(const LMValue *lm_vec1, const LMValue *lm_vec2)
{
  int wire_arc_count = sta::TimingArcSet::wireArcCount();
  size_t lm_vec_size = wire_arc_count * graph_->apCount();
  LMValue* merged_lm_vec = new LMValue[lm_vec_size];
  for (int i = 0; i < lm_vec_size; i++) {
    merged_lm_vec[i] = lm_vec1[i] + lm_vec2[i];
  }
  return merged_lm_vec;
}

void
LrRebuffer::bufferForTiming(const BnetPtr &tree, bool allow_topology_rewrite)
{
  sta::LibertyPort *strong_driver;
  {
    sta::LibertyPort* dummy;
    buffer_sizes_.back().cell->bufferPorts(dummy, strong_driver);
  }

  rsz::BnetSeq top_opts = visitTree(
      [&](auto& recurse, int level, const BnetPtr& node) -> BnetSeq {
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
              debugPrint(logger_,
                         RSZ,
                         "rebuffer",
                         4,
                         "{:{}s}inserting prebuffers",
                         "",
                         level);
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
                logger_->critical(RSZ,
                                  2008,
                                  "buffering pin {}: wire step options empty",
                                  network_->name(pin_));
              }
              return opts1;
            }
          }
        }
      }
  )
}



}
#include "position_driven.hh"

#include <algorithm>      // std::sort
#include <utility>        // std::pair, tuple interface
#include <cmath>          // std::ceil, std::floor
#include <cstdint>        // uint32_t
#include <csignal>        // strsignal
#include <cstdio>         // freopen
#include <limits>         // std::numeric_limits
#include <set>
#include <string>
#include <vector>
#include <cerrno>         // errno, EINTR
#include <fcntl.h>        // open, O_WRONLY
#include <unistd.h>       // fork, pipe, _exit, read, write, close, dup, dup2
#include <sys/wait.h>     // waitpid
#include <omp.h>          // omp_set_num_threads

#include "odb/db.h"
#include "cut/logic_cut.h"

#include "sta/Corner.hh"
#include "sta/DcalcAnalysisPt.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/InputDrive.hh"
#include "sta/Liberty.hh"
#include "sta/Parasitics.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/PortDirection.hh"
#include "sta/Sdc.hh"
#include "sta/Search.hh"
#include "sta/Sta.hh"
#include "sta/TimingArc.hh"
#include "sta/Transition.hh"
#include "sta/Units.hh"
#include "sta/VerilogWriter.hh"

#include "base/abc/abc.h"
#include "map/mapper/mapper.h"
#include "Strategy.hh"
#include "dpl/Opendp.h"
#include "rmp/SeqRemapper.hh"
#include "rsz/Resizer.hh"
#include "utils.h"

using sta::Edge;
using sta::fuzzyEqual;
using sta::fuzzyGreater;
using sta::fuzzyGreaterEqual;
using sta::fuzzyLess;
using sta::GraphDelayCalc;
using sta::InstancePinIterator;
using sta::NetConnectedPinIterator;
using sta::PathEndSeq;
using sta::PathExpanded;
using sta::Slew;
using sta::Slack;
using sta::Delay;
using sta::VertexInEdgeIterator;
using sta::VertexOutEdgeIterator;

namespace abc {
  //struct Map_MappingSolution_t;
  extern void * Abc_NtkMapEnumPassStore( Abc_Ntk_t * pNtk, int nMaxSolutions, int fVerbose );
  extern void Abc_NtkMapEnumFreeStore( void * pStore );
  extern void Abc_FrameSetLibGen( void * pLib );
  
} // namespace abc

/*
// Helper function to validate if a mapping solution has valid cut assignments
// Returns true if the solution is valid (all critical nodes have cuts assigned)
static bool IsValidMappingSolution(abc::Map_Man_t* pMan, abc::Map_MappingSolution_t* pSol) {
  if (!pSol || !pMan) {
    return false;
  }

  // Check each node in the mapping manager
  for (int i = 0; i < pMan->vMapObjs->nSize; i++) {
    abc::Map_Node_t* pNode = pMan->vMapObjs->pArray[i];
    
    // Skip non-AND nodes and nodes with representatives (they're part of choice nodes)
    if (!abc::Map_NodeIsAnd(pNode) || pNode->pRepr) {
      continue;
    }
    
    int idx = i * 2;
    abc::Map_Cut_t* pCut0 = pSol->pCutBest[idx];
    abc::Map_Cut_t* pCut1 = pSol->pCutBest[idx + 1];
    
    // Check if this node is used (referenced)
    // If it's used, at least one phase must have a cut
    if (pNode->nRefAct[2] > 0 && pCut0 == nullptr && pCut1 == nullptr) {
      return false;
    }
  }
  
  return true;
}
*/
// ---------------------------------------------------------------------------
// Pipe I/O helpers: loop until all bytes are transferred, retrying on EINTR.
// Returns true on success, false if an error or EOF occurs before completion.
// ---------------------------------------------------------------------------
static bool write_all(int fd, const void* buf, size_t count)
{
  const char* p = static_cast<const char*>(buf);
  size_t remaining = count;
  while (remaining > 0) {
    ssize_t n = write(fd, p, remaining);
    if (n <= 0) {
      if (n < 0 && errno == EINTR)
        continue;
      return false;
    }
    p += static_cast<size_t>(n);
    remaining -= static_cast<size_t>(n);
  }
  return true;
}

static bool read_all(int fd, void* buf, size_t count)
{
  char* p = static_cast<char*>(buf);
  size_t remaining = count;
  while (remaining > 0) {
    ssize_t n = read(fd, p, remaining);
    if (n <= 0) {
      if (n < 0 && errno == EINTR)
        continue;
      return false;  // EOF (n==0) or unrecoverable error (n<0)
    }
    p += static_cast<size_t>(n);
    remaining -= static_cast<size_t>(n);
  }
  return true;
}

namespace rmp {


sta::Vertex* PositionDrivenStrategy::getFarthestOutputVertex(
    SeqRemapper& remapper) {
  sta::dbNetwork* network = remapper.getSta()->getDbNetwork();
  sta::Graph* graph = remapper.getSta()->graph();
  sta::Vertex* farthest_vertex = nullptr;
  Slack worst_slack = std::numeric_limits<Slack>::infinity();

  cut::LogicCut bottleneck_cut = remapper.extractBottleneck(*this);

  for (sta::Net* output_net : bottleneck_cut.primary_outputs()) {
    sta::Vertex* output_vertex = nullptr;
    sta::Vertex* bidirect_vertex = nullptr;
    sta::NetPinIterator* pin_iter = network->pinIterator(output_net);
      while (pin_iter->hasNext()) {
        const sta::Pin* pin = pin_iter->next();
        sta::PortDirection* direction = network->direction(pin);
        if (direction->isAnyInput()) {
          graph->pinVertices(pin, output_vertex, bidirect_vertex);
          break;
        }
      }
    delete pin_iter;
    if (output_vertex == nullptr) {
      remapper.getLogger()->error(
          utl::RES, 330, "Output net {} has no vertex.", network->name(output_net));
    }
    Slack slack = remapper.getSta()->vertexSlack(output_vertex, sta::MinMax::max());
    if (slack < worst_slack) {
      worst_slack = slack;
      farthest_vertex = output_vertex;
    }
  }
  return farthest_vertex;
}

// Collect valid (non-don't-touch, input-direction) endpoints, sort worst-first,
// then select a subset based on the three optional parameters:
//   percentage >= 0  : take top N% (min 1)
//   max_percentage >= 0 && slack_threshold < FLT_MAX/2 : filter by threshold, cap at N%
//   otherwise        : return just the single worst endpoint
// Returns endpoint pins (not vertices) sorted worst-slack-first.
// Pins live in the network, not the timing graph, so they remain valid
// after Sta::networkChanged() deletes and rebuilds the graph.
static std::vector<sta::Pin*> selectCandidateEndpoints(
    sta::dbSta* sta,
    rsz::Resizer* resizer,
    float percentage,
    float max_percentage,
    float slack_threshold)
{
  sta::dbNetwork* network = sta->getDbNetwork();
  std::vector<sta::Pin*> all_pins;
  for (sta::Vertex* vertex : *sta->endpoints()) {
    sta::Pin* pin = vertex->pin();
    const sta::PortDirection* direction = network->direction(pin);
    if (!direction->isInput()) {
      continue;
    }
    // Skip clock endpoints — they are not candidates for logic remapping.
    if (vertex->isRegClk() || vertex->isCheckClk()) {
      continue;
    }
    if (resizer != nullptr) {
      if (resizer->dontTouch(pin) || resizer->dontTouch(network->net(pin))
          || resizer->dontTouch(network->instance(pin))) {
        continue;
      }
    }
    all_pins.push_back(pin);
  }

  if (all_pins.empty()) {
    return {};
  }

  // Precompute slack once per pin to avoid repeated STA queries during sort.
  using PinSlackPair = std::pair<sta::Pin*, sta::Slack>;
  std::vector<PinSlackPair> endpoint_slacks;
  endpoint_slacks.reserve(all_pins.size());
  for (sta::Pin* p : all_pins) {
    sta::Vertex* v = sta->graph()->pinLoadVertex(p);
    sta::Slack slack = v ? sta->vertexSlack(v, sta::MinMax::max())
                         : std::numeric_limits<sta::Slack>::infinity();
    endpoint_slacks.emplace_back(p, slack);
  }

  // Sort ascending by slack (most negative = worst first).
  std::sort(endpoint_slacks.begin(), endpoint_slacks.end(),
    [](const PinSlackPair& a, const PinSlackPair& b) {
      return a.second < b.second;
    });

  // Threshold above which slack_threshold is considered "not set".
  const float kNoThreshold = std::numeric_limits<float>::max() / 2.0f;

  if (percentage >= 0.0f) {
    // Percentage mode: fix top N% of all endpoints, at least 1.
    size_t n = static_cast<size_t>(
        std::ceil(static_cast<float>(endpoint_slacks.size()) * percentage / 100.0f));
    n = std::max(n, size_t(1));
    n = std::min(n, endpoint_slacks.size());
    std::vector<sta::Pin*> result;
    result.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      result.push_back(endpoint_slacks[i].first);
    }
    return result;
  }

  if (max_percentage >= 0.0f && slack_threshold < kNoThreshold) {
    // max_percentage + slack_threshold mode: keep endpoints below threshold,
    // capped at max_percentage of the total endpoint count.
    size_t max_n = static_cast<size_t>(
        std::floor(static_cast<float>(endpoint_slacks.size()) * max_percentage / 100.0f));
    std::vector<sta::Pin*> result;
    for (const auto& [p, slack] : endpoint_slacks) {
      if (result.size() >= max_n) {
        break;
      }
      if (slack < slack_threshold) {
        result.push_back(p);
      } else {
        break;  // sorted: no further endpoint will be below threshold
      }
    }
    return result;
  }

  // Default: fix only the single worst endpoint.
  return {endpoint_slacks[0].first};
}

std::vector<sta::Vertex*> PositionDrivenStrategy::getWorstVertices(
    SeqRemapper& remapper,
    float percentage,
    float max_percentage,
    float slack_threshold) {
  // Return vertices on the most critical path sorted by load-dependent delay
  // (largest first). If the top candidate yields a single-instance cut,
  // the caller can fall back to the next candidate.
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Graph* graph = sta->graph();

  // Disable incremental timing.
  sta->graphDelayCalc()->delaysInvalid();
  sta->search()->arrivalsInvalid();
  sta->search()->endpointsInvalid();

  cut::LogicExtractorFactory logic_extractor(sta, remapper.getLogger());
  auto candidate_pins = selectCandidateEndpoints(
      sta, remapper.getResizer(), percentage, max_percentage, slack_threshold);

  for (sta::Pin* endpoint_pin : candidate_pins) {
    sta::Vertex* v = nullptr;
    sta::Vertex* bidir_v = nullptr;
    graph->pinVertices(endpoint_pin, v, bidir_v);
    if (v) {
      logic_extractor.AppendEndpoint(v);
    }
  }
  abc_library_ = remapper.getAbcLibrary();
  cut::LogicCut bad_cut = logic_extractor.BuildLogicCut(*abc_library_);

  // 1) Find the worst (most negative slack) endpoint vertex among cut outputs.
  sta::Vertex* worst_end_vertex = nullptr;
  Slack worst_slack = std::numeric_limits<Slack>::infinity();

  for (sta::Net* output_net : bad_cut.primary_outputs()) {
    sta::Vertex* output_vertex = nullptr;
    sta::Vertex* bidirect_vertex = nullptr;

    sta::NetPinIterator* pin_iter = network->pinIterator(output_net);
    while (pin_iter->hasNext()) {
      const sta::Pin* pin = pin_iter->next();
      sta::PortDirection* direction = network->direction(pin);
      if (direction && direction->isAnyInput()) {
        graph->pinVertices(pin, output_vertex, bidirect_vertex);
        break;
      }
    }
    delete pin_iter;

    if (output_vertex == nullptr) {
      remapper.getLogger()->warn(
          utl::RES, 331, "Output net {} has no endpoint vertex.", network->name(output_net));
      continue;
    }

    const Slack slack = sta->vertexSlack(output_vertex, sta::MinMax::max());
    if (slack < worst_slack) {
      worst_slack = slack;
      worst_end_vertex = output_vertex;
    }
  }

  if (worst_end_vertex == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 332, "No valid endpoint vertex found in bottleneck cut outputs.");
    return {};
  }

  // 2) Get the worst-slack path to that endpoint and expand it.
  sta::Path* end_path = sta->vertexWorstSlackPath(worst_end_vertex, sta::MinMax::max());
  if (end_path == nullptr) {
    remapper.getLogger()->warn(
        utl::RES, 333, "No worst-slack path found for endpoint {}.",
        worst_end_vertex->name(network));
    return {};
  }

  sta::PathExpanded expanded(end_path, sta);

  // 3) Walk the path and collect all driver vertices with their load-dependent delay.
  using VertexDelayPair = std::pair<sta::Vertex*, Delay>;
  std::vector<VertexDelayPair> vertex_delays;

  if (expanded.size() > 1) {
    const int path_length = expanded.size();
    const int start_index = expanded.startIndex();
    const sta::DcalcAnalysisPt* dcalc_ap = end_path->dcalcAnalysisPt(sta);
    const int lib_ap = dcalc_ap ? dcalc_ap->libertyIndex() : 0;
    const int dcalc_index = dcalc_ap ? dcalc_ap->index() : 0;

    for (int i = start_index; i < path_length; i++) {
      const sta::Path* path_i = expanded.path(i);
      if (path_i == nullptr) {
        continue;
      }

      sta::Vertex* path_vertex = path_i->vertex(sta);
      const sta::Pin* path_pin = path_i->pin(sta);
      if (path_vertex == nullptr || path_pin == nullptr) {
        continue;
      }

      if (i > 0 && path_vertex->isDriver(network)
          && !network->isTopLevelPort(path_pin)
          && !sta->search()->isClock(path_vertex)) {
        const sta::TimingArc* prev_arc = path_i->prevArc(sta);
        sta::Edge* prev_edge = path_i->prevEdge(sta);
        if (prev_arc == nullptr || prev_edge == nullptr) {
          continue;
        }

        const sta::TimingArc* corner_arc = prev_arc->cornerArc(lib_ap);
        if (corner_arc == nullptr) {
          continue;
        }

        const Delay arc_delay = graph->arcDelay(prev_edge, prev_arc, dcalc_index);
        const Delay load_delay = arc_delay - corner_arc->intrinsicDelay();

        vertex_delays.emplace_back(path_vertex, load_delay);
      }
    }
  }

  // Sort by load-dependent delay descending (largest first).
  std::sort(vertex_delays.begin(), vertex_delays.end(),
    [](const VertexDelayPair& a, const VertexDelayPair& b) {
      return a.second > b.second;
    });

  // Debug: print detailed info for each vertex in vertex_delays.
  {
    const sta::DcalcAnalysisPt* dbg_dcalc_ap = end_path->dcalcAnalysisPt(sta);
    const int dcalc_index = dbg_dcalc_ap ? dbg_dcalc_ap->index() : 0;
    std::set<sta::Vertex*> printed;
    for (const auto& [vtx, load_delay] : vertex_delays) {
      if (!printed.insert(vtx).second) {
        continue;  // skip duplicates
      }
      const sta::Pin* vtx_pin = vtx->pin();
      sta::Instance* vtx_inst = network->instance(vtx_pin);
      if (vtx_inst == nullptr) {
        continue;
      }
      sta::LibertyCell* lib_cell = network->libertyCell(vtx_inst);
      const char* cell_type = lib_cell ? lib_cell->name() : "unknown";
      const char* inst_name = network->name(vtx_inst);

      // Instance position.
      odb::dbInst* db_inst = network->staToDb(vtx_inst);
      int inst_x = 0, inst_y = 0;
      if (db_inst) {
        db_inst->getLocation(inst_x, inst_y);
      }

      logger_->info(utl::RES, 380,
          "[VertexDelayInfo] Instance: {} | Cell: {} | Position: ({}, {}) | LoadDelay: {}",
          inst_name, cell_type, inst_x, inst_y, sta::delayAsFloat(load_delay));

      // Iterate over all pins of this instance.
      sta::InstancePinIterator* pin_it = network->pinIterator(vtx_inst);
      while (pin_it->hasNext()) {
        sta::Pin* pin = pin_it->next();
        sta::PortDirection* dir = network->direction(pin);
        const char* pin_name = network->portName(pin);
        bool is_input = dir && dir->isAnyInput();
        bool is_output = dir && dir->isAnyOutput();
        const char* dir_str = is_input ? "FANIN" : (is_output ? "FANOUT" : "OTHER");

        // Pin slack.
        sta::Vertex* pin_vertex = nullptr;
        sta::Vertex* pin_bidir = nullptr;
        graph->pinVertices(pin, pin_vertex, pin_bidir);
        Slack pin_slack = std::numeric_limits<Slack>::infinity();
        if (pin_vertex) {
          pin_slack = sta->vertexSlack(pin_vertex, sta::MinMax::max());
        }

        // For input pins: find fanin instance and compute HPWL distance + delay.
        // For output pins: find fanout instances and compute HPWL distance + delay.
        sta::Net* pin_net = network->net(pin);
        if (pin_net == nullptr) {
          logger_->info(utl::RES, 381,
              "  Pin: {} | Dir: {} | Slack: {} | (no net)",
              pin_name, dir_str, sta::delayAsFloat(pin_slack));
          continue;
        }

        // Collect connected instances with HPWL distance.
        std::string connected_info;
        sta::NetPinIterator* net_pin_it = network->pinIterator(pin_net);
        while (net_pin_it->hasNext()) {
          const sta::Pin* connected_pin = net_pin_it->next();
          sta::Instance* connected_inst = network->instance(connected_pin);
          if (connected_inst == nullptr || connected_inst == vtx_inst) {
            continue;
          }
          odb::dbInst* conn_db_inst = network->staToDb(connected_inst);
          int conn_x = 0, conn_y = 0;
          if (conn_db_inst) {
            conn_db_inst->getLocation(conn_x, conn_y);
          }
          int hpwl = std::abs(conn_x - inst_x) + std::abs(conn_y - inst_y);
          const char* conn_name = network->name(connected_inst);
          sta::LibertyCell* conn_cell = network->libertyCell(connected_inst);
          const char* conn_type = conn_cell ? conn_cell->name() : "unknown";
          if (!connected_info.empty()) {
            connected_info += "; ";
          }
          connected_info += std::string(conn_name) + "(" + conn_type + ") HPWL=" + std::to_string(hpwl);
        }
        delete net_pin_it;

        // Collect delay to fanin/fanout instances via graph edges.
        // Helper lambda: get max delay across all arcs of an edge.
        auto getEdgeMaxDelay = [&](sta::Edge* edge) -> float {
          float max_delay = 0.0f;
          if (edge->isWire()) {
            // Wire edges use wireArcDelay with rise/fall.
            for (const auto* rf : sta::RiseFall::range()) {
              float d = sta::delayAsFloat(graph->wireArcDelay(edge, rf, dcalc_index));
              if (d > max_delay) max_delay = d;
            }
          } else {
            // Cell edges use arcDelay with timing arcs.
            sta::TimingArcSet* arc_set = edge->timingArcSet();
            if (arc_set) {
              for (const sta::TimingArc* arc : arc_set->arcs()) {
                float d = sta::delayAsFloat(graph->arcDelay(edge, arc, dcalc_index));
                if (d > max_delay) max_delay = d;
              }
            }
          }
          return max_delay;
        };

        std::string delay_info;
        if (pin_vertex) {
          if (is_input) {
            // Walk incoming edges to find fanin drivers and their delays.
            VertexInEdgeIterator in_iter(pin_vertex, graph);
            while (in_iter.hasNext()) {
              sta::Edge* edge = in_iter.next();
              sta::Vertex* from_vtx = edge->from(graph);
              if (from_vtx == nullptr) continue;
              sta::Instance* from_inst = network->instance(from_vtx->pin());
              if (from_inst == nullptr) continue;
              const char* from_name = network->name(from_inst);
              float max_delay = getEdgeMaxDelay(edge);
              if (!delay_info.empty()) delay_info += "; ";
              delay_info += "from " + std::string(from_name)
                  + " delay=" + std::to_string(max_delay);
            }
          } else if (is_output) {
            // Walk outgoing edges to find fanout sinks and their delays.
            VertexOutEdgeIterator out_iter(pin_vertex, graph);
            while (out_iter.hasNext()) {
              sta::Edge* edge = out_iter.next();
              sta::Vertex* to_vtx = edge->to(graph);
              if (to_vtx == nullptr) continue;
              sta::Instance* to_inst = network->instance(to_vtx->pin());
              if (to_inst == nullptr) continue;
              const char* to_name = network->name(to_inst);
              float max_delay = getEdgeMaxDelay(edge);
              if (!delay_info.empty()) delay_info += "; ";
              delay_info += "to " + std::string(to_name)
                  + " delay=" + std::to_string(max_delay);
            }
          }
        }

        logger_->info(utl::RES, 382,
            "  Pin: {} | Dir: {} | Slack: {} | Connected: [{}] | Delays: [{}]",
            pin_name, dir_str, sta::delayAsFloat(pin_slack),
            connected_info.empty() ? "none" : connected_info,
            delay_info.empty() ? "none" : delay_info);
      }
      delete pin_it;
    }
  }

  // Build result vector (deduplicated, preserving order).
  std::vector<sta::Vertex*> result;
  std::set<sta::Vertex*> seen;
  const size_t max_candidates = 100;
  for (auto& [v, d] : vertex_delays) {
    if (result.size() >= max_candidates) break;
    if (seen.insert(v).second) {
      result.push_back(v);
    }
  }

  // If no driver vertices were found, fall back to the endpoint vertex itself.
  if (result.empty()) {
    result.push_back(worst_end_vertex);
  }

  return result;
}

std::vector<sta::Vertex*> PositionDrivenStrategy::getWorstVerticesForEndpoint(
    SeqRemapper& remapper,
    sta::Vertex* endpoint) {
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Graph* graph = sta->graph();

  abc_library_ = remapper.getAbcLibrary();

  sta::Path* end_path = sta->vertexWorstSlackPath(endpoint, sta::MinMax::max());
  if (end_path == nullptr) {
    logger_->warn(utl::RES, 350,
                  "No worst-slack path found for endpoint {}.",
                  endpoint->name(network));
    return {};
  }

  sta::PathExpanded expanded(end_path, sta);

  using VertexDelayPair = std::pair<sta::Vertex*, Delay>;
  std::vector<VertexDelayPair> vertex_delays;

  if (expanded.size() > 1) {
    const int path_length = expanded.size();
    const int start_index = expanded.startIndex();
    const sta::DcalcAnalysisPt* dcalc_ap = end_path->dcalcAnalysisPt(sta);
    const int lib_ap = dcalc_ap ? dcalc_ap->libertyIndex() : 0;
    const int dcalc_index = dcalc_ap ? dcalc_ap->index() : 0;

    for (int i = start_index; i < path_length; i++) {
      const sta::Path* path_i = expanded.path(i);
      if (path_i == nullptr) {
        continue;
      }

      sta::Vertex* path_vertex = path_i->vertex(sta);
      const sta::Pin* path_pin = path_i->pin(sta);
      if (path_vertex == nullptr || path_pin == nullptr) {
        continue;
      }

      if (i > 0 && path_vertex->isDriver(network)
          && !network->isTopLevelPort(path_pin)
          && !sta->search()->isClock(path_vertex)) {
        const sta::TimingArc* prev_arc = path_i->prevArc(sta);
        sta::Edge* prev_edge = path_i->prevEdge(sta);
        if (prev_arc == nullptr || prev_edge == nullptr) {
          continue;
        }

        const sta::TimingArc* corner_arc = prev_arc->cornerArc(lib_ap);
        if (corner_arc == nullptr) {
          continue;
        }

        const Delay arc_delay = graph->arcDelay(prev_edge, prev_arc, dcalc_index);
        const Delay load_delay = arc_delay - corner_arc->intrinsicDelay();

        vertex_delays.emplace_back(path_vertex, load_delay);
      }
    }
  }

  std::sort(vertex_delays.begin(), vertex_delays.end(),
    [](const VertexDelayPair& a, const VertexDelayPair& b) {
      return a.second > b.second;
    });

  // Debug: print detailed info for each vertex in vertex_delays.
  {
    const sta::DcalcAnalysisPt* dbg_dcalc_ap = end_path->dcalcAnalysisPt(sta);
    const int dcalc_index = dbg_dcalc_ap ? dbg_dcalc_ap->index() : 0;
    std::set<sta::Vertex*> printed;
    for (const auto& [vtx, load_delay] : vertex_delays) {
      if (!printed.insert(vtx).second) {
        continue;  // skip duplicates
      }
      const sta::Pin* vtx_pin = vtx->pin();
      sta::Instance* vtx_inst = network->instance(vtx_pin);
      if (vtx_inst == nullptr) {
        continue;
      }
      sta::LibertyCell* lib_cell = network->libertyCell(vtx_inst);
      const char* cell_type = lib_cell ? lib_cell->name() : "unknown";
      const char* inst_name = network->name(vtx_inst);

      // Instance position.
      odb::dbInst* db_inst = network->staToDb(vtx_inst);
      int inst_x = 0, inst_y = 0;
      if (db_inst) {
        db_inst->getLocation(inst_x, inst_y);
      }

      logger_->info(utl::RES, 388,
          "[VertexDelayInfo] Instance: {} | Cell: {} | Position: ({}, {}) | LoadDelay: {}",
          inst_name, cell_type, inst_x, inst_y, sta::delayAsFloat(load_delay));

      // Iterate over all pins of this instance.
      sta::InstancePinIterator* pin_it = network->pinIterator(vtx_inst);
      while (pin_it->hasNext()) {
        sta::Pin* pin = pin_it->next();
        sta::PortDirection* dir = network->direction(pin);
        const char* pin_name = network->portName(pin);
        bool is_input = dir && dir->isAnyInput();
        bool is_output = dir && dir->isAnyOutput();
        const char* dir_str = is_input ? "FANIN" : (is_output ? "FANOUT" : "OTHER");

        // Pin slack.
        sta::Vertex* pin_vertex = nullptr;
        sta::Vertex* pin_bidir = nullptr;
        graph->pinVertices(pin, pin_vertex, pin_bidir);
        Slack pin_slack = std::numeric_limits<Slack>::infinity();
        if (pin_vertex) {
          pin_slack = sta->vertexSlack(pin_vertex, sta::MinMax::max());
        }

        // For input pins: find fanin instance and compute HPWL distance + delay.
        // For output pins: find fanout instances and compute HPWL distance + delay.
        sta::Net* pin_net = network->net(pin);
        if (pin_net == nullptr) {
          logger_->info(utl::RES, 389,
              "  Pin: {} | Dir: {} | Slack: {} | (no net)",
              pin_name, dir_str, sta::delayAsFloat(pin_slack));
          continue;
        }

        // Collect connected instances with HPWL distance.
        std::string connected_info;
        sta::NetPinIterator* net_pin_it = network->pinIterator(pin_net);
        while (net_pin_it->hasNext()) {
          const sta::Pin* connected_pin = net_pin_it->next();
          sta::Instance* connected_inst = network->instance(connected_pin);
          if (connected_inst == nullptr || connected_inst == vtx_inst) {
            continue;
          }
          odb::dbInst* conn_db_inst = network->staToDb(connected_inst);
          int conn_x = 0, conn_y = 0;
          if (conn_db_inst) {
            conn_db_inst->getLocation(conn_x, conn_y);
          }
          int hpwl = std::abs(conn_x - inst_x) + std::abs(conn_y - inst_y);
          const char* conn_name = network->name(connected_inst);
          sta::LibertyCell* conn_cell = network->libertyCell(connected_inst);
          const char* conn_type = conn_cell ? conn_cell->name() : "unknown";
          if (!connected_info.empty()) {
            connected_info += "; ";
          }
          connected_info += std::string(conn_name) + "(" + conn_type + ") HPWL=" + std::to_string(hpwl);
        }
        delete net_pin_it;

        // Collect delay to fanin/fanout instances via graph edges.
        auto getEdgeMaxDelay = [&](sta::Edge* edge) -> float {
          float max_delay = 0.0f;
          if (edge->isWire()) {
            for (const auto* rf : sta::RiseFall::range()) {
              float d = sta::delayAsFloat(graph->wireArcDelay(edge, rf, dcalc_index));
              if (d > max_delay) max_delay = d;
            }
          } else {
            sta::TimingArcSet* arc_set = edge->timingArcSet();
            if (arc_set) {
              for (const sta::TimingArc* arc : arc_set->arcs()) {
                float d = sta::delayAsFloat(graph->arcDelay(edge, arc, dcalc_index));
                if (d > max_delay) max_delay = d;
              }
            }
          }
          return max_delay;
        };

        std::string delay_info;
        if (pin_vertex) {
          if (is_input) {
            VertexInEdgeIterator in_iter(pin_vertex, graph);
            while (in_iter.hasNext()) {
              sta::Edge* edge = in_iter.next();
              sta::Vertex* from_vtx = edge->from(graph);
              if (from_vtx == nullptr) continue;
              sta::Instance* from_inst = network->instance(from_vtx->pin());
              if (from_inst == nullptr) continue;
              const char* from_name = network->name(from_inst);
              float max_delay = getEdgeMaxDelay(edge);
              if (!delay_info.empty()) delay_info += "; ";
              delay_info += "from " + std::string(from_name)
                  + " delay=" + std::to_string(max_delay);
            }
          } else if (is_output) {
            VertexOutEdgeIterator out_iter(pin_vertex, graph);
            while (out_iter.hasNext()) {
              sta::Edge* edge = out_iter.next();
              sta::Vertex* to_vtx = edge->to(graph);
              if (to_vtx == nullptr) continue;
              sta::Instance* to_inst = network->instance(to_vtx->pin());
              if (to_inst == nullptr) continue;
              const char* to_name = network->name(to_inst);
              float max_delay = getEdgeMaxDelay(edge);
              if (!delay_info.empty()) delay_info += "; ";
              delay_info += "to " + std::string(to_name)
                  + " delay=" + std::to_string(max_delay);
            }
          }
        }

        logger_->info(utl::RES, 390,
            "  Pin: {} | Dir: {} | Slack: {} | Connected: [{}] | Delays: [{}]",
            pin_name, dir_str, sta::delayAsFloat(pin_slack),
            connected_info.empty() ? "none" : connected_info,
            delay_info.empty() ? "none" : delay_info);
      }
      delete pin_it;
    }
  }

  std::vector<sta::Vertex*> result;
  std::set<sta::Vertex*> seen;
  const size_t max_candidates = 100;
  for (auto& [v, d] : vertex_delays) {
    if (result.size() >= max_candidates) break;
    if (seen.insert(v).second) {
      result.push_back(v);
    }
  }

  if (result.empty()) {
    result.push_back(endpoint);
  }

  return result;
}

bool PositionDrivenStrategy::remapOneCut(
    SeqRemapper& remapper,
    std::vector<sta::Vertex*>& worst_vertices) {
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();

  // Try each candidate vertex in order of decreasing load-dependent delay.
  // Skip vertices whose extracted cut contains only 1 instance.
  sta::Instance* bad_instance = nullptr;
  cut::LogicCut candidate_cut({}, {}, {});
  bool found_valid_cut = false;

  for (size_t vi = 0; vi < worst_vertices.size(); ++vi) {
    sta::Vertex* bad_vertex = worst_vertices[vi];
    sta::Instance* inst = network->instance(bad_vertex->pin());
    if (inst == nullptr) {
      logger_->info(utl::RES, 397,
                    "[Step1] Candidate {} vertex {} has no instance, skipping.",
                    vi, bad_vertex->name(network));
      continue;
    }
    sta::LibertyCell* cell = network->libertyCell(inst);
    if (cell == nullptr
        || !remapper.getAbcLibrary()->IsSupportedCell(cell->name())) {
      logger_->info(utl::RES, 398,
                    "[Step1] Candidate {} vertex {} cell type ({}) not supported by ABC, skipping.",
                    vi, bad_vertex->name(network),
                    cell ? cell->name() : "unknown");
      continue;
    }

    logger_->info(utl::RES, 406,
                  "[Step1] Candidate {}: bad_instance name={}, type={}",
                  vi, network->name(inst), cell->name());

    // Fanin instances
    sta::InstancePinIterator* pin_it = network->pinIterator(inst);
    while (pin_it->hasNext()) {
      sta::Pin* pin = pin_it->next();
      sta::PortDirection* dir = network->direction(pin);
      if (!dir->isInput()) continue;
      sta::Net* net = network->net(pin);
      if (!net) continue;
      sta::NetPinIterator* npi = network->pinIterator(net);
      while (npi->hasNext()) {
        const sta::Pin* cp = npi->next();
        sta::Instance* ci = network->instance(cp);
        if (ci == inst || network->isTopInstance(ci)) continue;
        if (network->direction(cp)->isAnyOutput()) {
          sta::LibertyCell* fc = network->libertyCell(ci);
          logger_->info(utl::RES, 391,
                        "[Step1]   fanin: name={}, type={}",
                        network->name(ci), fc ? fc->name() : "unknown");
        }
      }
      delete npi;
    }
    delete pin_it;

    // Fanout instances
    pin_it = network->pinIterator(inst);
    while (pin_it->hasNext()) {
      sta::Pin* pin = pin_it->next();
      sta::PortDirection* dir = network->direction(pin);
      if (!dir->isAnyOutput()) continue;
      sta::Net* net = network->net(pin);
      if (!net) continue;
      sta::NetPinIterator* npi = network->pinIterator(net);
      while (npi->hasNext()) {
        const sta::Pin* cp = npi->next();
        sta::Instance* ci = network->instance(cp);
        if (ci == inst || network->isTopInstance(ci)) continue;
        if (network->direction(cp)->isInput()) {
          sta::LibertyCell* fc = network->libertyCell(ci);
          logger_->info(utl::RES, 392,
                        "[Step1]   fanout: name={}, type={}",
                        network->name(ci), fc ? fc->name() : "unknown");
        }
      }
      delete npi;
    }
    delete pin_it;

    // Extract the bottleneck cut around this vertex.
    setRefGate(inst);
    cut::LogicCut trial_cut = extractBottleneck(remapper);

    logger_->info(utl::RES, 393,
                  "[Step2] candidate_cut: {} instances, {} PIs, {} POs",
                  trial_cut.cut_instances().size(),
                  trial_cut.primary_inputs().size(),
                  trial_cut.primary_outputs().size());

    for (const sta::Instance* ci : trial_cut.cut_instances()) {
      if (!ci) continue;
      sta::LibertyCell* cc = network->libertyCell(ci);
      logger_->info(utl::RES, 394,
                    "[Step2]   instance: name={}, type={}",
                    network->name(ci), cc ? cc->name() : "unknown");

      sta::InstancePinIterator* pi = network->pinIterator(ci);
      while (pi->hasNext()) {
        sta::Pin* p = pi->next();
        sta::PortDirection* d = network->direction(p);
        if (!d->isInput()) continue;
        sta::Net* n = network->net(p);
        if (!n) continue;
        sta::NetPinIterator* npi = network->pinIterator(n);
        while (npi->hasNext()) {
          const sta::Pin* cp = npi->next();
          sta::Instance* fi = network->instance(cp);
          if (fi == ci || network->isTopInstance(fi)) continue;
          if (network->direction(cp)->isAnyOutput()) {
            sta::LibertyCell* fc = network->libertyCell(fi);
            logger_->info(utl::RES, 395,
                          "[Step2]     fanin: name={}, type={}",
                          network->name(fi), fc ? fc->name() : "unknown");
          }
        }
        delete npi;
      }
      delete pi;

      pi = network->pinIterator(ci);
      while (pi->hasNext()) {
        sta::Pin* p = pi->next();
        sta::PortDirection* d = network->direction(p);
        if (!d->isAnyOutput()) continue;
        sta::Net* n = network->net(p);
        if (!n) continue;
        sta::NetPinIterator* npi = network->pinIterator(n);
        while (npi->hasNext()) {
          const sta::Pin* cp = npi->next();
          sta::Instance* fo = network->instance(cp);
          if (fo == ci || network->isTopInstance(fo)) continue;
          if (network->direction(cp)->isInput()) {
            sta::LibertyCell* fc = network->libertyCell(fo);
            logger_->info(utl::RES, 396,
                          "[Step2]     fanout: name={}, type={}",
                          network->name(fo), fc ? fc->name() : "unknown");
          }
        }
        delete npi;
      }
      delete pi;
    }

    if (trial_cut.cut_instances().size() <= 1) {
      logger_->info(utl::RES, 399,
                    "[Step1] Candidate {} cut has only {} instance(s), trying next vertex.",
                    vi, trial_cut.cut_instances().size());
      continue;
    }

    bad_instance = inst;
    candidate_cut = std::move(trial_cut);
    found_valid_cut = true;
    break;
  }

  if (!found_valid_cut) {
    logger_->warn(utl::RES, 375,
        "All candidate vertices produced cuts with <= 1 instance, nothing to remap.");
    return false;
  }
  setCandidateCut(candidate_cut);

  // When not verbose, redirect stdout to suppress ABC output.
  int saved_stdout = -1;
  if (!verbose_) {
    fflush(stdout);
    saved_stdout = dup(STDOUT_FILENO);
    int devnull = open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      dup2(devnull, STDOUT_FILENO);
      close(devnull);
    }
  }

  // Build the ABC network from the candidate cut.
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network(
      candidate_cut.BuildMappedAbcNetwork(
          *remapper.getAbcLibrary(),
          network,
          remapper.getLogger()).release(),
      &abc::Abc_NtkDelete);

  if (mapped_abc_network == nullptr) {
    // Restore stdout before returning.
    if (saved_stdout >= 0) {
      fflush(stdout);
      dup2(saved_stdout, STDOUT_FILENO);
      close(saved_stdout);
    }
    logger_->error(utl::RES, 335, "Failed to build ABC network from candidate cut.");
    return false;
  }

  auto library = static_cast<abc::Mio_Library_t*>(mapped_abc_network.get()->pManFunc);
  if (library == nullptr) {
    if (saved_stdout >= 0) {
      fflush(stdout);
      dup2(saved_stdout, STDOUT_FILENO);
      close(saved_stdout);
    }
    logger_->error(utl::RES, 341, "ABC network does not have an associated library.");
    return false;
  }
  abc::Abc_FrameSetLibGen(library);

  // Convert the mapped network to logic (AIG) form for enumeration.
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> logic_network(
      abc::Abc_NtkToLogic(mapped_abc_network.get()),
      &abc::Abc_NtkDelete);

  if (logic_network == nullptr) {
    if (saved_stdout >= 0) {
      fflush(stdout);
      dup2(saved_stdout, STDOUT_FILENO);
      close(saved_stdout);
    }
    logger_->error(utl::RES, 340, "Failed to convert ABC network to logic form.");
    return false;
  }

  logger_->info(utl::RES, 351,
      "After step 4. ABC network converted to logic form with {} nodes.",
      abc::Abc_NtkNodeNum(logic_network.get()));

  // Enumerate all possible mapping solutions using ABC.
  int nMaxSolutions = 80;
  int fVerbose = verbose_ ? 1 : 0;

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> strashed_network(
      abc::Abc_NtkStrash(logic_network.get(), 0, 0, 0),
      &abc::Abc_NtkDelete);

  void* pMan = abc::Abc_NtkMapEnumPassStore(
      strashed_network.get(),
      nMaxSolutions,
      fVerbose);

  // Restore stdout after ABC calls.
  if (saved_stdout >= 0) {
    fflush(stdout);
    dup2(saved_stdout, STDOUT_FILENO);
    close(saved_stdout);
  }

  if (pMan == nullptr) {
    logger_->warn(utl::RES, 353, "ABC mapping enumeration returned no solutions.");
    return false;
  }

  logger_->info(utl::RES, 352, "After step 5.");

  // Evaluate each solution.
  abc::Map_Man_t* map_man = static_cast<abc::Map_Man_t*>(pMan);
  const int num_solutions = abc::Map_ManReadNumSolutions(map_man);
  if (num_solutions <= 0) {
    logger_->warn(utl::RES, 354, "ABC mapping enumeration returned no solutions.");
    abc::Abc_NtkMapEnumFreeStore(pMan);
    return false;
  }

  logger_->info(utl::RES, 359, "Found {} enumerated solutions to evaluate.", num_solutions);

  abc::Map_MappingSolution_t* pSolutionBest = nullptr;
  sta::Slack best_slack = std::numeric_limits<sta::Slack>::lowest();
  int best_solution_index = -1;
  int evaluated_count = 0;

  auto results = forkEvaluateSolutions(
      map_man, logic_network.get(), candidate_cut, remapper,
      0, num_solutions);

  for (auto& res : results) {
    int idx = res.solution_index;
    logger_->info(utl::RES, 355, "--- Solution {}/{} ---", idx + 1, num_solutions);

    if (!res.success) {
      logger_->warn(utl::RES, 358, "Solution {} child failed.", idx + 1);
      continue;
    }

    if (!res.log.empty()){
      logger_->reportLiteral(res.log);
    }

    abc::Map_MappingSolutionSetEvalResult(
        res.pSolution, static_cast<float>(res.slack));

    if (res.slack > best_slack) {
      best_slack = res.slack;
      pSolutionBest = res.pSolution;
      best_solution_index = idx;
      logger_->info(utl::RES, 362, "Solution {} is new best (slack={:.4e}).",
                    idx + 1, res.slack);
    } else {
      logger_->info(utl::RES, 363, "Solution {} (slack={:.4e}) not better than best ({:.4e}).",
                    idx + 1, res.slack, best_slack);
    }
    evaluated_count++;
  }

  logger_->info(utl::RES, 360,
               "Enumeration evaluation complete: {} solutions evaluated.",
               evaluated_count);

  // UCT iterative phase
  const int nBatchSize = 20;
  const int nRounds = 5;
  const double uctC = 1.414;

  abc::Map_ManSetMaxSolutions(map_man, nMaxSolutions + nBatchSize * nRounds);
  abc::Map_UctMan_t* pUct = abc::Map_UctBegin(map_man, uctC);

  if (pUct != nullptr) {
    for (int round = 0; round < nRounds; round++) {
      int nBefore = abc::Map_ManReadNumSolutions(map_man);

      int nNew = abc::Map_UctGenerateBatch(pUct, nBatchSize);
      if (nNew == 0) {
        logger_->info(utl::RES, 371,
                      "UCT round {}/{}: no new unique solutions, stopping.",
                      round + 1, nRounds);
        break;
      }

      int nAfter = abc::Map_ManReadNumSolutions(map_man);
      logger_->info(utl::RES, 373,
                    "UCT round {}/{}: generated {} new solutions.",
                    round + 1, nRounds, nNew);

      auto round_results = forkEvaluateSolutions(
          map_man, logic_network.get(), candidate_cut, remapper,
          nBefore, nAfter);

      for (auto& res : round_results) {
        int idx = res.solution_index;

        if (!res.success) {
          logger_->warn(utl::RES, 372,
                        "UCT solution {} child failed.", idx + 1);
          continue;
        }

        if (!res.log.empty()){
          logger_->reportLiteral(res.log);
        }

        abc::Map_MappingSolutionSetEvalResult(
            res.pSolution, static_cast<float>(res.slack));

        if (res.slack > best_slack) {
          best_slack = res.slack;
          pSolutionBest = res.pSolution;
          best_solution_index = idx;
          logger_->info(utl::RES, 368,
                        "UCT solution {} is new best (slack={:.4e}).",
                        idx + 1, res.slack);
        } else {
          logger_->info(utl::RES, 369,
                        "UCT solution {} (slack={:.4e}) not better than best ({:.4e}).",
                        idx + 1, res.slack, best_slack);
        }
        evaluated_count++;
      }

      abc::Map_UctUpdateRewards(pUct, nBefore, nAfter);
    }

    logger_->info(utl::RES, 370,
                 "UCT evaluation complete: total {} solutions evaluated.",
                 evaluated_count);

    abc::Map_UctEnd(pUct);
  } else {
    logger_->info(utl::RES, 374, "UCT initialization failed, skipping.");
  }

  // Apply the best solution.
  bool applied = false;
  if (pSolutionBest) {
    logger_->info(utl::RES, 346,
        "Best solution found (index {}) with worst slack = {:.4e}",
        best_solution_index + 1, best_slack);

    // Suppress ABC output when not verbose.
    int saved_stdout2 = -1;
    if (!verbose_) {
      fflush(stdout);
      saved_stdout2 = dup(STDOUT_FILENO);
      int devnull = open("/dev/null", O_WRONLY);
      if (devnull >= 0) {
        dup2(devnull, STDOUT_FILENO);
        close(devnull);
      }
    }

    candidate_cut.InsertAbcMapSolution(
        pSolutionBest,
        map_man,
        logic_network.get(),
        *remapper.getAbcLibrary(),
        network,
        sta,
        remapper.getNameGenerator(),
        logger_);

    if (saved_stdout2 >= 0) {
      fflush(stdout);
      dup2(saved_stdout2, STDOUT_FILENO);
      close(saved_stdout2);
    }

    // Place the newly inserted cells near the cut centroid.
    // Skip evaluateSolution() in the parent — STA graph operations
    // (networkChanged/updateTiming/repairSetup) are fragile and can crash.
    // The children already evaluated solutions safely via fork(); the parent
    // only needs the netlist and placement changes.
    remapper.performIncreDpl(candidate_cut, remapper.getDpl());

    logger_->info(utl::RES, 364, "Best solution permanently applied.");
    applied = true;
  } else {
    logger_->warn(utl::RES, 361, "No valid solution found to apply.");
  }

  abc::Abc_NtkMapEnumFreeStore(pMan);
  return applied;
}

void PositionDrivenStrategy::remap(SeqRemapper& remapper,
                                    float percentage,
                                    float max_percentage,
                                    float slack_threshold,
                                    bool run_detailed_placement,
                                    bool verbose) {
  verbose_ = verbose;
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();

  logger_->info(utl::RES, 408, "[remap] enter");

  // Invalidate timing so selectCandidateEndpoints sees fresh slacks.
  sta->graphDelayCalc()->delaysInvalid();
  sta->search()->arrivalsInvalid();
  sta->search()->endpointsInvalid();

  logger_->info(utl::RES, 409, "[remap] after invalidation, before selectCandidateEndpoints");

  // Get all candidate endpoints sorted by slack (worst first).
  // Store pins (not vertices): Sta::networkChanged() deletes and rebuilds the
  // timing graph on each successful remap, invalidating all Vertex* pointers.
  // Pins live in the network and survive graph rebuilds.
  auto candidate_pins = selectCandidateEndpoints(
      sta, remapper.getResizer(), percentage, max_percentage, slack_threshold);

  logger_->info(utl::RES, 410, "[remap] after selectCandidateEndpoints, got {} pins",
                candidate_pins.size());

  if (candidate_pins.empty()) {
    logger_->warn(utl::RES, 334, "No candidate endpoints found.");
    return;
  }

  logger_->info(utl::RES, 400,
                "Found {} candidate endpoints to process iteratively.",
                candidate_pins.size());

  // Maximum number of worst vertices to attempt fixing per endpoint.
  const size_t max_vertices_per_endpoint = 5;

  int remapped_count = 0;

  for (size_t ep_idx = 0; ep_idx < candidate_pins.size(); ++ep_idx) {
    sta::Pin* endpoint_pin = candidate_pins[ep_idx];

    logger_->info(utl::RES, 411, "[remap] iter {}: before graph lookup", ep_idx);

    // Look up a fresh vertex each iteration.  evaluateSolution() calls
    // sta->networkChanged() which deletes graph_ and rebuilds it via
    // updateTiming(), so any Vertex* held across iterations would be dangling.
    sta::Graph* iter_graph = sta->ensureGraph();
    if (iter_graph == nullptr) {
      logger_->warn(utl::RES, 412,
                    "Iteration {}: graph is null after ensureGraph, skipping.",
                    ep_idx + 1);
      continue;
    }
    sta::Vertex* endpoint = nullptr;
    sta::Vertex* bidir_ep = nullptr;
    iter_graph->pinVertices(endpoint_pin, endpoint, bidir_ep);
    if (endpoint == nullptr) {
      logger_->warn(utl::RES, 407,
                    "Iteration {}: endpoint pin has no vertex, skipping.",
                    ep_idx + 1);
      continue;
    }

    logger_->info(utl::RES, 413, "[remap] iter {}: before vertexSlack", ep_idx);
    const Slack ep_slack = sta->vertexSlack(endpoint, sta::MinMax::max());

    logger_->info(utl::RES, 401,
                  "=== Iteration {}/{}: endpoint {} (slack={:.4e}) ===",
                  ep_idx + 1, candidate_pins.size(),
                  endpoint->name(network), ep_slack);

    logger_->info(utl::RES, 414, "[remap] iter {}: before getWorstVerticesForEndpoint", ep_idx);
    // Get worst vertices along this endpoint's critical path.
    std::vector<sta::Vertex*> worst_vertices =
        getWorstVerticesForEndpoint(remapper, endpoint);

    if (worst_vertices.empty()) {
      logger_->info(utl::RES, 402,
                    "Iteration {}: no vertices found for endpoint, skipping.",
                    ep_idx + 1);
      continue;
    }

    logger_->info(utl::RES, 415, "[remap] iter {}: attempting to fix up to {} vertices out of {}",
                  ep_idx, max_vertices_per_endpoint, worst_vertices.size());

    // Try to remap worst vertices for this endpoint one at a time.
    // After each successful remap, InsertAbcMapSolution deletes/replaces
    // instances, invalidating all Vertex* AND Pin* in the cut neighbourhood.
    // We must re-derive the worst vertices from the (surviving) endpoint pin
    // after every successful remap.
    int ep_remapped = 0;
    int ep_attempted = 0;
    while (ep_remapped < static_cast<int>(max_vertices_per_endpoint)) {
      // (Re-)derive worst vertices from the endpoint pin.
      // On the first pass we already have them; on subsequent passes we
      // need a fresh graph and fresh vertex list.
      if (ep_attempted > 0) {
        sta::Graph* vg = sta->ensureGraph();
        if (vg == nullptr) {
          logger_->warn(utl::RES, 419,
                        "[remap] iter {}: graph null after remap {}, stopping.",
                        ep_idx, ep_remapped);
          break;
        }
        sta::Vertex* fresh_ep = nullptr;
        sta::Vertex* bidir_ep2 = nullptr;
        vg->pinVertices(endpoint_pin, fresh_ep, bidir_ep2);
        if (fresh_ep == nullptr) {
          logger_->warn(utl::RES, 420,
                        "[remap] iter {}: endpoint lost vertex after remap {}, stopping.",
                        ep_idx, ep_remapped);
          break;
        }
        worst_vertices = getWorstVerticesForEndpoint(remapper, fresh_ep);
        if (worst_vertices.empty()) {
          break;
        }
      }

      // Skip the first ep_attempted vertices (already tried in prior passes).
      // After a successful remap the list is re-derived, so previously-remapped
      // instances are gone and the new list starts fresh — reset the skip count.
      // We only need to skip when the previous attempt was NOT applied (the
      // vertex was not remapable and still appears in the re-derived list).
      // Use a simple approach: try only the first vertex in the list each time.
      // If it's not remapable, skip it by erasing and retry; if the list is
      // exhausted, stop.

      std::vector<sta::Vertex*> single_vertex = { worst_vertices[0] };
      logger_->info(utl::RES, 417,
                    "[remap] iter {}: remapOneCut attempt {} (ep_remapped={})",
                    ep_idx, ep_attempted + 1, ep_remapped);
      bool applied = remapOneCut(remapper, single_vertex);
      logger_->info(utl::RES, 416, "[remap] iter {}: after remapOneCut, applied={}",
                    ep_idx, applied);
      ep_attempted++;

      if (applied) {
        remapped_count++;
        ep_remapped++;
        sta->networkChanged();
      } else {
        // This vertex was not remapable; remove it and try the next one.
        worst_vertices.erase(worst_vertices.begin());
        if (worst_vertices.empty()) {
          break;
        }
        // Don't increment ep_attempted again — we'll retry with the new front
        // without re-deriving, since the netlist hasn't changed.
        continue;
      }
    }

    if (ep_remapped > 0) {
      logger_->info(utl::RES, 418,
                    "[remap] iter {}: remapped {} vertices for this endpoint.",
                    ep_idx, ep_remapped);
    }
  }

  logger_->info(utl::RES, 403,
                "Iterative remap complete: {}/{} pins successfully remapped.",
                remapped_count, candidate_pins.size());

  // Optionally run full detailed placement to resolve any overlaps from
  // iterative cell insertions, then improve wirelength with local optimizations.
  if (run_detailed_placement && remapped_count > 0) {
    dpl::Opendp* dpl = remapper.getDpl();
    if (dpl) {
      logger_->info(utl::RES, 404,
                    "Running detailed placement to resolve overlaps...");
      dpl->detailedPlacement(/*max_displacement_x=*/0,
                             /*max_displacement_y=*/0);
      logger_->info(utl::RES, 405,
                    "Running placement improvement for wirelength optimization...");
      dpl->improvePlacement(/*seed=*/42,
                            /*max_displacement_x=*/0,
                            /*max_displacement_y=*/0);
    }
  }

  // Rebuild the STA graph and timing ONCE at the very end, after all
  // netlist and placement changes are complete.  All heavy STA work during
  // the loop was done in fork()ed children; only this final rebuild runs
  // in the parent.
  if (remapped_count > 0) {
    logger_->info(utl::RES, 417, "[remap] rebuilding STA graph after {} remaps", remapped_count);
    sta->networkChanged();
    sta->updateTiming(false);
    logger_->info(utl::RES, 418, "[remap] STA rebuild complete");
  }
}

std::vector<SolutionEvalResult> PositionDrivenStrategy::forkEvaluateSolutions(
    abc::Map_Man_t* map_man,
    abc::Abc_Ntk_t* logic_network,
    cut::LogicCut& candidate_cut,
    SeqRemapper& remapper,
    int iStart,
    int iEnd)
{
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();

  struct ChildInfo {
    pid_t pid;
    int pipe_fd;
    int solution_index;
    abc::Map_MappingSolution_t* pSolution;
  };
  std::vector<ChildInfo> children;

  // Fork all children in parallel
  for (int i = iStart; i < iEnd; ++i) {
    abc::Map_MappingSolution_t* pSolution =
        abc::Map_MappingGetSolution(map_man, i);
    if (pSolution == nullptr) {
      logger_->warn(utl::RES, 349, "Solution {} is NULL, skipping.", i + 1);
      continue;
    }

    int pipefd[2];
    if (pipe(pipefd) == -1) {
      logger_->warn(utl::RES, 356, "Solution {} pipe() failed, skipping.", i + 1);
      continue;
    }

    pid_t pid = fork();
    if (pid == -1) {
      close(pipefd[0]);
      close(pipefd[1]);
      logger_->warn(utl::RES, 357, "Solution {} fork() failed, skipping.", i + 1);
      continue;
    }

    if (pid == 0) {
      // === CHILD PROCESS ===
      close(pipefd[0]);
      omp_set_num_threads(1);
      sta->setThreadCount(1);
      freopen("/dev/null", "w", stdout);
      // Keep stderr visible for crash diagnostics (e.g. assertion failures)
      // freopen("/dev/null", "w", stderr);
      logger_->redirectStringBegin();

      try {
        candidate_cut.InsertAbcMapSolution(
            pSolution,
            map_man,
            logic_network,
            *remapper.getAbcLibrary(),
            network,
            sta,
            remapper.getNameGenerator(),
            logger_);

        sta::Slack slack = evaluateSolution(
            pSolution,
            map_man,
            logic_network,
            candidate_cut,
            remapper);

        std::string log_output = logger_->redirectStringEnd();

        uint32_t log_len = static_cast<uint32_t>(log_output.size());
        write_all(pipefd[1], &slack, sizeof(slack));
        write_all(pipefd[1], &log_len, sizeof(log_len));
        if (log_len > 0)
          write_all(pipefd[1], log_output.data(), log_len);
      } catch (const std::exception& e) {
        // Write error info back through the pipe so parent can report it.
        // Use a sentinel slack value to indicate failure, then send the
        // exception message as the log.
        std::string log_output = logger_->redirectStringEnd();
        std::string err_msg = log_output
            + "\n[CHILD EXCEPTION] " + e.what() + "\n";
        sta::Slack sentinel = std::numeric_limits<sta::Slack>::lowest();
        uint32_t log_len = static_cast<uint32_t>(err_msg.size());
        write_all(pipefd[1], &sentinel, sizeof(sentinel));
        write_all(pipefd[1], &log_len, sizeof(log_len));
        if (log_len > 0)
          write_all(pipefd[1], err_msg.data(), log_len);
      }
      close(pipefd[1]);
      _exit(0);
    }

    // === PARENT PROCESS ===
    close(pipefd[1]);
    children.push_back({pid, pipefd[0], i, pSolution});
  }

  // Collect results from all children
  std::vector<SolutionEvalResult> results;
  results.reserve(children.size());

  for (size_t j = 0; j < children.size(); ++j) {
    auto& child = children[j];
    SolutionEvalResult res;
    res.solution_index = child.solution_index;
    res.pSolution = child.pSolution;
    res.success = false;

    sta::Slack slack;
    uint32_t log_len;

    if (!read_all(child.pipe_fd, &slack, sizeof(slack))) {
      logger_->warn(utl::RES, 383,
                    "Solution {} pipe read for slack failed.",
                    child.solution_index + 1);
      close(child.pipe_fd);
      results.push_back(std::move(res));
      continue;
    }

    if (!read_all(child.pipe_fd, &log_len, sizeof(log_len))) {
      logger_->warn(utl::RES, 384,
                    "Solution {} pipe read for log_len failed.",
                    child.solution_index + 1);
      close(child.pipe_fd);
      results.push_back(std::move(res));
      continue;
    }

    std::string log(log_len, '\0');
    bool log_ok = (log_len == 0) || read_all(child.pipe_fd, log.data(), log_len);
    close(child.pipe_fd);

    if (log_ok) {
      res.slack = slack;
      res.log = std::move(log);
      res.success = true;
    }
    results.push_back(std::move(res));
  }

  // Reap all children
  for (size_t j = 0; j < children.size(); ++j) {
    int status;
    waitpid(children[j].pid, &status, 0);
    if (WIFEXITED(status)) {
      int exit_code = WEXITSTATUS(status);
      if (exit_code != 0) {
        logger_->warn(utl::RES, 385,
                      "Solution {} child exited with code {}.",
                      children[j].solution_index + 1, exit_code);
        results[j].success = false;
      }
    } else if (WIFSIGNALED(status)) {
      int sig = WTERMSIG(status);
      logger_->warn(utl::RES, 386,
                    "Solution {} child killed by signal {} ({}).",
                    children[j].solution_index + 1, sig, strsignal(sig));
      results[j].success = false;
    } else {
      logger_->warn(utl::RES, 387,
                    "Solution {} child ended with unknown status 0x{:x}.",
                    children[j].solution_index + 1, status);
      results[j].success = false;
    }
  }

  return results;
}

sta::Slack PositionDrivenStrategy::evaluateSolution(
    abc::Map_MappingSolution_t* pSolution,
    abc::Map_Man_t* pMan,
    abc::Abc_Ntk_t* pOriginalNetwork,
    cut::LogicCut& candidate_cut,
    SeqRemapper& remapper)
{
  if (!pSolution) {
    remapper.getLogger()->error(
        utl::RES, 342, "Solution pointer is NULL.");
    return std::numeric_limits<sta::Slack>::lowest();
  }

  if (!pMan || !pOriginalNetwork) {
    remapper.getLogger()->error(
        utl::RES, 343,
        "Map manager or original network not available.");
    return std::numeric_limits<sta::Slack>::lowest();
  }

  sta::dbSta* sta = remapper.getSta();
  utl::Logger* logger = remapper.getLogger();

  // Run DPL to legalize placement of newly inserted cut instances.
  // Each call is either in an isolated child (fork-based evaluation) or
  // in the parent for the final permanent apply.
  remapper.performIncreDpl(candidate_cut, remapper.getDpl());

  // Recompute timing from the current network state.
  // updateTiming(false) does an incremental update: arrivals + required times.
  // findDelays() alone only computes gate delays, not required times/slack.
  sta->networkChanged();
  sta->updateTiming(false);

  // Collect output pins of the cut to find affected endpoints
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::PinSeq cut_output_pins;
  for (sta::Net* output_net : candidate_cut.primary_outputs()) {
    sta::NetPinIterator* pin_iter = network->pinIterator(output_net);
    while (pin_iter->hasNext()) {
      const sta::Pin* pin = pin_iter->next();
      if (network->direction(pin)->isAnyOutput()) {
        cut_output_pins.push_back(pin);
        break;
      }
    }
    delete pin_iter;
  }

  // Find endpoints reachable from the cut outputs
  sta::PinSet fanout_endpoints = sta->findFanoutPins(
      &cut_output_pins,
      /*flat=*/true,
      /*endpoints_only=*/true,
      /*inst_levels=*/-1,
      /*pin_levels=*/-1,
      /*thru_disabled=*/false,
      /*thru_constants=*/false);

  // Find worst slack among cut-affected endpoints only
  sta::Slack worst_slack = std::numeric_limits<sta::Slack>::infinity();
  sta::Vertex* worst_vertex = nullptr;
  int endpoint_count = fanout_endpoints.size();

  sta::Graph* graph = sta->ensureGraph();
  for (const sta::Pin* pin : fanout_endpoints) {
    sta::Vertex* vertex = nullptr;
    sta::Vertex* bidir = nullptr;
    graph->pinVertices(pin, vertex, bidir);
    if (!vertex)
      continue;
    sta::Slack slack = sta->vertexSlack(vertex, sta::MinMax::max());
    if (slack < worst_slack) {
      worst_slack = slack;
      worst_vertex = vertex;
    }
  }

  // Attempt repair moves on cut instances if there are setup violations:
  // UnbufferMove -> VTSwapSpeed -> SizeUpMove -> SwapPinsMove -> BufferMove -> SplitLoadMove
  if (worst_vertex && fuzzyLess(worst_slack, 0.0f)) {
    rsz::Resizer* resizer = remapper.getResizer();
    const sta::InstanceSet& cut_insts = candidate_cut.cut_instances();
    resizer->setSizeUpInstanceFilter(&cut_insts);
    resizer->repairSetup(worst_vertex->pin(), /*size_up_only=*/false);
    resizer->setSizeUpInstanceFilter(nullptr);
    // Recompute timing after size-up
    sta->networkChanged();
    sta->updateTiming(false);
    graph = sta->ensureGraph();
    worst_slack = std::numeric_limits<sta::Slack>::infinity();
    for (const sta::Pin* pin : fanout_endpoints) {
      sta::Vertex* vertex = nullptr;
      sta::Vertex* bidir2 = nullptr;
      graph->pinVertices(pin, vertex, bidir2);
      if (!vertex)
        continue;
      sta::Slack slack = sta->vertexSlack(vertex, sta::MinMax::max());
      if (slack < worst_slack) {
        worst_slack = slack;
      }
    }
  }

  // Log the evaluation result
  logger->info(utl::RES, 345,
               "Solution evaluated: {} endpoints, Worst Slack = {:.4e}",
               endpoint_count, worst_slack);

  return worst_slack;
}
} // namespace rmp

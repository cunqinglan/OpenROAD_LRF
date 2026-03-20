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
#include <unistd.h>       // fork, pipe, _exit, read, write, close
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
#include "sta/Units.hh"
#include "sta/VerilogWriter.hh"

#include "base/abc/abc.h"
#include "map/mapper/mapper.h"
#include "Strategy.hh"
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
static std::vector<sta::Vertex*> selectCandidateEndpoints(
    sta::dbSta* sta,
    rsz::Resizer* resizer,
    float percentage,
    float max_percentage,
    float slack_threshold)
{
  sta::dbNetwork* network = sta->getDbNetwork();
  std::vector<sta::Vertex*> all_endpoints;
  for (sta::Vertex* vertex : *sta->endpoints()) {
    sta::Pin* pin = vertex->pin();
    const sta::PortDirection* direction = network->direction(pin);
    if (!direction->isInput()) {
      continue;
    }
    if (resizer != nullptr) {
      if (resizer->dontTouch(pin) || resizer->dontTouch(network->net(pin))
          || resizer->dontTouch(network->instance(pin))) {
        continue;
      }
    }
    all_endpoints.push_back(vertex);
  }

  if (all_endpoints.empty()) {
    return {};
  }

  // Precompute slack once per vertex to avoid repeated STA queries during sort.
  using VertexSlackPair = std::pair<sta::Vertex*, sta::Slack>;
  std::vector<VertexSlackPair> endpoint_slacks;
  endpoint_slacks.reserve(all_endpoints.size());
  for (sta::Vertex* v : all_endpoints) {
    endpoint_slacks.emplace_back(v, sta->vertexSlack(v, sta::MinMax::max()));
  }

  // Sort ascending by slack (most negative = worst first).
  std::sort(endpoint_slacks.begin(), endpoint_slacks.end(),
    [](const VertexSlackPair& a, const VertexSlackPair& b) {
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
    std::vector<sta::Vertex*> result;
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
    std::vector<sta::Vertex*> result;
    for (const auto& [v, slack] : endpoint_slacks) {
      if (result.size() >= max_n) {
        break;
      }
      if (slack < slack_threshold) {
        result.push_back(v);
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
  auto candidate_vertices = selectCandidateEndpoints(
      sta, remapper.getResizer(), percentage, max_percentage, slack_threshold);

  for (sta::Vertex* negative_endpoint : candidate_vertices) {
    logic_extractor.AppendEndpoint(negative_endpoint);
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
          && !network->isTopLevelPort(path_pin)) {
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

void PositionDrivenStrategy::remap(SeqRemapper& remapper,
                                    float percentage,
                                    float max_percentage,
                                    float slack_threshold) {
  // Step 1: Get worst vertices on the most critical path, sorted by
  // load-dependent delay (largest first).
  std::vector<sta::Vertex*> worst_vertices =
      getWorstVertices(remapper, percentage, max_percentage, slack_threshold);
  if (worst_vertices.empty()) {
    remapper.getLogger()->warn(
      utl::RES, 334, "No worst-slack path found.");
    return;
  }

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

    // Debug: Print bad_instance info
    logger_->info(utl::RES, 390,
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

    // Step 2: Extract the bottleneck cut around this vertex.
    setRefGate(inst);
    cut::LogicCut trial_cut = extractBottleneck(remapper);

    // Debug: Print cut info
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

    // Found a valid cut with >1 instance.
    bad_instance = inst;
    candidate_cut = std::move(trial_cut);
    found_valid_cut = true;
    break;
  }

  if (!found_valid_cut) {
    remapper.getLogger()->warn(
        utl::RES, 366,
        "All candidate vertices produced cuts with <= 1 instance, nothing to remap.");
    return;
  }
  setCandidateCut(candidate_cut);

  // Step 3: Build the ABC network from the candidate cut.
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network(
      candidate_cut.BuildMappedAbcNetwork(
          *remapper.getAbcLibrary(),
          network,
          remapper.getLogger()).release(),
      &abc::Abc_NtkDelete);

  if (mapped_abc_network == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 335, "Failed to build ABC network from candidate cut.");
    return;
  }

  // Must set the global ABC library BEFORE Abc_NtkToLogic, because
  // Abc_NtkAlloc(ABC_FUNC_MAP) initializes pManFunc = Abc_FrameReadLibGen().
  // If the global is not set first, the new network gets a NULL/stale pManFunc
  // which later causes the assertion in Abc_NtkMapToSopUsingLibrary to fail.
  auto library = static_cast<abc::Mio_Library_t*>(mapped_abc_network.get()->pManFunc);
  if (library == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 341, "ABC network does not have an associated library.");
    return;
  }
  abc::Abc_FrameSetLibGen(library);

  // Step 4: Convert the mapped network to logic (AIG) form for enumeration.
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> logic_network(
      abc::Abc_NtkToLogic(mapped_abc_network.get()),
      &abc::Abc_NtkDelete);

  if (logic_network == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 340, "Failed to convert ABC network to logic form.");
    return;
  }

  logger_->info(
      utl::RES, 351, "After step 4. ABC network converted to logic form with {} nodes.",
      abc::Abc_NtkNodeNum(logic_network.get()));

  // Step 5: Enumerate all possible mapping solutions using ABC.
  int nMaxSolutions = 80;
  int fVerbose = 1;

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> strashed_network(
      abc::Abc_NtkStrash(logic_network.get(), 0, 0, 0),
      &abc::Abc_NtkDelete);

  void* pMan = abc::Abc_NtkMapEnumPassStore(
      strashed_network.get(),
      nMaxSolutions,
      fVerbose);

  if (pMan == nullptr) {
    remapper.getLogger()->warn(
        utl::RES, 353, "ABC mapping enumeration returned no solutions.");
    return;
  }

  logger_->info(utl::RES, 352, "After step 5.");

  // Step 6: Evaluate each solution using fork() for isolation.
  abc::Map_Man_t* map_man = static_cast<abc::Map_Man_t*>(pMan);
  const int num_solutions = abc::Map_ManReadNumSolutions(map_man);
  if (num_solutions <= 0) {
    remapper.getLogger()->warn(
        utl::RES, 354, "ABC mapping enumeration returned no solutions.");
    abc::Abc_NtkMapEnumFreeStore(pMan);
    return;
  }

  logger_->info(utl::RES, 359, "Found {} enumerated solutions to evaluate.", num_solutions);

  abc::Map_MappingSolution_t* pSolutionBest = nullptr;
  sta::Slack best_slack = std::numeric_limits<sta::Slack>::lowest();
  int best_solution_index = -1;
  int evaluated_count = 0;

  // --- Evaluate enumerated solutions ---
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

    if (!res.log.empty())
      logger_->reportLiteral(res.log);

    // Store evaluation result back into the solution for UCT
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

  // --- UCT iterative phase: generate → evaluate → update rewards → repeat ---
  const int nBatchSize = 20;
  const int nRounds = 5;
  const double uctC = 1.414;  // sqrt(2)

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

      // Evaluate the new batch via fork (parallel)
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

        if (!res.log.empty())
          logger_->reportLiteral(res.log);

        // Store result back for reward update
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

      // Feed results back into UCT reward stats for next round
      abc::Map_UctUpdateRewards(pUct, nBefore, nAfter);
    }

    logger_->info(utl::RES, 370,
                 "UCT evaluation complete: total {} solutions evaluated.",
                 evaluated_count);

    abc::Map_UctEnd(pUct);
  } else {
    logger_->info(utl::RES, 374, "UCT initialization failed, skipping.");
  }

  // Step 7: Permanently apply the best solution in the parent process.
  if (pSolutionBest) {
    remapper.getLogger()->info(
        utl::RES, 346,
        "Best solution found (index {}) with worst slack = {:.4e}",
        best_solution_index + 1, best_slack);

    candidate_cut.InsertAbcMapSolution(
        pSolutionBest,
        map_man,
        logic_network.get(),
        *remapper.getAbcLibrary(),
        network,
        sta,
        remapper.getNameGenerator(),
        logger_);

    // Final placement and timing with GPL in the parent.
    evaluateSolution(
        pSolutionBest,
        map_man,
        logic_network.get(),
        candidate_cut,
        remapper);

    logger_->info(utl::RES, 364, "Best solution permanently applied.");
  } else {
    remapper.getLogger()->warn(
        utl::RES, 361,
        "No valid solution found to apply.");
  }

  // Step 8: Clean up the mapping manager.
  abc::Abc_NtkMapEnumFreeStore(pMan);
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
        write(pipefd[1], &slack, sizeof(slack));
        write(pipefd[1], &log_len, sizeof(log_len));
        if (log_len > 0)
          write(pipefd[1], log_output.data(), log_len);
      } catch (const std::exception& e) {
        // Write error info back through the pipe so parent can report it.
        // Use a sentinel slack value to indicate failure, then send the
        // exception message as the log.
        std::string log_output = logger_->redirectStringEnd();
        std::string err_msg = log_output
            + "\n[CHILD EXCEPTION] " + e.what() + "\n";
        sta::Slack sentinel = std::numeric_limits<sta::Slack>::lowest();
        uint32_t log_len = static_cast<uint32_t>(err_msg.size());
        write(pipefd[1], &sentinel, sizeof(sentinel));
        write(pipefd[1], &log_len, sizeof(log_len));
        if (log_len > 0)
          write(pipefd[1], err_msg.data(), log_len);
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
    ssize_t n;

    n = read(child.pipe_fd, &slack, sizeof(slack));
    if (n != static_cast<ssize_t>(sizeof(slack))) {
      logger_->warn(utl::RES, 383,
                    "Solution {} pipe read for slack returned {} bytes (expected {}).",
                    child.solution_index + 1, static_cast<long>(n),
                    sizeof(slack));
      close(child.pipe_fd);
      results.push_back(std::move(res));
      continue;
    }

    n = read(child.pipe_fd, &log_len, sizeof(log_len));
    if (n != static_cast<ssize_t>(sizeof(log_len))) {
      logger_->warn(utl::RES, 384,
                    "Solution {} pipe read for log_len returned {} bytes (expected {}).",
                    child.solution_index + 1, static_cast<long>(n),
                    sizeof(log_len));
      close(child.pipe_fd);
      results.push_back(std::move(res));
      continue;
    }

    std::string log(log_len, '\0');
    size_t total_read = 0;
    while (total_read < log_len) {
      n = read(child.pipe_fd, log.data() + total_read, log_len - total_read);
      if (n <= 0) break;
      total_read += n;
    }
    close(child.pipe_fd);

    if (total_read == log_len) {
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
        logger_->warn(utl::RES, 380,
                      "Solution {} child exited with code {}.",
                      children[j].solution_index + 1, exit_code);
        results[j].success = false;
      }
    } else if (WIFSIGNALED(status)) {
      int sig = WTERMSIG(status);
      logger_->warn(utl::RES, 381,
                    "Solution {} child killed by signal {} ({}).",
                    children[j].solution_index + 1, sig, strsignal(sig));
      results[j].success = false;
    } else {
      logger_->warn(utl::RES, 382,
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
        cut_output_pins.push_back(const_cast<sta::Pin*>(pin));
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

  sta::Graph* graph = sta->graph();
  for (const sta::Pin* pin : fanout_endpoints) {
    sta::Vertex* vertex = graph->pinDrvrVertex(pin);
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
    worst_slack = std::numeric_limits<sta::Slack>::infinity();
    for (const sta::Pin* pin : fanout_endpoints) {
      sta::Vertex* vertex = graph->pinDrvrVertex(pin);
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

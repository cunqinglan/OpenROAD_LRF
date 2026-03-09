#include "position_driven.hh"

#include <algorithm>      // std::sort
#include <cmath>          // std::ceil, std::floor
#include <cstdint>        // uint32_t
#include <cstdio>         // freopen
#include <limits>         // std::numeric_limits
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
  Slack worst_slack = 0.0;

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

  // Sort ascending by slack (most negative = worst first).
  std::sort(all_endpoints.begin(), all_endpoints.end(),
    [&sta](sta::Vertex* a, sta::Vertex* b) {
      return sta->vertexSlack(a, sta::MinMax::max())
           < sta->vertexSlack(b, sta::MinMax::max());
    });

  // Threshold above which slack_threshold is considered "not set".
  const float kNoThreshold = std::numeric_limits<float>::max() / 2.0f;

  if (percentage >= 0.0f) {
    // Percentage mode: fix top N% of all endpoints, at least 1.
    size_t n = static_cast<size_t>(
        std::ceil(static_cast<float>(all_endpoints.size()) * percentage / 100.0f));
    n = std::max(n, size_t(1));
    n = std::min(n, all_endpoints.size());
    return {all_endpoints.begin(), all_endpoints.begin() + static_cast<ptrdiff_t>(n)};
  }

  if (max_percentage >= 0.0f && slack_threshold < kNoThreshold) {
    // max_percentage + slack_threshold mode: keep endpoints below threshold,
    // capped at max_percentage of the total endpoint count.
    size_t max_n = static_cast<size_t>(
        std::floor(static_cast<float>(all_endpoints.size()) * max_percentage / 100.0f));
    std::vector<sta::Vertex*> result;
    for (sta::Vertex* v : all_endpoints) {
      if (result.size() >= max_n) {
        break;
      }
      if (sta->vertexSlack(v, sta::MinMax::max()) < slack_threshold) {
        result.push_back(v);
      } else {
        break;  // sorted: no further endpoint will be below threshold
      }
    }
    return result;
  }

  // Default: fix only the single worst endpoint.
  return {all_endpoints[0]};
}

sta::Vertex* PositionDrivenStrategy::getWorstVertex(
    SeqRemapper& remapper,
    float percentage,
    float max_percentage,
    float slack_threshold) {
  // Return the vertex on the most critical path (within the cut outputs)
  // that has the largest load-dependent delay (arc delay - intrinsic delay).
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

    // Find a load pin on the net (an input pin) and map to a graph vertex.
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
    return nullptr;
  }

  // 2) Get the worst-slack path to that endpoint and expand it.
  sta::Path* end_path = sta->vertexWorstSlackPath(worst_end_vertex, sta::MinMax::max());
  if (end_path == nullptr) {
    remapper.getLogger()->warn(
        utl::RES, 333, "No worst-slack path found for endpoint {}.",
        worst_end_vertex->name(network));
    return worst_end_vertex;
  }

  sta::PathExpanded expanded(end_path, sta);

  // 3) Walk the path and compute load-dependent delay for each driver vertex,
  //     pick the maximum (like RepairSetup::repairPath).
  sta::Vertex* worst_vertex = worst_end_vertex;
  Delay max_load_delay = -std::numeric_limits<Delay>::infinity();

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

      // Same conditions as RepairSetup: ignore the first element and top-level ports.
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

        // Break ties by choosing the more downstream (larger i), matching RSZ logic.
        if (load_delay > max_load_delay
            || (load_delay == max_load_delay && i > 0)) {
          max_load_delay = load_delay;
          worst_vertex = path_vertex;
        }
      }
    }
  }

  return worst_vertex;
}

void PositionDrivenStrategy::remap(SeqRemapper& remapper,
                                    float percentage,
                                    float max_percentage,
                                    float slack_threshold) {
  // Step 1: Get the worst vertex on the most critical path.
  sta::Vertex* bad_vertex = getWorstVertex(remapper, percentage, max_percentage, slack_threshold);
  if (bad_vertex == nullptr) {
    remapper.getLogger()->warn(
      utl::RES, 334, "No worst-slack path found.");
    return;
  }
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Instance* bad_instance = network->instance(bad_vertex->pin());
  if (bad_instance == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 350, "Worst vertex {} is not driven by an instance.",
        bad_vertex->name(network));
    return;
  }
  setRefGate(bad_instance);

  // Step 2: Extract the bottleneck cut around that vertex.
  cut::LogicCut candidate_cut = extractBottleneck(remapper);
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
  int nMaxSolutions = 20;
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
  // Each child inherits the full database via COW, freely modifies it
  // (insert solution, run GPL, run STA), writes the slack result back
  // via a pipe, and _exit()s.  The parent's state is never touched.
  abc::Map_Man_t* map_man = static_cast<abc::Map_Man_t*>(pMan);
  const int num_solutions = abc::Map_ManReadNumSolutions(map_man);
  if (num_solutions <= 0) {
    remapper.getLogger()->warn(
        utl::RES, 354, "ABC mapping enumeration returned no solutions.");
    abc::Abc_NtkMapEnumFreeStore(pMan);
    return;
  }

  logger_->info(utl::RES, 359, "Found {} solutions to evaluate.", num_solutions);

  abc::Map_MappingSolution_t* pSolutionBest = nullptr;
  sta::Slack best_slack = std::numeric_limits<sta::Slack>::lowest();
  int best_solution_index = -1;
  int evaluated_count = 0;

  // --- Phase 1: Fork all children in parallel ---
  struct ChildInfo {
    pid_t pid;
    int pipe_fd;        // read end
    int solution_index;
    abc::Map_MappingSolution_t* pSolution;
  };
  std::vector<ChildInfo> children;

  for (int i = 0; i < num_solutions; ++i) {
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
      close(pipefd[0]);  // close read end

      // Force single-threaded to avoid fork+threads issues.
      omp_set_num_threads(1);
      sta->setThreadCount(1);

      // Suppress raw stdout/stderr (GPL/ABC may printf).
      freopen("/dev/null", "w", stdout);
      freopen("/dev/null", "w", stderr);

      // Capture all Logger output to a string.
      logger_->redirectStringBegin();

      candidate_cut.InsertAbcMapSolution(
          pSolution,
          map_man,
          logic_network.get(),
          *remapper.getAbcLibrary(),
          network,
          sta,
          remapper.getNameGenerator(),
          logger_);

      sta::Slack slack = evaluateSolution(
          pSolution,
          map_man,
          logic_network.get(),
          candidate_cut,
          remapper);

      std::string log_output = logger_->redirectStringEnd();

      // Write to pipe: slack, then log length, then log content.
      uint32_t log_len = static_cast<uint32_t>(log_output.size());
      write(pipefd[1], &slack, sizeof(slack));
      write(pipefd[1], &log_len, sizeof(log_len));
      if (log_len > 0)
        write(pipefd[1], log_output.data(), log_len);
      close(pipefd[1]);
      _exit(0);
    }

    // === PARENT PROCESS ===
    close(pipefd[1]);  // close write end
    children.push_back({pid, pipefd[0], i, pSolution});
  }

  // --- Phase 2: Collect results from all children ---
  struct ChildResult {
    sta::Slack slack;
    std::string log;
    bool success;
  };
  std::vector<ChildResult> results(children.size(), {0.0, "", false});

  for (size_t j = 0; j < children.size(); ++j) {
    auto& child = children[j];
    auto& result = results[j];

    sta::Slack slack;
    uint32_t log_len;
    ssize_t n;

    n = read(child.pipe_fd, &slack, sizeof(slack));
    if (n != static_cast<ssize_t>(sizeof(slack))) {
      close(child.pipe_fd);
      continue;
    }

    n = read(child.pipe_fd, &log_len, sizeof(log_len));
    if (n != static_cast<ssize_t>(sizeof(log_len))) {
      close(child.pipe_fd);
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
      result.slack = slack;
      result.log = std::move(log);
      result.success = true;
    }
  }

  // Reap all children.
  for (size_t j = 0; j < children.size(); ++j) {
    int status;
    waitpid(children[j].pid, &status, 0);
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      results[j].success = false;
    }
  }

  // --- Phase 3: Print results as coherent blocks, find best ---
  for (size_t j = 0; j < children.size(); ++j) {
    int idx = children[j].solution_index;
    auto& result = results[j];

    logger_->info(utl::RES, 355, "--- Solution {}/{} ---", idx + 1, num_solutions);

    if (!result.success) {
      logger_->warn(utl::RES, 358, "Solution {} child failed.", idx + 1);
      continue;
    }

    // Print captured log as one coherent block.
    if (!result.log.empty())
      logger_->reportLiteral(result.log);

    if (result.slack > best_slack) {
      best_slack = result.slack;
      pSolutionBest = children[j].pSolution;
      best_solution_index = idx;
      logger_->info(utl::RES, 362, "Solution {} is new best (slack={:.4f}).",
                    idx + 1, result.slack);
    } else {
      logger_->info(utl::RES, 363, "Solution {} (slack={:.4f}) not better than best ({:.4f}).",
                    idx + 1, result.slack, best_slack);
    }

    evaluated_count++;
  }

  logger_->info(utl::RES, 360,
               "Evaluation complete: {} solutions evaluated.",
               evaluated_count);

  // Step 7: Permanently apply the best solution in the parent process.
  if (pSolutionBest) {
    remapper.getLogger()->info(
        utl::RES, 346,
        "Best solution found (index {}) with worst slack = {:.4f}",
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

  // Always run GPL placement — each call is either in an isolated child
  // (fork-based evaluation) or in the parent for the final permanent apply.
  remapper.performIncrePlace(candidate_cut, remapper.getGpl(), remapper.getDpl());
  //remapper.performIncreDpl(candidate_cut);

  // Recompute timing from the current network state.
  sta->networkChanged();
  sta->findDelays();
  
  // Find worst slack and worst endpoint vertex
  sta::Slack worst_slack = std::numeric_limits<sta::Slack>::infinity();
  sta::Vertex* worst_vertex = nullptr;

  sta::VertexSet* endpoints = sta->search()->endpoints();
  int endpoint_count = 0;
  if (endpoints) {
    endpoint_count = endpoints->size();
    for (sta::Vertex* endpoint : *endpoints) {
      sta::Slack slack = sta->vertexSlack(endpoint, sta::MinMax::max());
      if (slack < worst_slack) {
        worst_slack = slack;
        worst_vertex = endpoint;
      }
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
    sta->findDelays();
    worst_slack = std::numeric_limits<sta::Slack>::infinity();
    for (sta::Vertex* endpoint : *endpoints) {
      sta::Slack slack = sta->vertexSlack(endpoint, sta::MinMax::max());
      if (slack < worst_slack) {
        worst_slack = slack;
      }
    }
  }

  // Log the evaluation result
  logger->info(utl::RES, 345,
               "Solution evaluated: {} endpoints, Worst Slack = {:.4f}",
               endpoint_count, worst_slack);

  return worst_slack;
}
} // namespace rmp

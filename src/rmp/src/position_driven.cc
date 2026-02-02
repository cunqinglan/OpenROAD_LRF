#include "position_driven.hh"

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
  extern void * Abc_NtkMapEnumPassStore( Abc_Ntk_t * pNtk, int nMaxSolutions, int fVerbose );
  extern void Abc_NtkMapEnumFreeStore( void * pStore );
} // namespace abc

namespace rmp {

sta::Vertex* PositionDrivenStrategy::getFarthestOutputVertex(
    SeqRemapper& remapper) {
  sta::dbNetwork* network = remapper.getSta()->getDbNetwork();
  sta::Graph* graph = remapper.getSta()->graph();
  sta::Vertex* farthest_vertex = nullptr;
  Slack worst_slack = 0.0;

  cut::LogicCut bottleneck_cut = remapper.extractBottleneck();

  for (sta::Net* output_net : bottleneck_cut.primary_outputs()) {
    sta::Vertex* output_vertex = nullptr;
    sta::Vertex* bidirect_vertex = nullptr;
    sta::NetPinIterator* pin_iter = network->pinIterator(output_net);
      while (pin_iter->hasNext()) {
        sta::Pin* pin = pin_iter->next();
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
    Slack slack = remapper.getSta()->vertexSlack(output_vertex, remapper.getMax());
    if (slack < worst_slack) {
      worst_slack = slack;
      farthest_vertex = output_vertex;
    }
  }
  return farthest_vertex;
}

///////////
// Maybe it's better to put the cut variable insider the remapper class,
// instead of passing it around.
///////////

sta::Vertex* PositionDrivenStrategy::getWorstVertex(
    SeqRemapper& remapper) {
  // Return the vertex on the most critical path (within the cut outputs)
  // that has the largest load-dependent delay (arc delay - intrinsic delay).
  sta::Sta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Graph* graph = sta->graph();

  // Disable incremental timing.
  sta->graphDelayCalc()->delaysInvalid();
  sta->search()->arrivalsInvalid();
  sta->search()->endpointsInvalid();

  cut::LogicExtractorFactory logic_extractor(sta, logger);
  for (sta::Vertex* negative_endpoint : candidate_vertices) {
    logic_extractor.AppendEndpoint(negative_endpoint);
  }

  cut::LogicCut bad_cut = logic_extractor.BuildLogicCut(abc_library);

  // 1) Find the worst (most negative slack) endpoint vertex among cut outputs.
  sta::Vertex* worst_end_vertex = nullptr;
  Slack worst_slack = std::numeric_limits<Slack>::infinity();

  for (sta::Net* output_net : bad_cut.primary_outputs()) {
    sta::Vertex* output_vertex = nullptr;
    sta::Vertex* bidirect_vertex = nullptr;

    // Find a load pin on the net (an input pin) and map to a graph vertex.
    sta::NetPinIterator* pin_iter = network->pinIterator(output_net);
    while (pin_iter->hasNext()) {
      sta::Pin* pin = pin_iter->next();
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

    const Slack slack = sta->vertexSlack(output_vertex, remapper.getMax());
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
  sta::Path* end_path = sta->vertexWorstSlackPath(worst_end_vertex, remapper.getMax());
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

void PositionDrivenStrategy::remap(
    SeqRemapper& remapper) {
  // Step 1: Get the worst vertex on the most critical path.
  sta::Vertex* bad_vertex = getWorstVertex(remapper);
  if (bad_vertex == nullptr) {
    remapper.getLogger()->warn(
      utl::RES, 334, "No worst-slack path found.");
    return;
  }
  remapper.setRefGate(bad_vertex);

  // Step 2: Extract the bottleneck cut around that vertex.
  cut::LogicCut candidate_cut = remapper.extractBottleneck();
  remapper.setCandidateCut(candidate_cut);

  // Step 3: Build the ABC network from the candidate cut.
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network(
      candidate_cut.BuildMappedAbcNetwork(
          *remapper.getAbcLibrary(),
          remapper.getSta()->getDbNetwork(),
          remapper.getLogger()).release(),
      &abc::Abc_NtkDelete);

  if (mapped_abc_network == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 335, "Failed to build ABC network from candidate cut.");
    return;
  }

  // Step 4: Convert the mapped network to logic (AIG) form for enumeration.
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> logic_network(
      abc::Abc_NtkToLogic(mapped_abc_network.get()),
      &abc::Abc_NtkDelete);

  if (logic_network == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 336, "Failed to convert ABC network to logic form.");
    return;
  }

  // Step 5: Enumerate all possible mapping solutions using ABC.
  int nMaxSolutions = 100;  // Configure as needed
  int fVerbose = remapper.getLogger()->debugCheck(utl::RES, "remap", 1);
  
  void* pMan = abc::Abc_NtkMapEnumPassStore(
      logic_network.get(), 
      nMaxSolutions, 
      fVerbose);

  if (pMan == nullptr) {
    remapper.getLogger()->warn(
        utl::RES, 337, "ABC mapping enumeration returned no solutions.");
    return;
  }

  // Step 6: Process the mapping solutions.
  // TODO: Evaluate each solution and select the best one based on timing/area.
  // You'll need to:
  // - Iterate through solutions in pMan
  // - For each solution, build a network and evaluate its cost
  // - Select the best solution
  // - Apply the best solution back to the original network

  // Step 7: Clean up the mapping manager.
  abc::Abc_NtkMapEnumFreeStore(pMan);

  // Additional work needed:
  // - Implement solution evaluation logic
  // - Implement network patching to replace the bottleneck with the best solution
}

}  // namespace rmp
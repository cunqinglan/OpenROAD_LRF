

#include "cut/abc_library_factory.h"
#include "cut/logic_cut.h"
#include "cut/logic_extractor.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "sta/Bfs.hh"
#include "sta/Graph.hh"
#include "sta/GraphClass.hh"
#include "sta/Liberty.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/PortDirection.hh"
#include "sta/SearchPred.hh"
#include "utl/Logger.h"
#include "sta/GraphDelayCalc.hh"
#include "sta/Search.hh"

#include "Strategy.hh"
#include "rmp/SeqRemapper.hh"
#include "utils.h"


namespace rmp {



bool 
SearchABCCompatiblePred::searchThru(sta::Edge* edge) {
  sta::Network* network = sta_->network();
  sta::Instance* to_inst = network->instance(edge->to(graph_)->pin());
  if (to_inst == nullptr) {
    return false;
  }
  sta::Instance* from_inst = network->instance(edge->from(graph_)->pin());
  if (from_inst == nullptr) {
    return false;
  }
  sta::LibertyCell* cell = network->libertyCell(to_inst);
  if (cell == nullptr) {
    return false;
  }
  if (!abc_library_->IsSupportedCell(cell->name())) {
    return false;
  }
  cell = network->libertyCell(from_inst);
  if (cell == nullptr) {
    return false;
  }
  if (!abc_library_->IsSupportedCell(cell->name())) {
    return false;
  }

  return sta::SearchPredNonReg2::searchThru(edge);
}

LogicExtractorFactoryPro::LogicExtractorFactoryPro(sta::dbSta* sta, utl::Logger* logger)
    : cut::LogicExtractorFactory(sta, logger) 
{
}

cut::LogicCut
LogicExtractorFactoryPro::buildLogicCutFromCutVertices(sta::VertexSet &cut_vertices, 
                                                       cut::AbcLibrary &abc_network) 
{
  open_sta_->ensureGraph();
  open_sta_->ensureLevelized();

  std::vector<sta::Vertex*> cut_vertices_vec;
  for (auto vertex : cut_vertices) {
    cut_vertices_vec.push_back(vertex);
  }
  // Dealing with constant cells 1/0 and disabled timing paths.
  cut_vertices_vec = AddMissingVertices(cut_vertices_vec, abc_network);

  std::vector<sta::Pin*> primary_inputs = GetPrimaryInputs(cut_vertices_vec);
  std::vector<sta::Pin*> primary_outputs = GetPrimaryOutputs(cut_vertices_vec);
  sta::InstanceSet cut_instances = GetCutInstances(cut_vertices_vec);

  // Remove primary outputs who are undriven. This can happen when a flop feeds
  // into another flop where the logic cone is essentially just a wire. Just
  // remove them.
  std::vector<sta::Pin*> filtered_primary_outputs
      = FilterUndrivenOutputs(primary_outputs, cut_instances);

  std::vector<sta::Net*> primary_input_nets
      = ConvertIoPinsToNets(primary_inputs);
  std::vector<sta::Net*> primary_output_nets
      = ConvertIoPinsToNets(filtered_primary_outputs);

  return cut::LogicCut(std::move(primary_input_nets),
                      std::move(primary_output_nets),
                      std::move(cut_instances));
}

//////////////////////////////////////////////////////////////////////////
// ExtractFaninConeOfBadEndPoints

bool Strategy::isExtractFaninConeOfBadEndPoints() {
  return false;
}

std::string
ExtractFaninConeOfBadEndPoints::to_string() {
  return "ExtractFaninConeOfBadEndPoints";
}

bool 
ExtractFaninConeOfBadEndPoints::isExtractFaninConeOfBadEndPoints() 
{
  return true;
}

cut::LogicCut
ExtractFaninConeOfBadEndPoints::extractBottleneck(SeqRemapper& remapper) 
{
  auto candidate_endpoints =
      GetEndpoints(remapper.getSta(), remapper.getResizer(), remapper.getSlackThreshold());
  if (candidate_endpoints.empty()) {
    remapper.getLogger()->info(
        utl::RES, 306, "No negative slack endpoints found for remapping.");
    return cut::LogicCut({}, {}, {});
  }

  remapper.getSta()->graphDelayCalc()->delaysInvalid();
  remapper.getSta()->search()->arrivalsInvalid();
  remapper.getSta()->search()->endpointsInvalid();

  cut::LogicExtractorFactory logic_extractor(remapper.getSta(), remapper.getLogger());
  for (sta::Vertex* negative_endpoint : candidate_endpoints) {
    logic_extractor.AppendEndpoint(negative_endpoint);
  }

  cut::LogicCut cut = logic_extractor.BuildLogicCut(*remapper.getAbcLibrary());

  return cut;
}

//////////////////////////////////////////////////////////////////////////
// ExtractLocalWindow
//////////////////////////////////////////////////////////////////////////

void
ExtractLocalWindow::collectAdjacentInsts(odb::dbInst* inst, 
                                         size_t window_size, 
                                        //  Return value
                                         sta::VertexSet &cut_vertices) {
  sta::Instance* sta_inst = sta_->getDbNetwork()->dbToSta(inst);
  collectAdjacentInsts(sta_inst, window_size, cut_vertices);
}

void
ExtractLocalWindow::collectAdjacentInsts(sta::Instance* inst, 
                                         size_t window_size, 
                                        //  Return value
                                         sta::VertexSet &cut_vertices) {
  cut_vertices.clear();
  sta::InstanceSet adj_inst_seq(sta_->network());
  sta::dbNetwork* network = sta_->getDbNetwork();
  
  sta::InstancePinIterator *pin_iter = network->pinIterator(inst);
  while (pin_iter->hasNext()) {
    sta::Pin* pin = pin_iter->next();
    sta::Vertex *vertex, *bidirect_vertex;
    sta_->graph()->pinVertices(pin, vertex, bidirect_vertex);
    if (vertex == nullptr || bidirect_vertex != nullptr) {
      logger_->error(utl::RES, 308, "Pin of ref gate {} has bidirectional or no vertex.", network->name(pin));
      return;
    }
    cut_vertices.insert(vertex);

    // Collect fanin vertices if vertex is a gate output
    // or fanout vertices if vertex is a gate input
    if (network->direction(pin)->isOutput())   // difference between driver and output?
      collectFanoutVerticesInWindow(vertex, window_size, cut_vertices);
    else if (network->direction(pin)->isInput()) // difference between load and input?
      collectFaninVerticesInWindow(vertex, window_size, cut_vertices);
  }
  delete pin_iter;
}

void 
ExtractLocalWindow::collectFaninVerticesInWindow(sta::Vertex* input_vertex, 
                   size_t current_depth,
                  //  Return value
                   sta::VertexSet &cut_vertices) 
{
  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Graph* graph = sta_->graph();
  printf("collectFaninVerticesInWindow depth=%zu at vertex %s\n", current_depth, input_vertex->name(network));
  fflush(stdout);
  if (current_depth == 0) {
    return;
  }
  sta::VertexInEdgeIterator in_edge_iter(input_vertex, graph);
  while (in_edge_iter.hasNext()) {
    sta::VertexSet fanin_vertices_set(graph);
    sta::Edge* edge = in_edge_iter.next();
    if (!abc_search_pred_->searchThru(edge)) {
      continue;
    }
    sta::Vertex* fanin_vertex = edge->from(graph);
    // Avoid revisiting vertices
    if (cut_vertices.find(fanin_vertex) != cut_vertices.end()) {
      continue;
    }
    sta::Instance *fanin_inst = network->instance(fanin_vertex->pin());
    sta::InstancePinIterator *pin_iter = network->pinIterator(fanin_inst);
    while (pin_iter->hasNext()) {
      sta::Pin* pin = pin_iter->next();
      sta::Vertex *adj_vertex, *bidirect_vertex;
      graph->pinVertices(pin, adj_vertex, bidirect_vertex);
      if (adj_vertex == nullptr || bidirect_vertex != nullptr) {
        logger_->error(utl::RES, 310, "Pin of adjacent gate {} has bidirectional or no vertex.", network->name(pin));
        fanin_vertices_set.clear();
        break;
      }
      fanin_vertices_set.insert(adj_vertex);
    }
    delete pin_iter;
    for (sta::Vertex* fanin_gate_vertex : fanin_vertices_set) {
      cut_vertices.insert(fanin_gate_vertex);
    }
    for (sta::Vertex* fanin_gate_vertex : fanin_vertices_set) {
      if (network->direction(fanin_gate_vertex->pin())->isOutput())
        collectFanoutVerticesInWindow(fanin_gate_vertex, current_depth - 1, cut_vertices);
      else if (network->direction(fanin_gate_vertex->pin())->isInput())
        collectFaninVerticesInWindow(fanin_gate_vertex, current_depth - 1, cut_vertices);
    }
  }
}

void 
ExtractLocalWindow::collectFanoutVerticesInWindow(sta::Vertex* output_vertex, 
                   size_t current_depth,
                  //  Return value
                   sta::VertexSet &cut_vertices) 
{
  sta::dbNetwork* network = sta_->getDbNetwork();
  sta::Graph* graph = sta_->graph();
  printf("collectFanoutVerticesInWindow depth=%zu at vertex %s\n", current_depth, output_vertex->name(network));
  fflush(stdout);
  if (current_depth == 0) {
    return;
  }
  sta::VertexOutEdgeIterator out_edge_iter(output_vertex, graph);
  while (out_edge_iter.hasNext()) {
    sta::VertexSet fanout_vertices_set(graph);
    sta::Edge* edge = out_edge_iter.next();
    if (!abc_search_pred_->searchThru(edge)) {
      continue;
    }
    sta::Vertex* fanout_vertex = edge->to(graph);
    // Avoid revisiting vertices
    if (cut_vertices.find(fanout_vertex) != cut_vertices.end()) {
      continue;
    }
    sta::Instance *fanout_inst = network->instance(fanout_vertex->pin());
    sta::InstancePinIterator *pin_iter = network->pinIterator(fanout_inst);
    while (pin_iter->hasNext()) {
      sta::Pin* pin = pin_iter->next();
      sta::Vertex *adj_vertex, *bidirect_vertex;
      graph->pinVertices(pin, adj_vertex, bidirect_vertex);
      if (adj_vertex == nullptr || bidirect_vertex != nullptr) {
        logger_->error(utl::RES, 309, "Pin of adjacent gate {} has bidirectional or no vertex.", network->name(pin));
        fanout_vertices_set.clear();
        break;
      }
      fanout_vertices_set.insert(adj_vertex);
    }
    delete pin_iter;
    for (sta::Vertex* fanout_gate_vertex : fanout_vertices_set) {
      cut_vertices.insert(fanout_gate_vertex);
    }
    for (sta::Vertex* fanout_gate_vertex : fanout_vertices_set) {
      if (network->direction(fanout_gate_vertex->pin())->isOutput())
        collectFanoutVerticesInWindow(fanout_gate_vertex, current_depth - 1, cut_vertices);
      else if (network->direction(fanout_gate_vertex->pin())->isInput())
        collectFaninVerticesInWindow(fanout_gate_vertex, current_depth - 1, cut_vertices);
    }
  }
}

cut::LogicCut 
ExtractLocalWindow::extractBottleneck(SeqRemapper& remapper) 
{
  if (ref_gate_ == nullptr) {
    remapper.getLogger()->error(
        utl::RES, 307, "Reference gate not set for ExtractLocalWindow strategy.");
  }

  sta_ = remapper.getSta();
  abc_library_ = remapper.getAbcLibrary();
  abc_search_pred_ = new SearchABCCompatiblePred(sta_, abc_library_, sta_->graph());

  sta::Graph* graph = sta_->graph();
  sta::VertexSet cut_vertices(graph);
  collectAdjacentInsts(ref_gate_, window_size_, cut_vertices);
  LogicExtractorFactoryPro logic_extractor(sta_, logger_);
  cut::LogicCut cut = logic_extractor.buildLogicCutFromCutVertices(cut_vertices, *remapper.getAbcLibrary());
  return cut;
}

std::string
ExtractLocalWindow::to_string() {
  return "ExtractLocalWindow";
}



} // namespace rmp
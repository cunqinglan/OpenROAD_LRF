// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2025, The OpenROAD Authors

#include "ord/Timing.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "grt/GRoute.h"
#include "grt/GlobalRouter.h"
#include "sta/Network.hh"
#include "sta/Parasitics.hh"
#include "sta/ParasiticsClass.hh"
#include "sta/PortDirection.hh"

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/PtrSetMap.h"
#include "odb/db.h"
#include "ord/Design.h"
#include "ord/OpenRoad.hh"
#include "ord/Tech.h"
#include "rsz/Resizer.hh"
#include "sta/Clock.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/Mode.hh"
#include "sta/Path.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/PathGroup.hh"
#include "sta/PowerClass.hh"
#include "sta/Scene.hh"
#include "sta/Sdc.hh"
#include "sta/SdcClass.hh"
#include "sta/Search.hh"
#include "sta/SearchClass.hh"
#include "sta/StringUtil.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "sta/Graph.hh"
#include "utl/Logger.h"

#include "sta/Graph.hh"
#include "sta/ArcDelayCalc.hh"
#include "lrf/LrConfig.hh"
#include "lrf/IncreSta.hh"
#include "lrf/TestLrf.hh"


namespace ord {

Timing::Timing(Design* design) : design_(design)
{
}

sta::dbSta* Timing::getSta()
{
  return design_->getTech()->getSta();
}

std::pair<odb::dbITerm*, odb::dbBTerm*> Timing::staToDBPin(const sta::Pin* pin)
{
  sta::dbNetwork* db_network = getSta()->getDbNetwork();
  odb::dbITerm* iterm;
  odb::dbBTerm* bterm;
  odb::dbModITerm* moditerm;
  db_network->staToDb(pin, iterm, bterm, moditerm);
  return std::make_pair(iterm, bterm);
}

bool Timing::isEndpoint(odb::dbITerm* db_pin)
{
  sta::Pin* sta_pin = getSta()->getDbNetwork()->dbToSta(db_pin);
  return isEndpoint(sta_pin);
}

bool Timing::isEndpoint(odb::dbBTerm* db_pin)
{
  sta::Pin* sta_pin = getSta()->getDbNetwork()->dbToSta(db_pin);
  return isEndpoint(sta_pin);
}

bool Timing::isEndpoint(sta::Pin* sta_pin)
{
  auto search = getSta()->search();
  auto vertex_array = vertices(sta_pin);
  for (auto vertex : vertex_array) {
    if (vertex != nullptr && search->isEndpoint(vertex)) {
      return true;
    }
  }
  return false;
}

float Timing::slewAllCorners(sta::Vertex* vertex, const sta::MinMax* minmax)
{
  auto sta = getSta();
  return sta::delayAsFloat(
      sta->slew(vertex, sta::RiseFallBoth::riseFall(), sta->scenes(), minmax));
}

float Timing::getPinSlew(odb::dbITerm* db_pin, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Pin* sta_pin = sta->getDbNetwork()->dbToSta(db_pin);
  return getPinSlew(sta_pin, minmax);
}

float Timing::getPinSlew(odb::dbBTerm* db_pin, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Pin* sta_pin = sta->getDbNetwork()->dbToSta(db_pin);
  return getPinSlew(sta_pin, minmax);
}

float Timing::getPinSlew(sta::Pin* sta_pin, MinMax minmax)
{
  auto vertex_array = vertices(sta_pin);
  float pin_slew = (minmax == Max) ? -sta::INF : sta::INF;
  for (auto vertex : vertex_array) {
    if (vertex != nullptr) {
      const float pin_slew_temp = slewAllCorners(vertex, getMinMax(minmax));
      pin_slew = (minmax == Max) ? std::max(pin_slew, pin_slew_temp)
                                 : std::min(pin_slew, pin_slew_temp);
    }
  }
  return pin_slew;
}

sta::Network* Timing::cmdLinkedNetwork()
{
  sta::Network* network = getSta()->cmdNetwork();
  if (network->isLinked()) {
    return network;
  }

  design_->getLogger()->error(utl::ORD, 104, "STA network is not linked.");
}

sta::Graph* Timing::cmdGraph()
{
  cmdLinkedNetwork();
  return getSta()->ensureGraph();
}

std::array<sta::Vertex*, 2> Timing::vertices(const sta::Pin* pin)
{
  sta::Vertex *vertex, *vertex_bidirect_drvr;
  std::array<sta::Vertex*, 2> vertices;

  cmdGraph()->pinVertices(pin, vertex, vertex_bidirect_drvr);
  vertices[0] = vertex;
  vertices[1] = vertex_bidirect_drvr;
  return vertices;
}

bool Timing::isTimeInf(float time)
{
  return (time > 1e+10 || time < -1e+10);
}

float Timing::getPinArrivalTime(sta::Clock* clk,
                                const sta::RiseFall* clk_rf,
                                sta::Vertex* vertex,
                                const sta::RiseFall* rf)
{
  sta::dbSta* sta = getSta();
  (void) clk;
  (void) clk_rf;
  return sta::delayAsFloat(sta->arrival(
      vertex, rf->asRiseFallBoth(), sta->scenes(), sta::MinMax::max()));
}

sta::ClockSeq Timing::findClocksMatching(const char* pattern,
                                         bool regexp,
                                         bool nocase)
{
  auto sta = getSta();
  cmdLinkedNetwork();
  sta::PatternMatch matcher(pattern, regexp, nocase, sta->tclInterp());
  return sta->cmdMode()->sdc()->findClocksMatching(&matcher);
}

float Timing::getPinArrival(odb::dbITerm* db_pin, RiseFall rf, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Pin* sta_pin = sta->getDbNetwork()->dbToSta(db_pin);
  return getPinArrival(sta_pin, rf, minmax);
}

float Timing::getPinArrival(odb::dbBTerm* db_pin, RiseFall rf, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Pin* sta_pin = sta->getDbNetwork()->dbToSta(db_pin);
  return getPinArrival(sta_pin, rf, minmax);
}

float Timing::getPinArrival(sta::Pin* sta_pin, RiseFall rf, MinMax minmax)
{
  auto vertex_array = vertices(sta_pin);
  float delay = (minmax == Max) ? -sta::INF : sta::INF;
  float d1, d2;
  sta::Clock* default_arrival_clock
      = getSta()->cmdMode()->sdc()->defaultArrivalClock();
  for (auto vertex : vertex_array) {
    if (vertex == nullptr) {
      continue;
    }
    const sta::RiseFall* clk_r = sta::RiseFall::rise();
    const sta::RiseFall* clk_f = sta::RiseFall::fall();
    const sta::RiseFall* arrive_hold = (rf == Rise) ? clk_r : clk_f;
    d1 = getPinArrivalTime(nullptr, clk_r, vertex, arrive_hold);
    d2 = getPinArrivalTime(default_arrival_clock, clk_r, vertex, arrive_hold);
    delay = (minmax == Max) ? std::max({d1, d2, delay})
                            : std::min({d1, d2, delay});
    for (auto clk : findClocksMatching("*", false, false)) {
      d1 = getPinArrivalTime(clk, clk_r, vertex, arrive_hold);
      d2 = getPinArrivalTime(clk, clk_f, vertex, arrive_hold);
      delay = (minmax == Max) ? std::max({d1, d2, delay})
                              : std::min({d1, d2, delay});
    }
  }
  return delay;
}

std::vector<sta::Scene*> Timing::getCorners()
{
  auto& corners = getSta()->scenes();
  return {corners.begin(), corners.end()};
}

sta::Scene* Timing::cmdCorner()
{
  return getSta()->cmdScene();
}

sta::Scene* Timing::findCorner(const char* name)
{
  for (auto* corner : getCorners()) {
    if (strcmp(corner->name().c_str(), name) == 0) {
      return corner;
    }
  }

  return nullptr;
}

float Timing::getPinSlack(odb::dbITerm* db_pin, RiseFall rf, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Pin* sta_pin = sta->getDbNetwork()->dbToSta(db_pin);
  return getPinSlack(sta_pin, rf, minmax);
}

float Timing::getPinSlack(odb::dbBTerm* db_pin, RiseFall rf, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Pin* sta_pin = sta->getDbNetwork()->dbToSta(db_pin);
  return getPinSlack(sta_pin, rf, minmax);
}

float Timing::getPinSlack(sta::Pin* sta_pin, RiseFall rf, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  auto sta_rf = (rf == Rise) ? sta::RiseFall::rise() : sta::RiseFall::fall();
  return sta->slack(
      sta_pin, sta_rf->asRiseFallBoth(), sta->scenes(), getMinMax(minmax));
}

// I'd like to return a std::set but swig gave me way too much grief
// so I just copy the set to a vector.
std::vector<odb::dbMTerm*> Timing::getTimingFanoutFrom(odb::dbMTerm* input)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();

  odb::dbMaster* master = input->getMaster();
  sta::Cell* cell = network->dbToSta(master);
  if (!cell) {
    return {};
  }

  sta::LibertyCell* lib_cell = network->libertyCell(cell);
  if (!lib_cell) {
    return {};
  }

  sta::Port* port = network->dbToSta(input);
  sta::LibertyPort* lib_port = network->libertyPort(port);

  odb::PtrSet<odb::dbMTerm> outputs;
  for (auto arc_set : lib_cell->timingArcSets(lib_port, /* to */ nullptr)) {
    const sta::TimingRole* role = arc_set->role();
    if (role->isTimingCheck() || role->isAsyncTimingCheck()
        || role->isNonSeqTimingCheck() || role->isDataCheck()) {
      continue;
    }
    sta::LibertyPort* to_port = arc_set->to();
    odb::dbMTerm* to_mterm = master->findMTerm(to_port->name().c_str());
    if (to_mterm) {
      outputs.insert(to_mterm);
    }
  }
  return {outputs.begin(), outputs.end()};
}

const sta::MinMax* Timing::getMinMax(MinMax type)
{
  return type == Max ? sta::MinMax::max() : sta::MinMax::min();
}

float Timing::getNetCap(odb::dbNet* net, sta::Scene* corner, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Net* sta_net = sta->getDbNetwork()->dbToSta(net);

  float pin_cap;
  float wire_cap;
  sta->connectedCap(sta_net, corner, getMinMax(minmax), pin_cap, wire_cap);
  return pin_cap + wire_cap;
}

float Timing::getPortCap(odb::dbITerm* pin, sta::Scene* corner, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Pin* sta_pin = network->dbToSta(pin);
  sta::LibertyPort* lib_port = network->libertyPort(sta_pin);
  return sta->capacitance(lib_port, corner, getMinMax(minmax));
}

float Timing::getMaxCapLimit(odb::dbMTerm* pin)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Port* port = network->dbToSta(pin);
  sta::LibertyPort* lib_port = network->libertyPort(port);
  sta::LibertyLibrary* lib = network->defaultLibertyLibrary();
  float max_cap = 0.0;
  bool max_cap_exists = false;
  if (!pin->getSigType().isSupply()) {
    lib_port->capacitanceLimit(sta::MinMax::max(), max_cap, max_cap_exists);
    if (!max_cap_exists) {
      lib->defaultMaxCapacitance(max_cap, max_cap_exists);
    }
  }
  return max_cap;
}

float Timing::getMaxSlewLimit(odb::dbMTerm* pin)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Port* port = network->dbToSta(pin);
  sta::LibertyPort* lib_port = network->libertyPort(port);
  sta::LibertyLibrary* lib = network->defaultLibertyLibrary();
  float max_slew = 0.0;
  bool max_slew_exists = false;
  if (!pin->getSigType().isSupply()) {
    lib_port->slewLimit(sta::MinMax::max(), max_slew, max_slew_exists);
    if (!max_slew_exists) {
      lib->defaultMaxSlew(max_slew, max_slew_exists);
    }
  }
  return max_slew;
}

float Timing::staticPower(odb::dbInst* inst, sta::Scene* corner)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();

  sta::Instance* sta_inst = network->dbToSta(inst);
  if (!sta_inst) {
    return 0.0;
  }
  sta::PowerResult power = sta->power(sta_inst, corner);
  return power.leakage();
}

float Timing::dynamicPower(odb::dbInst* inst, sta::Scene* corner)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();

  sta::Instance* sta_inst = network->dbToSta(inst);
  if (!sta_inst) {
    return 0.0;
  }
  sta::PowerResult power = sta->power(sta_inst, corner);
  return (power.internal() + power.switching());
}

void Timing::makeEquivCells()
{
  rsz::Resizer* resizer = design_->getResizer();
  resizer->makeEquivCells();
}

std::vector<odb::dbMaster*> Timing::equivCells(odb::dbMaster* master)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Cell* cell = network->dbToSta(master);
  std::vector<odb::dbMaster*> master_seq;
  if (cell) {
    sta::LibertyCell* libcell = network->libertyCell(cell);
    sta::LibertyCellSeq* equiv_cells = sta->equivCells(libcell);
    if (equiv_cells) {
      for (sta::LibertyCell* equiv_cell : *equiv_cells) {
        odb::dbMaster* equiv_master = network->staToDb(equiv_cell);
        master_seq.emplace_back(equiv_master);
      }
    } else {
      master_seq.emplace_back(master);
    }
  }
  return master_seq;
}
float Timing::getWorstSlack(MinMax minmax)
{
  sta::dbSta* sta = getSta();
  cmdLinkedNetwork();
  return sta->worstSlack(getMinMax(minmax));
}

float Timing::getTotalNegativeSlack(MinMax minmax)
{
  sta::dbSta* sta = getSta();
  cmdLinkedNetwork();
  return sta->totalNegativeSlack(getMinMax(minmax));
}

int Timing::getEndpointCount()
{
  sta::dbSta* sta = getSta();
  cmdLinkedNetwork();
  return sta->endpoints().size();
}

std::vector<EndpointSlack> Timing::getEndpointSlacks(MinMax minmax)
{
  sta::dbSta* sta = getSta();
  cmdLinkedNetwork();

  std::vector<EndpointSlack> result;
  for (sta::Vertex* vertex : sta->endpoints()) {
    const sta::Pin* pin = vertex->pin();
    float slack = sta->slack(
        pin, sta::RiseFallBoth::riseFall(), sta->scenes(), getMinMax(minmax));
    auto [iterm, bterm] = staToDBPin(pin);
    result.push_back({iterm, bterm, slack});
  }
  return result;
}

std::vector<ClockInfo> Timing::getClockInfo()
{
  sta::dbSta* sta = getSta();
  cmdLinkedNetwork();

  std::vector<ClockInfo> result;
  for (const sta::Clock* clk : sta->cmdMode()->sdc()->clocks()) {
    ClockInfo info;
    info.name = clk->name();
    info.period = clk->period();
    info.waveform = clk->waveform();
    for (const sta::Pin* pin : clk->pins()) {
      auto [iterm, bterm] = staToDBPin(pin);
      if (iterm) {
        info.source_iterms.push_back(iterm);
      }
      if (bterm) {
        info.source_bterms.push_back(bterm);
      }
    }
    result.push_back(std::move(info));
  }
  return result;
}

std::vector<TimingPathInfo> Timing::getTimingPaths(MinMax minmax,
                                                   int max_paths,
                                                   float slack_threshold)
{
  sta::dbSta* sta = getSta();
  cmdLinkedNetwork();
  sta::dbNetwork* network = sta->getDbNetwork();

  const bool is_setup = (minmax == Max);
  sta::SceneSeq scenes = sta->scenes();
  sta::StringSeq group_names;

  sta->ensureGraph();
  sta->searchPreamble();

  sta::Search* search = sta->search();
  sta::PathEndSeq path_ends = search->findPathEnds(
      nullptr,  // from
      nullptr,  // thrus
      nullptr,  // to
      false,    // unconstrained
      scenes,
      is_setup ? sta::MinMaxAll::max() : sta::MinMaxAll::min(),
      max_paths,        // group_count
      1,                // endpoint_count (one per endpoint)
      true,             // unique_pins
      true,             // unique_edges
      -sta::INF,        // slack_min
      slack_threshold,  // slack_max
      true,             // sort_by_slack
      group_names,
      is_setup,   // setup
      !is_setup,  // hold
      false,      // recovery
      false,      // removal
      false,      // clk_gating_setup
      false);     // clk_gating_hold

  std::vector<TimingPathInfo> result;
  auto* graph = sta->graph();
  const sta::Sdc* sdc = sta->cmdScene()->sdc();
  sta::Mode* mode = sta->cmdScene()->mode();
  sta::GraphDelayCalc* gdc = sta->graphDelayCalc();
  sta::dbNetwork* db_network = sta->getDbNetwork();

  for (auto& path_end : path_ends) {
    TimingPathInfo path_info;
    sta::Path* path = path_end->path();

    path_info.slack = path_end->slack(sta);
    path_info.arrival = path_end->dataArrivalTime(sta);
    path_info.required = path_end->requiredTime(sta);
    path_info.skew = path_end->clkSkew(sta);

    auto* path_delay = path_end->pathDelay();
    path_info.path_delay = path_delay ? path_delay->delay() : 0.0f;

    auto* start_clk_edge = path_end->sourceClkEdge(sta);
    path_info.start_clock
        = start_clk_edge ? start_clk_edge->clock()->name() : "";

    auto* end_clk = path_end->targetClk(sta);
    path_info.end_clock = end_clk ? end_clk->name() : "";

    auto* path_group = path_end->pathGroup();
    path_info.path_group = path_group ? path_group->name() : "";

    // Expand path to get arc detail
    sta::PathExpanded expand(path, sta);
    float arrival_prev = 0.0f;
    float logic_delay_total = 0.0f;
    int logic_depth_count = 0;
    int max_fanout = 0;
    std::unordered_set<sta::Instance*> logic_insts;

    for (size_t i = 0; i < expand.size(); i++) {
      const auto* ref = expand.path(i);
      sta::Vertex* vertex = ref->vertex(sta);
      const sta::Pin* pin = vertex->pin();
      const bool is_rising = ref->transition(sta) == sta::RiseFall::rise();
      const float arr = sta::delayAsFloat(ref->arrival());
      const float slw = sta::delayAsFloat(ref->slew(sta));
      const float pin_delay = arr - arrival_prev;

      // Compute fanout
      int node_fanout = 0;
      sta::VertexOutEdgeIterator iter(vertex, graph);
      while (iter.hasNext()) {
        sta::Edge* edge = iter.next();
        if (edge->isWire()) {
          const sta::Pin* to_pin = edge->to(graph)->pin();
          if (network->isTopLevelPort(to_pin)) {
            sta::Port* port = network->port(to_pin);
            node_fanout += sdc->portExtFanout(port, sta::MinMax::max()) + 1;
          } else {
            node_fanout++;
          }
        }
      }
      max_fanout = std::max(node_fanout, max_fanout);

      // Compute load capacitance
      float cap = 0.0f;
      const bool is_driver = network->isDriver(pin);
      if (is_driver && i > 0) {
        cap = gdc->loadCap(
            pin, ref->transition(sta), ref->scene(sta), ref->minMax(sta));
      }

      // Determine master, net arcs, logic depth, and build arc info
      if (i > 0) {
        const auto* prev_ref = expand.path(i - 1);
        sta::Vertex* prev_vertex = prev_ref->vertex(sta);
        const sta::Pin* prev_pin = prev_vertex->pin();
        sta::Instance* inst = network->instance(pin);
        sta::Instance* prev_inst = network->instance(prev_pin);

        const bool same_inst = (inst == prev_inst && inst != nullptr);

        // Track logic depth (non-clock, non-net arcs)
        bool pin_is_clock = sta->isClock(pin, mode);
        if (same_inst && !pin_is_clock) {
          if (logic_insts.find(inst) == logic_insts.end()) {
            logic_insts.insert(inst);
            logic_depth_count++;
            logic_delay_total += pin_delay;
          }
        }

        TimingArcInfo arc;
        odb::dbModITerm* mod_iterm;
        db_network->staToDb(
            prev_pin, arc.from_iterm, arc.from_bterm, mod_iterm);
        db_network->staToDb(pin, arc.to_iterm, arc.to_bterm, mod_iterm);
        if (same_inst && arc.to_iterm) {
          arc.master = arc.to_iterm->getInst()->getMaster();
        }
        arc.delay = pin_delay;
        arc.slew = slw;
        arc.load = cap;
        arc.fanout = node_fanout;
        arc.is_rising = is_rising;
        path_info.arcs.push_back(arc);
      }

      arrival_prev = arr;
    }

    // Get startpoint/endpoint objects
    odb::dbModITerm* mod_iterm;
    db_network->staToDb(expand.path(0)->vertex(sta)->pin(),
                        path_info.start_iterm,
                        path_info.start_bterm,
                        mod_iterm);
    db_network->staToDb(path_end->vertex(sta)->pin(),
                        path_info.end_iterm,
                        path_info.end_bterm,
                        mod_iterm);

    path_info.logic_delay = logic_delay_total;
    path_info.logic_depth = logic_depth_count;
    path_info.fanout = max_fanout;

    result.push_back(std::move(path_info));
  }
  return result;
}


//////////////////////////////////////////
// Functions for LR sizing
//////////////////////////////////////////
float
Timing::getLmDelaySum(odb::dbInst* inst, const sta::MinMax *minmax) {
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Instance* sta_inst = network->dbToSta(inst);
  float delay_lambda_sum = 0.0;
  sta->getIncreSta()->delayLmSum(sta_inst, minmax, delay_lambda_sum);
  return delay_lambda_sum;
}

bool
Timing::checkErcViolations(odb::dbInst* inst, sta::Scene* corner) {
  bool violated = false;
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  sta::Instance* sta_inst = network->dbToSta(inst);

  sta::InstancePinIterator* pin_iterator = sta->network()->pinIterator(sta_inst);
  while (pin_iterator->hasNext()) {
    sta::Pin* pin = pin_iterator->next();
    // Check max slew
    float limit = sta->getIncreSta()->maxInputSlew(pin, corner);
    for (const sta::RiseFall* rf : sta::RiseFall::range()) {
      if (network->isLoad(pin)) {
        sta::Vertex *vertex = sta->graph()->pinLoadVertex(pin);
        const sta::DcalcAPIndex dcalc_ap = corner->dcalcAnalysisPtIndex(sta::MinMax::max());
        float actual_slew = sta->graph()->slew(vertex, rf, dcalc_ap);
        if (actual_slew > limit) {
          violated = true;
          // Use the project's logger (fmt-style) instead of printf to avoid
          // format-string/type mismatches and integrate with logging.
          design_->getLogger()->report(
            "ERC Violation: Instance {} Pin {} exceeds max slew limit {:.3f} with actual slew {:.3f}",
            inst->getName(),
            network->name(pin),
            limit,
            actual_slew);
        }
      }
    }
    // Check max capacitance
    if (network->isDriver(pin)) {
      const sta::Scene* corner1 = nullptr;
      float cap1, max_cap1, cap_slack1;
      const sta::RiseFall* rf;
      sta->checkCapacitance(pin, sta->scenes(), sta::MinMax::max(),
                            cap1, max_cap1, cap_slack1, rf, corner1);
      if (cap_slack1 < 0.0) {
        violated = true;
        design_->getLogger()->report(
          "ERC Violation: Instance {} Pin {} exceeds max capacitance limit {:.3f} with actual capacitance {:.3f}",
          inst->getName(),
          network->name(pin),
          max_cap1,
          cap1);
      }
    }
  }
  return violated;
}

void
Timing::lmUpdate() {
  sta::dbSta* sta = getSta();
  sta->getIncreSta()->lmUpdate();
}

bool
Timing::saveLmSnapshot(const char *path) {
  sta::dbSta* sta = getSta();
  odb::dbBlock* block = design_->getBlock();
  std::string design_name = block->getName();
  return sta->getIncreSta()->saveLmToFile(path, design_name);
}

bool
Timing::loadLmSnapshot(const char *path) {
  sta::dbSta* sta = getSta();
  odb::dbBlock* block = design_->getBlock();
  std::string design_name = block->getName();
  int frame_id = sta->getIncreSta()->loadLmFromFile(path, design_name);
  return frame_id >= 0;
}

float 
Timing::averageDelayOnCritPath() {
  sta::dbSta* sta = getSta();
  return sta->getIncreSta()->averageDelayOnCritPath();
}


////////////////////////////////////////////
// Functions of testing IncreSta
////////////////////////////////////////////
void
Timing::testLocalDelayCompute(char *inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testLocalDelayCompute(inst_name, sta, resizer, design_->getBlock());
}

void 
Timing::testLocalArrivalCompute(char *inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testLocalArrivalCompute(inst_name, sta, resizer, design_->getBlock());
}

void 
Timing::testLocalSlewCompute(char *inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testLocalSlewCompute(inst_name, sta, resizer, design_->getBlock());
}

void
Timing::testPtGraphErrors(char* inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testDifferenceBetweenLocalAndOpen(inst_name, sta, resizer, design_->getBlock());
}

void
Timing::testMEEAssignments() {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testMEEAssignments(sta, resizer, design_->getBlock());
}

void
Timing::testReportFFEndpointLMs(char *inst_name, int lm_iters) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testReportFFEndpointLMs(inst_name, sta, resizer,
                                   design_->getBlock(),
                                   static_cast<size_t>(lm_iters));
}

void
Timing::testReportFFPtGraph(char *inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testReportFFPtGraph(inst_name, sta, resizer, design_->getBlock());
}

void
Timing::testFFLocalDelay(char *inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testFFLocalDelay(inst_name, sta, resizer, design_->getBlock());
}

void
Timing::runLr(const lrf::LrConfig &cfg) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting runLr (mode=%d) with %zu threads\n",
         static_cast<int>(cfg.mode), thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.runLr(sta, resizer, design_->getBlock(), thread_num, cfg);
}

void
Timing::runLr(int mode, size_t iterations, size_t max_resize_num,
              size_t num_no_improve_tolerance, float PT_tradeoff,
              float density_weight, bool ratcons,
              const char *lr_helper_method, float top_ratio,
              const char *checkpoint_dir,
              bool debug, size_t buffering_start_iter,
              float timing_margin, bool resize_ff, bool verbose) {
  lrf::LrConfig cfg;
  cfg.mode = static_cast<lrf::LrMode>(mode);
  cfg.iterations = iterations;
  cfg.max_resize_num = max_resize_num;
  cfg.num_no_improve_tolerance = num_no_improve_tolerance;
  cfg.PT_tradeoff = PT_tradeoff;
  cfg.density_weight = density_weight;
  cfg.ratcons = ratcons;
  cfg.lr_helper_method = lr_helper_method;
  cfg.top_ratio = top_ratio;
  cfg.checkpoint_dir = checkpoint_dir ? checkpoint_dir : "";
  cfg.debug = debug;
  cfg.buffering_start_iter = buffering_start_iter;
  cfg.timing_margin = timing_margin;
  cfg.resize_ff = resize_ff;
  cfg.verbose = verbose;
  runLr(cfg);
}

void
Timing::runInitialization(bool minimize_leakage) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.runInitializationStandalone(sta, resizer, design_->getBlock(),
                                       thread_num, minimize_leakage);
}

void
Timing::debugPrecheckAccuracy(float PT_tradeoff, float top_ratio) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting debugPrecheckAccuracy with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.debugPrecheckAccuracy(sta, resizer, design_->getBlock(),
    thread_num, PT_tradeoff, top_ratio);
}

void
Timing::testParallelResizeByArray(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, float density_weight,
  bool resize_ff) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArray(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, density_weight,
    /* checkpoint_dir */ "", /* timing_margin */ 0.01f, resize_ff);
}

void
Timing::testParallelResizeByArrayWithBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, float density_weight) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, density_weight);
}

void
Timing::testParallelResizeByArrayWithRszBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, float density_weight) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithRszBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithRszBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, density_weight);
}

void
Timing::testParallelResizeByArrayWithSdpBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, float density_weight,
  float timing_margin) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithSdpBuffering with %zu threads, timing_margin=%.4f\n",
         thread_num, timing_margin);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithSdpBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, density_weight,
    /*debug=*/false, /*buffering_start_iter=*/5, timing_margin);
}

void
Timing::testInitResizeThenSdpBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, float density_weight,
  float timing_margin, float erc_violation_weight, float erc_limit_scale,
  bool resize_ff) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testInitResizeThenSdpBuffering with %zu threads, "
         "timing_margin=%.4f, erc_violation_weight=%.3g, erc_limit_scale=%.3f, "
         "resize_ff=%d\n",
         thread_num, timing_margin, erc_violation_weight, erc_limit_scale,
         resize_ff);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testInitResizeThenSdpBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, density_weight,
    /*debug=*/false, /*buffering_start_iter=*/5, timing_margin,
    erc_violation_weight, erc_limit_scale, resize_ff);
}

void
Timing::testEcoResizeNoHalve(size_t iterations, float PT_tradeoff,
  const char *lr_helper_method, float halve_factor, bool use_precheck) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testEcoResizeNoHalve with %zu threads, halve_factor=%.2f, precheck=%s\n",
         thread_num, halve_factor, use_precheck ? "yes" : "no");
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testEcoResizeNoHalve(sta, resizer, design_->getBlock(),
    thread_num, iterations, PT_tradeoff, lr_helper_method, halve_factor,
    use_precheck);
}

void
Timing::testCombinedResizeBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testCombinedResizeBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrCombinedResizeBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method);
}

void
Timing::testBufferOnly(size_t iterations, float PT_tradeoff,
  const char *lr_helper_method, float bakoglu_k, bool debug) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testBufferOnly with %zu threads, bakoglu_k=%.2f, debug=%d\n",
         thread_num, bakoglu_k, debug);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testBufferOnly(sta, resizer, design_->getBlock(),
    thread_num, iterations, PT_tradeoff, lr_helper_method, bakoglu_k, debug);
}

void
Timing::testSingleBufferPass(bool use_sdp, float PT_tradeoff,
  const char *lr_helper_method, size_t lm_warmup_rounds) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testSingleBufferPass with %zu threads, use_sdp=%d\n",
         thread_num, use_sdp);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testSingleBufferPass(sta, resizer, design_->getBlock(),
    thread_num, use_sdp, PT_tradeoff, lr_helper_method, lm_warmup_rounds);
}

void
Timing::testParallelResizeByArrayWithPrecheck(size_t max_resize_num,
  size_t iterations, size_t num_no_improve_tolerance, bool ratcons,
  float PT_tradeoff, const char *lr_helper_method, float top_ratio) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithPrecheck with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithPrecheck(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, top_ratio);
}

void
Timing::testParallelResizeByArrayWithPrecheckBuffering(size_t max_resize_num,
  size_t iterations, size_t num_no_improve_tolerance, bool ratcons,
  float PT_tradeoff, const char *lr_helper_method, float top_ratio) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithPrecheckBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithPrecheckBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, top_ratio);
}

void
Timing::testPrecedingResizeCheck(float PT_tradeoff, float top_ratio)
{
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testPrecedingResizeCheck with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testPrecedingResizeCheck(sta, resizer, design_->getBlock(),
    thread_num, PT_tradeoff, top_ratio);
}

void
Timing::testParallelKKTProjection(const char *lr_helper_method)
{
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelKKTProjection with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelKKTProjection(sta, resizer, design_->getBlock(),
    thread_num, lr_helper_method);
}

void
Timing::testLocalStaAccuracy(size_t max_steps)
{
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testLocalStaAccuracy(sta, resizer, design_->getBlock(), max_steps);
}


void
Timing::testSlewViolationFeasibility()
{
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testSlewViolationFeasibility(sta, resizer, design_->getBlock());
}

void
Timing::testRepairSlew()
{
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testRepairSlew(sta, resizer, design_->getBlock());
}

void
Timing::testBufferingRsz(float PT_tradeoff, int top_n)
{
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  lrf::TestLrf test_lrf;
  test_lrf.testBufferingRsz(sta, resizer, design_->getBlock(),
                             thread_num, PT_tradeoff, top_n);
}

void
Timing::probeRszBnet() {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  lrf::TestLrf test_lrf;
  test_lrf.probeRszBnet(sta, resizer, design_->getBlock(), thread_num);
}

void
Timing::probeBufferOneByOne(bool use_rsz) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  lrf::TestLrf test_lrf;
  test_lrf.probeBufferOneByOne(sta, resizer, design_->getBlock(),
                                thread_num, use_rsz);
}

void
Timing::probeBufferDeep(const char *pin_names_csv) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  // Parse CSV into vector
  std::vector<std::string> pin_names;
  std::string csv(pin_names_csv);
  size_t pos = 0;
  while ((pos = csv.find(',')) != std::string::npos) {
    std::string token = csv.substr(0, pos);
    if (!token.empty()) pin_names.push_back(token);
    csv.erase(0, pos + 1);
  }
  if (!csv.empty()) pin_names.push_back(csv);
  lrf::TestLrf test_lrf;
  test_lrf.probeBufferDeep(sta, resizer, design_->getBlock(),
                           thread_num, pin_names);
}

void
Timing::probeAllOptions(const char *pin_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  lrf::TestLrf test_lrf;
  test_lrf.probeAllOptions(sta, resizer, design_->getBlock(),
                           thread_num, pin_name);
}

void
Timing::probeAllOptionsBySensitivity(int top_n) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  lrf::TestLrf test_lrf;
  test_lrf.probeAllOptionsBySensitivity(sta, resizer, design_->getBlock(),
                                         thread_num, top_n);
}

void
Timing::testReportVertices() {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  odb::dbBlock* block = design_->getBlock();
  test_lrf.testReportVertices(sta, resizer, block);
  // grep a inst and print its vertices
  const char *inst_name = "u_NV_NVDLA_sdp_u_wdma_u_intr_stl_cnt_cur_reg[22]";
  odb::dbInst* inst = block->findInst(inst_name);
  if (inst) {
    sta::InstancePinIterator* pin_iterator = sta->network()->pinIterator(
        sta->getDbNetwork()->dbToSta(inst));
    while (pin_iterator->hasNext()) {
      sta::Pin* pin = pin_iterator->next();
      sta::Vertex *vertex, *bidirect_vertex;
      sta->graph()->pinVertices(pin, vertex, bidirect_vertex);
      if (vertex) {
        printf("Pin %s Vertex ID: %s\n", sta->network()->name(pin), vertex->to_string(sta->graph()).c_str());
      }
      sta::VertexOutEdgeIterator out_edge_iterator(vertex, sta->graph());
      while (out_edge_iterator.hasNext()) {
        sta::Edge* edge = out_edge_iterator.next();
        printf("  Out Edge %s\n", edge->to_string(sta->graph()).c_str());
      }
      sta::VertexInEdgeIterator in_edge_iterator(vertex, sta->graph());
      while (in_edge_iterator.hasNext()) {
        sta::Edge* edge = in_edge_iterator.next();
        printf("  In Edge %s\n", edge->to_string(sta->graph()).c_str());
      }
    }
  }
}


float Timing::getTns(sta::Scene* corner, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  float tns = sta->totalNegativeSlack(corner, getMinMax(minmax));
  return tns;
}

float Timing::getTns(MinMax minmax)
{
  sta::dbSta* sta = getSta();
  return sta->totalNegativeSlack(getMinMax(minmax));
}

////////////////////////////////////////////////////////////////
// dumpDiagBundle — ML/diagnostic CSV dump.
// TODO(LRF-migration): port to master STA Scene/index API + new GRT
// route/parasitic accessors. Stubbed during phyls->master migration; not
// on the resize/golden path. See git history in OpenROAD_phyls for body.
////////////////////////////////////////////////////////////////
void Timing::dumpDiagBundle(const std::string& prefix)
{
  design_->getLogger()->warn(utl::ORD, 9001,
      "dumpDiagBundle is not yet ported to the current STA API (prefix={}).",
      prefix);
}

}  // namespace ord

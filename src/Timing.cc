// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023-2025, The OpenROAD Authors

#include "ord/Timing.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <set>
#include <utility>
#include <vector>
#include <unordered_map>
#include <map>
#include <string>

#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "ord/Design.h"
#include "ord/OpenRoad.hh"
#include "ord/Tech.h"
#include "rsz/Resizer.hh"
#include "sta/Clock.hh"
#include "sta/Corner.hh"
#include "sta/Liberty.hh"
#include "sta/LibertyClass.hh"
#include "sta/MinMax.hh"
#include "sta/PowerClass.hh"
#include "sta/Search.hh"
#include "sta/TimingArc.hh"
#include "sta/TimingRole.hh"
#include "sta/Graph.hh"
#include "utl/Logger.h"


#include "sta/DcalcAnalysisPt.hh"
#include "sta/ArcDelayCalc.hh"
#include "sta/PortDirection.hh"
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
  bool max = (minmax == sta::MinMax::max());
  float slew = (max) ? -sta::INF : sta::INF;
  float slew_corner;
  for (auto corner : getCorners()) {
    slew_corner = sta::delayAsFloat(
        sta->vertexSlew(vertex, sta::RiseFall::rise(), corner, minmax));
    slew = (max) ? std::max(slew, slew_corner) : std::min(slew, slew_corner);
  }
  return slew;
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

std::vector<float> Timing::arrivalsClk(const sta::RiseFall* rf,
                                       sta::Clock* clk,
                                       const sta::RiseFall* clk_rf,
                                       sta::Vertex* vertex)
{
  auto sta = getSta();
  std::vector<float> arrivals;
  const sta::ClockEdge* clk_edge = nullptr;
  if (clk) {
    clk_edge = clk->edge(clk_rf);
  }
  for (auto path_ap : sta->corners()->pathAnalysisPts()) {
    arrivals.push_back(sta::delayAsFloat(
        sta->vertexArrival(vertex, rf, clk_edge, path_ap, nullptr)));
  }
  return arrivals;
}

bool Timing::isTimeInf(float time)
{
  return (time > 1e+10 || time < -1e+10);
}

float Timing::getPinArrivalTime(sta::Clock* clk,
                                const sta::RiseFall* clk_rf,
                                sta::Vertex* vertex,
                                const sta::RiseFall* arrive_hold)
{
  std::vector<float> times = arrivalsClk(arrive_hold, clk, clk_rf, vertex);
  float delay = -sta::INF;
  for (float delay_time : times) {
    if (!isTimeInf(delay_time)) {
      delay = std::max(delay, delay_time);
    }
  }
  return delay;
}

sta::ClockSeq Timing::findClocksMatching(const char* pattern,
                                         bool regexp,
                                         bool nocase)
{
  auto sta = getSta();
  cmdLinkedNetwork();
  sta::PatternMatch matcher(pattern, regexp, nocase, sta->tclInterp());
  return sta->sdc()->findClocksMatching(&matcher);
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
  sta::Clock* default_arrival_clock = getSta()->sdc()->defaultArrivalClock();
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

std::vector<sta::Corner*> Timing::getCorners()
{
  sta::Corners* corners = getSta()->corners();
  return {corners->begin(), corners->end()};
}

sta::Corner* Timing::cmdCorner()
{
  return getSta()->cmdCorner();
}

sta::Corner* Timing::findCorner(const char* name)
{
  for (auto* corner : getCorners()) {
    if (strcmp(corner->name(), name) == 0) {
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
  return sta->pinSlack(sta_pin, sta_rf, getMinMax(minmax));
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

  std::set<odb::dbMTerm*> outputs;
  for (auto arc_set : lib_cell->timingArcSets(lib_port, /* to */ nullptr)) {
    const sta::TimingRole* role = arc_set->role();
    if (role->isTimingCheck() || role->isAsyncTimingCheck()
        || role->isNonSeqTimingCheck() || role->isDataCheck()) {
      continue;
    }
    sta::LibertyPort* to_port = arc_set->to();
    odb::dbMTerm* to_mterm = master->findMTerm(to_port->name());
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

float Timing::getNetCap(odb::dbNet* net, sta::Corner* corner, MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Net* sta_net = sta->getDbNetwork()->dbToSta(net);

  float pin_cap;
  float wire_cap;
  sta->connectedCap(sta_net, corner, getMinMax(minmax), pin_cap, wire_cap);
  return pin_cap + wire_cap;
}

float Timing::getPortCap(odb::dbITerm* pin, sta::Corner* corner, MinMax minmax)
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

float Timing::staticPower(odb::dbInst* inst, sta::Corner* corner)
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

float Timing::dynamicPower(odb::dbInst* inst, sta::Corner* corner)
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
Timing::checkErcViolations(odb::dbInst* inst, sta::Corner* corner) {
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
        const sta::DcalcAnalysisPt* dcalc_ap = corner->findDcalcAnalysisPt(sta::MinMax::max());
        float actual_slew = sta->graph()->slew(vertex, rf, dcalc_ap->index());
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
      const sta::Corner* corner1 = nullptr;
      float cap1, max_cap1, cap_slack1;
      const sta::RiseFall* rf;
      sta->checkCapacitance(pin, corner, sta::MinMax::max(), corner1, rf, cap1, max_cap1, cap_slack1);
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

float 
Timing::averageDelayOnCritPath() {
  sta::dbSta* sta = getSta();
  return sta->getIncreSta()->averageDelayOnCritPath();
}


////////////////////////////////////////////
// Functions of testing IncreSta
////////////////////////////////////////////
void 
Timing::testBufferInsertion(char *inst_name) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testBufferInsertion(inst_name, sta, resizer, design_->getBlock());
}

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
Timing::testParallelVisitor(const std::vector<odb::dbInst*>& db_insts) {
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelVisitor(db_insts, sta, resizer, design_->getBlock());
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
Timing::testParallelResize() {
  printf("Starting testParallelResize\n");
  printf("First compute all parasitic networks...\n");
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  printf("Starting testParallelResize\n");
  fflush(stdout);
  test_lrf.testParallelResize(sta, resizer, design_->getBlock());
}

void
Timing::testParallelLrResizing(size_t max_resize_num, size_t iterations, 
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelLrResizing with %zu threads\n", thread_num);
  printf("First compute all parasitic networks...\n");
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  printf("Starting testParallelLrResizing\n");
  fflush(stdout);
  test_lrf.testParallelLrResizing(sta, resizer, design_->getBlock(), 
thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons, 
  PT_tradeoff, lr_helper_method);
}

void 
Timing::testTimingComputeAndWriteBack(const std::vector<odb::dbInst*> &insts)
{
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testTimingComputeAndWriteBack(sta, resizer, design_->getBlock(), insts);
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


float Timing::getWorstSlack(sta::MinMax minmax)
{
  sta::dbSta* sta = getSta();
  sta::Vertex* vertex;
  sta::Slack worstSlack;
  sta->worstSlack(getMinMax(minmax), worstSlack, vertex);
  return worstSlack;
}

float Timing::getTns(sta::Corner* corner, MinMax minmax)
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

// Heuristic parser for ASAP7-style names, eg
//   O2A1O1Ixp33_ASAP7_75t_R
//   O2A1O1Ixp33_ASAP7_75t_L
//   O2A1O1Ixp5_ASAP7_75t_SRAM
// We treat everything before the first "_ASAP7_" as the structural prefix.
struct CellNameParts
{
  std::string prefix;  // structure (horizontal axis)
  std::string vt;      // R/L/SL/SRAM/... (vertical axis)
};

CellNameParts parseCellName(const char* name)
{
  CellNameParts parts;
  if (name == nullptr) {
    return parts;
  }
  const std::string s{name};

  // prefix: up to _ASAP7_ if present, else up to last '_' (best-effort)
  const std::string anchor = "_ASAP7_";
  const size_t anchor_pos = s.find(anchor);
  if (anchor_pos != std::string::npos) {
    parts.prefix = s.substr(0, anchor_pos);
  } else {
    const size_t last_us = s.rfind('_');
    parts.prefix = (last_us == std::string::npos) ? s : s.substr(0, last_us);
  }

  // vt: token after last '_'
  const size_t last_us = s.rfind('_');
  if (last_us != std::string::npos && last_us + 1 < s.size()) {
    parts.vt = s.substr(last_us + 1);
  } else {
    parts.vt = "";
  }
  return parts;
}

// A coarse VT ordering (top->bottom). If unknown, keep after known values.
// NOTE: You said index increasing corresponds to VT rising / input cap decreasing.
// We'll compute per-prefix order late using measured input cap, but this provides
// a stable tie-breaker.
int vtRank(const std::string& vt)
{
  if (vt == "SRAM") {
    return 0;
  }
  if (vt == "R") {
    return 1;
  }
  if (vt == "L") {
    return 2;
  }
  if (vt == "SL") {
    return 3;
  }
  // unknown
  return 100;
}

double avgInputCap(const sta::LibertyCell* corner_cell)
{
  if (corner_cell == nullptr) {
    return 0.0;
  }
  double sum = 0.0;
  int cnt = 0;
  sta::LibertyCellPortIterator port_iter(corner_cell);
  while (port_iter.hasNext()) {
    sta::LibertyPort* port = port_iter.next();
    if (!port)
      continue;
    if (port->isPwrGnd() || port->isClock())
      continue;
    if (port->direction() == sta::PortDirection::input()) {
      sum += port->capacitance();
      cnt++;
    }
  }
  return (cnt > 0) ? (sum / cnt) : 0.0;
}

void
Timing::getEquivCellArray(std::vector<std::vector<sta::LibertyCell*>> &array,
                          std::unordered_map<sta::LibertyCell*, std::pair<int, int>> &pos_map)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  utl::Logger* logger = design_->getLogger();

  sta::Corner* corner = sta->cmdCorner();
  const sta::DcalcAnalysisPt* dcalc_ap
      = corner ? corner->findDcalcAnalysisPt(sta::MinMax::max()) : nullptr;
  const int lib_ap = dcalc_ap ? dcalc_ap->libertyIndex() : 0;

  // Iterate all liberty cells and get their equiv group.
  std::set<sta::LibertyCellSeq*> seen_groups;
  sta::LibertyLibraryIterator* lib_iter = network->libertyLibraryIterator();
  while (lib_iter->hasNext()) {
    sta::LibertyLibrary* lib = lib_iter->next();
    sta::LibertyCellIterator cell_iter(lib);
    while (cell_iter.hasNext()) {
      sta::LibertyCell* any_cell = cell_iter.next();
      if (!any_cell) {
        continue;
      }
      sta::LibertyCellSeq* group = sta->equivCells(any_cell);
      if (!group || group->empty() || seen_groups.find(group) != seen_groups.end()) {
        continue;
      }
      seen_groups.insert(group);

      // Build columns by prefix.
      // column_key = prefix; rows determined by VT order (or cap order).
      std::map<std::string, std::vector<sta::LibertyCell*>> cols;

      // Track per-cell metrics to drive sorting.
      std::map<sta::LibertyCell*, double> cell_incap;
      std::map<sta::LibertyCell*, CellNameParts> cell_parts;

      for (sta::LibertyCell* c : *group) {
        if (!c)
          continue;
        const sta::LibertyCell* corner_cell = c->cornerCell(lib_ap);
        const double incap = avgInputCap(corner_cell);
        cell_incap[c] = incap;
        cell_parts[c] = parseCellName(c->name());
        cols[cell_parts[c].prefix].push_back(c);
      }

      // Horizontal axis: sort structural prefixes by smallest input cap inside
      // that prefix ("input cap smaller on the left").
      std::vector<std::string> prefixes;
      prefixes.reserve(cols.size());
      for (const auto& [prefix, _] : cols) {
        prefixes.push_back(prefix);
      }
      std::sort(prefixes.begin(), prefixes.end(), [&](const std::string& a, const std::string& b) {
        auto best_cap = [&](const std::string& p) {
          double best = std::numeric_limits<double>::infinity();
          for (sta::LibertyCell* c : cols[p]) {
            best = std::min(best, cell_incap[c]);
          }
          return best;
        };
        const double ca = best_cap(a);
        const double cb = best_cap(b);
        if (ca != cb)
          return ca < cb;
        return a < b;
      });

      // For each prefix (column), vertical axis: sort by (vtRank, input cap desc)
      // to encourage higher vt (smaller cap) at larger row index.
      // If the naming doesn't capture VT well, the cap tie-breaker still gives a
      // sensible ordering.
      for (auto& [prefix, vec] : cols) {
        std::sort(vec.begin(), vec.end(), [&](sta::LibertyCell* x, sta::LibertyCell* y) {
          const auto& px = cell_parts[x];
          const auto& py = cell_parts[y];
          const int rx = vtRank(px.vt);
          const int ry = vtRank(py.vt);
          // smaller cap should be lower (larger index), so sort cap descending.
          const double cx = cell_incap[x];
          const double cy = cell_incap[y];
          if (rx < ry && cx > cy) {
            logger->report("  [Warning: unexpected ranking] {} vs {}: vtRank wins ({} vs {})",
                           x->name(), y->name(), rx, ry);
          } else if (rx > ry && cx < cy) {
            logger->report("  [Warning: unexpected ranking] {} vs {}: cap wins ({} vs {})",
                           x->name(), y->name(), cx, cy);

          }
          if (rx != ry)
            return rx < ry;
          if (cx != cy)
            return cx > cy;
          return std::string(x->name()) < std::string(y->name());
        });
      }

      // Build a rectangular matrix [rows][cols]. Missing entries are nullptr.
      size_t max_rows = 0;
      for (const auto& p : prefixes) {
        max_rows = std::max(max_rows, cols[p].size());
      }
      std::vector<std::vector<sta::LibertyCell*>> matrix(
          max_rows, std::vector<sta::LibertyCell*>(prefixes.size(), nullptr));

      // Fill matrix and build per-cell position.
      std::unordered_map<sta::LibertyCell*, std::pair<int, int>> pos_map;
      for (size_t col = 0; col < prefixes.size(); col++) {
        const auto& prefix = prefixes[col];
        const auto& vec = cols[prefix];
        for (size_t row = 0; row < vec.size(); row++) {
          matrix[row][col] = vec[row];
          pos_map[vec[row]] = {static_cast<int>(row), static_cast<int>(col)};
        }
      }

      // Append this group's matrix into the global output as block rows.
      // This keeps a single return value without inventing a complex nested dict.
      // If you need per-group separation, we can add group boundaries later.
      const int row_offset = static_cast<int>(array.size());
      array.insert(array.end(), matrix.begin(), matrix.end());
      for (const auto& [cell, rc] : pos_map) {
        // Keep first occurrence if duplicates (shouldn't happen if equiv groups disjoint).
        if (pos_map.find(cell) == pos_map.end()) {
          pos_map[cell]
              = {rc.first + row_offset, rc.second};
        }
      }

      // Log one group matrix.
      logger->report("EquivCellArrayGroup: {} ({} cells, {} cols, {} rows)",
                     group->front() ? group->front()->name() : "<null>",
                     group->size(),
                     prefixes.size(),
                     max_rows);
      // Print header row (prefixes)
      std::string header = "  col:";
      for (size_t c = 0; c < prefixes.size(); c++) {
        header += (c == 0 ? " " : " | ");
        header += prefixes[c];
      }
      logger->report("{}", header);
      for (size_t r = 0; r < max_rows; r++) {
        std::string line = fmt::format("  row{:>2d}:", static_cast<int>(r));
        for (size_t c = 0; c < prefixes.size(); c++) {
          line += (c == 0 ? " " : " | ");
          if (matrix[r][c]) {
            const double cap = cell_incap[matrix[r][c]];
            line += fmt::format("{}({:.3g})", matrix[r][c]->name(), cap);
          } else {
            line += "<null>";
          }
        }
        logger->report("{}", line);
      }
      logger->report("  (pos mapping size = {})", pos_map.size());
      logger->report("  --");
    }
  }
  delete lib_iter;
}

}  // namespace ord

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
#include <utility>
#include <vector>

#include "grt/GRoute.h"
#include "grt/GlobalRouter.h"
#include "grt/Rudy.h"
#include "sta/Network.hh"
#include "sta/Parasitics.hh"
#include "sta/ParasiticsClass.hh"
#include "sta/PortDirection.hh"

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
#include "lrf/LrConfig.hh"
#include "lrf/IncreSta.hh"
#include "lrf/TestLrf.hh"

// Forward decls of free functions defined in lrf::ViolationCheck.cc.
// (Avoids pulling LocalSta.hh / Timing.cc circular include surface.)
namespace lrf {
size_t loadMlCapAugmentRatioCsv(odb::dbBlock* block, const std::string& path);
void   clearMlCapAugmentRatio();
size_t mlCapAugmentRatioSize();
size_t loadMlSlewAugmentRatioCsv(odb::dbBlock* block, const std::string& path);
void   clearMlSlewAugmentRatio();
size_t mlSlewAugmentRatioSize();
}

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
              bool initialize, const char *checkpoint_dir,
              bool debug, size_t buffering_start_iter,
              float timing_margin) {
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
  cfg.initialize = initialize;
  cfg.checkpoint_dir = checkpoint_dir ? checkpoint_dir : "";
  cfg.debug = debug;
  cfg.buffering_start_iter = buffering_start_iter;
  cfg.timing_margin = timing_margin;
  runLr(cfg);
}

void
Timing::testParallelInitializer(bool minimize_leakage) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelInitializer(sta, resizer, design_->getBlock(),
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
  const char *lr_helper_method, bool initialize, float density_weight) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArray with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArray(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, initialize, density_weight);
}

void
Timing::testParallelResizeByArrayWithBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, bool initialize, float density_weight) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, initialize, density_weight);
}

void
Timing::testParallelResizeByArrayWithRszBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, bool initialize, float density_weight) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testParallelResizeByArrayWithRszBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrResizeByArrayWithRszBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, initialize, density_weight);
}

void
Timing::testParallelResizeByArrayWithSdpBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, bool initialize, float density_weight,
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
    PT_tradeoff, lr_helper_method, initialize, density_weight,
    /*debug=*/false, /*buffering_start_iter=*/5, timing_margin);
}

void
Timing::testInitResizeThenSdpBuffering(size_t max_resize_num, size_t iterations,
  size_t num_no_improve_tolerance, bool ratcons, float PT_tradeoff,
  const char *lr_helper_method, bool initialize, float density_weight,
  float timing_margin, float erc_violation_weight) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testInitResizeThenSdpBuffering with %zu threads, "
         "timing_margin=%.4f, erc_violation_weight=%.3g\n",
         thread_num, timing_margin, erc_violation_weight);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testInitResizeThenSdpBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, initialize, density_weight,
    /*debug=*/false, /*buffering_start_iter=*/5, timing_margin,
    erc_violation_weight);
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
  const char *lr_helper_method, bool initialize) {
  size_t thread_num = ord::OpenRoad::openRoad()->getThreadCount();
  printf("Starting testCombinedResizeBuffering with %zu threads\n", thread_num);
  fflush(stdout);
  design_->updateParasiticsNoDeleteNetwork();
  rsz::Resizer* resizer = design_->getResizer();
  sta::dbSta* sta = getSta();
  lrf::TestLrf test_lrf;
  test_lrf.testParallelLrCombinedResizeBuffering(sta, resizer, design_->getBlock(),
    thread_num, max_resize_num, iterations, num_no_improve_tolerance, ratcons,
    PT_tradeoff, lr_helper_method, initialize);
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


float Timing::getWorstSlack(MinMax minmax)
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

////////////////////////////////////////////////////////////////
// dumpDiagBundle — ML / diagnostic feature dump.
//
// Caller is expected to have already done:
//   global_route -guide_file ... (or grt.globalRoute(true))
//   estimate_parasitics -global_routing
// Then a single call writes four CSVs at <prefix>_{nets,segments,sinks,
// congestion}.csv. All numeric values are emitted in SI base units (s, F,
// Ohm) plus dbu for coordinates; downstream analysis can rescale freely.
// Missing values (e.g. parasitic not found) are written as the literal
// "nan" so pandas read_csv treats them as NaN by default.
////////////////////////////////////////////////////////////////
namespace {

inline void writeFloat(std::ostream& os, float v)
{
  if (std::isfinite(v)) {
    os << v;
  } else {
    os << "nan";
  }
}

}  // namespace

void Timing::dumpDiagBundle(const std::string& prefix)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  odb::dbBlock* block = network->block();
  if (block == nullptr) {
    return;
  }

  sta::Parasitics* parasitics = sta->parasitics();
  sta::Graph* graph = sta->graph();
  sta::Corner* corner = sta->corners()->findCorner(0);
  const sta::ParasiticAnalysisPt* ap
      = corner->findParasiticAnalysisPt(sta::MinMax::max());
  const sta::MinMax* mm_max = sta::MinMax::max();

  grt::GlobalRouter* grouter = OpenRoad::openRoad()->getGlobalRouter();
  // Reference (not copy) — getRoutes() returns NetRouteMap&.
  const grt::NetRouteMap* routes
      = grouter ? &grouter->getRoutes() : nullptr;

  std::ofstream nets_csv(prefix + "_nets.csv");
  std::ofstream segs_csv(prefix + "_segments.csv");
  std::ofstream sinks_csv(prefix + "_sinks.csv");
  std::ofstream cong_csv(prefix + "_congestion.csv");

  nets_csv
      << "net_name,sig_type,driver_inst,driver_master,fanout,bterm_count,"
         "hpwl_dbu,num_segments,"
         "pi_c2_rise_F,pi_rpi_rise_Ohm,pi_c1_rise_F,"
         "pi_c2_fall_F,pi_rpi_fall_Ohm,pi_c1_fall_F,"
         "drvr_slew_rise_s,drvr_slew_fall_s,"
         "net_cap_F,drvr_max_cap_F\n";
  segs_csv << "net_name,seg_idx,init_x_dbu,init_y_dbu,init_layer,"
              "final_x_dbu,final_y_dbu,final_layer,length_dbu,is_via\n";
  sinks_csv << "net_name,sink_pin_name,sink_inst,sink_master,"
               "sink_input_cap_F,sink_max_slew_s,"
               "sink_slew_rise_s,sink_slew_fall_s,"
               "wire_delay_rise_s,wire_delay_fall_s\n";
  cong_csv << "layer_name,gx,gy,capacity,usage\n";

  const float kNaN = std::numeric_limits<float>::quiet_NaN();

  // Helper to translate dbNet sig type to a short string.
  auto sigTypeName = [](odb::dbSigType t) -> const char* {
    switch (t.getValue()) {
      case odb::dbSigType::SIGNAL: return "SIGNAL";
      case odb::dbSigType::POWER:  return "POWER";
      case odb::dbSigType::GROUND: return "GROUND";
      case odb::dbSigType::CLOCK:  return "CLOCK";
      case odb::dbSigType::ANALOG: return "ANALOG";
      case odb::dbSigType::RESET:  return "RESET";
      case odb::dbSigType::SCAN:   return "SCAN";
      case odb::dbSigType::TIEOFF: return "TIEOFF";
      default: return "OTHER";
    }
  };

  size_t n_nets = 0, n_segs = 0, n_sinks = 0;

  for (odb::dbNet* db_net : block->getNets()) {
    if (db_net->isSpecial()) {
      continue;
    }

    sta::Net* sta_net = network->dbToSta(db_net);

    // Pass 1: find driver, count fanout, compute HPWL bbox.
    odb::dbITerm* drvr_iterm = nullptr;
    int fanout = 0;
    int bterm_count = 0;
    int xmin = std::numeric_limits<int>::max();
    int xmax = std::numeric_limits<int>::min();
    int ymin = std::numeric_limits<int>::max();
    int ymax = std::numeric_limits<int>::min();
    bool has_xy = false;

    for (odb::dbITerm* it : db_net->getITerms()) {
      if (it->isOutputSignal()) {
        if (drvr_iterm == nullptr) {
          drvr_iterm = it;
        }
      } else {
        ++fanout;
      }
      int x = 0, y = 0;
      if (it->getAvgXY(&x, &y)) {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
        has_xy = true;
      }
    }
    for (odb::dbBTerm* bt : db_net->getBTerms()) {
      ++bterm_count;
      int x = 0, y = 0;
      if (bt->getFirstPinLocation(x, y)) {
        xmin = std::min(xmin, x); xmax = std::max(xmax, x);
        ymin = std::min(ymin, y); ymax = std::max(ymax, y);
        has_xy = true;
      }
    }

    int hpwl_dbu = has_xy ? ((xmax - xmin) + (ymax - ymin)) : 0;

    // Driver-derived data.
    sta::Pin* drvr_pin = nullptr;
    odb::dbInst* drvr_inst = nullptr;
    odb::dbMaster* drvr_master = nullptr;
    odb::dbMTerm* drvr_mterm = nullptr;
    if (drvr_iterm != nullptr) {
      drvr_pin = network->dbToSta(drvr_iterm);
      drvr_inst = drvr_iterm->getInst();
      drvr_master = drvr_inst->getMaster();
      drvr_mterm = drvr_iterm->getMTerm();
    }

    float c2_rise = kNaN, rpi_rise = kNaN, c1_rise = kNaN;
    float c2_fall = kNaN, rpi_fall = kNaN, c1_fall = kNaN;
    sta::Parasitic* pi_rise = nullptr;
    sta::Parasitic* pi_fall = nullptr;
    if (drvr_pin != nullptr && ap != nullptr) {
      pi_rise = parasitics->findPiElmore(
          drvr_pin, sta::RiseFall::rise(), ap);
      pi_fall = parasitics->findPiElmore(
          drvr_pin, sta::RiseFall::fall(), ap);
      if (pi_rise && parasitics->isPiModel(pi_rise)) {
        parasitics->piModel(pi_rise, c2_rise, rpi_rise, c1_rise);
      }
      if (pi_fall && parasitics->isPiModel(pi_fall)) {
        parasitics->piModel(pi_fall, c2_fall, rpi_fall, c1_fall);
      }
    }

    float drvr_slew_rise = kNaN, drvr_slew_fall = kNaN;
    if (drvr_pin != nullptr) {
      sta::Vertex* drvr_v = graph->pinDrvrVertex(drvr_pin);
      if (drvr_v != nullptr) {
        drvr_slew_rise = sta->vertexSlew(
            drvr_v, sta::RiseFall::rise(), mm_max);
        drvr_slew_fall = sta->vertexSlew(
            drvr_v, sta::RiseFall::fall(), mm_max);
      }
    }

    float drvr_max_cap = kNaN;
    if (drvr_mterm != nullptr) {
      drvr_max_cap = getMaxCapLimit(drvr_mterm);
    }

    float net_cap = kNaN;
    if (ap != nullptr) {
      net_cap = getNetCap(db_net, corner, MinMax::Max);
    }

    int num_segments = 0;
    const grt::GRoute* groute = nullptr;
    if (routes != nullptr) {
      auto it = routes->find(db_net);
      if (it != routes->end()) {
        groute = &it->second;
        num_segments = static_cast<int>(groute->size());
      }
    }

    // Write nets row.
    nets_csv << db_net->getName() << ','
             << sigTypeName(db_net->getSigType()) << ','
             << (drvr_inst ? drvr_inst->getName() : std::string("")) << ','
             << (drvr_master ? drvr_master->getName() : std::string("")) << ','
             << fanout << ','
             << bterm_count << ','
             << hpwl_dbu << ','
             << num_segments << ',';
    writeFloat(nets_csv, c2_rise);  nets_csv << ',';
    writeFloat(nets_csv, rpi_rise); nets_csv << ',';
    writeFloat(nets_csv, c1_rise);  nets_csv << ',';
    writeFloat(nets_csv, c2_fall);  nets_csv << ',';
    writeFloat(nets_csv, rpi_fall); nets_csv << ',';
    writeFloat(nets_csv, c1_fall);  nets_csv << ',';
    writeFloat(nets_csv, drvr_slew_rise); nets_csv << ',';
    writeFloat(nets_csv, drvr_slew_fall); nets_csv << ',';
    writeFloat(nets_csv, net_cap);  nets_csv << ',';
    writeFloat(nets_csv, drvr_max_cap); nets_csv << '\n';
    ++n_nets;

    // Write segments rows.
    if (groute != nullptr) {
      for (size_t i = 0; i < groute->size(); ++i) {
        const grt::GSegment& seg = (*groute)[i];
        segs_csv << db_net->getName() << ',' << i << ','
                 << seg.init_x << ',' << seg.init_y << ',' << seg.init_layer
                 << ',' << seg.final_x << ',' << seg.final_y << ','
                 << seg.final_layer << ',' << seg.length() << ','
                 << (seg.isVia() ? 1 : 0) << '\n';
        ++n_segs;
      }
    }

    // Write sinks rows.
    if (drvr_pin != nullptr) {
      for (odb::dbITerm* sink_it : db_net->getITerms()) {
        if (sink_it == drvr_iterm) {
          continue;
        }
        sta::Pin* sink_pin = network->dbToSta(sink_it);

        // getPortCap takes corner+min_max (ITerm pin only).
        float sink_input_cap = getPortCap(sink_it, corner, MinMax::Max);
        float sink_max_slew = kNaN;
        odb::dbMTerm* sink_mt = sink_it->getMTerm();
        if (sink_mt != nullptr) {
          sink_max_slew = getMaxSlewLimit(sink_mt);
        }

        float sink_slew_rise = kNaN, sink_slew_fall = kNaN;
        if (sink_pin != nullptr) {
          sta::Vertex* sink_v = graph->pinLoadVertex(sink_pin);
          if (sink_v != nullptr) {
            sink_slew_rise = sta->vertexSlew(
                sink_v, sta::RiseFall::rise(), mm_max);
            sink_slew_fall = sta->vertexSlew(
                sink_v, sta::RiseFall::fall(), mm_max);
          }
        }

        float wd_rise = kNaN, wd_fall = kNaN;
        if (pi_rise != nullptr && sink_pin != nullptr) {
          float v; bool exists = false;
          parasitics->findElmore(pi_rise, sink_pin, v, exists);
          if (exists) wd_rise = v;
        }
        if (pi_fall != nullptr && sink_pin != nullptr) {
          float v; bool exists = false;
          parasitics->findElmore(pi_fall, sink_pin, v, exists);
          if (exists) wd_fall = v;
        }

        sinks_csv << db_net->getName() << ','
                  << sink_it->getName() << ','
                  << sink_it->getInst()->getName() << ','
                  << sink_it->getInst()->getMaster()->getName() << ',';
        writeFloat(sinks_csv, sink_input_cap); sinks_csv << ',';
        writeFloat(sinks_csv, sink_max_slew);  sinks_csv << ',';
        writeFloat(sinks_csv, sink_slew_rise); sinks_csv << ',';
        writeFloat(sinks_csv, sink_slew_fall); sinks_csv << ',';
        writeFloat(sinks_csv, wd_rise); sinks_csv << ',';
        writeFloat(sinks_csv, wd_fall); sinks_csv << '\n';
        ++n_sinks;
      }
    }
  }

  // Congestion CSV: per (layer, gcell_x, gcell_y).
  odb::dbGCellGrid* grid = block->getGCellGrid();
  if (grid != nullptr) {
    odb::dbTech* tech = block->getDataBase()->getTech();
    for (odb::dbTechLayer* layer : tech->getLayers()) {
      if (layer->getType() != odb::dbTechLayerType::ROUTING) {
        continue;
      }
      odb::dbMatrix<odb::dbGCellGrid::GCellData> mat
          = grid->getLayerCongestionMap(layer);
      const int rows = mat.numRows();
      const int cols = mat.numCols();
      for (int x = 0; x < rows; ++x) {
        for (int y = 0; y < cols; ++y) {
          const auto& d = mat(x, y);
          cong_csv << layer->getName() << ',' << x << ',' << y << ','
                   << d.capacity << ',' << d.usage << '\n';
        }
      }
    }
  }

  utl::Logger* logger = OpenRoad::openRoad()->getLogger();
  if (logger) {
    logger->report(
        "[dumpDiagBundle] wrote {}_nets.csv ({} rows), "
        "{}_segments.csv ({} rows), {}_sinks.csv ({} rows), "
        "{}_congestion.csv",
        prefix, n_nets, prefix, n_segs, prefix, n_sinks, prefix);
  }
}

////////////////////////////////////////////////////////////////
// dumpFeatureBundle — per-net feature CSV for ML cap_ratio prediction.
//
// Caller controls parasitic state (estimate_parasitics -placement OR
// -global_routing) before each call. Static features (geometry, layers,
// macro distance, RUDY) are placement-independent — emitted both times for
// easy join. Dynamic features (cap, drvr_slew, max_sink_slew, viol flags)
// reflect the current parasitic state.
//
// Skip rules:
//   - special / POWER / GROUND / CLOCK nets
//   - multi-driver nets (n_drvr != 1)
//   - fanout > max_fanout (these get GRT-skipped via -skip_large_fanout_nets)
//   - net names matching pre-CTS clock pattern (defensive: clk/clock/rst/
//     reset/scan even if sigType==SIGNAL)
////////////////////////////////////////////////////////////////
namespace {

// Defensive pre-CTS clock/reset/scan detector — net names like
// `clk_i`, `nvdla_core_clk`, `bit_clk_pad_i`, `rst_i`, `scan_*`.
inline bool looksLikeClockOrReset(const std::string& name)
{
  // Lowercase scan; stop at first hit.
  auto find_ci = [&](const char* needle) -> bool {
    const size_t nl = std::strlen(needle);
    for (size_t i = 0; i + nl <= name.size(); ++i) {
      bool ok = true;
      for (size_t k = 0; k < nl; ++k) {
        char c = name[i + k];
        if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
        if (c != needle[k]) { ok = false; break; }
      }
      if (!ok) continue;
      // Boundary check: previous char and next char should be non-alnum
      // (so "clkgate_in" doesn't match "clk").
      char prev = (i == 0) ? '/' : name[i - 1];
      char next = (i + nl >= name.size()) ? '/' : name[i + nl];
      auto is_word = [](char c) {
        return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
               || (c >= '0' && c <= '9') || c == '_';
      };
      // We require the BEFORE boundary to be non-word (so the keyword starts
      // a token), but allow the AFTER to be anything (digits/underscore/end).
      if (!is_word(prev)) return true;
    }
    return false;
  };
  return find_ci("clk") || find_ci("clock") || find_ci("rst")
      || find_ci("reset") || find_ci("scan");
}

// Routing levels touched by an iterm's pin geometry (1-based, M1=1).
inline std::set<int> itermPinLayers(odb::dbITerm* it)
{
  std::set<int> ls;
  odb::dbMTerm* mt = it->getMTerm();
  if (mt == nullptr) return ls;
  for (odb::dbMPin* mp : mt->getMPins()) {
    for (odb::dbBox* box : mp->getGeometry()) {
      odb::dbTechLayer* tl = box->getTechLayer();
      if (tl == nullptr) continue;
      int lvl = tl->getRoutingLevel();
      if (lvl > 0) ls.insert(lvl);
    }
  }
  return ls;
}

}  // namespace

void Timing::dumpFeatureBundle(const std::string& prefix, int max_fanout)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  odb::dbBlock* block = network->block();
  if (block == nullptr) return;

  sta::Parasitics* parasitics = sta->parasitics();
  sta::Graph* graph = sta->graph();
  sta::Corner* corner = sta->corners()->findCorner(0);
  const sta::ParasiticAnalysisPt* ap
      = corner ? corner->findParasiticAnalysisPt(sta::MinMax::max()) : nullptr;
  const sta::MinMax* mm_max = sta::MinMax::max();

  // ── Pre-compute design-level state (cheap; once per call) ──

  // (a) macro rectangles
  std::vector<odb::Rect> macros;
  for (odb::dbInst* inst : block->getInsts()) {
    if (!inst->isPlaced()) continue;
    odb::dbMaster* m = inst->getMaster();
    if (m == nullptr || !m->getType().isBlock()) continue;
    odb::dbBox* bb = inst->getBBox();
    macros.emplace_back(bb->xMin(), bb->yMin(), bb->xMax(), bb->yMax());
  }

  // (b) RUDY — uses GRT internal implementation. Constructor will
  //     auto-initFastRoute if GRT not yet initialized. Robust to "no GRT
  //     run yet" — perfect for placement-stage feature extraction.
  grt::GlobalRouter* grouter = OpenRoad::openRoad()->getGlobalRouter();
  grt::Rudy* rudy = nullptr;
  int rudy_tcx = 0, rudy_tcy = 0;
  int rudy_tile_size = 1;
  int rudy_origin_x = 0, rudy_origin_y = 0;
  if (grouter != nullptr) {
    try {
      rudy = grouter->getRudy();
      if (rudy != nullptr) {
        rudy->calculateRudy();
        auto gs = rudy->getGridSize();
        rudy_tcx = gs.first;
        rudy_tcy = gs.second;
        rudy_tile_size = std::max(rudy->getTileSize(), 1);
        odb::Rect die = block->getDieArea();
        rudy_origin_x = die.xMin();
        rudy_origin_y = die.yMin();
      }
    } catch (...) {
      rudy = nullptr;
    }
  }

  auto rudyTileIdx = [&](int x, int y, int& tx, int& ty) -> bool {
    if (rudy == nullptr) return false;
    tx = (x - rudy_origin_x) / rudy_tile_size;
    ty = (y - rudy_origin_y) / rudy_tile_size;
    if (tx < 0) tx = 0; if (tx >= rudy_tcx) tx = rudy_tcx - 1;
    if (ty < 0) ty = 0; if (ty >= rudy_tcy) ty = rudy_tcy - 1;
    return true;
  };

  // ── (c) Post-GRT routes + per-gcell congestion ──
  // If GRT has been run (caller invoked global_route already), per-net routes
  // are populated and dbGCellGrid has capacity/usage. If not, all per-route
  // features below are emitted as NaN/0.
  const grt::NetRouteMap* routes = (grouter != nullptr)
                                       ? &grouter->getRoutes() : nullptr;
  odb::dbGCellGrid* gcell_grid = block->getGCellGrid();

  // Cache routing-level → tech layer pointer (M1=1, M2=2, ...).
  std::vector<odb::dbTechLayer*> layer_by_level;
  layer_by_level.push_back(nullptr);  // index 0 unused
  if (gcell_grid != nullptr) {
    odb::dbTech* tech = block->getDataBase()->getTech();
    for (odb::dbTechLayer* tl : tech->getLayers()) {
      if (tl->getType() != odb::dbTechLayerType::ROUTING) continue;
      int lvl = tl->getRoutingLevel();
      while ((int)layer_by_level.size() <= lvl)
        layer_by_level.push_back(nullptr);
      layer_by_level[lvl] = tl;
    }
  }

  std::ofstream csv(prefix + "_features.csv");
  csv << "net,fanout,hpwl_dbu,bbox_w_dbu,bbox_h_dbu,bbox_aspect_ratio,"
         "driver_pin_layer,min_sink_pin_layer,max_sink_pin_layer,"
         "layer_span_count,"
         "macro_dist_dbu,bbox_overlaps_macro,"
         "rudy_avg,rudy_max,rudy_at_drvr,"
         "route_length_dbu,route_detour_ratio,via_count,route_max_layer,"
         "route_overflow_sum,route_overflow_max,route_congestion_sum,"
         "cap_F,drvr_max_cap_F,"
         "pi_c2_rise_F,pi_rpi_rise_Ohm,pi_c1_rise_F,"
         "pi_c2_fall_F,pi_rpi_fall_Ohm,pi_c1_fall_F,"
         "drvr_slew_rise_s,drvr_slew_fall_s,"
         "max_sink_slew_rise_s,max_sink_slew_fall_s,"
         "slew_viol,cap_viol\n";

  size_t n_emit = 0;

  for (odb::dbNet* db_net : block->getNets()) {
    if (db_net->isSpecial()) continue;
    odb::dbSigType st = db_net->getSigType();
    if (st == odb::dbSigType::POWER || st == odb::dbSigType::GROUND
        || st == odb::dbSigType::CLOCK)
      continue;
    if (looksLikeClockOrReset(db_net->getName())) continue;

    // ── Pass 1: driver, fanout, bbox ──
    odb::dbITerm* drvr = nullptr;
    int n_drvr = 0;
    int n_sinks = 0;
    int xmin = std::numeric_limits<int>::max();
    int xmax = std::numeric_limits<int>::min();
    int ymin = std::numeric_limits<int>::max();
    int ymax = std::numeric_limits<int>::min();
    bool has_xy = false;
    for (odb::dbITerm* it : db_net->getITerms()) {
      if (it->isOutputSignal()) {
        if (drvr == nullptr) drvr = it;
        ++n_drvr;
      } else {
        ++n_sinks;
      }
      int x = 0, y = 0;
      if (it->getAvgXY(&x, &y)) {
        if (x < xmin) xmin = x; if (x > xmax) xmax = x;
        if (y < ymin) ymin = y; if (y > ymax) ymax = y;
        has_xy = true;
      }
    }
    int n_bterms = 0;
    for (odb::dbBTerm* bt : db_net->getBTerms()) {
      ++n_bterms;
      int x = 0, y = 0;
      if (bt->getFirstPinLocation(x, y)) {
        if (x < xmin) xmin = x; if (x > xmax) xmax = x;
        if (y < ymin) ymin = y; if (y > ymax) ymax = y;
        has_xy = true;
      }
    }

    if (n_drvr != 1 || drvr == nullptr || !has_xy) continue;
    int fanout = n_sinks + n_bterms;
    if (fanout < 1 || fanout > max_fanout) continue;

    // ── Geometry ──
    int bbox_w = xmax - xmin;
    int bbox_h = ymax - ymin;
    int hpwl = bbox_w + bbox_h;
    double aspect = 1000.0;  // clip per plan §Open Q 2
    int bw_min = std::min(bbox_w, bbox_h);
    int bw_max = std::max(bbox_w, bbox_h);
    if (bw_min > 0) aspect = double(bw_max) / double(bw_min);

    // ── Pin layers ──
    auto drvr_layers = itermPinLayers(drvr);
    int driver_pin_layer = drvr_layers.empty() ? 0 : *drvr_layers.begin();
    std::set<int> sink_layers;
    for (odb::dbITerm* it : db_net->getITerms()) {
      if (it == drvr) continue;
      auto sl = itermPinLayers(it);
      sink_layers.insert(sl.begin(), sl.end());
    }
    int min_sink_layer = sink_layers.empty() ? 0 : *sink_layers.begin();
    int max_sink_layer = sink_layers.empty() ? 0 : *sink_layers.rbegin();
    std::set<int> all_layers = drvr_layers;
    all_layers.insert(sink_layers.begin(), sink_layers.end());
    int layer_span = static_cast<int>(all_layers.size());

    // ── Macro distance (Manhattan, dbu) ──
    int macro_dist = -1;
    int overlaps = 0;
    for (const odb::Rect& m : macros) {
      int dx = std::max({m.xMin() - xmax, xmin - m.xMax(), 0});
      int dy = std::max({m.yMin() - ymax, ymin - m.yMax(), 0});
      int d = dx + dy;
      if (d == 0) { overlaps = 1; macro_dist = 0; break; }
      if (macro_dist < 0 || d < macro_dist) macro_dist = d;
    }

    // ── RUDY ──
    float rudy_avg = 0.0f, rudy_max = 0.0f, rudy_at_drvr = 0.0f;
    if (rudy != nullptr) {
      int tx0, ty0, tx1, ty1;
      rudyTileIdx(xmin, ymin, tx0, ty0);
      rudyTileIdx(xmax, ymax, tx1, ty1);
      if (tx1 < tx0) std::swap(tx0, tx1);
      if (ty1 < ty0) std::swap(ty0, ty1);
      double sum = 0.0;
      int cnt = 0;
      for (int tx = tx0; tx <= tx1; ++tx) {
        for (int ty = ty0; ty <= ty1; ++ty) {
          float v = rudy->getTile(tx, ty).getRudy();
          sum += v;
          if (v > rudy_max) rudy_max = v;
          ++cnt;
        }
      }
      rudy_avg = (cnt > 0) ? static_cast<float>(sum / cnt) : 0.0f;

      int dx_pin = 0, dy_pin = 0, dtx, dty;
      if (drvr->getAvgXY(&dx_pin, &dy_pin)
          && rudyTileIdx(dx_pin, dy_pin, dtx, dty)) {
        rudy_at_drvr = rudy->getTile(dtx, dty).getRudy();
      }
    }

    // ── Post-GRT route features (NaN if GRT not yet run) ──
    float route_length_dbu = std::numeric_limits<float>::quiet_NaN();
    float route_detour_ratio = std::numeric_limits<float>::quiet_NaN();
    int via_count = 0;
    int route_max_layer = 0;
    float route_overflow_sum = std::numeric_limits<float>::quiet_NaN();
    float route_overflow_max = std::numeric_limits<float>::quiet_NaN();
    float route_congestion_sum = std::numeric_limits<float>::quiet_NaN();
    if (routes != nullptr) {
      auto rit = routes->find(db_net);
      if (rit != routes->end() && !rit->second.empty()) {
        const grt::GRoute& groute = rit->second;
        long long len_sum = 0;
        double of_sum = 0.0, of_max = 0.0, cong_sum = 0.0;
        bool have_cong = (gcell_grid != nullptr);
        for (const grt::GSegment& seg : groute) {
          if (seg.isVia()) {
            ++via_count;
          } else {
            len_sum += static_cast<long long>(seg.length());
          }
          int max_lvl_seg = std::max(seg.init_layer, seg.final_layer);
          if (max_lvl_seg > route_max_layer) route_max_layer = max_lvl_seg;

          if (!have_cong) continue;
          // Walk gcells the segment traverses; for via, single tile both
          // layers; for wire on one layer, walk x/y range.
          int lvl_lo = std::min(seg.init_layer, seg.final_layer);
          int lvl_hi = std::max(seg.init_layer, seg.final_layer);
          int x_lo = std::min(seg.init_x, seg.final_x);
          int x_hi = std::max(seg.init_x, seg.final_x);
          int y_lo = std::min(seg.init_y, seg.final_y);
          int y_hi = std::max(seg.init_y, seg.final_y);
          uint32_t gx_lo = gcell_grid->getXIdx(x_lo);
          uint32_t gx_hi = gcell_grid->getXIdx(x_hi);
          uint32_t gy_lo = gcell_grid->getYIdx(y_lo);
          uint32_t gy_hi = gcell_grid->getYIdx(y_hi);
          for (int lvl = lvl_lo; lvl <= lvl_hi; ++lvl) {
            if (lvl <= 0 || lvl >= (int)layer_by_level.size()) continue;
            odb::dbTechLayer* tl = layer_by_level[lvl];
            if (tl == nullptr) continue;
            for (uint32_t gx = gx_lo; gx <= gx_hi; ++gx) {
              for (uint32_t gy = gy_lo; gy <= gy_hi; ++gy) {
                float cap   = gcell_grid->getCapacity(tl, gx, gy);
                float usage = gcell_grid->getUsage(tl, gx, gy);
                float of    = std::max(0.0f, usage - cap);
                of_sum += of;
                if (of > of_max) of_max = of;
                if (cap > 0.0f) cong_sum += static_cast<double>(usage) / cap;
              }
            }
          }
        }
        route_length_dbu = static_cast<float>(len_sum);
        if (hpwl > 0)
          route_detour_ratio = static_cast<float>(len_sum) / float(hpwl);
        if (have_cong) {
          route_overflow_sum   = static_cast<float>(of_sum);
          route_overflow_max   = static_cast<float>(of_max);
          route_congestion_sum = static_cast<float>(cong_sum);
        }
      } else {
        // GRT object exists but no route for this net (skipped or pre-route).
        route_length_dbu = 0.0f;
        route_detour_ratio = 0.0f;
        route_overflow_sum = 0.0f;
        route_overflow_max = 0.0f;
        route_congestion_sum = 0.0f;
      }
    }

    // ── Electrical (depends on currently-active parasitic state) ──
    sta::Pin* drvr_pin = network->dbToSta(drvr);
    odb::dbMTerm* drvr_mterm = drvr->getMTerm();

    float cap_F = std::numeric_limits<float>::quiet_NaN();
    cap_F = getNetCap(db_net, corner, MinMax::Max);

    float drvr_max_cap = std::numeric_limits<float>::quiet_NaN();
    if (drvr_mterm != nullptr) {
      drvr_max_cap = getMaxCapLimit(drvr_mterm);
    }

    // π model: per RiseFall, decompose into (C2 near-source, Rpi, C1 far).
    // Captures R/C distribution that cap_F alone loses — needed to predict
    // sink delay/slew, not just net cap.
    float c2_rise = std::numeric_limits<float>::quiet_NaN();
    float rpi_rise = std::numeric_limits<float>::quiet_NaN();
    float c1_rise = std::numeric_limits<float>::quiet_NaN();
    float c2_fall = std::numeric_limits<float>::quiet_NaN();
    float rpi_fall = std::numeric_limits<float>::quiet_NaN();
    float c1_fall = std::numeric_limits<float>::quiet_NaN();
    if (drvr_pin != nullptr && ap != nullptr && parasitics != nullptr) {
      sta::Parasitic* pi_r
          = parasitics->findPiElmore(drvr_pin, sta::RiseFall::rise(), ap);
      sta::Parasitic* pi_f
          = parasitics->findPiElmore(drvr_pin, sta::RiseFall::fall(), ap);
      if (pi_r && parasitics->isPiModel(pi_r)) {
        parasitics->piModel(pi_r, c2_rise, rpi_rise, c1_rise);
      }
      if (pi_f && parasitics->isPiModel(pi_f)) {
        parasitics->piModel(pi_f, c2_fall, rpi_fall, c1_fall);
      }
    }

    float drvr_slew_rise = std::numeric_limits<float>::quiet_NaN();
    float drvr_slew_fall = std::numeric_limits<float>::quiet_NaN();
    if (drvr_pin != nullptr) {
      sta::Vertex* dv = graph->pinDrvrVertex(drvr_pin);
      if (dv != nullptr) {
        drvr_slew_rise = sta->vertexSlew(dv, sta::RiseFall::rise(), mm_max);
        drvr_slew_fall = sta->vertexSlew(dv, sta::RiseFall::fall(), mm_max);
      }
    }

    // Sink slews + slew violation across all signal sinks.
    float max_sink_slew_rise = drvr_slew_rise;
    float max_sink_slew_fall = drvr_slew_fall;
    int slew_viol = 0;
    for (odb::dbITerm* it : db_net->getITerms()) {
      if (it == drvr || it->isOutputSignal()) continue;
      sta::Pin* sink_pin = network->dbToSta(it);
      odb::dbMTerm* sink_mt = it->getMTerm();
      float sink_limit = (sink_mt != nullptr) ? getMaxSlewLimit(sink_mt)
                                              : std::numeric_limits<float>::quiet_NaN();
      if (sink_pin != nullptr) {
        sta::Vertex* sv = graph->pinLoadVertex(sink_pin);
        if (sv != nullptr) {
          float sr = sta->vertexSlew(sv, sta::RiseFall::rise(), mm_max);
          float sf = sta->vertexSlew(sv, sta::RiseFall::fall(), mm_max);
          if (std::isfinite(sr)) {
            if (sr > max_sink_slew_rise) max_sink_slew_rise = sr;
            if (std::isfinite(sink_limit) && sr > sink_limit) slew_viol = 1;
          }
          if (std::isfinite(sf)) {
            if (sf > max_sink_slew_fall) max_sink_slew_fall = sf;
            if (std::isfinite(sink_limit) && sf > sink_limit) slew_viol = 1;
          }
        }
      }
    }

    int cap_viol = (std::isfinite(drvr_max_cap) && std::isfinite(cap_F)
                    && cap_F > drvr_max_cap) ? 1 : 0;

    // ── Emit row ──
    csv << db_net->getName() << ',' << fanout << ','
        << hpwl << ',' << bbox_w << ',' << bbox_h << ',';
    csv << aspect << ',';
    csv << driver_pin_layer << ',' << min_sink_layer << ','
        << max_sink_layer << ',' << layer_span << ',';
    csv << macro_dist << ',' << overlaps << ',';
    writeFloat(csv, rudy_avg);     csv << ',';
    writeFloat(csv, rudy_max);     csv << ',';
    writeFloat(csv, rudy_at_drvr); csv << ',';
    writeFloat(csv, route_length_dbu);    csv << ',';
    writeFloat(csv, route_detour_ratio);  csv << ',';
    csv << via_count << ',';
    csv << route_max_layer << ',';
    writeFloat(csv, route_overflow_sum);   csv << ',';
    writeFloat(csv, route_overflow_max);   csv << ',';
    writeFloat(csv, route_congestion_sum); csv << ',';
    writeFloat(csv, cap_F);        csv << ',';
    writeFloat(csv, drvr_max_cap); csv << ',';
    writeFloat(csv, c2_rise);  csv << ',';
    writeFloat(csv, rpi_rise); csv << ',';
    writeFloat(csv, c1_rise);  csv << ',';
    writeFloat(csv, c2_fall);  csv << ',';
    writeFloat(csv, rpi_fall); csv << ',';
    writeFloat(csv, c1_fall);  csv << ',';
    writeFloat(csv, drvr_slew_rise); csv << ',';
    writeFloat(csv, drvr_slew_fall); csv << ',';
    writeFloat(csv, max_sink_slew_rise); csv << ',';
    writeFloat(csv, max_sink_slew_fall); csv << ',';
    csv << slew_viol << ',' << cap_viol << '\n';
    ++n_emit;
  }

  utl::Logger* logger = OpenRoad::openRoad()->getLogger();
  if (logger) {
    logger->report("[dumpFeatureBundle] wrote {}_features.csv ({} nets, "
                   "{} macros, RUDY {}x{})",
                   prefix, n_emit, macros.size(), rudy_tcx, rudy_tcy);
  }
}

size_t Timing::loadMlCapAugmentRatio(const std::string& csv_path)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  odb::dbBlock* block = network ? network->block() : nullptr;
  if (block == nullptr) {
    printf("[ML] Timing::loadMlCapAugmentRatio: no block; aborting\n");
    return 0;
  }
  return ::lrf::loadMlCapAugmentRatioCsv(block, csv_path);
}

void Timing::clearMlCapAugmentRatio()
{
  ::lrf::clearMlCapAugmentRatio();
}

size_t Timing::mlCapAugmentRatioSize() const
{
  return ::lrf::mlCapAugmentRatioSize();
}

size_t Timing::loadMlSlewAugmentRatio(const std::string& csv_path)
{
  sta::dbSta* sta = getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  odb::dbBlock* block = network ? network->block() : nullptr;
  if (block == nullptr) {
    printf("[ML] Timing::loadMlSlewAugmentRatio: no block; aborting\n");
    return 0;
  }
  return ::lrf::loadMlSlewAugmentRatioCsv(block, csv_path);
}

void Timing::clearMlSlewAugmentRatio()
{
  ::lrf::clearMlSlewAugmentRatio();
}

size_t Timing::mlSlewAugmentRatioSize() const
{
  return ::lrf::mlSlewAugmentRatioSize();
}

}  // namespace ord

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2022-2025, The OpenROAD Authors

#include "ord/Design.h"

#include <cmath>
#include <cstdint>
#include <istream>
#include <mutex>
#include <ostream>
#include <string>

#include "ant/AntennaChecker.hh"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "grt/GlobalRouter.h"
#include "ifp/InitFloorplan.hh"
#include "odb/db.h"
#include "ord/OpenRoad.hh"
#include "ord/Tech.h"
#include "tcl.h"
#include "utl/Logger.h"

namespace ord {

std::mutex Design::interp_mutex;

Design::Design(Tech* tech) : tech_(tech)
{
}

ord::OpenRoad* Design::getOpenRoad()
{
  return tech_->app_;
}

odb::dbBlock* Design::getBlock()
{
  auto chip = tech_->getDB()->getChip();
  return chip ? chip->getBlock() : nullptr;
}

void Design::readVerilog(const std::string& file_name)
{
  // DB will change; clear any cached instance list
  sorted_instances_.clear();
  auto chip = tech_->getDB()->getChip();
  if (chip && chip->getBlock()) {
    getLogger()->error(utl::ORD, 36, "A block already exists in the db");
  }

  getOpenRoad()->readVerilog(file_name.c_str());
}

void Design::readDef(const std::string& file_name,
                     bool continue_on_errors,  // = false
                     bool floorplan_init,      // = false
                     bool incremental          // = false
)
{
  // DB will change; clear any cached instance list
  sorted_instances_.clear();
  if (floorplan_init && incremental) {
    getLogger()->error(utl::ORD,
                       101,
                       "Only one of the options -incremental and"
                       " -floorplan_init can be set at a time");
  }
  if (tech_->getDB()->getTech() == nullptr) {
    getLogger()->error(utl::ORD, 102, "No technology has been read.");
  }
  auto chip = tech_->getDB()->getChip();
  if (chip == nullptr) {
    chip = odb::dbChip::create(tech_->getDB(), tech_->getDB()->getTech());
  }
  getOpenRoad()->readDef(
      file_name.c_str(), chip, continue_on_errors, floorplan_init, incremental);
}

void Design::link(const std::string& design_name)
{
  // linking may change the working db/design; clear cache
  sorted_instances_.clear();
  getOpenRoad()->linkDesign(design_name.c_str(), false);
}

void Design::readDb(std::istream& stream)
{
  // DB will change; clear any cached instance list
  sorted_instances_.clear();
  getOpenRoad()->readDb(stream);
}

void Design::readDb(const std::string& file_name)
{
  // DB will change; clear any cached instance list
  sorted_instances_.clear();
  getOpenRoad()->readDb(file_name.c_str());
}

void Design::writeDb(std::ostream& stream)
{
  getOpenRoad()->writeDb(stream);
}

void Design::writeDb(const std::string& file_name)
{
  getOpenRoad()->writeDb(file_name.c_str());
}

void Design::writeDef(const std::string& file_name)
{
  getOpenRoad()->writeDef(file_name.c_str(), "5.8");
}

ifp::InitFloorplan Design::getFloorplan()
{
  auto block = getBlock();
  if (!block) {
    getLogger()->error(utl::ORD, 37, "No block loaded.");
  }
  return ifp::InitFloorplan(block, getLogger(), getSta()->getDbNetwork());
}

utl::Logger* Design::getLogger()
{
  return getOpenRoad()->getLogger();
}

int Design::micronToDBU(double coord)
{
  const int dbu_per_micron = getBlock()->getDbUnitsPerMicron();
  return round(coord * dbu_per_micron);
}

ant::AntennaChecker* Design::getAntennaChecker()
{
  return getOpenRoad()->getAntennaChecker();
}

std::string Design::evalTclString(const std::string& cmd)
{
  const std::lock_guard<std::mutex> lock(interp_mutex);
  auto openroad = getOpenRoad();
  ord::OpenRoad::setOpenRoad(openroad, /* reinit_ok */ true);
  Tcl_Interp* tcl_interp = openroad->tclInterp();
  sta::Sta::setSta(openroad->getSta());
  Tcl_Eval(tcl_interp, cmd.c_str());
  return std::string(Tcl_GetStringResult(tcl_interp));
}

Tech* Design::getTech()
{
  return tech_;
}

sta::dbSta* Design::getSta()
{
  return tech_->getSta();
}

sta::LibertyCell* Design::getLibertyCell(odb::dbMaster* master)
{
  sta::dbNetwork* network = getSta()->getDbNetwork();

  sta::Cell* cell = network->dbToSta(master);
  if (!cell) {
    return nullptr;
  }
  return network->libertyCell(cell);
}

bool Design::isBuffer(odb::dbMaster* master)
{
  auto lib_cell = getLibertyCell(master);
  if (!lib_cell) {
    return false;
  }
  return lib_cell->isBuffer();
}

bool Design::isInverter(odb::dbMaster* master)
{
  auto lib_cell = getLibertyCell(master);
  if (!lib_cell) {
    return false;
  }
  return lib_cell->isInverter();
}

bool Design::isSequential(odb::dbMaster* master)
{
  auto lib_cell = getLibertyCell(master);
  if (!lib_cell) {
    return false;
  }
  return lib_cell->hasSequentials();
}

bool Design::isInClock(odb::dbInst* inst)
{
  for (auto* iterm : inst->getITerms()) {
    auto* net = iterm->getNet();
    if (net != nullptr && net->getSigType() == odb::dbSigType::CLOCK) {
      return true;
    }
  }
  return false;
}

bool Design::isInClock(odb::dbITerm* iterm)
{
  auto* net = iterm->getNet();
  if (net != nullptr && net->getSigType() == odb::dbSigType::CLOCK) {
    return true;
  }
  return false;
}

std::string Design::getITermName(odb::dbITerm* pin)
{
  return pin->getName();
}

bool Design::isInSupply(odb::dbITerm* pin)
{
  return pin->getSigType().isSupply();
}

std::uint64_t Design::getNetRoutedLength(odb::dbNet* net)
{
  std::uint64_t route_length = 0;
  if (net->getSigType().isSupply()) {
    for (odb::dbSWire* swire : net->getSWires()) {
      for (odb::dbSBox* wire : swire->getWires()) {
        if (wire != nullptr && !(wire->isVia())) {
          route_length += wire->getLength();
        }
      }
    }
  } else {
    auto* wire = net->getWire();
    if (wire != nullptr) {
      route_length += wire->getLength();
    }
  }
  return route_length;
}

grt::GlobalRouter* Design::getGlobalRouter()
{
  return getOpenRoad()->getGlobalRouter();
}

gpl::Replace* Design::getReplace()
{
  return getOpenRoad()->getReplace();
}

dpl::Opendp* Design::getOpendp()
{
  return getOpenRoad()->getOpendp();
}

exa::Example* Design::getExample()
{
  return getOpenRoad()->getExample();
}

mpl::MacroPlacer* Design::getMacroPlacer()
{
  return getOpenRoad()->getMacroPlacer();
}

ppl::IOPlacer* Design::getIOPlacer()
{
  return getOpenRoad()->getIOPlacer();
}

tap::Tapcell* Design::getTapcell()
{
  return getOpenRoad()->getTapcell();
}

cgt::ClockGating* Design::getClockGating()
{
  return getOpenRoad()->getClockGating();
}

cts::TritonCTS* Design::getTritonCts()
{
  return getOpenRoad()->getTritonCts();
}

drt::TritonRoute* Design::getTritonRoute()
{
  return getOpenRoad()->getTritonRoute();
}

fin::Finale* Design::getFinale()
{
  return getOpenRoad()->getFinale();
}

par::PartitionMgr* Design::getPartitionMgr()
{
  return getOpenRoad()->getPartitionMgr();
}

rcx::Ext* Design::getOpenRCX()
{
  return getOpenRoad()->getOpenRCX();
}

rmp::Restructure* Design::getRestructure()
{
  return getOpenRoad()->getRestructure();
}

stt::SteinerTreeBuilder* Design::getSteinerTreeBuilder()
{
  return getOpenRoad()->getSteinerTreeBuilder();
}

psm::PDNSim* Design::getPDNSim()
{
  return getOpenRoad()->getPDNSim();
}

pdn::PdnGen* Design::getPdnGen()
{
  return getOpenRoad()->getPdnGen();
}

pad::ICeWall* Design::getICeWall()
{
  return getOpenRoad()->getICeWall();
}

dft::Dft* Design::getDft()
{
  return getOpenRoad()->getDft();
}

odb::dbDatabase* Design::getDb()
{
  return getOpenRoad()->getDb();
}

rsz::Resizer* Design::getResizer()
{
  return getOpenRoad()->getResizer();
}

/* static */
odb::dbDatabase* Design::createDetachedDb()
{
  auto app = OpenRoad::openRoad();
  auto db = odb::dbDatabase::create();
  db->setLogger(app->getLogger());
  return db;
}

/////////////////////////////////////////////////////////////
// Functions for LR sizing
/////////////////////////////////////////////////////////////
std::vector<odb::dbInst*> Design::sortedInstances()
{
  // Note: returns a Python-friendly container via SWIG std_vector wrapper
  // or a typemap; see Design.i for vector exposure.
  printf("Design::sortedInstances called\n");
  fflush(stdout);
  sta::dbSta* sta = getSta();
  sta->searchPreamble();
  sta::dbNetwork* network = sta->getDbNetwork();

  sta::InstanceSeq &sorted_instances = sta->getSortedInstances();
  std::vector<odb::dbInst*> instances;
  instances.reserve(sorted_instances.size());
  for (auto* inst : sorted_instances) {
    odb::dbInst* db_inst = network->staToDb(inst);
    if (db_inst == nullptr) {
      // STA instance has no DB mapping; skip safely
      printf("Warning: staToDb returned nullptr for an instance %s\n",
             network->name(inst));
      continue;
    }
    odb::dbMaster* master = db_inst->getMaster();
    if (master) {
      instances.push_back(db_inst);
    }
  }
  // Cache the result so subsequent calls don't recompute unless the
  // database/design changes (see places that clear the cache above).
  sorted_instances_ = instances;
  return sorted_instances_;
}


/////////////////////////////////////////////////////////////
// End functions for LR sizing
/////////////////////////////////////////////////////////////

}  // namespace ord

// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors
#include <fcntl.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <vector>

#include "Strategy.hh"
#include "aig/gia/gia.h"
#include "aig/gia/giaAig.h"
#include "base/abc/abc.h"
#include "base/main/main.h"
#include "cut/abc_library_factory.h"
#include "cut/logic_cut.h"
#include "cut/logic_extractor.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "map/if/if.h"
#include "map/scl/sclSize.h"
#include "misc/vec/vecPtr.h"
#include "odb/db.h"
#include "proof/dch/dch.h"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/MinMax.hh"
#include "sta/PortDirection.hh"
#include "sta/Search.hh"
#include "utils.h"
#include "utl/Logger.h"
#include "utl/SuppressStdout.h"
#include "utl/deleter.h"
#include "utl/unique_name.h"
#include "gpl/Replace.h"
#include "dpl/Opendp.h"

#include "rmp/SeqRemapper.hh"

namespace abc {
extern Abc_Ntk_t* Abc_NtkFromAigPhase(Aig_Man_t* pMan);
extern Abc_Ntk_t* Abc_NtkFromCellMappedGia(Gia_Man_t* p, int fUseBuffs);
extern Abc_Ntk_t* Abc_NtkFromDarChoices(Abc_Ntk_t* pNtkOld, Aig_Man_t* pMan);
extern Abc_Ntk_t* Abc_NtkFromMappedGia(Gia_Man_t* p,
                                       int fFindEnables,
                                       int fUseBuffs);
extern Abc_Ntk_t* Abc_NtkMap(Abc_Ntk_t* pNtk,
                             Mio_Library_t* userLib,
                             double DelayTarget,
                             double AreaMulti,
                             double DelayMulti,
                             float LogFan,
                             float Slew,
                             float Gain,
                             int nGatesMin,
                             int fRecovery,
                             int fSwitching,
                             int fSkipFanout,
                             int fUseProfile,
                             int fUseBuffs,
                             int fVerbose);
extern Aig_Man_t* Abc_NtkToDar(Abc_Ntk_t* pNtk, int fExors, int fRegisters);
extern Aig_Man_t* Abc_NtkToDarChoices(Abc_Ntk_t* pNtk);
extern Gia_Man_t* Gia_ManAigSynch2(Gia_Man_t* p,
                                   void* pPars,
                                   int nLutSize,
                                   int nRelaxRatio);
extern Gia_Man_t* Gia_ManCheckFalse(Gia_Man_t* p,
                                    int nSlackMax,
                                    int nTimeOut,
                                    int fVerbose,
                                    int fVeryVerbose);
extern Vec_Ptr_t* Abc_NtkCollectCiNames(Abc_Ntk_t* pNtk);
extern Vec_Ptr_t* Abc_NtkCollectCoNames(Abc_Ntk_t* pNtk);
extern void Abc_NtkRedirectCiCo(Abc_Ntk_t* pNtk);
}  // namespace abc

namespace rmp {

// Definition of helper declared in SeqRemapper.hh.
// Keeps name vectors when replacing the gia pointer.
void replaceGia(abc::Gia_Man_t*& gia, abc::Gia_Man_t* new_gia)
{
  if (new_gia == nullptr) {
    printf("Warning: replaceGia called with nullptr new_gia\n");
    fflush(stdout);
    return;
  }
  if (gia == nullptr) {
    printf("Warning: replaceGia called with nullptr gia\n");
    fflush(stdout);
    gia = new_gia;
    return;
  }
  if (gia == new_gia) {
    return;
  }
  if (gia->vNamesIn && !new_gia->vNamesIn) {
    std::swap(gia->vNamesIn, new_gia->vNamesIn);
  }
  if (gia->vNamesOut && !new_gia->vNamesOut) {
    std::swap(gia->vNamesOut, new_gia->vNamesOut);
  }
  if (gia->vNamesNode && !new_gia->vNamesNode) {
    std::swap(gia->vNamesNode, new_gia->vNamesNode);
  }
  abc::Gia_ManStop(gia);
  gia = new_gia;
}

void 
MappingResult::show(utl::Logger* logger)
{
  // Use existing RMP tool category instead of undefined RES.
  logger->info(utl::RES, 305, "Mapping Result: WNS = {:.3f}, TNS = {:.3f}, Area = {:.3f}, Power = {:.3f}",
               wns * 1e12, tns * 1e12, area, power);
}

SeqRemapper::SeqRemapper(sta::dbSta* sta, odb::dbDatabase* db, 
                         sta::Corner* corner, rsz::Resizer* resizer, 
                         utl::Logger* logger, gpl::Replace* gpl,
                         dpl::Opendp* dpl, est::EstimateParasitics* est)
    : db_(db),  
      corner_(corner),
      resizer_(resizer),
      logger_(logger),
      gpl_(gpl),
      dpl_(dpl),
      est_(est),
      opt_operator_factory_(new GiaOptOperator())
{
  dbStaState::init(sta);
  buildAbcLibrary(); 
  block_ = db_->getChip()->getBlock();
  if (est_ == nullptr) {
    logger_->error(utl::RES, 311, "EstimateParasitics is null in SeqRemapper");
  }
  checkTracksAndRows();
}

SeqRemapper::~SeqRemapper()
{
  delete abc_library_;
  delete opt_operator_factory_;
  sta_->unregisterStaState(this);
}

void
SeqRemapper::remapPreamble()
{
  sta_->ensureGraph();
  sta_->ensureLevelized();
  sta_->searchPreamble();
  sta_->ensureClkNetwork();
}

void
SeqRemapper::buildAbcLibrary()
{
  cut::AbcLibraryFactory factory(logger_);
  factory.AddDbSta(sta_);
  factory.AddResizer(resizer_);
  factory.SetCorner(corner_);
  abc_library_ = new cut::AbcLibrary(factory.Build());
}

cut::LogicCut
SeqRemapper::extractBottleneck(Strategy &strategy)
{
  // Assume 
  //sta::dbNetwork* network = sta_->getDbNetwork();
  //sta::Instance* ref_gate = network->findInstance("g218487");  // 示例实例名，可替换为实际需要的名称
  //if (ref_gate == nullptr) {
    //logger_->error(
        //utl::RES, 318, "Reference gate 'ref_gate' not found in the design.");
  //}
  //strategy->setRefGate(ref_gate);
  sta_->ensureGraph();
  sta_->ensureLevelized();
  return strategy.extractBottleneck(*this);
}

utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>
SeqRemapper::cutToAig(cut::LogicCut& logic_cut)
{
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_ntk =
      logic_cut.BuildMappedAbcNetwork(*abc_library_, sta_->getDbNetwork(), logger_);

  // Get the library before converting (it's stored in mapped_abc_ntk->pManFunc)
  auto library
        = static_cast<abc::Mio_Library_t*>(mapped_abc_ntk->pManFunc);
  // Install library for NtkMap (needed for unmap and later remap)
  abc::Abc_FrameSetLibGen(library);

  // Convert to Logic network - this preserves ABC_FUNC_MAP
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> aig_abc_ntk = WrapUnique(
      abc::Abc_NtkToLogic(const_cast<abc::Abc_Ntk_t*>(mapped_abc_ntk.get())));

  // Unmap the network: convert from ABC_FUNC_MAP to ABC_FUNC_SOP
  // This is necessary because Abc_NtkStrash expects SOP/BDD/AIG, not MAP
  if (abc::Abc_NtkHasMapping(aig_abc_ntk.get())) {
    printf("DEBUG: Before unmap - ntkFunc=%d (MAP=%d, SOP=%d)\n", 
           aig_abc_ntk->ntkFunc, abc::ABC_FUNC_MAP, abc::ABC_FUNC_SOP);
    printf("DEBUG: pManFunc=%p, FrameLibGen=%p\n", 
           aig_abc_ntk->pManFunc, abc::Abc_FrameReadLibGen());
    fflush(stdout);
    int result = abc::Abc_NtkMapToSop(aig_abc_ntk.get());
    printf("DEBUG: After unmap - result=%d, ntkFunc=%d\n", 
           result, aig_abc_ntk->ntkFunc);
    fflush(stdout);
  }

  printf("Original mapped ABC network info:\n");
  fflush(stdout);
  printNtkInfo(aig_abc_ntk, logger_);
      
  return aig_abc_ntk;
}

abc::Gia_Man_t*
SeqRemapper::aigToGia(
    utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>& strashed_aig)
{
  auto ntk_ptr = strashed_aig.get();
  printf("DEBUG aigToGia: ntkType=%d (LOGIC=%d), ntkFunc=%d (SOP=%d, MAP=%d, AIG=%d)\n",
         ntk_ptr->ntkType, abc::ABC_NTK_LOGIC,
         ntk_ptr->ntkFunc, abc::ABC_FUNC_SOP, abc::ABC_FUNC_MAP, abc::ABC_FUNC_AIG);
  printf("DEBUG aigToGia: IsStrash=%d, HasMapping=%d, HasSop=%d, HasAig=%d\n",
         Abc_NtkIsStrash(ntk_ptr), abc::Abc_NtkHasMapping(ntk_ptr),
         abc::Abc_NtkHasSop(ntk_ptr), abc::Abc_NtkHasAig(ntk_ptr));
  fflush(stdout);
  assert(!Abc_NtkIsStrash(ntk_ptr) && "AIG network is expected to be not strashed.");
  
  // Check network before strashing
  printf("DEBUG: Checking network before Abc_NtkStrash...\n");
  printf("DEBUG: Abc_NtkIsLogic=%d, Abc_NtkCheck=%d\n", 
         abc::Abc_NtkIsLogic(ntk_ptr), abc::Abc_NtkCheck(ntk_ptr));
  fflush(stdout);
  
  // Check if we need to convert to AIG first for SOP networks
  if (abc::Abc_NtkHasSop(ntk_ptr)) {
    printf("DEBUG: Network has SOP, converting to AIG first...\n");
    fflush(stdout);
    if (!abc::Abc_NtkToAig(ntk_ptr)) {
      printf("ERROR: Abc_NtkToAig failed!\n");
      fflush(stdout);
      return nullptr;
    }
    printf("DEBUG: After Abc_NtkToAig - HasAig=%d, HasSop=%d\n",
           abc::Abc_NtkHasAig(ntk_ptr), abc::Abc_NtkHasSop(ntk_ptr));
    fflush(stdout);
  }
  
  printf("DEBUG: Calling Abc_NtkStrash...\n");
  fflush(stdout);
  auto stash  = Abc_NtkStrash(ntk_ptr, false, true, false);
  auto aig = Abc_NtkToDar(stash, false, false);
  Abc_NtkDelete(stash);  // Fix memory leak
  abc::Gia_Man_t* gia = Gia_ManFromAig(aig);
  Aig_ManStop(aig);
  // Perform undc/zero
  auto inits = Abc_NtkCollectLatchValuesStr(ntk_ptr);
  auto temp = gia;
  gia = Gia_ManDupZeroUndc(gia, inits, 0, false, false);
  Gia_ManStop(temp);
  ABC_FREE(inits);
  // copy names
  gia->vNamesIn = abc::Abc_NtkCollectCiNames(ntk_ptr);
  gia->vNamesOut = abc::Abc_NtkCollectCoNames(ntk_ptr);
  return gia;
}

void
SeqRemapper::checkTracksAndRows()
{
  bool has_valid_site = false;
  for (auto* row : block_->getRows()) {
    if (row->getSite()->getClass() != odb::dbSiteClass::PAD) {
      has_valid_site = true;
      break;
    }
  }
  if (!has_valid_site) {
    logger_->warn(utl::RES, 339, "No valid site found for GPL, skipping incremental global placement");
    return;
  }
}
/*
void 
SeqRemapper::setIncrePlaceParam(PlaceMode::Mode mode, float density_penalty,
                                 int place_iter)
{
  if (gpl_ != nullptr) {
    switch (mode) {
      case PlaceMode::ROUTE_DRIVEN:
        gpl_->setRoutabilityDrivenMode(true);
        break;
      case PlaceMode::TIMING_DRIVEN:
        gpl_->setRoutabilityDrivenMode(false);
        break;
    }
    gpl_->setInitDensityPenalityFactor(density_penalty);
    gpl_->setInitialPlaceMaxIter(place_iter);
  }
}
*/
utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>
SeqRemapper::giaToAig(abc::Gia_Man_t* gia)
{
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> aig_ntk;
  abc::Extra_UtilGetoptReset();

  if (Gia_ManHasCellMapping(gia)) {
    aig_ntk = WrapUnique(abc::Abc_NtkFromCellMappedGia(gia, false));
  } else if (Gia_ManHasMapping(gia) || gia->pMuxes) {
    aig_ntk = WrapUnique(Abc_NtkFromMappedGia(gia, false, false));
  } else {
    if (Gia_ManHasDangling(gia) != 0) {
      debugPrint(
          logger_, utl::RES, "annealing", 6, "Rehashing before conversion");
      replaceGia(gia, Gia_ManRehash(gia, false));
    }
    assert(Gia_ManHasDangling(gia) == 0);
    auto aig = Gia_ManToAig(gia, false);
    if (aig == nullptr) {
      logger_->error(utl::RES, 314, "Gia_ManToAig returned null");
    }
    // Debug: print AIG statistics before calling Abc_NtkFromAigPhase
    printNtkInfo(aig, logger_);

    // Check if AIG is valid
    if (abc::Aig_ManObjNum(aig) == 0) {
      logger_->warn(utl::RES, 316, "AIG has 0 objects, network may be empty");
    }
    aig_ntk = WrapUnique(Abc_NtkFromAigPhase(aig));
    if (aig_ntk == nullptr) {
      Aig_ManStop(aig);
      logger_->error(utl::RES, 315, "Abc_NtkFromAigPhase returned null");
    }
    aig_ntk->pName = abc::Extra_UtilStrsav(aig->pName);
    Aig_ManStop(aig);
  }

  assert(gia->vNamesIn);
  for (int i = 0; i < abc::Abc_NtkCiNum(aig_ntk.get()); i++) {
    assert(i < Vec_PtrSize(gia->vNamesIn));
    abc::Abc_Obj_t* obj = abc::Abc_NtkCi(aig_ntk.get(), i);
    assert(obj);
    Nm_ManDeleteIdName(aig_ntk->pManName, obj->Id);
    Abc_ObjAssignName(
        obj, static_cast<char*>(Vec_PtrEntry(gia->vNamesIn, i)), nullptr);
  }
  assert(gia->vNamesOut);
  for (int i = 0; i < abc::Abc_NtkCoNum(aig_ntk.get()); i++) {
    assert(i < Vec_PtrSize(gia->vNamesOut));
    abc::Abc_Obj_t* obj = Abc_NtkCo(aig_ntk.get(), i);
    assert(obj);
    Nm_ManDeleteIdName(aig_ntk->pManName, obj->Id);
    assert(Abc_ObjIsPo(obj));
    Abc_ObjAssignName(
        obj, static_cast<char*>(Vec_PtrEntry(gia->vNamesOut, i)), nullptr);
  }

  // decouple CI/CO with the same name
  if (!Abc_NtkIsStrash(aig_ntk.get())
      && (gia->vNamesIn || gia->vNamesOut)) {
    abc::Abc_NtkRedirectCiCo(aig_ntk.get());
  }
  Gia_ManStop(gia);

  if (!Abc_NtkIsStrash(aig_ntk.get())) {
      aig_ntk = WrapUnique(
          abc::Abc_NtkStrash(aig_ntk.get(), false, true, false));
  }

  return aig_ntk;
}

utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>
SeqRemapper::performMapping(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &aig_ntk)
{
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> temp;
  if (!Abc_NtkIsStrash(aig_ntk.get())) {
    aig_ntk = WrapUnique(
        abc::Abc_NtkStrash(aig_ntk.get(), false, true, false));
  }
  {
    utl::SuppressStdout nostdout(logger_);
    temp = WrapUnique(abc::Abc_NtkMap(aig_ntk.get(),
                                      nullptr,
                                      /*DelayTarget=*/1.0,
                                      /*AreaMulti=*/0.0,
                                      /*DelayMulti=*/2.5,
                                      /*LogFan=*/0.0,
                                      /*Slew=*/0.0,
                                      /*Gain=*/250.0,
                                      /*nGatesMin=*/0,
                                      /*fRecovery=*/true,
                                      /*fSwitching=*/false,
                                      /*fSkipFanout=*/false,
                                      /*fUseProfile=*/false,
                                      /*fUseBuffs=*/false,
                                      /*fVerbose=*/false));
  }
  
  abc::Abc_NtkCleanup(temp.get(), /*fVerbose=*/false);

  temp = WrapUnique(abc::Abc_NtkDupDfs(temp.get()));

  if (iters_ > 0) {
    // All the magic numbers are defaults from abc/src/base/abci/abc.c
    utl::SuppressStdout nostdout(logger_);
    abc::SC_SizePars pars = {};
    pars.nIters = iters_;
    pars.nIterNoChange = 50;
    pars.Window = 1;
    pars.Ratio = 10;
    pars.Notches = 1000;
    pars.DelayUser = 0;
    pars.DelayGap = 0;
    pars.TimeOut = 0;
    pars.BuffTreeEst = 0;
    pars.BypassFreq = 0;
    pars.fUseDept = true;
    abc::Abc_SclUpsizePerform(
        abc_library_->abc_library(), temp.get(), &pars, nullptr);
    abc::Abc_SclDnsizePerform(
        abc_library_->abc_library(), temp.get(), &pars, nullptr);
  }

  temp = WrapUnique(abc::Abc_NtkToNetlist(temp.get()));
  return temp;
}

void
SeqRemapper::insertMappedAbcNetwork(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &mapped_ntk, cut::LogicCut& logic_cut)
{
  logic_cut.InsertMappedAbcNetwork(mapped_ntk.get(), *abc_library_, 
                            sta_->getDbNetwork(), sta_, name_generator_,
                            logger_);
}

void SeqRemapper::performIncrePlace(cut::LogicCut& logic_cut, gpl::Replace *gpl, dpl::Opendp* dpl)
{
  if (gpl == nullptr) {
    logger_->warn(utl::RES, 336, "GPL is nullptr, cannot perform incremental global placement");
    return;
  }
  
  size_t thread_count = 4;
  gpl::PlaceOptions options;
  
  // Call the existing incremental placement implementation
  gpl->doIncrementalPlace(thread_count, options);
  
  logger_->info(utl::RES, 312, "Incremental global placement completed");
}

void SeqRemapper::performPlace(cut::LogicCut& logic_cut, gpl::Replace *gpl, dpl::Opendp* dpl)
{
  if (gpl == nullptr) {
    logger_->warn(utl::RES, 348, "GPL is nullptr, cannot perform global placement");
    return;
  }

  size_t thread_count = 4;
  gpl::PlaceOptions options;

  // Call the existing incremental placement implementation
  gpl->doPlace(thread_count, options);

  logger_->info(utl::RES, 347, "Global placement completed");
}

void SeqRemapper::performIncreDpl(cut::LogicCut& logic_cut, dpl::Opendp* dpl)
{
  if (dpl == nullptr) {
    logger_->warn(utl::RES, 337, "DPL is nullptr, cannot perform incremental detailed placement");
    return;
  }

  sta::dbNetwork* network = sta_->getDbNetwork();

  // Step 1: Compute a centroid seed position from the boundary pins of the cut.
  //   - primary_input nets: the driver pin (output direction) is an external instance.
  //   - primary_output nets: the load pins (input direction) are external instances.
  int sum_x = 0, sum_y = 0, count = 0;

  auto accumulate_pin_location = [&](sta::Net* net, bool want_driver) {
    sta::NetPinIterator* pin_iter = network->pinIterator(net);
    while (pin_iter->hasNext()) {
      const sta::Pin* pin = pin_iter->next();
      sta::PortDirection* dir = network->direction(pin);
      if (dir == nullptr) {
        continue;
      }
      bool is_driver = dir->isAnyOutput();
      if (is_driver != want_driver) {
        continue;
      }
      if (network->isTopLevelPort(pin)) {
        continue;
      }
      sta::Instance* sta_inst = network->instance(pin);
      if (sta_inst == nullptr) {
        continue;
      }
      odb::dbInst* db_inst = network->staToDb(sta_inst);
      if (db_inst == nullptr) {
        continue;
      }
      int x, y;
      db_inst->getLocation(x, y);
      sum_x += x;
      sum_y += y;
      ++count;
    }
    delete pin_iter;
  };

  for (sta::Net* net : logic_cut.primary_inputs()) {
    accumulate_pin_location(net, /*want_driver=*/true);
  }
  for (sta::Net* net : logic_cut.primary_outputs()) {
    accumulate_pin_location(net, /*want_driver=*/false);
  }

  odb::Point centroid(0, 0);
  if (count > 0) {
    centroid = odb::Point(sum_x / count, sum_y / count);
  } else if (block_ != nullptr) {
    odb::Rect core = block_->getCoreArea();
    centroid = odb::Point(core.xMin(), core.yMin());
  }

  // Step 2: Seed each new cut instance at the centroid and mark it PLACED
  //         so DPL can legally snap it to a valid row/site.
  for (const sta::Instance* sta_inst : logic_cut.cut_instances()) {
    odb::dbInst* db_inst = network->staToDb(sta_inst);
    if (db_inst == nullptr) {
      continue;
    }
    db_inst->setLocation(centroid.x(), centroid.y());
    db_inst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  }

  // Step 3: Legalize each new cut instance in place — snaps to the nearest
  //         legal row/site and resolves overlaps locally, like rsz does after
  //         cell insertion.
  for (const sta::Instance* sta_inst : logic_cut.cut_instances()) {
    odb::dbInst* db_inst = network->staToDb(sta_inst);
    if (db_inst == nullptr) {
      continue;
    }
    dpl->legalCellPos(db_inst);
  }

  logger_->info(utl::RES, 338, "Incremental detailed placement completed");
}

/*
void 
SeqRemapper::performIncrePlace(cut::LogicCut& logic_cut, gpl::Replace *gpl, dpl::Opendp* dpl)
{ 
  // Proceed with placement without the guard for now to avoid EST-0104
  size_t cut_size = logic_cut.cut_instances().size();
  // To test incremental placement, first set it to 5
 
  if (cut_size > 1) {
    performIncreGpl(logic_cut, gpl);
  } else {
    performIncreDpl(logic_cut, dpl);
  }
}

void 
SeqRemapper::performIncreGpl(cut::LogicCut& logic_cut, gpl::Replace *gpl)
{
  if (gpl == nullptr) {
    logger_->warn(utl::RES, 344, "GPL is nullptr, cannot perform incremental global placement");
    return;
  }
  
  size_t thread_count = 4;
  auto db_insts = block_->getInsts();

  setIncrePlaceParam(PlaceMode::ROUTE_DRIVEN, 0.5, 10);
  gpl->doNesterovPlace(thread_count);
  logger_->info(utl::RES, 318, "Incremental global placement completed");
}

void
SeqRemapper::performIncreDpl(cut::LogicCut& logic_cut, dpl::Opendp* dpl)
{
  if (dpl == nullptr) {
    logger_->warn(utl::RES, 337, "DPL is nullptr, cannot perform incremental detailed placement");
    return;
  }
  // TODO: develop incremental DPL placement in two steps:
  // 1. Assign a location to these new cells
  // 2. Legalize the placement with dpl
  // 3. Optionally perform dpl optimization
  std::unordered_map<const sta::Instance*, odb::Point> inst_location_map;
  for (auto* sta_inst : logic_cut.cut_instances()) {
    auto* db_inst = sta_->getDbNetwork()->staToDb(sta_inst);
    if (db_inst) {
      odb::Point &loc = inst_location_map[sta_inst];
      db_inst->setLocation(loc.x(), loc.y());
    }
  }
  // Legalize placement with dpl
  for (auto* sta_inst : logic_cut.cut_instances()) {
    auto* db_inst = sta_->getDbNetwork()->staToDb(sta_inst);
    if (db_inst) {
      dpl->legalCellPos(db_inst);
    }
  }
  logger_->info(utl::RES, 338, "Incremental detailed placement completed");
}

void 
SeqRemapper::runOpt() {
  remapPreamble();

  ExtractLocalWindow extrac_strategy(logger_);
  cut::LogicCut logic_cut = extractBottleneck(&extrac_strategy);
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> strashed_aig = 
                                        cutToAig(logic_cut);
  printNtkInfo(strashed_aig, logger_);
  for (size_t i = 0; i < 10; ++i)
    TryOptWithAig(strashed_aig, NtkType::GIA, i, logic_cut);
}

void 
SeqRemapper::TryOptWithAig(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>& aig_ntk,
                           NtkType::Type ntk_type, size_t action, 
                           cut::LogicCut& logic_cut)
{
  if (ntk_type == NtkType::GIA) {
    std::vector<GiaOp> ops = GiaOptOperator::getAllOperations(logger_);
    auto gia = aigToGia(aig_ntk);
    for (size_t i = 0; i < action && i < ops.size(); ++i) {
      ops[i](gia);
      logger_->info(utl::RES, 313, "Applied GiaOp {}", i);
    }
    aig_ntk = giaToAig(gia);
    evaluateTemporary(aig_ntk, logic_cut);
  } else if (ntk_type == NtkType::AIG) {
    printf("AIG optimization not implemented yet.\n");
    fflush(stdout);
  }
}

MappingResult
SeqRemapper::evaluateTemporary(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &aig_ntk, cut::LogicCut& logic_cut) 
{
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_aig = performMapping(aig_ntk);
    // Temporarily insert to get metrics
  insertMappedAbcNetwork(mapped_aig, logic_cut);
  
  performIncrePlace(logic_cut, gpl_, dpl_);
  performTimingRepair(logic_cut);
  MappingResult result;
  getMetrics(logic_cut, result);
  result.show(logger_);
  return result;
}

MappingResult
SeqRemapper::applyBestResult(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &aig_ntk, cut::LogicCut& logic_cut) 
{
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_aig = performMapping(aig_ntk);
  {
    // Permanently insert the best result
    insertMappedAbcNetwork(mapped_aig, logic_cut);
    
    // Update STA graph after instance deletion/insertion
    sta_->ensureGraph();
    sta_->ensureLevelized();
    
    performIncrePlace(logic_cut, gpl_, dpl_);
  }
  performTimingRepair(logic_cut);
  MappingResult result;
  getMetrics(logic_cut, result);
  result.show(logger_);
  return result;
}

MappingResult
SeqRemapper::evaluate(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &aig_ntk, cut::LogicCut& logic_cut) 
{
  return evaluateTemporary(aig_ntk, logic_cut);
}
*/
void 
SeqRemapper::performTimingRepair(cut::LogicCut& logic_cut)
{
  // TODO: implement timing repair after remapping and placement
  // 1. We should keep tracking the instance change in resizer?
}

void 
SeqRemapper::getMetrics(cut::LogicCut& logic_cut, MappingResult& result)
{
  result.wns = sta_->worstSlack(sta::MinMax::max());
  result.tns = sta_->totalNegativeSlack(sta::MinMax::max());
  // TODO: get area and power metrics
}


}  // namespace rmp

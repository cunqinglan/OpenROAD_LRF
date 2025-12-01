// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include "rmp/SeqRemapper.hh"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <vector>

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
#include "sta/Search.hh"
#include "utils.h"
#include "utl/Logger.h"
#include "utl/SuppressStdout.h"
#include "utl/deleter.h"
#include "utl/unique_name.h"

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

/////////////////////////////////////////////////////////////////
// Operator base implementation
/////////////////////////////////////////////////////////////////
void 
Operator::runOptOperator(abc::Gia_Man_t*& gia, int action, utl::Logger* logger)
{
  if (action >= action_count_) {
    logger->info(utl::RES, 301, "Action {} is out of range [0, {})", action, action_count_);
  }
  if (action < 0) {
    logger->error(utl::RES, 302, "Action {} is negative", action);
  }
}

void 
Operator::runOptOperator(abc::Abc_Ntk_t*& ntk, int action, utl::Logger* logger)
{
  if (action >= action_count_) {
    logger->info(utl::RES, 303, "Action {} is out of range [0, {})", action, action_count_);
  }
  if (action < 0) {
    logger->error(utl::RES, 304, "Action {} is negative", action);
  }
}

/////////////////////////////////////////////////////////////////
// GiaOptOperator implementation
/////////////////////////////////////////////////////////////////

GiaOp GiaOptOperator::getRehashOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &st
    logger->info(utl::RES, 317, "Starting rehashing");
    replaceGia(gia, Gia_ManRehash(gia, false));
  };
}

GiaOp GiaOptOperator::getDchOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &dch
    if (!gia->pReprs) {
      logger->info(utl::RES,
                   227,
                   "Computing choices before equiv reduce");
      abc::Dch_Pars_t pars = {};
      Dch_ManSetDefaultParams(&pars);
      replaceGia(gia, Gia_ManPerformDch(gia, &pars));
    }
    debugPrint(logger, utl::RES, "gia_ops", 1, "Starting equiv reduce");
    replaceGia(gia, Gia_ManEquivReduce(gia, true, false, false, false));
  };
}

GiaOp GiaOptOperator::getSyn2Op(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &syn2
    logger->info(utl::RES, 319, "Starting syn2");
    replaceGia(gia,
               Gia_ManAigSyn2(gia, false, true, 0, 20, 0, false, false));
  };
}

GiaOp GiaOptOperator::getSyn3Op(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &syn3
    logger->info(utl::RES, 320, "Starting syn3");
    replaceGia(gia, Gia_ManAigSyn3(gia, false, false));
  };
}

GiaOp GiaOptOperator::getSyn4Op(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &syn4
    logger->info(utl::RES, 321, "Starting syn4");
    replaceGia(gia, Gia_ManAigSyn4(gia, false, false));
  };
}

GiaOp GiaOptOperator::getRetimeOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &retime
    logger->info(utl::RES, 322, "Starting retime");
    replaceGia(gia, Gia_ManRetimeForward(gia, 100, false));
  };
}

GiaOp GiaOptOperator::getCompress2Op(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &dc2
    logger->info(utl::RES, 323, "Starting heavy rewriting");
    replaceGia(gia, Gia_ManCompress2(gia, true, false));
  };
}

GiaOp GiaOptOperator::getAreaBalanceOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &b
    logger->info(utl::RES, 324, "Starting &b");
    replaceGia(
        gia, Gia_ManAreaBalance(gia, false, ABC_INFINITY, false, false));
  };
}

GiaOp GiaOptOperator::getBalanceOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &b -d
    logger->info(utl::RES, 325, "Starting &b -d");
    replaceGia(gia, Gia_ManBalance(gia, false, false, false));
  };
}

GiaOp GiaOptOperator::getFalsePathOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &false
    logger->info(utl::RES, 326, "Starting false path elimination");
    utl::SuppressStdout nostdout(logger);
    replaceGia(gia, Gia_ManCheckFalse(gia, 0, 0, false, false));
  };
}

GiaOp GiaOptOperator::getEquivReduceOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &reduce
    if (!gia->pReprs) {
      debugPrint(logger,
                 utl::RES,
                 "gia_ops",
                 1,
                 "Computing choices before equiv reduce");
      abc::Dch_Pars_t pars = {};
      Dch_ManSetDefaultParams(&pars);
      replaceGia(gia, Gia_ManPerformDch(gia, &pars));
    }
    logger->info(utl::RES, 327, "Starting equiv reduce and remap");
    replaceGia(gia, Gia_ManEquivReduceAndRemap(gia, true, false));
  };
}

GiaOp GiaOptOperator::getSopBalancingOp(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &if -g -K 6
    if (Gia_ManHasMapping(gia)) {
      debugPrint(logger,
                 utl::RES,
                 "gia_ops",
                 1,
                 "GIA has mapping - rehashing before mapping");
      replaceGia(gia, Gia_ManRehash(gia, false));
    }
    abc::If_Par_t pars = {};
    Gia_ManSetIfParsDefault(&pars);
    pars.fDelayOpt = true;
    pars.nLutSize = 6;
    pars.fTruth = true;
    pars.fCutMin = true;
    pars.fExpRed = false;
    logger->info(utl::RES, 328, "Starting SOP balancing");
    replaceGia(gia, Gia_ManPerformMapping(gia, &pars));
  };
}

GiaOp GiaOptOperator::getSynch2Op(utl::Logger* logger)
{
  return [logger](auto& gia) {
    // &synch2
    abc::Dch_Pars_t pars = {};
    Dch_ManSetDefaultParams(&pars);
    pars.nBTLimit = 100;
    logger->info(utl::RES, 329, "Starting synch2");
    replaceGia(gia, Gia_ManAigSynch2(gia, &pars, 6, 20));
  };
}

std::vector<GiaOp> GiaOptOperator::getAllOperations(utl::Logger* logger)
{
  return {
      getRehashOp(logger),
      getDchOp(logger),
      getSyn2Op(logger),
      getSyn3Op(logger),
      getSyn4Op(logger),
      getRetimeOp(logger),
      getCompress2Op(logger),
      getAreaBalanceOp(logger),
      getBalanceOp(logger),
      getFalsePathOp(logger),
      getEquivReduceOp(logger),
      getSopBalancingOp(logger),
      getSynch2Op(logger)
  };
}

GiaOp 
GiaOptOperator::getOptOperator(int index, utl::Logger* logger)
{
  auto all_ops = getAllOperations(logger);
  if (index < 0 || index >= static_cast<int>(all_ops.size())) {
    throw std::out_of_range("Operator index out of range");
  }
  return all_ops[index];
}

void 
GiaOptOperator::runOptOperator(abc::Gia_Man_t*& gia, int action, utl::Logger* logger)
{
  printf("Running Gia optimization operator %d\n", action);
  auto gia_op = getOptOperator(action, logger);
  gia_op(gia);
  printNtkInfo(gia, logger);
}

}  // namespace rmp

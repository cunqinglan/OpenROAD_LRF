// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#pragma once

#include <random>

#include "aig/aig/aig.h"
#include "base/abc/abc.h"
#include "db_sta/dbSta.hh"
#include "resynthesis_strategy.h"
#include "rsz/Resizer.hh"
#include "sta/Corner.hh"
#include "sta/Delay.hh"
#include "sta/Liberty.hh"
#include "utl/Logger.h"
#include "utl/deleter.h"

namespace cut {
class LogicCut;
}  // namespace cut

namespace rmp {

utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> WrapUnique(abc::Abc_Ntk_t* ntk);

std::vector<sta::Vertex*> GetEndpoints(sta::dbSta* sta,
                                       rsz::Resizer* resizer,
                                       sta::Slack slack_threshold);

void checkNtkType(abc::Abc_Ntk_t* ntk, utl::Logger* logger);

void printNtkInfo(abc::Abc_Ntk_t* ntk, utl::Logger* logger);
void printNtkInfo(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>& ntk,
                  utl::Logger* logger);
void printNtkInfo(abc::Gia_Man_t* gia, utl::Logger* logger);
void printNtkInfo(abc::Aig_Man_t* aig, utl::Logger* logger);
int CountInputPins(const sta::LibertyCell* cell);

bool HasLargeInputCells(const cut::LogicCut& cut,
                        sta::dbNetwork* network,
                        int min_inputs,
                        int* large_cell_count = nullptr);

}  // namespace rmp

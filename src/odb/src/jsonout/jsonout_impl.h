// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <list>
#include <map>
#include <ostream>
#include <string>

#include "odb/db.h"
#include "odb/dbMap.h"
#include "odb/dbObject.h"
#include "odb/jsonout.h"
#include "odb/odb.h"
namespace utl {
class Logger;
}

namespace odb {

class dbBlock;
class dbBTerm;
class dbInst;
class dbTechNonDefaultRule;
class dbTechLayerRule;

class JsonOut::Impl
{
 public:
  Impl(utl::Logger* logger) : _logger(logger) {}

  ~Impl() = default;

  void selectNet(dbNet* net);
  void selectInst(dbInst* inst);

  bool writeBlock(dbBlock* block, const char* def_file);
  bool writeBlock(dbBlock* block, std::ostream& stream);

 private:
  std::list <dbInst*> _inst_list;
  double _dist_factor{0};
  std::ostream* _out{nullptr};
  dbMap<dbNet, char>* _select_net_map{nullptr};
  dbMap<dbInst, char>* _select_inst_map{nullptr};
  dbTechNonDefaultRule* _non_default_rule{nullptr};
  utl::Logger* _logger;
};

}  // namespace odb

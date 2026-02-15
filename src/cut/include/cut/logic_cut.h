// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2024-2025, The OpenROAD Authors

#pragma once

#include <utility>
#include <vector>

#include "base/abc/abc.h"
#include "cut/abc_library_factory.h"
#include "db_sta/dbNetwork.hh"
#include "sta/NetworkClass.hh"
#include "utl/Logger.h"
#include "utl/deleter.h"
#include "utl/unique_name.h"

#include "map/mapper/mapper.h"

namespace cut {
class LogicCut
{
 public:
  LogicCut(std::vector<sta::Net*>&& primary_inputs,
           std::vector<sta::Net*>&& primary_outputs,
           sta::InstanceSet&& cut_instances)
      : primary_inputs_(std::move(primary_inputs)),
        primary_outputs_(std::move(primary_outputs)),
        cut_instances_(std::move(cut_instances))
  {
  }
  ~LogicCut() = default;

  const std::vector<sta::Net*>& primary_inputs() const
  {
    return primary_inputs_;
  }
  const std::vector<sta::Net*>& primary_outputs() const
  {
    return primary_outputs_;
  }
  const sta::InstanceSet& cut_instances() const { return cut_instances_; }
  void set_cut_instances(const sta::InstanceSet& instances) { cut_instances_ = instances; }

  bool IsEmpty() const
  {
    return primary_inputs_.empty() && primary_outputs_.empty()
           && cut_instances_.empty();
  }

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> BuildMappedAbcNetwork(
      AbcLibrary& abc_library,
      sta::dbNetwork* network,
      utl::Logger* logger);
  /*
  utl::UniquePtrWithDeleter<abc::Design_Info_t> BuildAbcDesignInfo(
      AbcLibrary& abc_library,
      sta::dbNetwork* network,
      utl::Logger* logger);
  */
  void InsertMappedAbcNetwork(abc::Abc_Ntk_t* abc_network,
                              AbcLibrary& abc_library,
                              sta::dbNetwork* network,
                              sta::dbSta* sta,
                              utl::UniqueName& unique_name,
                              utl::Logger* logger);
  
  void InsertAbcMapSolution(abc::Map_MappingSolution_t* pSolution,
                           abc::Map_Man_t* pMan,
                           abc::Abc_Ntk_t* pOriginalNetwork,
                           AbcLibrary& abc_library,
                           sta::dbNetwork* network,
                           sta::dbSta* sta,
                           utl::UniqueName& unique_name,
                           utl::Logger* logger);
  /*
  utl::UniquePtrWithDeleter<std::pair<abc::Abc_Ntk_t*, abc::Design_Info_t*>> 
       BuildAbcNetworkWithPositions(
           AbcLibrary& abc_library,
           sta::dbNetwork* network,
           utl::Logger* logger);
  */

 private:
  std::vector<sta::Net*> primary_inputs_;
  std::vector<sta::Net*> primary_outputs_;
  sta::InstanceSet cut_instances_;
};
}  // namespace cut

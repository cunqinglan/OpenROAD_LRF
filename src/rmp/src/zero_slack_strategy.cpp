// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include "zero_slack_strategy.h"

#include <optional>
#include <vector>

#include "cut/abc_library_factory.h"
#include "cut/logic_cut.h"
#include "cut/logic_extractor.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "delay_optimization_strategy.h"
#include "map/mio/mio.h"
#include "rsz/Resizer.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Search.hh"
#include "utils.h"
#include "utl/Logger.h"
#include "utl/deleter.h"
#include "utl/unique_name.h"

namespace rmp {

void ZeroSlackStrategy::OptimizeDesign(sta::dbSta* sta,
                                       utl::UniqueName& name_generator,
                                       rsz::Resizer* resizer,
                                       utl::Logger* logger)
{
  sta->ensureGraph();
  sta->ensureLevelized();
  sta->searchPreamble();
  sta->ensureClkNetwork();

  sta::dbNetwork* network = sta->getDbNetwork();

  std::vector<sta::Vertex*> candidate_vertices
      = GetEndpoints(sta, resizer, 0.0);

  if (candidate_vertices.empty()) {
    logger->info(utl::RMP,
                 50,
                 "All candidate endpoints have positive slack, nothing to do.");
    return;
  }

  cut::AbcLibraryFactory factory(logger);
  factory.AddDbSta(sta);
  factory.AddResizer(resizer);
  factory.SetCorner(corner_);
  cut::AbcLibrary abc_library = factory.Build();

  // Disable incremental timing.
  sta->graphDelayCalc()->delaysInvalid();
  sta->search()->arrivalsInvalid();
  sta->search()->endpointsInvalid();

  cut::LogicExtractorFactory logic_extractor(sta, logger);
  for (sta::Vertex* negative_endpoint : candidate_vertices) {
    logic_extractor.AppendEndpoint(negative_endpoint);
  }

  cut::LogicCut cut = logic_extractor.BuildLogicCut(abc_library);

  if (cut.IsEmpty()) {
    logger->warn(
        utl::RMP, 1032, "Logic cut is empty after extraction, nothing to do.");
    return;
  }

  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> mapped_abc_network
      = cut.BuildMappedAbcNetwork(abc_library, network, logger);

  cut::AbcLibrary* map_library = &abc_library;
  abc::Mio_Library_t* map_mio
      = static_cast<abc::Mio_Library_t*>(mapped_abc_network->pManFunc);
  std::optional<cut::AbcLibrary> small_library;

  if (split_large_inputs_k_ && *split_large_inputs_k_ >= 2) {
    const int kLargeInputThreshold = *split_large_inputs_k_;
    const int kMaxSmallInputs = *split_large_inputs_k_ - 1;
    int large_cell_count = 0;
    const bool has_large_inputs = HasLargeInputCells(
        cut, network, kLargeInputThreshold, &large_cell_count);

    if (has_large_inputs) {
      cut::AbcLibraryFactory small_factory(logger);
      small_factory.AddDbSta(sta);
      small_factory.AddResizer(resizer);
      small_factory.SetCorner(corner_);
      small_factory.SetMaxInputCount(kMaxSmallInputs);
      small_library.emplace(small_factory.Build());
      map_library = &*small_library;
      map_mio = map_library->mio_library();

      if (abc::Mio_LibraryReadBuf(map_mio) == nullptr) {
        logger->warn(
            utl::RMP,
            1033,
            "No buffer cell found after limiting to <= {} input gates; "
            "falling back to full library mapping.",
            kMaxSmallInputs);
        map_library = &abc_library;
        map_mio
            = static_cast<abc::Mio_Library_t*>(mapped_abc_network->pManFunc);
      } else {
        logger->info(
            utl::RMP,
            1034,
            "Found {} cells with >= {} inputs in the cut; "
            "remapping with <= {} input gates.",
            large_cell_count,
            kLargeInputThreshold,
            kMaxSmallInputs);
      }
    }
  }

  DelayOptimizationStrategy strategy;
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> remapped
      = strategy.Optimize(mapped_abc_network.get(),
                          *map_library,
                          map_mio,
                          logger);

  cut.InsertMappedAbcNetwork(
      remapped.get(), abc_library, network, sta, name_generator, logger);
}
}  // namespace rmp

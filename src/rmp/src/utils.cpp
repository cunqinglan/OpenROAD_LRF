// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#include "utils.h"

#include <fcntl.h>

#include <vector>

#include "base/abc/abc.h"
#include "cut/abc_library_factory.h"
#include "cut/logic_cut.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Delay.hh"
#include "sta/Graph.hh"
#include "sta/MinMax.hh"
#include "sta/PortDirection.hh"
#include "utl/deleter.h"

namespace rmp {

utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> WrapUnique(abc::Abc_Ntk_t* ntk)
{
  return utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>(ntk, &abc::Abc_NtkDelete);
}

std::vector<sta::Vertex*> GetEndpoints(sta::dbSta* sta,
                                       rsz::Resizer* resizer,
                                       sta::Slack slack_threshold)
{
  std::vector<sta::Vertex*> result;

  sta::dbNetwork* network = sta->getDbNetwork();
  for (sta::Vertex* vertex : *sta->endpoints()) {
    sta::Pin* pin = vertex->pin();
    sta::PortDirection* direction = network->direction(vertex->pin());
    if (!direction->isInput()) {
      continue;
    }

    if (resizer != nullptr) {
      if (resizer->dontTouch(pin) || resizer->dontTouch(network->net(pin))
          || resizer->dontTouch(network->instance(pin))) {
        continue;
      }
    }

    const sta::Slack slack = sta->vertexSlack(vertex, sta::MinMax::max());

    if (slack < slack_threshold) {
      result.push_back(vertex);
    }
  }

  return result;
}

int CountInputPins(const sta::LibertyCell* cell)
{
  if (!cell) {
    return 0;
  }

  sta::LibertyCellPortIterator cell_port_iterator(
      const_cast<sta::LibertyCell*>(cell));
  int input_count = 0;
  while (cell_port_iterator.hasNext()) {
    sta::LibertyPort* port = cell_port_iterator.next();
    if (port->direction()->isInput()) {
      input_count++;
    }
  }

  return input_count;
}

bool HasLargeInputCells(const cut::LogicCut& cut,
                        sta::dbNetwork* network,
                        int min_inputs,
                        int* large_cell_count)
{
  int count = 0;
  for (const sta::Instance* instance : cut.cut_instances()) {
    if (!instance) {
      continue;
    }
    sta::LibertyCell* cell = network->libertyCell(instance);
    if (!cell) {
      continue;
    }
    if (CountInputPins(cell) >= min_inputs) {
      count++;
      if (large_cell_count == nullptr) {
        return true;
      }
    }
  }

  if (large_cell_count != nullptr) {
    *large_cell_count = count;
  }
  return count > 0;
}

}  // namespace rmp

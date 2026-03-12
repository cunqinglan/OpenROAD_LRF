// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "jsonout_impl.h"

#include <sys/stat.h>

#include <algorithm>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include "odb/db.h"
#include "odb/dbMap.h"
#include "odb/dbObject.h"
#include "odb/dbSet.h"
#include "odb/dbTypes.h"
#include "odb/dbWireCodec.h"
#include "odb/defout.h"
#include "odb/geom.h"
#include "utl/Logger.h"
#include "utl/ScopedTemporaryFile.h"
//#include "sta/Liberty.hh"
//#include "sta/Network.hh"
//#include "sta/Graph.hh"
//#include "sta/Sta.hh"

namespace odb {

namespace {

template <typename T>
std::vector<T*> sortedSet(dbSet<T>& to_sort)
{
  std::vector<T*> sorted(to_sort.begin(), to_sort.end());
  std::sort(sorted.begin(), sorted.end(), [](T* a, T* b) {
    return a->getName() < b->getName();
  });
  return sorted;
}

}  // namespace

void JsonOut::Impl::selectNet(dbNet* net)
{
  if (!net) {
    return;
  }
  if (!_select_net_map) {
    return;
  }
  (*_select_net_map)[net] = 1;
}

void JsonOut::Impl::selectInst(dbInst* inst)
{
  if (!inst) {
    return;
  }
  _inst_list.push_back(inst);
}

bool JsonOut::Impl::writeBlock(dbBlock* block, const char* json_file)
{
  std::ofstream outfile(json_file);
  if (!outfile.is_open()) {
    _logger->error(utl::ODB, 1, "Cannot open file {} for writing", json_file);
    return false;
  }
  return writeBlock(block, outfile);
}

bool JsonOut::Impl::writeBlock(dbBlock* block, std::ostream& stream)
{
  _out = &stream;
  _dist_factor = 1.0 / (double) block->getDbUnitsPerMicron();

//  // Get STA database
//  sta::dbSta* sta = block->getDataBase()->getDbSta();
//  sta::dbNetwork* network = sta ? sta->getDbNetwork() : nullptr;

  // Get all instances
  dbSet<dbInst> insts = block->getInsts();
  auto sorted_insts = sortedSet(insts);

  // Start JSON output
  *_out << "{\n";
  *_out << "  \"design\": \"" << block->getName() << "\",\n";
  *_out << "  \"units\": \"microns\",\n";
  *_out << "  \"gates\": [\n";

  bool first_inst = true;
  for (dbInst* inst : sorted_insts) {
    if (!first_inst) {
      *_out << ",\n";
    }
    first_inst = false;

    // Get instance information
    std::string inst_name = inst->getName();
    dbMaster* master = inst->getMaster();
    std::string gate_type = master->getName();
    
    int x, y;
    inst->getLocation(x, y);
    double x_micron = x * _dist_factor;
    double y_micron = y * _dist_factor;

    *_out << "    {\n";
    *_out << "      \"name\": \"" << inst_name << "\",\n";
    *_out << "      \"type\": \"" << gate_type << "\",\n";
    *_out << "      \"position\": {\n";
    *_out << "        \"x\": " << x_micron << ",\n";
    *_out << "        \"y\": " << y_micron << "\n";
    *_out << "      },\n";
    *_out << "      \"connections\": {\n";

    // Get all pins (iterms) of this instance
    dbSet<dbITerm> iterms = inst->getITerms();
    auto sorted_iterms = sortedSet(iterms);
    
    bool first_iterm = true;
    for (dbITerm* iterm : sorted_iterms) {
      dbMTerm* mterm = iterm->getMTerm();
      std::string pin_name = mterm->getName();
      
      if (!first_iterm) {
        *_out << ",\n";
      }
      first_iterm = false;

      *_out << "        \"" << pin_name << "\": ";

      dbNet* net = iterm->getNet();
      if (net) {
        *_out << "{\n";
        *_out << "          \"net\": \"" << net->getName() << "\",\n";
        *_out << "          \"connected_gates\": [";
        // Get all other instances connected to this net
        std::set<std::string> connected_insts;
        for (dbITerm* other_iterm : net->getITerms()) {
          dbInst* other_inst = other_iterm->getInst();
          if (other_inst != inst) {
            connected_insts.insert(other_inst->getName());
          }
        }

        bool first_conn = true;
        for (const auto& conn_name : connected_insts) {
          if (!first_conn) {
            *_out << ", ";
          }
          first_conn = false;
          *_out << "\"" << conn_name << "\"";
        }

        *_out << "]\n";
        *_out << "        }";
      } else {
        *_out << "null";
      }
    }

    *_out << "\n      }\n";
    *_out << "    }";
  }

  *_out << "\n  ]\n";
  *_out << "}\n";

  _out = nullptr;
  return true;
}

}  // namespace odb

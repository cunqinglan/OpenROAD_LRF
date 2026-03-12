// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <functional>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "db_sta/dbSta.hh"
#include "rsz/Resizer.hh"
#include "sta/Corner.hh"
#include "sta/Delay.hh"
#include "sta/Liberty.hh"
#include "sta/NetworkClass.hh"
#include "utl/unique_name.h"

namespace abc {
}  // namespace abc

namespace utl {
class Logger;
}

namespace odb {
class dbDatabase;
class dbBlock;
class dbInst;
class dbNet;
class dbITerm;
}  // namespace odb

namespace est {
class EstimateParasitics;
}

namespace sta {
class dbSta;
}  // namespace sta

namespace gpl {
class Replace;
}

namespace dpl {
class Opendp;
}

namespace rmp {

using utl::Logger;

enum class Mode
{
  AREA_1 = 0,
  AREA_2,
  AREA_3,
  DELAY_1,
  DELAY_2,
  DELAY_3,
  DELAY_4
};

class Restructure
{
 public:
  Restructure(utl::Logger* logger,
              sta::dbSta* open_sta,
              odb::dbDatabase* db,
              rsz::Resizer* resizer,
              est::EstimateParasitics* estimate_parasitics,
              gpl::Replace* replace = nullptr,
              dpl::Opendp* opendp = nullptr);
  ~Restructure();

  void reset();
  void resynth(sta::Corner* corner);
  void resynthAnnealing(sta::Corner* corner);
  void run(char* liberty_file_name,
           float slack_threshold,
           unsigned max_depth,
           char* workdir_name,
           char* abc_logfile);

  void setAnnealingSeed(std::mt19937::result_type seed)
  {
    annealing_seed_ = seed;
  }
  void setAnnealingTemp(float temp) { annealing_temp_ = temp; }
  void setAnnealingIters(unsigned iters) { annealing_iters_ = iters; }
  void setAnnealingRevertAfter(unsigned revert_after)
  {
    annealing_revert_after_ = revert_after;
  }
  void setAnnealingInitialOps(unsigned ops) { annealing_init_ops_ = ops; }
  void setSlackThreshold(sta::Slack thresh) { slack_threshold_ = thresh; }
  void setMode(const char* mode_name);
  void setSplitLargeInputs(int k);
  void setTieLoPort(sta::LibertyPort* loport);
  void setTieHiPort(sta::LibertyPort* hiport);
  
  // Position-driven remapping strategy.
  // Endpoint selection (pass -1.0 / FLT_MAX to leave unset):
  //   percentage      >= 0 : fix top N% of all endpoints (min 1), overrides others
  //   max_percentage  >= 0 : cap count at N% of all endpoints (used with slack_threshold)
  //   slack_threshold      : select endpoints with slack < threshold (used with max_percentage)
  void positionDrivenRemap(sta::Corner* corner,
                           float percentage = -1.0f,
                           float max_percentage = -1.0f,
                           float slack_threshold = std::numeric_limits<float>::max());

 private:
  void deleteComponents();
  void getBlob(unsigned max_depth);
  void runABC();
  void postABC(float worst_slack);
  bool writeAbcScript(std::string file_name);
  void writeOptCommands(std::ofstream& script);
  void initDB();
  void getEndPoints(sta::PinSet& ends, bool area_mode, unsigned max_depth);
  int countConsts(odb::dbBlock* top_block);
  void removeConstCells();
  void removeConstCell(odb::dbInst* inst);
  bool readAbcLog(std::string abc_file_name, int& level_gain, float& delay_val);
  void collectLargeInputDontUse();

  Logger* logger_;
  utl::UniqueName name_generator_;
  std::string logfile_;
  std::string locell_;
  std::string loport_;
  std::string hicell_;
  std::string hiport_;
  std::string work_dir_name_;

  // db vars
  sta::dbSta* open_sta_;
  odb::dbDatabase* db_;
  rsz::Resizer* resizer_;
  est::EstimateParasitics* estimate_parasitics_;
  gpl::Replace* replace_ = nullptr;
  dpl::Opendp* opendp_ = nullptr;
  odb::dbBlock* block_ = nullptr;

  // Annealing
  std::optional<std::mt19937::result_type> annealing_seed_;
  std::optional<float> annealing_temp_;
  unsigned annealing_iters_ = 100;
  std::optional<unsigned> annealing_revert_after_;
  unsigned annealing_init_ops_ = 10;
  sta::Slack slack_threshold_ = 0;

  std::string input_blif_file_name_;
  std::string output_blif_file_name_;
  std::vector<std::string> lib_file_names_;
  std::set<odb::dbInst*> path_insts_;
  std::set<std::string> abc_dont_use_;
  std::optional<int> split_large_inputs_k_;

  Mode opt_mode_{Mode::DELAY_1};
  bool is_area_mode_{false};
  int blif_call_id_{0};
};

}  // namespace rmp

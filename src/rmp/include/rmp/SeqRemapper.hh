// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2025-2025, The OpenROAD Authors

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <string>

#include "cut/abc_library_factory.h"
#include "db_sta/dbSta.hh"
#include "sta/Corner.hh"
#include "sta/Delay.hh"
#include "utl/Logger.h"
#include "utl/unique_name.h"

#include "db_sta/dbSta.hh"

namespace odb {
class dbDatabase;
}

namespace cut {
class LogicCut;
}

namespace rsz {
class Resizer;
}  // namespace rsz

namespace dpl {
class Opendp;
}  // namespace dpl

namespace gpl {
class Replace;
}  // namespace gpl

namespace est {
class EstimateParasitics;
}  // namespace est

namespace rmp {

class Strategy;
class GiaOptOperator;
class Operator;

using GiaOp = std::function<void(abc::Gia_Man_t*&)>; 

struct MappingResult
{
  float wns;
  float tns;
  float area;
  float power;
  void show(utl::Logger* logger);
};

class NtkType
{
 public:
  enum Type
  {
    AIG,
    GIA,
    NETLIST
  };
};

class PlaceMode
{
 public:
  enum Mode
  {
    ROUTE_DRIVEN,
    TIMING_DRIVEN
  };
};

// Provide a single definition in SeqRemapper.cc with external linkage.
void replaceGia(abc::Gia_Man_t*& gia, abc::Gia_Man_t* new_gia);

class SeqRemapper : public sta::dbStaState
{
 public:
  SeqRemapper(sta::dbSta* sta, odb::dbDatabase* db, 
              sta::Corner* corner, rsz::Resizer* resizer, 
              utl::Logger* logger, gpl::Replace* gpl,
              dpl::Opendp* dpl, est::EstimateParasitics* est);
  ~SeqRemapper();

  // Run optimization
  void runOpt();   // Automatically run the whole optimization flow
  void TryOptWithAig(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>& aig_ntk,
                     NtkType::Type ntk_type, size_t action, 
                     cut::LogicCut& logic_cut);

  // Get attributes
  sta::dbSta* getSta() const { return sta_; };
  sta::Corner* getCorner() const { return corner_; };
  rsz::Resizer* getResizer() const { return resizer_; };
  utl::Logger* getLogger() const { return logger_; };
  gpl::Replace* getGpl() const { return gpl_; };
  dpl::Opendp* getDpl() const { return dpl_; };
  est::EstimateParasitics* getEstimateParasitics() const { return est_; }
  float getSlackThreshold() const { return slack_threshold_; }
  cut::AbcLibrary *getAbcLibrary() { return abc_library_; }
  utl::UniqueName& getNameGenerator() { return name_generator_; }

  void checkTracksAndRows();
  void setIncrePlaceParam(PlaceMode::Mode mode, float density_penalty,
                          int place_iter);

  // Preparation before remapping
  void remapPreamble();
  // Identify bottleneck area and extract subcircuit
  cut::LogicCut extractBottleneck(Strategy &strategy);
  // Convert subsircuit into AIG representation
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> netlistToAig(cut::LogicCut& logic_cut);
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> cutToAig(cut::LogicCut& logic_cut);
  // Convert AIG to GIA format
  abc::Gia_Man_t* aigToGia(
      utl::UniquePtrWithDeleter<abc::Abc_Ntk_t>& strashed_aig);
  // Build ABC library with libs
  void buildAbcLibrary();

  // Run sequential logic optimization on AIG
  // void runGiaOptAction(abc::Gia_Man_t* gia, GiaOp& gia_op);

  // Convert optimized GIA back to AIG to enable mapping
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> giaToAig(abc::Gia_Man_t* gia);
  
  ////////////////////////////////////////////////
  // Evaluate the optimized subcircuit
  MappingResult evaluate(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &aig_ntk,
                cut::LogicCut& logic_cut);
  // Function to perform the evaluation process
  utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> 
        performMapping(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &aig_ntk);
  void insertMappedAbcNetwork(utl::UniquePtrWithDeleter<abc::Abc_Ntk_t> &mapped_ntk, cut::LogicCut& logic_cut);
  void performIncrePlace(cut::LogicCut& logic_cut, gpl::Replace *gpl, dpl::Opendp* dpl);
  void performPlace(cut::LogicCut& logic_cut, gpl::Replace *gpl, dpl::Opendp* dpl);
  void performIncreGpl(cut::LogicCut& logic_cut, gpl::Replace *gpl);
  void performIncreDpl(cut::LogicCut& logic_cut, dpl::Opendp* dpl);
  void performTimingRepair(cut::LogicCut& logic_cut);
  void getMetrics(cut::LogicCut& logic_cut, MappingResult& result);


protected:
  odb::dbDatabase* db_;
  sta::Corner* corner_;
  rsz::Resizer* resizer_;
  utl::Logger* logger_;
  gpl::Replace* gpl_;
  dpl::Opendp* dpl_;
  est::EstimateParasitics* est_;
  cut::AbcLibrary *abc_library_; 
  Operator *opt_operator_factory_;
  utl::UniqueName name_generator_;

  odb::dbBlock* block_ = nullptr;

  float slack_threshold_ = 0.0f;

  // Metrics
  sta::Slack worst_slack_;
  float area_ = 0.0f;
  float static_power_ = 0.0f;
  size_t iters_ = 0;

private:
  friend class Strategy;
};

/////////////////////////////////////////////////
// Operator class of logic optimization
////////////////////////////////////////////////
class Operator
{
 public:
  Operator() = default;
  Operator(size_t action_count) : action_count_(action_count) {}
  virtual ~Operator() = default;
  virtual void runOptOperator(abc::Gia_Man_t*& gia, int action, utl::Logger* logger);
  virtual void runOptOperator(abc::Abc_Ntk_t*& ntk, int action, utl::Logger* logger);
  // virtual void runOptOperator(abc::Aig_Man_t* aig, int action, utl::Logger* logger);

  private:
  size_t action_count_ = 0;
};

class GiaOptOperator : public Operator
{
 public:
  GiaOptOperator() : Operator(13) {}
  void runOptOperator(abc::Gia_Man_t*& gia, int action, utl::Logger* logger) override;
  static std::vector<GiaOp> getAllOperations(utl::Logger* logger);

  GiaOp getOptOperator(int action, utl::Logger* logger);
  
  // 单个操作
  static GiaOp getRehashOp(utl::Logger* logger);
  static GiaOp getDchOp(utl::Logger* logger);
  static GiaOp getSyn2Op(utl::Logger* logger);
  static GiaOp getSyn3Op(utl::Logger* logger);
  static GiaOp getSyn4Op(utl::Logger* logger);
  static GiaOp getRetimeOp(utl::Logger* logger);
  static GiaOp getCompress2Op(utl::Logger* logger);
  static GiaOp getAreaBalanceOp(utl::Logger* logger);
  static GiaOp getBalanceOp(utl::Logger* logger);
  static GiaOp getFalsePathOp(utl::Logger* logger);
  static GiaOp getEquivReduceOp(utl::Logger* logger);
  static GiaOp getSopBalancingOp(utl::Logger* logger);
  static GiaOp getSynch2Op(utl::Logger* logger);
  
 private:
};


}  // namespace rmp

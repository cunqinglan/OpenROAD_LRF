
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <random>
#include <string>

#include "cut/abc_library_factory.h"
#include "cut/logic_extractor.h"
#include "db_sta/dbSta.hh"
#include "sta/Corner.hh"
#include "sta/Delay.hh"
#include "utl/Logger.h"
#include "utl/unique_name.h"

#include "db_sta/dbSta.hh"

namespace sta {
class SearchPred;
}

namespace cut {
class AbcLibrary;
class LogicCut;
}

namespace rmp {

class SearchABCCompatiblePred : public cut::SearchPredNonReg2AbcSupport
{
 public:
  SearchABCCompatiblePred(sta::dbSta* open_sta,
                          cut::AbcLibrary* abc_library,
                          sta::Graph* graph)
      : cut::SearchPredNonReg2AbcSupport(open_sta, abc_library, graph)
  {
  }
  bool searchThru(sta::Edge* edge) override;
};

// Forward declaration
class SeqRemapper;

class LogicExtractorFactoryPro : public cut::LogicExtractorFactory
{
 public:
  LogicExtractorFactoryPro(sta::dbSta* sta, utl::Logger* logger);
  void clear();
  void setCutVertices(const sta::VertexSet &cut_vertices);
  cut::LogicCut buildLogicCutFromCutVertices(sta::VertexSet &cut_vertices, 
                                    cut::AbcLibrary &abc_network);
};

class Strategy
{
 public:
  Strategy(utl::Logger *logger) : logger_(logger) {}
  virtual ~Strategy() = default;

  virtual std::string to_string() = 0;
  virtual bool isExtractFaninConeOfBadEndPoints();
  virtual void setRefGate(odb::dbInst *ref_gate) {}
  virtual void setRefGate(sta::Instance *ref_gate) {}

  // Function to extract cut according to strategy
  virtual cut::LogicCut extractBottleneck(SeqRemapper& remapper) = 0;
protected:
  utl::Logger *logger_;
};

class ExtractFaninConeOfBadEndPoints : public Strategy
{
 public:
  ExtractFaninConeOfBadEndPoints(utl::Logger *logger) : Strategy(logger) {}
  ~ExtractFaninConeOfBadEndPoints() override = default;

  std::string to_string() override;

  bool isExtractFaninConeOfBadEndPoints() override;
  cut::LogicCut extractBottleneck(SeqRemapper& remapper) override;
};

class ExtractLocalWindow : public Strategy
{
public:
  ExtractLocalWindow(utl::Logger *logger) : Strategy(logger) {}
  ~ExtractLocalWindow() { delete abc_search_pred_; }

  std::string to_string() override;

  void setRefGate(sta::Instance *ref_gate) override { ref_gate_ = ref_gate; }
  void setRefGate(odb::dbInst *ref_gate) override {
    sta::Instance* instance = sta_->getDbNetwork()->dbToSta(ref_gate);
    ref_gate_ = instance;
  }
  void collectAdjacentInsts(sta::Instance* inst, 
                            size_t window_size, 
                          //  Return value
                            sta::VertexSet &cut_vertices);
  void collectAdjacentInsts(odb::dbInst* inst, size_t window_size,
                           //  Return value
                           sta::VertexSet &cut_vertices);
  void collectFanoutVerticesInWindow(sta::Vertex* drvr_vertex, size_t current_depth,
                                //  Return value
                                 sta::VertexSet &cut_vertices);
  void collectFaninVerticesInWindow(sta::Vertex* load_vertex, size_t current_depth,
                                //  Return value  
                                sta::VertexSet &cut_vertices);

  cut::LogicCut extractBottleneck(SeqRemapper& remapper) override;
  
  protected:
    sta::dbSta* sta_;
    cut::AbcLibrary* abc_library_;
    size_t window_size_ = 1;
    sta::SearchPred *abc_search_pred_ = nullptr;
    sta::Instance* ref_gate_ = nullptr;
};





} // namespace rmp
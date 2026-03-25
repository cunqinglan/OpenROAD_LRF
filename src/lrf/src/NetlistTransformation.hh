#pragma once

#include "lrf/LrfClass.hh"
#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/Delay.hh"
#include "LibertyClass.hh"

#include <memory>
#include <map>
#include <unordered_map>
#include <vector>
#include <limits>
#include <mutex>

namespace sta {
  class ArcDelayCalc;
  class dbSta;
}

namespace rsz {
  class Resizer;
  class BufferedNet;
  using BufferedNetPtr = std::shared_ptr<BufferedNet>;
}

namespace lrf {

class LocalSta;
class PtGraph;
class PtVertex;
class LrRebuffer;
class LrRebufferV2;
class LocalCellInfo;
class TaskArranger;

// ═══════════════════════════════════════════════════════════
// Layer 1: EvalContext — per-thread evaluation context
// ═══════════════════════════════════════════════════════════
struct EvalContext {
  PtGraph *pt_graph = nullptr;
  sta::ArcDelayCalc *arc_delay_calc = nullptr;
  float average_delay = 1.0f;
  float average_leakage = 1.0f;
  float PT_tradeoff = 100.0f;
  std::map<std::string, double> *runtime_map = nullptr;

  float swapCost(float delay_lm_sum, float power) const;
};

// ═══════════════════════════════════════════════════════════
// Layer 2: MoveOption — unified action protocol
// ═══════════════════════════════════════════════════════════
struct MoveOption {
  enum Type { NONE, RESIZE_ONLY, BUFFER_ONLY, COMBINED };
  Type type = NONE;
  float cost = std::numeric_limits<float>::max();
  float slack = 0.0f;
  sta::LibertyCell *target_cell = nullptr;
  rsz::BufferedNetPtr buffer_tree = nullptr;

  bool hasChange() const { return type != NONE; }
  // Update only if cost improves AND slack margin is respected.
  // slack_before: original cell slack; slack_margin: e.g. 1.05
  void updateIfBetter(Type t, float c, float s,
                      float slack_before, float slack_margin,
                      sta::LibertyCell *cell,
                      rsz::BufferedNetPtr bnet);
};

// ═══════════════════════════════════════════════════════════
// Layer 3: LrOperator — stateless operator interface
// ═══════════════════════════════════════════════════════════
class LrOperator {
public:
  virtual ~LrOperator() = default;
  virtual MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                              EvalContext &ctx) = 0;
  virtual std::unique_ptr<LrOperator> copy() const = 0;
};

// ─── ResizeOperator ──────────────────────────────────────
class ResizeOperator : public LrOperator {
public:
  ResizeOperator(sta::dbSta *db_sta, LocalSta *local_sta);
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  std::unique_ptr<LrOperator> copy() const override;

  void setEquivCellArray(LibertyCellArray *array, PosMap *pos_map);
  void setInstInfoMap(std::unordered_map<sta::Instance*, LocalCellInfo*> *map);
  void setSlackMargin(float margin) { slack_margin_ = margin; }
  void setColPadding(int p) { col_padding_ = p; }
  void setRowPadding(int p) { row_padding_ = p; }

  friend class CombinedOperator;

protected:
  // Shared: collect candidates within neighborhood
  std::vector<sta::LibertyCell*> collectCandidates(
      sta::LibertyCell *ori_cell) const;

  // Shared: look up pre-computed leakage for a candidate cell
  float lookupLeakage(sta::Instance *inst, sta::LibertyCell *cand) const;

  sta::dbSta *db_sta_;
  LocalSta *local_sta_;
  LibertyCellArray *equiv_cell_array_ = nullptr;
  PosMap *equiv_cell_pos_map_ = nullptr;
  std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map_ = nullptr;
  float slack_margin_ = 0.0f;
  int col_padding_ = 3;
  int row_padding_ = 1;
};

// ─── ResizePrecheckOperator ──────────────────────────────
// Lightweight precheck: same eval loop but smaller neighborhood,
// no final timing recompute, returns cost delta (positive = beneficial).
class ResizePrecheckOperator : public ResizeOperator {
public:
  using ResizeOperator::ResizeOperator;
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  std::unique_ptr<LrOperator> copy() const override;
};

// ─── BufferOperator ──────────────────────────────────────
class BufferOperator : public LrOperator {
public:
  BufferOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                 rsz::Resizer *resizer, EvalContext *ctx);
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  std::unique_ptr<LrOperator> copy() const override;

  LrRebufferV2 *rebuffer() { return rebuffer_.get(); }
  rsz::Resizer *resizer() { return resizer_; }
  // Update the EvalContext pointer (called after copy() when new visitor's ctx is ready)
  void setEvalContext(EvalContext *ctx);

protected:
  sta::dbSta *db_sta_;
  LocalSta *local_sta_;
  rsz::Resizer *resizer_;
  std::unique_ptr<LrRebufferV2> rebuffer_;
};

// ─── BufferSensitivityOperator ───────────────────────────
// Lightweight precheck: computes net sensitivity score without
// inserting buffers. Returns score in MoveOption::cost (type=NONE).
class BufferSensitivityOperator : public BufferOperator {
public:
  using BufferOperator::BufferOperator;
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  std::unique_ptr<LrOperator> copy() const override;
};

// ─── CombinedOperator ────────────────────────────────────
class CombinedOperator : public LrOperator {
public:
  CombinedOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                   rsz::Resizer *resizer, EvalContext *ctx);
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  std::unique_ptr<LrOperator> copy() const override;

  void setEquivCellArray(LibertyCellArray *array, PosMap *pos_map);
  void setInstInfoMap(std::unordered_map<sta::Instance*, LocalCellInfo*> *map);
  void setSlackMargin(float margin);

private:
  std::unique_ptr<ResizeOperator> resize_op_;
  std::unique_ptr<BufferOperator> buffer_op_;
  sta::dbSta *db_sta_;
  LocalSta *local_sta_;
};

// ═══════════════════════════════════════════════════════════
// Layer 4: ParallelVisitor — thin dispatcher
// ═══════════════════════════════════════════════════════════

extern std::mutex g_odb_sta_access_mutex;

class ParallelVisitor {
public:
  ParallelVisitor(sta::dbSta *db_sta, LocalSta *local_sta,
                  rsz::Resizer *resizer);
  virtual ~ParallelVisitor();

  // Initialize visitor: compute slack_margin from WNS/clock_period,
  // set average_delay/leakage, configure operators.
  void init(float average_delay, float average_power, float wns,
            float PT_tradeoff,
            std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map);

  virtual bool visit(sta::Instance *inst, sta::VertexId vid);
  bool singleGateSizing(sta::Instance *inst);
  void visitSlewOnly(sta::Instance *inst);
  virtual void applyChangesToDb(rsz::Resizer *resizer);
  virtual ParallelVisitor *copy() const;

  // Operator setup
  void setResizeOperator(std::unique_ptr<ResizeOperator> op);
  void setBufferOperator(std::unique_ptr<BufferOperator> op);
  void setCombinedOperator(std::unique_ptr<CombinedOperator> op);

  void setTaskArranger(TaskArranger *ta) { task_arranger_ = ta; }

  // Precheck mode: when set, visit() stores cost in results vector
  // indexed by vertex ID instead of applying changes to DB.
  void setPrecheckResults(std::vector<ResizeBenefit> *results) {
    precheck_results_ = results;
  }

  // Context configuration
  void setAverageDelay(float v) { eval_ctx_.average_delay = v; }
  void setAverageLeakage(float v) { eval_ctx_.average_leakage = v; }
  void setPTTradeoff(float v) { eval_ctx_.PT_tradeoff = v; }

  // Results
  const MoveOption &bestMove() const { return best_move_; }
  PtGraph *ptGraph() const { return pt_graph_; }
  sta::LibertyCell *bestCell() const { return best_move_.target_cell; }
  EvalContext &evalContext() { return eval_ctx_; }

  // Profiling
  void printRuntimeProfile() const;
  const std::map<std::string, double> &runtimeMap() const { return runtime_map_; }

protected:
  void updateTimingFromPtGraph();
  void updateVertexInfo(sta::VertexId vertex_id);
  void applyResizeToDb(rsz::Resizer *resizer);
  void applyBufferingToDb(rsz::Resizer *resizer);

  sta::dbSta *db_sta_;
  LocalSta *local_sta_;
  rsz::Resizer *resizer_;
  PtGraph *pt_graph_ = nullptr;
  EvalContext eval_ctx_;
  MoveOption best_move_;
  TaskArranger *task_arranger_ = nullptr;

  std::unique_ptr<ResizeOperator> resize_op_;
  std::unique_ptr<BufferOperator> buffer_op_;
  std::unique_ptr<CombinedOperator> combined_op_;
  std::vector<ResizeBenefit> *precheck_results_ = nullptr;

  std::map<std::string, double> runtime_map_ = {
    {"visit", 0.0},
    {"pt_graph_construction", 0.0},
    {"equiv_cell_check", 0.0},
    {"equiv_cell_count", 0.0},
    {"swap", 0.0},
    {"writeTimingToDb", 0.0},
    {"applyDb", 0.0},
    {"buffer_insertion", 0.0},
    {"buffer_count", 0.0},
    {"precheck", 0.0},
    {"rebuffer_total", 0.0},
    {"rebuffer_setup", 0.0},
    {"rebuffer_coarse", 0.0},
    {"rebuffer_precise", 0.0},
    {"rebuffer_pin_count", 0.0}
  };
};

}  // namespace lrf
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
  bool allow_resize = true;
  bool allow_buffer = true;
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

// Helper: write slew+paths from PtGraph vertices to sta::Graph.
void updateTimingFromPtGraph(PtGraph *pt_graph);

// ═══════════════════════════════════════════════════════════
// Layer 3: LrOperator — operator interface
// ═══════════════════════════════════════════════════════════
class LrOperator {
public:
  virtual ~LrOperator() = default;
  virtual MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                              EvalContext &ctx) = 0;
  virtual void apply(const MoveOption &move, PtGraph *pt_graph,
                     std::map<std::string, double> &runtime_map) {}
  virtual std::unique_ptr<LrOperator> copy() const = 0;

  // Configuration (override in subclasses that need them)
  virtual void setEquivCellArray(LibertyCellArray *, PosMap *) {}
  virtual void setInstInfoMap(std::unordered_map<sta::Instance*, LocalCellInfo*> *) {}
  virtual void setSlackMargin(float) {}
  virtual void setEvalContext(EvalContext *) {}
  virtual PruningControl *pruningControl() const { return nullptr; }
};

// ─── ResizeOperator ──────────────────────────────────────
class ResizeOperator : public LrOperator {
public:
  ResizeOperator(sta::dbSta *db_sta, LocalSta *local_sta);
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  // Return top-N resize candidates (sorted by cost, ascending).
  // Used by CombinedOperator to try buffering on multiple candidates.
  std::vector<MoveOption> evaluateTopN(PtGraph *pt_graph, sta::Instance *inst,
                                       EvalContext &ctx, int n);
  void apply(const MoveOption &move, PtGraph *pt_graph,
             std::map<std::string, double> &runtime_map) override;
  std::unique_ptr<LrOperator> copy() const override;

  void setEquivCellArray(LibertyCellArray *array, PosMap *pos_map) override;
  void setInstInfoMap(std::unordered_map<sta::Instance*, LocalCellInfo*> *map) override;
  void setSlackMargin(float margin) override { slack_margin_ = margin; }
  void setColPadding(int p) { col_padding_ = p; }
  void setRowPadding(int p) { row_padding_ = p; }
  void setPruningControl(PruningControl *prune_control) { pruning_control_ = prune_control; }
  PruningControl *pruningControl() const override { return pruning_control_; }

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
  PruningControl *pruning_control_ = nullptr;
};

// ─── ResizePrecheckOperator ──────────────────────────────
// Lightweight precheck: same eval loop but smaller neighborhood,
// no final timing recompute, returns cost delta (positive = beneficial).
class ResizePrecheckOperator : public ResizeOperator {
public:
  using ResizeOperator::ResizeOperator;
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  void apply(const MoveOption &, PtGraph *,
             std::map<std::string, double> &) override {}
  std::unique_ptr<LrOperator> copy() const override;
};

// ─── BufferOperator ──────────────────────────────────────
class BufferOperator : public LrOperator {
public:
  BufferOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                 rsz::Resizer *resizer, EvalContext *ctx);
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  void apply(const MoveOption &move, PtGraph *pt_graph,
             std::map<std::string, double> &runtime_map) override;
  std::unique_ptr<LrOperator> copy() const override;

  LrRebufferV2 *rebuffer() { return rebuffer_.get(); }
  rsz::Resizer *resizer() { return resizer_; }
  // Update the EvalContext pointer (called after copy() when new visitor's ctx is ready)
  void setEvalContext(EvalContext *ctx) override;

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
  void apply(const MoveOption &, PtGraph *,
             std::map<std::string, double> &) override {}
  std::unique_ptr<LrOperator> copy() const override;
};

// ─── CombinedOperator ────────────────────────────────────
class CombinedOperator : public LrOperator {
public:
  CombinedOperator(sta::dbSta *db_sta, LocalSta *local_sta,
                   rsz::Resizer *resizer, EvalContext *ctx);
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;
  void apply(const MoveOption &move, PtGraph *pt_graph,
             std::map<std::string, double> &runtime_map) override;
  std::unique_ptr<LrOperator> copy() const override;

  void setEquivCellArray(LibertyCellArray *array, PosMap *pos_map) override;
  void setInstInfoMap(std::unordered_map<sta::Instance*, LocalCellInfo*> *map) override;
  void setSlackMargin(float margin) override;
  void setEvalContext(EvalContext *ctx) override;
  LrRebufferV2 *rebuffer() { return buffer_op_ ? buffer_op_->rebuffer() : nullptr; }

private:
  // Two-phase buffering: try buffering on each candidate cell, return
  // the best buffer result. Returns MoveOption with type==NONE if no
  // buffering improvement found.
  MoveOption tryBufferingOnCandidates(
      PtGraph *pt_graph, sta::Instance *inst, EvalContext &ctx,
      const std::vector<MoveOption> &resize_candidates,
      sta::LibertyCell *ori_cell, float ori_cost);
  MoveOption tryBufferingOnTop1AndSmaller(
      PtGraph *pt_graph, sta::Instance *inst, EvalContext &ctx,
      const std::vector<MoveOption> &resize_candidates,
      sta::LibertyCell *ori_cell, float baseline_cost);

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
  // set average_delay/leakage, configure operator.
  void init(float average_delay, float average_power, float wns,
            float PT_tradeoff,
            std::unordered_map<sta::Instance*, LocalCellInfo*> *inst_info_map);

  virtual bool visit(sta::Instance *inst, sta::VertexId vid);
  bool singleGateSizing(sta::Instance *inst);
  void visitSlewOnly(sta::Instance *inst);
  virtual void applyChangesToDb(rsz::Resizer *resizer);
  virtual ParallelVisitor *copy() const;

  // Single operator slot — the visitor only calls evaluate/apply on this.
  void setOperator(std::unique_ptr<LrOperator> op) { operator_ = std::move(op); }

  void setTaskArranger(TaskArranger *ta) { task_arranger_ = ta; }
  PruningControl *pruningControl() const {
    return operator_ ? operator_->pruningControl() : nullptr;
  }

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
  PtGraph *ptGraph() const { return pt_graph_.get(); }
  sta::LibertyCell *bestCell() const { return best_move_.target_cell; }
  EvalContext &evalContext() { return eval_ctx_; }

  // Pruning stats (aggregated across threads)
  int resizeVisitCount() const { return resize_visit_count_; }
  int resizeChangeCount() const { return resize_change_count_; }

  // Profiling
  void printRuntimeProfile() const;
  const std::map<std::string, double> &runtimeMap() const { return runtime_map_; }

protected:
  sta::dbSta *db_sta_;
  LocalSta *local_sta_;
  rsz::Resizer *resizer_;
  std::unique_ptr<PtGraph> pt_graph_;
  EvalContext eval_ctx_;
  MoveOption best_move_;
  TaskArranger *task_arranger_ = nullptr;
  std::unique_ptr<LrOperator> operator_;
  std::vector<ResizeBenefit> *precheck_results_ = nullptr;
  int resize_visit_count_ = 0;
  int resize_change_count_ = 0;

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

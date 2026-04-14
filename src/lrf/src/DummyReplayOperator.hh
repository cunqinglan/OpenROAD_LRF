#pragma once

#include "NetlistTransformation.hh"

namespace lrf {

// Mock operator for isolating thread idle causes.
//
// Replaces ResizeOperator::evaluate() with a busy-spin (or sleep) of the
// recorded duration from a previous real run. Preserves:
//   - PtGraph construction (still in ParallelVisitor::visit())
//   - TaskArranger dispatch & conflict graph
//   - MEE locks
//   - applyDb serialization
//
// Returns MoveOption{NONE} so no DB mutations happen; apply() is a no-op.
// This isolates "framework overhead" from "actual computation variance".
class DummyReplayOperator : public ResizeOperator {
 public:
  using ResizeOperator::ResizeOperator;

  // Mock evaluate: look up vid -> eval_ns, busy-spin, return NONE.
  MoveOption evaluate(PtGraph *pt_graph, sta::Instance *inst,
                      EvalContext &ctx) override;

  // Apply is a no-op (no DB changes in replay mode).
  void apply(const MoveOption &, PtGraph *,
             std::map<std::string, double> &) override {}

  std::unique_ptr<LrOperator> copy() const override;

  // Set vid for this task (must be called before evaluate()).
  // Since evaluate()'s signature doesn't pass vid, we stash it via thread_local
  // set by ParallelVisitor::visit() — see TaskTrace.cc.
  static void setCurrentVid(uint32_t vid);

 private:
  static thread_local uint32_t current_vid_;
};

}  // namespace lrf

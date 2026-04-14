#include "DummyReplayOperator.hh"

#include "TaskTrace.hh"
#include "lrf/LrfClass.hh"

namespace lrf {

thread_local uint32_t DummyReplayOperator::current_vid_ = 0;

void DummyReplayOperator::setCurrentVid(uint32_t vid) {
  current_vid_ = vid;
}

MoveOption DummyReplayOperator::evaluate(PtGraph * /*pt_graph*/,
                                         sta::Instance * /*inst*/,
                                         EvalContext &ctx) {
  auto &collector = TaskTraceCollector::instance();
  uint64_t eval_ns = collector.lookupEvalNs(current_vid_);

  // Fallback: if vid not in replay table (new instance / missing record),
  // use a small default so we still exercise the dispatcher.
  if (eval_ns == 0) eval_ns = 1000;  // 1us

  uint64_t t0 = nowNs();
  if (collector.dummyMode() == TaskTraceCollector::BUSY_SPIN) {
    busySpinNs(eval_ns);
  } else {
    sleepNs(eval_ns);
  }
  uint64_t t1 = nowNs();

  // Report the mocked cost into the runtime_map so downstream profiling
  // still sees a populated equiv_cell_check bucket.
  if (ctx.runtime_map) {
    double sec = static_cast<double>(t1 - t0) * 1e-9;
    (*ctx.runtime_map)["equiv_cell_check"] += sec;
    (*ctx.runtime_map)["equiv_cell_count"] += 1.0;
  }

  // NONE -> caller skips apply(), no DB modification.
  return MoveOption{};
}

std::unique_ptr<LrOperator> DummyReplayOperator::copy() const {
  auto op = std::make_unique<DummyReplayOperator>(db_sta_, local_sta_);
  op->equiv_cell_array_ = equiv_cell_array_;
  op->equiv_cell_pos_map_ = equiv_cell_pos_map_;
  op->inst_info_map_ = inst_info_map_;
  op->slack_margin_ = slack_margin_;
  op->col_padding_ = col_padding_;
  op->row_padding_ = row_padding_;
  op->pruning_control_ = pruning_control_;
  return op;
}

}  // namespace lrf

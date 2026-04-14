#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace lrf {

struct TaskTraceRecord {
  uint32_t vid;         // VertexId (stable instance identifier)
  uint32_t tid;         // Worker thread index (0..N-1)
  uint64_t start_ns;    // visit() entry timestamp (steady_clock epoch)
  uint64_t end_ns;      // visit() exit timestamp
  uint64_t eval_ns;     // inner evaluate() duration (PtGraph build + equiv_cell_check)
  uint32_t iter_idx;    // LR iteration index (0-based)
  uint32_t sub_iter;    // sub-iteration within LR iter
};

// Singleton trace collector.
//
// Thread safety: append() takes a short mutex (fine-grained, called once per task).
// For ~100K tasks per sub-iter across 12 threads that's ~100K lock acquires,
// overhead negligible vs the ms-scale tasks themselves.
class TaskTraceCollector {
 public:
  enum Mode { OFF, RECORD, REPLAY };

  static TaskTraceCollector &instance();

  // Initialize from env vars:
  //   LRF_TRACE_DUMP=/path.csv    -> RECORD mode
  //   LRF_TRACE_REPLAY=/path.csv  -> REPLAY mode
  // Call once at start of each LR run.
  void initFromEnv();

  Mode mode() const { return mode_; }
  bool isRecord() const { return mode_ == RECORD; }
  bool isReplay() const { return mode_ == REPLAY; }

  // ----- Recording -----
  void append(const TaskTraceRecord &rec);
  void setIterIdx(uint32_t iter, uint32_t sub) { iter_idx_ = iter; sub_iter_ = sub; }
  uint32_t iterIdx() const { return iter_idx_; }
  uint32_t subIter() const { return sub_iter_; }

  // Dump all records to CSV at the configured path.
  // Called at IncreSta destruction or explicitly.
  void dumpCsv();

  // ----- Replay -----
  // Lookup recorded eval duration for this vid.
  // Returns 0 if not found (dummy op should then use a small default).
  uint64_t lookupEvalNs(uint32_t vid) const;
  size_t replayTableSize() const { return replay_table_.size(); }

  // Mode for dummy operator: busy-spin CPU or sleep (thread yields).
  enum DummyMode { BUSY_SPIN, SLEEP };
  DummyMode dummyMode() const { return dummy_mode_; }

  // Reset for a new run (keeps mode/paths).
  void resetRecords();

 private:
  TaskTraceCollector() = default;

  void loadReplayTable(const std::string &path);

  Mode mode_ = OFF;
  DummyMode dummy_mode_ = BUSY_SPIN;
  std::string dump_path_;
  std::string replay_path_;

  std::mutex mtx_;
  std::vector<TaskTraceRecord> records_;

  // vid -> average eval_ns (built from replay CSV at load time).
  // If a vid was visited multiple times, we average.
  std::unordered_map<uint32_t, uint64_t> replay_table_;

  uint32_t iter_idx_ = 0;
  uint32_t sub_iter_ = 0;
};

// Helpers

// High-resolution timestamp in ns since steady_clock epoch.
uint64_t nowNs();

// Busy-spin for exactly `ns` nanoseconds (CPU-bound wait, preserves cache contention).
void busySpinNs(uint64_t ns);

// Sleep for `ns` nanoseconds (yields CPU, simulates pure scheduling wait).
// Note: Linux sleep granularity is ~1us; avoid for ns-scale durations.
void sleepNs(uint64_t ns);

}  // namespace lrf

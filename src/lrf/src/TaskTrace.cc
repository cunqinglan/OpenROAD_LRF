#include "TaskTrace.hh"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <thread>

namespace lrf {

TaskTraceCollector &TaskTraceCollector::instance() {
  static TaskTraceCollector s;
  return s;
}

void TaskTraceCollector::initFromEnv() {
  // Idempotent: re-reading env is cheap; allows reinit per LR run.
  const char *dump = std::getenv("LRF_TRACE_DUMP");
  const char *replay = std::getenv("LRF_TRACE_REPLAY");
  const char *dmode = std::getenv("LRF_DUMMY_MODE");

  if (dmode && std::strcmp(dmode, "sleep") == 0) {
    dummy_mode_ = SLEEP;
  } else {
    dummy_mode_ = BUSY_SPIN;
  }

  if (replay && *replay) {
    mode_ = REPLAY;
    replay_path_ = replay;
    if (replay_table_.empty()) {
      loadReplayTable(replay_path_);
    }
    printf("[TaskTrace] REPLAY mode, table=%s entries=%zu dummy=%s\n",
           replay_path_.c_str(), replay_table_.size(),
           dummy_mode_ == BUSY_SPIN ? "busy_spin" : "sleep");
    fflush(stdout);
  } else if (dump && *dump) {
    mode_ = RECORD;
    dump_path_ = dump;
    printf("[TaskTrace] RECORD mode, output=%s\n", dump_path_.c_str());
    fflush(stdout);
  } else {
    mode_ = OFF;
  }
}

void TaskTraceCollector::append(const TaskTraceRecord &rec) {
  std::lock_guard<std::mutex> lock(mtx_);
  records_.push_back(rec);
}

void TaskTraceCollector::resetRecords() {
  std::lock_guard<std::mutex> lock(mtx_);
  records_.clear();
}

void TaskTraceCollector::dumpCsv() {
  std::lock_guard<std::mutex> lock(mtx_);
  if (dump_path_.empty() || records_.empty()) return;

  std::ofstream out(dump_path_);
  if (!out.is_open()) {
    fprintf(stderr, "[TaskTrace] ERROR: cannot open %s for writing\n",
            dump_path_.c_str());
    return;
  }

  out << "vid,tid,iter,sub_iter,start_ns,end_ns,eval_ns,wall_ns\n";
  for (const auto &r : records_) {
    out << r.vid << ',' << r.tid << ',' << r.iter_idx << ',' << r.sub_iter
        << ',' << r.start_ns << ',' << r.end_ns << ',' << r.eval_ns << ','
        << (r.end_ns - r.start_ns) << '\n';
  }
  printf("[TaskTrace] dumped %zu records to %s\n", records_.size(),
         dump_path_.c_str());
  fflush(stdout);
}

void TaskTraceCollector::loadReplayTable(const std::string &path) {
  std::ifstream in(path);
  if (!in.is_open()) {
    fprintf(stderr, "[TaskTrace] ERROR: cannot open replay table %s\n",
            path.c_str());
    return;
  }

  std::string line;
  std::getline(in, line);  // header

  // Accumulate sum + count per vid, then store average.
  std::unordered_map<uint32_t, uint64_t> sum;
  std::unordered_map<uint32_t, uint32_t> count;

  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    uint32_t vid = 0;
    uint64_t eval_ns = 0;
    int col = 0;
    while (std::getline(ss, tok, ',')) {
      switch (col) {
        case 0: vid = static_cast<uint32_t>(std::stoul(tok)); break;
        case 6: eval_ns = std::stoull(tok); break;  // eval_ns column
      }
      ++col;
    }
    sum[vid] += eval_ns;
    count[vid] += 1;
  }

  replay_table_.reserve(sum.size());
  for (const auto &kv : sum) {
    uint32_t vid = kv.first;
    uint64_t avg = kv.second / count[vid];
    replay_table_[vid] = avg;
  }
}

uint64_t TaskTraceCollector::lookupEvalNs(uint32_t vid) const {
  auto it = replay_table_.find(vid);
  return (it == replay_table_.end()) ? 0 : it->second;
}

// -------------------- Helpers --------------------

uint64_t nowNs() {
  using clk = std::chrono::steady_clock;
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             clk::now().time_since_epoch())
      .count();
}

void busySpinNs(uint64_t ns) {
  if (ns == 0) return;
  uint64_t start = nowNs();
  uint64_t deadline = start + ns;
  // Volatile accumulator prevents the compiler from eliminating the loop.
  volatile uint64_t sink = 0;
  while (nowNs() < deadline) {
    for (int i = 0; i < 32; ++i) sink += i;
  }
  (void)sink;
}

void sleepNs(uint64_t ns) {
  if (ns == 0) return;
  std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
}

}  // namespace lrf

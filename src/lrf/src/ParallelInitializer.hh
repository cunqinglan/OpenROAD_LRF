#pragma once

#include "db_sta/dbSta.hh"
#include "sta/MinMax.hh"
#include "sta/StaState.hh"
#include "sta/VertexVisitor.hh"

#include <mutex>
#include <vector>

namespace rsz {
class Resizer;
}

namespace odb {
class dbBlock;
}

namespace lrf {

class IncreSta;

/// Level-parallel Initializer using STA BfsIterator + VertexVisitor.
///
/// Same 4-step Sharma initialization as Initializer, but Step 2 and
/// Step 3 use BfsBkwdIterator / BfsFwdIterator with visitParallel()
/// to process all cells at the same topological level concurrently.
///
///   Step 1: Downsize all gates to min-leakage
///   Step 2: Fix cap violations PO→PI  (BfsBkwdIterator::visitParallel)
///   Step 3: Fix slew violations PI→PO (BfsFwdIterator::visitParallel)
///   Step 4: Fix remaining cap violations by buffer insertion
class ParallelInitializer : public sta::dbStaState
{
public:
  ParallelInitializer(sta::dbSta* sta, IncreSta* incre_sta,
                      rsz::Resizer* resizer, odb::dbBlock* block,
                      int thread_count,
                      bool minimize_leakage = true);
  ~ParallelInitializer();

  void run();

private:
  // Step 1: Assign every combinational gate to its min-leakage equiv cell.
  void downsizeToMinLeakage();

  // Step 2: Fix cap violations by upsizing (PO→PI).
  // Also converts slew limit to cap requirement: cell must satisfy
  // both maxcap AND slew-derived cap limit.
  // Uses BfsBkwdIterator::visitParallel for level-parallel traversal.
  void fixLoadViolationsParallel();

  // Step 3: Fix slew violations by upsizing (PI→PO).
  // Uses BfsFwdIterator::visitParallel for level-parallel traversal.
  // Multi-pass: repeat until convergence (max 5 passes).
  void fixSlewViolationsParallel();

  // Step 4: Fix remaining cap violations by buffer insertion.
  // Targets nets where the driver cell has no larger equivalent
  // (single-size family) and load_cap > max_capacitance.
  void fixCapByBuffering();

  // Count remaining cap and slew violations on output pins.
  void countViolations(int& cap_cnt, int& slew_cnt);

  // Estimate max output slew for a candidate port driving load_cap.
  // When inst is provided, uses actual input slew from graph;
  // otherwise falls back to a fixed 50ps estimate.
  float estimateMaxSlew(sta::LibertyPort* port, float load_cap,
                        const sta::Scene* scene, const sta::MinMax *min_max,
                        sta::Instance* inst = nullptr);

  IncreSta* incre_sta_;
  rsz::Resizer* resizer_;
  odb::dbBlock* block_;
  int thread_count_;
  sta::dbNetwork* db_network_ = nullptr;

  bool minimize_leakage_;

  // Cap/slew margins (percentage, same convention as RepairDesign).
  // max_cap_effective = max_cap * (1.0 - cap_margin_ / 100.0)
  float cap_margin_ = 5.0;   // 5% cap margin
  float slew_margin_ = 5.0;  // 5% slew margin

  // Shared mutex for replaceCell + parasitic/delay updates.
  std::mutex modify_mutex_;

  friend class FixLoadVisitor;
  friend class FixSlewVisitor;
};

} // namespace lrf

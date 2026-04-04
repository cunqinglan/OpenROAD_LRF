#pragma once

#include "db_sta/dbSta.hh"
#include "sta/MinMax.hh"
#include "sta/StaState.hh"

namespace rsz {
class Resizer;
}

namespace odb {
class dbBlock;
}

namespace lrf {

class LocalSta;

class Initializer : public sta::dbStaState
{
public:
  Initializer(sta::dbSta* sta, rsz::Resizer* resizer, odb::dbBlock* block);
  ~Initializer();

  // Sharma et al. three-step initialization:
  //   Step 1: Downsize all gates to min-leakage cell
  //   Step 2: Fix load+slew violations (reverse topo PO→PI)
  //   Step 3: Fix remaining slew violations (forward topo PI→PO)
  void run();

private:
  // Step 1: Assign every combinational gate to its min-leakage equiv cell.
  void downsizeToMinLeakage();

  // Step 2: Fix cap violations by upsizing (PO→PI).
  // Also converts slew limit to cap requirement: cell must satisfy
  // both maxcap AND slew-derived cap limit.
  void fixLoadViolations();

  // Step 3: Fix slew violations by upsizing (PI→PO).
  void fixSlewViolations();

  // Estimate max output slew for a candidate port driving load_cap.
  static float estimateMaxSlew(sta::LibertyPort* port, float load_cap,
                               const sta::DcalcAnalysisPt* dcalc_ap);

  rsz::Resizer* resizer_;
  odb::dbBlock* block_;
  LocalSta* local_sta_;
};

} // namespace lrf

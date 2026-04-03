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

  // Fix maxcap and maxslew violations by multi-pass upsizing.
  // Uses replaceCell + delaysInvalid for accurate incremental updates.
  // Uses fixed 50ps input slew for output slew estimation.
  void run();

private:
  rsz::Resizer* resizer_;
  odb::dbBlock* block_;
  LocalSta* local_sta_;
};

} // namespace lrf

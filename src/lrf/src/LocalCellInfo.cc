

#include "lrf/LrfClass.hh"

namespace lrf {

LocalCellInfo::~LocalCellInfo() {
  if (cell_leakages) {
    delete[] cell_leakages;
    cell_leakages = nullptr;
  }
}

} // namespace lrf
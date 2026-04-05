#pragma once

#include <vector>
#include "odb/db.h"

namespace lrf {

// Lightweight grid-based placement density map.
// Divides die area into uniform bins and computes cell-area density per bin.
// Thread-safe for read after build(); not thread-safe during build().
class PlacementDensityMap {
public:
  PlacementDensityMap() = default;

  // Build the density grid from current placement.
  // Bin size is derived automatically from average cell area:
  //   idealBinArea = avgCellArea / targetDensity
  //   binCnt = sqrt(dieArea / idealBinArea), rounded to power-of-2
  void build(odb::dbBlock* block);

  // Query normalized density at a physical location (DBU).
  // Returns instArea / binArea for the bin containing (x, y).
  float getDensity(int x, int y) const;

  // Query density at a dbInst's center location.
  float getDensity(odb::dbInst* inst) const;

  bool isBuilt() const { return !bins_.empty(); }
  int binCntX() const { return bin_cnt_x_; }
  int binCntY() const { return bin_cnt_y_; }
  double binSizeX() const { return bin_size_x_; }
  double binSizeY() const { return bin_size_y_; }

private:
  int lx_ = 0, ly_ = 0;
  int bin_cnt_x_ = 0, bin_cnt_y_ = 0;
  double bin_size_x_ = 0, bin_size_y_ = 0;  // DBU
  std::vector<float> bins_;  // density per bin, row-major [y * bin_cnt_x + x]
};

}  // namespace lrf

#include "PlacementDensityMap.hh"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace lrf {

void
PlacementDensityMap::build(odb::dbBlock* block)
{
  odb::Rect die = block->getDieArea();
  lx_ = die.xMin();
  ly_ = die.yMin();
  int die_dx = die.dx();
  int die_dy = die.dy();
  int dbu_per_micron = block->getDbUnitsPerMicron();

  // Compute average placed-instance area (in DBU^2).
  int64_t total_inst_area = 0;
  int inst_count = 0;
  for (odb::dbInst* inst : block->getInsts()) {
    if (!inst->isPlaced())
      continue;
    odb::dbBox* bbox = inst->getBBox();
    total_inst_area += static_cast<int64_t>(bbox->getDX()) * bbox->getDY();
    inst_count++;
  }
  if (inst_count == 0) {
    printf("PlacementDensityMap: no placed instances, skipping.\n");
    return;
  }

  // GPL-style adaptive bin count:
  //   idealBinArea = avgCellArea / targetDensity  (target ~0.7)
  //   idealBinCnt  = dieArea / idealBinArea
  //   then round to a power-of-2 for each axis, adjusted by aspect ratio.
  int64_t die_area = static_cast<int64_t>(die_dx) * die_dy;
  int64_t avg_inst_area = total_inst_area / inst_count;
  double target_density = 0.7;
  int64_t ideal_bin_area = std::max(
      static_cast<int64_t>(avg_inst_area / target_density),
      static_cast<int64_t>(1));
  int ideal_bin_cnt = std::max(static_cast<int>(die_area / ideal_bin_area), 4);

  // Find power-of-2 base bin count such that base^2 <= idealBinCnt.
  int base = 2;
  while (base * base * 4 <= ideal_bin_cnt)
    base *= 2;

  // Adjust for aspect ratio.
  double aspect = static_cast<double>(die_dx) / die_dy;
  if (aspect >= 1.0) {
    bin_cnt_x_ = static_cast<int>(std::round(base * aspect));
    bin_cnt_y_ = base;
  } else {
    bin_cnt_x_ = base;
    bin_cnt_y_ = static_cast<int>(std::round(base / aspect));
  }
  bin_cnt_x_ = std::max(bin_cnt_x_, 2);
  bin_cnt_y_ = std::max(bin_cnt_y_, 2);

  bin_size_x_ = static_cast<double>(die_dx) / bin_cnt_x_;
  bin_size_y_ = static_cast<double>(die_dy) / bin_cnt_y_;

  bins_.assign(bin_cnt_x_ * bin_cnt_y_, 0.0f);

  // Accumulate instance areas into bins by overlap.
  for (odb::dbInst* inst : block->getInsts()) {
    if (!inst->isPlaced())
      continue;
    odb::dbBox* bbox = inst->getBBox();
    int inst_lx = bbox->xMin();
    int inst_ly = bbox->yMin();
    int inst_ux = bbox->xMax();
    int inst_uy = bbox->yMax();

    // Find overlapping bin range.
    int bx0 = std::max(0, static_cast<int>((inst_lx - lx_) / bin_size_x_));
    int by0 = std::max(0, static_cast<int>((inst_ly - ly_) / bin_size_y_));
    int bx1 = std::min(bin_cnt_x_,
                       static_cast<int>(std::ceil((inst_ux - lx_) / bin_size_x_)));
    int by1 = std::min(bin_cnt_y_,
                       static_cast<int>(std::ceil((inst_uy - ly_) / bin_size_y_)));

    for (int by = by0; by < by1; by++) {
      double bin_ly_dbu = ly_ + by * bin_size_y_;
      double bin_uy_dbu = ly_ + (by + 1) * bin_size_y_;
      for (int bx = bx0; bx < bx1; bx++) {
        double bin_lx_dbu = lx_ + bx * bin_size_x_;
        double bin_ux_dbu = lx_ + (bx + 1) * bin_size_x_;

        // Rectangular overlap area.
        double ox0 = std::max(static_cast<double>(inst_lx), bin_lx_dbu);
        double oy0 = std::max(static_cast<double>(inst_ly), bin_ly_dbu);
        double ox1 = std::min(static_cast<double>(inst_ux), bin_ux_dbu);
        double oy1 = std::min(static_cast<double>(inst_uy), bin_uy_dbu);

        if (ox1 > ox0 && oy1 > oy0) {
          float overlap = static_cast<float>((ox1 - ox0) * (oy1 - oy0));
          bins_[by * bin_cnt_x_ + bx] += overlap;
        }
      }
    }
  }

  // Normalize: density = accumulated_area / bin_area.
  float bin_area = static_cast<float>(bin_size_x_ * bin_size_y_);
  if (bin_area > 0) {
    for (float& d : bins_) {
      d /= bin_area;
    }
  }

  printf("PlacementDensityMap: %d x %d bins (bin_size %.1f x %.1f um), "
         "%d instances, avg_inst_area %.1f um^2\n",
         bin_cnt_x_, bin_cnt_y_,
         bin_size_x_ / dbu_per_micron, bin_size_y_ / dbu_per_micron,
         inst_count,
         static_cast<double>(avg_inst_area) / dbu_per_micron / dbu_per_micron);
}

float
PlacementDensityMap::getDensity(int x, int y) const
{
  if (bins_.empty())
    return 0.0f;
  int bx = static_cast<int>((x - lx_) / bin_size_x_);
  int by = static_cast<int>((y - ly_) / bin_size_y_);
  bx = std::max(0, std::min(bx, bin_cnt_x_ - 1));
  by = std::max(0, std::min(by, bin_cnt_y_ - 1));
  return bins_[by * bin_cnt_x_ + bx];
}

float
PlacementDensityMap::getDensity(odb::dbInst* inst) const
{
  if (!inst)
    return 0.0f;
  int x, y;
  inst->getLocation(x, y);
  return getDensity(x, y);
}

}  // namespace lrf

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

  // Rebuild density values using existing bin grid (preserves bin_lambda_).
  // Must be called after build() has been called at least once.
  void rebuild(odb::dbBlock* block);

  // Query normalized density at a physical location (DBU).
  // Returns instArea / binArea for the bin containing (x, y).
  float getDensity(int x, int y) const;

  // Query density at a dbInst's center location.
  float getDensity(odb::dbInst* inst) const;

  // ── Per-bin Lagrange multiplier for density constraint ──

  // Initialize per-bin lambda.
  // headroom: threshold = max(density) × headroom. E.g. 1.05 = allow 5%
  //           above initial max density before penalizing.
  // lambda_init: starting value for all bins (must be > 0 for
  // multiplicative updates).
  void initLambda(float headroom = 1.05f, float lambda_init = 0.1f);

  // Multiplicative update (RapidLRHelper style):
  //   lambda[b] *= (density[b] / threshold)^k
  // Violated bins (density > threshold) → lambda grows exponentially.
  // Met bins (density < threshold) → lambda decays.
  void updateLambda();

  // O(1) incremental density update after a single cell swap.
  // delta_area = new_area - old_area  (in DBU^2).
  void incrementalUpdate(odb::dbInst* inst, int64_t delta_area_dbu2);

  // Query lambda for the bin containing inst.  Returns 0 if lambda
  // is not initialized (backward-compatible with old density_weight path).
  float getLambda(odb::dbInst* inst) const;
  float getLambda(int x, int y) const;

  // Whether per-bin lambda is active.
  bool hasLambda() const { return !bin_lambda_.empty(); }

  // Whether the bin containing (x, y) or inst has density > its threshold.
  bool isViolated(int x, int y) const;
  bool isViolated(odb::dbInst* inst) const;

  // Snapshot / restore for ECO revert.
  void snapshotState();
  void restoreState();

  void setK(int k) { k_ = k; }
  int getK() const { return k_; }

  bool isBuilt() const { return !bins_.empty(); }
  int binCntX() const { return bin_cnt_x_; }
  int binCntY() const { return bin_cnt_y_; }
  double binSizeX() const { return bin_size_x_; }
  double binSizeY() const { return bin_size_y_; }
  float threshold() const { return threshold_; }

private:
  int getBinIndex(int x, int y) const;

  int lx_ = 0, ly_ = 0;
  int bin_cnt_x_ = 0, bin_cnt_y_ = 0;
  double bin_size_x_ = 0, bin_size_y_ = 0;  // DBU
  std::vector<float> bins_;  // density per bin, row-major [y * bin_cnt_x + x]

  // Per-bin density Lagrange multipliers.
  std::vector<float> bin_lambda_;
  float threshold_ = 0.0f;    // global: max(initial_density) × headroom
  int k_ = 2;                 // exponent for multiplicative update
  static constexpr float LAMBDA_FLOOR = 1e-6f;

  // Snapshot for ECO revert.
  std::vector<float> snap_bins_;
  std::vector<float> snap_lambda_;
};

}  // namespace lrf

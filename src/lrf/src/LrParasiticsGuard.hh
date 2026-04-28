// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#pragma once

#include "est/EstimateParasitics.h"
#include "grt/GlobalRouter.h"

namespace lrf {

// RAII guard for the LR-flavored incremental-parasitics scope used by the
// LR ISTA loop.
//
// Why not reuse est::IncrementalParasiticsGuard?
//   Its dtor calls updateParasitics(), which routes through the delete-
//   network path (estimateGlobalRouteRC / estimateWireParasitics) and
//   tears down the per-net ParasiticNetworks the LR loop builds via
//   updateWireParasiticsNoDeleteNetwork*. We want the same callback +
//   IncrementalGRoute lifecycle but no dtor-time updateParasitics().
//
// Reentrancy: if an outer guard already owns the session
// (isIncrementalParasiticsEnabled() == true), this guard no-ops so it
// never yanks state out from under the outer owner.
class LrParasiticsGuard
{
 public:
  explicit LrParasiticsGuard(est::EstimateParasitics* est) : est_(est)
  {
    if (est_->isIncrementalParasiticsEnabled())
      return;

    switch (est_->getParasiticsSrc()) {
      case est::ParasiticsSrc::global_routing:
      case est::ParasiticsSrc::detailed_routing:
        est_->setIncrementalGRT(
            new grt::IncrementalGRoute(est_->getGlobalRouter(),
                                       est_->getBlock()));
        est_->getGlobalRouter()->setVerbose(false);
        break;
      case est::ParasiticsSrc::placement:
      case est::ParasiticsSrc::none:
        break;
    }
    est_->setIncrementalParasiticsEnabled(true);
    est_->setDbCbkOwner(est_->getBlock());
    owns_ = true;
  }

  ~LrParasiticsGuard()
  {
    if (!owns_)
      return;
    est_->removeDbCbkOwner();
    if (auto* g = est_->getIncrementalGRT()) {
      delete g;
      est_->setIncrementalGRT(nullptr);
    }
    est_->setIncrementalParasiticsEnabled(false);
  }

  LrParasiticsGuard(const LrParasiticsGuard&) = delete;
  LrParasiticsGuard& operator=(const LrParasiticsGuard&) = delete;

 private:
  est::EstimateParasitics* est_;
  bool owns_ = false;
};

}  // namespace lrf

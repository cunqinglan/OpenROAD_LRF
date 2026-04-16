// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2026, The OpenROAD Authors

#include "lrf/LrConfig.hh"

namespace lrf {

LrConfig& getConfig()
{
  static LrConfig instance;
  return instance;
}

}  // namespace lrf

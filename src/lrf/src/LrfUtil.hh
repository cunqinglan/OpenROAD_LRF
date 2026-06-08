
#pragma once

#include <cstdlib>


namespace sta {
class LibertyCell;
class Sdc;
class Network;
} // namespace sta

namespace lrf {

// Verbose switch for LR debug prints. Controlled by the LRF_VERBOSE
// environment variable (set it to anything to enable), read once on first
// use. Lives here (not on a class) so any LR file can gate diagnostics with
// `if (lrf::lrfVerbose())` without coupling to an unrelated owner such as
// TaskArranger. The static-local init is thread-safe (C++11 magic statics).
inline bool lrfVerbose() {
  static const bool v = (std::getenv("LRF_VERBOSE") != nullptr);
  return v;
}

// Prune illegal equivalent cells and get the violation type.
// Return true if cell are legal
bool isLegalEquivCells(sta::LibertyCell *ori_cell, sta::Sdc *sdc,
                       bool slew_violated, bool cap_violated);

} // namespace lrf
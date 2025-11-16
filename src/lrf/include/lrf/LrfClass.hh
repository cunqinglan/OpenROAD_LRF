#pragma once

#include <vector>

#include "sta/Delay.hh" // ArcDelay, Slew, Arrival typedefs
#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/LibertyClass.hh"

namespace sta {
  class StaState;
  class Sta;
} // namespace sta

namespace lrf {

// Keep LM value typedefs local to lrf. Use fully qualified sta:: types elsewhere instead of using declarations
typedef float LMValue;
typedef double LMValueDBL;
typedef std::vector<LMValue> LMValueSeq;

using sta::ArcDelay;
using sta::DcalcAPIndex;
using sta::Edge;
using sta::EdgeId;
using sta::Graph;
using sta::Instance;
using sta::InstanceSet;
using sta::Level;
using sta::MinMax;
using sta::RiseFall;
using sta::VertexId;
using sta::Vertex;
using sta::StaState;
using sta::Sta;
} // namespace lrf


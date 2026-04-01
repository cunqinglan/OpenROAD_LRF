#pragma once

#include <string>
#include <vector>
#include "lrf/LrfClass.hh"
#include "../../rsz/src/Rebuffer.hh"

namespace rsz {
class Resizer;
class BufferedNet;
using BufferedNetPtr = std::shared_ptr<BufferedNet>;
using BufferedNetSeq = std::vector<BufferedNetPtr>;
}

namespace lrf {

struct VirtualBufferInfo {
  std::vector<sta::VertexId> vertex_ids;
  std::vector<sta::EdgeId> edge_ids;
  std::vector<sta::EdgeId> orig_wire_edge_ids;
  bool failed = false;
};

} // namespace lrf

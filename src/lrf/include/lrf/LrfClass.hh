#pragma once

#include <vector>

#include "sta/Delay.hh" // ArcDelay, Slew, Arrival typedefs
#include "sta/GraphClass.hh"
#include "sta/NetworkClass.hh"
#include "sta/LibertyClass.hh"

#include <vector>
#include <map>
#include <unordered_map>
#include <tuple>

#include "sta/Graph.hh"
#include "sta/TimingArc.hh"
#include "sta/Map.hh"

namespace sta {
  class StaState;
  class Sta;
  class LibertyCell;
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

class PtEdge;
class PtVertex;
class DelayLmSumResult;

typedef std::vector<PtEdge> PtEdgeSeq;
typedef std::vector<PtVertex> PtVertexSeq;
typedef std::map<const sta::Vertex*, sta::VertexId> VertexPtToIdMap;
typedef std::vector<std::vector<sta::LibertyCell*>> LibertyCellArray;

struct CellArrayPos {
  int row;
  int col;
  int group_start;  // first global row of this equiv group (inclusive)
  int group_end;    // last global row of this equiv group (exclusive)
};
typedef std::unordered_map<sta::LibertyCell*, CellArrayPos> PosMap;

enum class MoveType {
  Resizing,
  BufferInsertion,
  None
};

enum class PtVertexType : uint8_t {
  Sentinel,     // index-0 null sentinel (placeholder, always skipped)
  RefDriver,    // vertices that drive the reference instance, should update parasitics of the these vertices
  RefInput,     // fanin vertices of the reference instance
  RefOutput,    // fanout vertices of the reference instance
  VirtualInput, // virtual device input pin (no base sta::Vertex)
  VirtualOutput,// virtual device output pin (no base sta::Vertex)
  Sibling,      // sibling load/driver on shared fanin net (second-order timing impact)
  None
};

enum class PtEdgeType : uint8_t {
  Sentinel,        // index-0 null sentinel (placeholder, always skipped)
  RefInstEdge,     // edges that belong to the reference instance
  VirtualGateEdge, // virtual device gate edge (no base sta::Edge)
  VirtualWireEdge, // wire edge to/from virtual device (no base sta::Edge)
  SiblingEdge,     // gate edge between sibling vertices on fanin side
  None
};

enum class PinType : uint8_t {
  NONE,
  DRIVER,
  LOAD,
  INPUT,
  OUTPUT,
  BIDIRECT
};

struct DelayLmSumResult {
  float delay_lm_sum = 1000000000.0;
  std::vector<float> vec_lms;
  std::vector<float> vec_delays;
};

inline size_t lmIndex(const sta::TimingArc *timing_arc,
                 sta::DcalcAPIndex ap_index,
                 size_t ap_count)
{
  size_t index = timing_arc->index() * ap_count + ap_index;
  return index;
}


enum class TimingType {
  EDGE,
  VERTEX,
  UNSET
};

struct TimingInfo {
  TimingType type = TimingType::UNSET;
  std::vector<sta::Slew> slews;
  std::vector<sta::Path> paths;
  std::vector<sta::Delay> delays;
  sta::TagGroupIndex tag_group_index = sta::tag_group_index_max;
  float total_cap = -1.0;
};

struct GraphTiming {
  sta::LibertyCell *cell = nullptr;
  std::unordered_map<std::string, TimingInfo> vertex_timing_map;
  std::unordered_map<std::string, TimingInfo> edge_timing_map;
};
  
struct TimingRecord {
  sta::Instance *inst = nullptr;
  sta::LibertyCell *orig_cell = nullptr;
  std::unordered_map<std::string, GraphTiming> liberty_timing_map;
};

class LocalCellInfo {
public:
  ~LocalCellInfo();
  sta::LibertyCellSeq *equiv_cells = nullptr;
  float *cell_leakages = nullptr;
};

class ParallelLocalCellInfo {
public:
  std::vector<sta::LibertyCellSeq> *equiv_cells = nullptr;
  std::vector<std::vector<float>> cell_leakages;
};

struct ResizeBenefit {
  sta::Instance* inst;
  float cost_change;    // original_cost - best_cost (positive = beneficial)
  size_t vertex_idx;    // index into TaskArranger::vertices_ for direct access
};

} // namespace lrf


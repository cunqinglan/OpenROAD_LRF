
#pragma once

#include <vector>
#include <map>

#include "sta/Graph.hh"
#include "sta/Delay.hh"
#include "sta/TimingArc.hh"
#include "sta/Map.hh"
#include "Delay.hh"

namespace lrf {

class PtEdge;
class PtVertex;
class DelayLmSumResult;

typedef std::vector<PtEdge> PtEdgeSeq;
typedef std::vector<PtVertex> PtVertexSeq;
typedef std::map<const sta::Vertex*, sta::VertexId> VertexPtToIdMap;


enum class PtVertexType : uint8_t {
  RefDriver, // vertices that drive the reference instance, should update parasitics of the these vertices
  RefInput,  // fanin vertices of the reference instance
  RefOutput, // fanout vertices of the reference instance
  None
};

enum class PtEdgeType : uint8_t {
  RefInstEdge, // edges that belong to the reference instance
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
  float delay_lm_sum;
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

}
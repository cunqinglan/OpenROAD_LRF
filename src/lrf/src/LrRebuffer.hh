
#include <string>
#include <vector>
#include "lrf/LrfClass.hh"
#include "../../rsz/src/Rebuffer.hh"
// #include "LocalSta.hh"

namespace rsz {
class Resizer;
class BufferedNet;
using BufferedNetPtr = std::shared_ptr<BufferedNet>;
using BufferedNetSeq = std::vector<BufferedNetPtr>;
}

namespace utl {

}


namespace lrf {

class ParallelLrVisitor;
class PtGraph;
class LocalSta;
class TestLrf;

struct VirtualBufferInfo {
  std::vector<sta::VertexId> vertex_ids;
  std::vector<sta::EdgeId> edge_ids;
  std::vector<sta::EdgeId> orig_wire_edge_ids;
  bool failed = false;
};

class LrRebuffer : public rsz::Rebuffer
{
  friend class TestLrf;
public:
  LrRebuffer(rsz::Resizer* resizer, ParallelLrVisitor* parallel_visitor);
  // Call once in serial before creating any LrRebuffer instances in parallel.
  static void initGlobalPreamble(sta::dbSta* sta, rsz::Resizer* resizer);
  void init();
  // Compute the best buffering option and save it at best_bnet_.
  void rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);
  // void annoataLoadSlacks();
  rsz::BufferedNetPtr bufferForTiming(sta::VertexId drvr_vertex_id, const rsz::BufferedNetPtr& tree, bool allow_topology_rewrite);
  void annotateLoadLMs(PtVertex &drvr_pt_vertex, const rsz::BufferedNetPtr& tree);
  void insertBufferOptions(rsz::BufferedNetSeq& opts,
                           int level,
                           int next_segment_wl = 0);
  rsz::BufferedNetPtr addWire(const rsz::BufferedNetPtr& p,
                       odb::Point wire_end,
                       int wire_layer,
                       int level = -1);
  int applyBufferingToDb();
  const sta::Pin *drvrPin() const { return drvr_pin_; }
  const rsz::BufferedNetPtr& bestBnet() const { return best_bnet_; }

protected:
  void localAnnotateLoadSlacks(const rsz::BufferedNetPtr& tree, PtVertex &drvr_pt_vertex);

  // Cost computation: delay_LM_sum + leakage
  float computeBufferAddedCost(float buffer_delay_seconds,
                                float buffer_leakage,
                                const rsz::BufferedNetPtr& load_opt);
  void propagateLmsThroughBuffer(rsz::BufferedNetPtr& buffer_node,
                                 const rsz::BufferedNetPtr& load_opt);
  std::vector<float> mergeLmVectors(const std::vector<float>& lm1,
                                    const std::vector<float>& lm2);
  LMValue evaluateOption(sta::VertexId pt_vertex_id, const rsz::BufferedNetPtr& option,
                       float original_slack);
  float cellDelayLmSum(sta::VertexId pt_vertex_id,
                       const rsz::BufferedNetPtr& load_opt,
                       sta::Slew &max_slew);
  bool hasViolation(const rsz::BufferedNetPtr& option, sta::Slew max_slew);
  VirtualBufferInfo buildVirtualBuffer(sta::VertexId drvr_vertex_id,
                                       const rsz::BufferedNetPtr& option);
  void removeVirtualBuffer(VirtualBufferInfo &info);
  float computeVirtualSlack(const VirtualBufferInfo &info);
  rsz::BufferedNetPtr attemptTopologyRewrite(const rsz::BufferedNetPtr& node,
                                             const rsz::BufferedNetPtr& left,
                                             const rsz::BufferedNetPtr& right,
                                             float best_cap);
  int bufferNum(const rsz::BufferedNetPtr& tree);
private:
  LocalSta *local_sta_;
  ParallelLrVisitor* visitor_;
  const sta::Pin *drvr_pin_ = nullptr;
  rsz::BufferedNetPtr best_bnet_ = nullptr;
  bool verbose_ = false;
};




} // namespace lrf

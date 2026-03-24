// LrRebufferV2: clean copy of LrRebuffer using EvalContext instead of ParallelLrVisitor.
// No dependency on old visitor architecture.
#pragma once

#include <string>
#include <vector>
#include "lrf/LrfClass.hh"
#include "../../rsz/src/Rebuffer.hh"
#include "LrRebuffer.hh"  // for VirtualBufferInfo

namespace lrf {

struct EvalContext;
class PtGraph;
class LocalSta;

class LrRebufferV2 : public rsz::Rebuffer
{
public:
  LrRebufferV2(rsz::Resizer* resizer, LocalSta* local_sta, EvalContext* eval_ctx);
  static void initGlobalPreamble(sta::dbSta* sta, rsz::Resizer* resizer);
  void init();
  void rebufferPin(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);
  rsz::BufferedNetPtr bufferForTiming(sta::VertexId drvr_vertex_id, const rsz::BufferedNetPtr& tree, bool allow_topology_rewrite, bool last_iteration);
  void annotateLoadLMs(PtVertex &drvr_pt_vertex, const rsz::BufferedNetPtr& tree);
  void insertBufferOptions(rsz::BufferedNetSeq& opts, int level, int next_segment_wl = 0);
  rsz::BufferedNetPtr addWire(const rsz::BufferedNetPtr& p, odb::Point wire_end, int wire_layer, int level = -1);
  int applyBufferingToDb();
  const sta::Pin *drvrPin() const { return drvr_pin_; }
  const rsz::BufferedNetPtr& bestBnet() const { return best_bnet_; }
  float bestCost() const { return best_cost_; }
  rsz::Resizer *resizer() const { return resizer_; }

  float computeNetSensitivity(const sta::Pin *drvr_pin,
                              PtVertex &drvr_pt_vertex,
                              float avg_delay, float avg_leakage);

protected:
  void localAnnotateLoadSlacks(const rsz::BufferedNetPtr& tree, PtVertex &drvr_pt_vertex);
  float computeBufferAddedCost(float buffer_delay_seconds,
                                float buffer_leakage,
                                const rsz::BufferedNetPtr& load_opt);
  void propagateLmsThroughBuffer(rsz::BufferedNetPtr& buffer_node,
                                 const rsz::BufferedNetPtr& load_opt);
  std::vector<float> mergeLmVectors(const std::vector<float>& lm1,
                                    const std::vector<float>& lm2);
  LMValue evaluateOptionCoarse(sta::VertexId pt_vertex_id, const rsz::BufferedNetPtr& option);
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
  void writeLmsToGraph();
  void writeTimingToGraph();
  void initNewStaVertexPaths(const PtVertex &pt_vertex, sta::Vertex *sta_vertex);
  void buildVirtualParasitics(VertexId drvr_vertex_id,
                              const rsz::BufferedNetPtr& option,
                              const VirtualBufferInfo &vinfo);
  void buildSyntheticParasitics(VertexId drvr_vertex_id,
                                const rsz::BufferedNetPtr& option,
                                const VirtualBufferInfo &vinfo);

private:
  LocalSta *local_sta_;
  EvalContext *eval_ctx_;
  const sta::Pin *drvr_pin_ = nullptr;
  rsz::BufferedNetPtr best_bnet_ = nullptr;
  float best_cost_ = std::numeric_limits<float>::max();
  VirtualBufferInfo best_vinfo_;
  bool verbose_ = true;
};

} // namespace lrf

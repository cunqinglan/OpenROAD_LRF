// TestRebuffer: subclass of LrRebuffer for buffer probing and comparison.
// Inherits all Rebuffer + LrRebuffer protected methods, enabling direct
// access to RSZ and LRF bufferForTiming, virtual buffer, etc.
#pragma once

#include "LrRebuffer.hh"

namespace odb {
class dbBlock;
}

namespace lrf {

class TestRebuffer : public LrRebuffer
{
public:
  using LrRebuffer::LrRebuffer;  // inherit constructor

  struct GlobalBaseline {
    double wns, tns, worst_sink, sum_sink;
    std::vector<sta::Vertex*> orig_sink_vertices;  // pre-captured before buffer
  };

  // ── Verified probe: one method, local + global ──
  // method: 0=RSZ, 1=LRF-worst, 2=LRF-sum
  // Local: virtual buffer → local worst/sum slack delta
  // Global: applyBufferingToDb → estimate_parasitics → global timing → revert
  void rebufferPinVG(const sta::Pin *drvr_pin, sta::Instance *inst,
                     odb::dbBlock *block, int method,
                     const GlobalBaseline &baseline);

  // ── RSZ vs LRF local-only comparison (diagnostic, no DB modification) ──
  void probeRszBnetWithLocalEval(const sta::Pin *drvr_pin, PtVertex &drvr_pt_vertex);

  // ── Enumerate ALL LRF bnet options for one pin ──
  // For each option: local worst/sum delta + global worst_sink/sum_sink/WNS/TNS delta
  void probeAllOptions(const sta::Pin *drvr_pin, sta::Instance *inst,
                       odb::dbBlock *block, const GlobalBaseline &baseline);
};

}  // namespace lrf

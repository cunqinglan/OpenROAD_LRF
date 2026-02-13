


#include "Rebuffer.hh"
// #include "LocalSta.hh"



namespace lrf {

class LrRebuffer : public rsz::Rebuffer
{
public:
  LrRebuffer(rsz::Resizer* resizer, ParallelLrVisitor* parallel_visitor);
  void init();
  int rebufferPin(const sta::Pin *drvr_pin, PtVertex *drvr_pt_vertex, PtGraph *pt_graph);  // Return the inserted buffer count.
  // void annoataLoadSlacks();
  rsz::BnetPtr bufferForTiming(const rsz::BnetPtr& tree, bool allow_topology_rewrite);
  void annotateLoadLMs(PtVertex &drvr_pt_vertex, PtGraph *pt_graph, sta::Vertex *root_vertex, const rsz::BnetPtr& tree);
  void insertBufferOptions(rsz::BnetSeq& opts,
                           int level,
                           int next_segment_wl = 0);
  rsz::BnetPtr addWire(const rsz::BnetPtr& p,
                       odb::Point wire_end,
                       int wire_layer,
                       int level = -1);

protected:
  void localAnnotateLoadSlacks(const rsz::BnetPtr& tree, PtVertex *drvr_pt_vertex, PtGraph *pt_graph);
  
  // LM sum computation functions
  float computeBufferAddedLmSum(sta::LibertyCell* buffer_cell, 
                                const rsz::BnetPtr& load_opt,
                                const FixedDelay& buffer_delay);
  void propagateLmsThroughBuffer(rsz::BnetPtr& buffer_node,
                                 sta::LibertyCell* buffer_cell,
                                 const rsz::BnetPtr& load_opt);
  std::vector<float> mergeLmVectors(const std::vector<float>& lm1, 
                                    const std::vector<float>& lm2);

private:
  LocalSta *local_sta_;
  ParallelLrVisitor* visitor_;
};




} // namespace lrf
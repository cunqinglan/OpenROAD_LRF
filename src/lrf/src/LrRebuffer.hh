


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
  LMValue *mergeWireLMVec(const LMValue *lm_vec1, const LMValue *lm_vec2);
  void clear();

protected:
  void localAnnotateLoadSlacks(const rsz::BnetPtr& tree, PtVertex *drvr_pt_vertex, PtGraph *pt_graph);

private:
  LocalSta *local_sta_;
  ParallelLrVisitor* visitor_;
  std::unordered_map<rsz::BnetPtr, LMValue*> bnet_lm_map_;
};




} // namespace lrf
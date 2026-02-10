


#include "Rebuffer.hh"
// #include "LocalSta.hh"



namespace lrf {

class LrRebuffer : public rsz::Rebuffer
{
public:
  LrRebuffer(rsz::Resizer* resizer, LocalSta * local_sta);

  int rebufferPin(const sta::Pin *drvr_pin);
  void annoataLoadSlacks();
  BnetPtr bufferForTiming(const BnetPtr& tree, bool allow_topology_rewrite);

private:
  LocalSta * local_sta_;
};




} // namespace lrf
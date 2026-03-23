#include "PtPiElmore.hh"

namespace lrf {

PtPiElmore::PtPiElmore()
  : ConcretePi(0.0f, 0.0f, 0.0f),
    ConcreteParasitic()
{
}

PtPiElmore::PtPiElmore(float c2, float rpi, float c1)
  : ConcretePi(c2, rpi, c1),
    ConcreteParasitic()
{
}

float
PtPiElmore::capacitance() const
{
  return c1_ + c2_;
}

void
PtPiElmore::piModel(float &c2, float &rpi, float &c1) const
{
  c2 = c2_;
  rpi = rpi_;
  c1 = c1_;
}

void
PtPiElmore::setPiModel(float c2, float rpi, float c1)
{
  c2_ = c2;
  rpi_ = rpi;
  c1_ = c1;
}

void
PtPiElmore::findElmore(const Pin *load_pin,
                        float &elmore, bool &exists) const
{
  exists = false;
  elmore = 0.0f;
  if (!load_pin) return;
  auto it = pin_index_.find(load_pin);
  if (it != pin_index_.end()) {
    elmore = loads_[it->second].elmore;
    exists = true;
  }
}

void
PtPiElmore::setElmore(const Pin *load_pin, float elmore)
{
  if (!load_pin) return;
  auto it = pin_index_.find(load_pin);
  if (it != pin_index_.end()) {
    loads_[it->second].elmore = elmore;
  }
}

sta::PinSet
PtPiElmore::unannotatedLoads(const Pin *, const Parasitics *) const
{
  return sta::PinSet();
}

float
PtPiElmore::findElmoreByVertexId(VertexId vid, bool &exists) const
{
  exists = false;
  auto it = vid_index_.find(vid);
  if (it != vid_index_.end()) {
    exists = true;
    return loads_[it->second].elmore;
  }
  return 0.0f;
}

void
PtPiElmore::setElmoreByVertexId(VertexId vid, float elmore)
{
  auto it = vid_index_.find(vid);
  if (it != vid_index_.end()) {
    loads_[it->second].elmore = elmore;
  }
}

void
PtPiElmore::addLoad(VertexId vid, const Pin *pin, float elmore)
{
  size_t idx = loads_.size();
  loads_.push_back({vid, pin, elmore});
  if (pin)
    pin_index_[pin] = idx;
  vid_index_[vid] = idx;
}

void
PtPiElmore::clear()
{
  loads_.clear();
  pin_index_.clear();
  vid_index_.clear();
  c2_ = rpi_ = c1_ = 0.0f;
}

} // namespace lrf

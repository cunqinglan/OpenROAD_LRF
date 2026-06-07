#pragma once

#include <vector>
#include <unordered_map>

#include "parasitics/ConcreteParasiticsPvt.hh"
#include "sta/GraphClass.hh"

namespace lrf {

using sta::VertexId;
using sta::Pin;
using sta::PinSet;
using sta::Parasitics;
using sta::ConcreteParasitic;
using sta::ConcretePi;

struct PtElmoreLoad {
  VertexId vertex_id;
  const Pin *pin;    // nullptr for virtual loads
  float elmore;
};

// PtGraph-local PiElmore parasitic model.
// Inherits ConcretePi + ConcreteParasitic so it can be passed as Parasitic*
// to DmpCeff gateDelay, enabling full Ceff iteration + Elmore wire delays.
class PtPiElmore : public ConcretePi, public ConcreteParasitic
{
public:
  PtPiElmore();
  PtPiElmore(float c2, float rpi, float c1);

  // ConcreteParasitic virtual interface (used by DmpCeff via static_cast)
  bool isPiElmore() const override { return true; }
  bool isPiModel() const override { return true; }
  float capacitance() const override;
  void piModel(float &c2, float &rpi, float &c1) const override;
  void setPiModel(float c2, float rpi, float c1) override;
  bool isReducedParasiticNetwork() const override { return true; }
  void setIsReduced(bool) override {}

  // Pin*-based Elmore lookup (called by DmpCeff for real loads)
  void findElmore(const Pin *load_pin,
                  float &elmore, bool &exists) const override;
  void setElmore(const Pin *load_pin, float elmore) override;
  PinSet unannotatedLoads(const Pin *drvr_pin,
                          const Parasitics *parasitics) const override;

  // VertexId-based Elmore lookup (for virtual loads without Pin*)
  float findElmoreByVertexId(VertexId vid, bool &exists) const;
  void setElmoreByVertexId(VertexId vid, float elmore);

  // Add a load with both Pin* and VertexId indexing
  void addLoad(VertexId vid, const Pin *pin, float elmore);

  void clear();
  size_t loadCount() const { return loads_.size(); }
  const std::vector<PtElmoreLoad>& loads() const { return loads_; }

private:
  std::vector<PtElmoreLoad> loads_;
  std::unordered_map<const Pin*, size_t> pin_index_;
  std::unordered_map<VertexId, size_t> vid_index_;
};

} // namespace lrf



#include "LocalSta.hh"
#include "sta/ClkNetwork.hh"
#include "sta/Sdc.hh"
#include "sta/PortDirection.hh"
#include "sta/InputDrive.hh"
#include "sta/Fuzzy.hh"

namespace lrf {

void
LocalSta::checkSlew(const sta::Pin *pin,
                    const sta::LibertyCell *lib_cell,
                    const sta::Corner * corner,
                    const sta::MinMax *min_max,
                    bool check_clks,
                    PtGraph *pt_graph,
                    // retrun values
                    const sta::Corner *&corner1,
                    const sta::RiseFall *&rf1,
                    float &slew1,
                    float &limit1,
                    float &slack1) const
{
  corner1 = nullptr;
  rf1 = nullptr;
  slew1 = 0.0;
  limit1 = 0.0;
  slack1 = sta::MinMax::min()->initValue();

  Vertex *vertex, *bidirect_vertex;
  graph_->pinVertices(pin, vertex, bidirect_vertex);
  if (vertex) {
    checkSlew1(pin, vertex, lib_cell, corner, min_max, check_clks,
      pt_graph, corner1, rf1, slew1, limit1, slack1);
  }
}

ClockSet
LocalSta::clockDomains(const sta::Vertex *vertex) const
{
  ClockSet clks;
  VertexPathIterator path_iter(const_cast<Vertex*>(vertex), sta_);
  while (path_iter.hasNext()) {
    Path *path = path_iter.next();
    const Clock *clk = path->clock(sta_);
    if (clk)
      clks.insert(const_cast<Clock*>(clk));
  }
  return clks;
}

void
LocalSta::checkSlew1(const sta::Pin *pin,
                    Vertex *vertex,
                    const sta::LibertyCell *lib_cell,
                    const sta::Corner * corner,
                    const sta::MinMax *min_max,
                    bool check_clks,
                    PtGraph *pt_graph,
                    // retrun values
                    const sta::Corner *&corner1,
                    const sta::RiseFall *&rf1,
                    float &slew1,
                    float &limit1,
                    float &slack1) const
{
  if (!vertex->isDisabledConstraint()
      && !vertex->isConstant()
      && !sta_->clkNetwork()->isIdealClock(pin)) {
    ClockSet clks;
    if (check_clks)
      clks = clockDomains(vertex);
    if (corner)
      checkSlew2(pin, vertex, lib_cell, corner, min_max, clks, pt_graph,
                 corner1, rf1, slew1, limit1, slack1);
    else {
      for (auto corner : *sta_->corners()) {
        checkSlew2(pin, vertex, lib_cell, corner, min_max, clks, pt_graph,
                   corner1, rf1, slew1, limit1, slack1);
      }
    }
  }
}

void
LocalSta::checkSlew2(const sta::Pin *pin,
                    Vertex *vertex,
                    const sta::LibertyCell *lib_cell,
                    const sta::Corner * corner,
                    const sta::MinMax *min_max,
                    const ClockSet &clks,
                    PtGraph *pt_graph,
                    // retrun values
                    const sta::Corner *&corner1,
                    const sta::RiseFall *&rf1,
                    float &slew1,
                    float &limit1,
                    float &slack1) const
{
  for (auto rf : RiseFall::range()) {
    float limit;
    bool exists;
    localFindSlewLimit(pin, lib_cell, corner, min_max, rf, clks, limit, exists);
    if (exists) {
      checkSlew3(pin, vertex, lib_cell, corner, rf, min_max, limit, pt_graph,
                 corner1, rf1, slew1, slack1, limit1);
    }
  }
}

void
LocalSta::localFindSlewLimit(const sta::LibertyPort *lib_port,
                const sta::Corner *corner,
                const sta::MinMax *min_max,
                // Return values
                float &limit,
                bool &exists) const
{
  limit = INF;
  exists = false;

  const Network *network = network_;
  Sdc *sdc = sdc_;
  float limit1;
  bool exists1;

  // Default to top ("design") limit.
  Cell *top_cell = network->cell(network->topInstance());
  sdc->slewLimit(top_cell, min_max,
		 limit1, exists1);
  if (exists1) {
    limit = limit1;
    exists = true;
  }

  if (lib_port) {
    const LibertyPort *corner_port = lib_port->cornerPort(corner, min_max);
    corner_port->slewLimit(min_max, limit1, exists1);
    if (!exists1
        // default_max_transition only applies to outputs.
        && corner_port->direction()->isAnyOutput()
        && min_max == MinMax::max())
      corner_port->libertyLibrary()->defaultMaxSlew(limit1, exists1);
    if (exists1
        && (!exists
            || min_max->compare(limit, limit1))) {
      limit = limit1;
      exists = true;
    }
  }
}

void
LocalSta::localFindSlewLimit(const sta::Pin *pin,
                const sta::LibertyCell *lib_cell,
                const sta::Corner *corner,
                const sta::MinMax *min_max,
                const sta::RiseFall *rf,
                const ClockSet &clks,
                // Return values
                float &limit,
                bool &exists) const
{
  sta::Sdc *sdc = sdc_;
  sta::Network *network = network_;
  LibertyPort *lib_port = lib_cell->findLibertyPort(network_->portName(pin));
  if (!lib_port) {
    printf("ERROR: no port found for pin %s\n", network_->pathName(pin));
    throw std::runtime_error("no port found for pin");
  }
  localFindSlewLimit(lib_port, corner, min_max, limit, exists);

  float limit1;
  bool exists1;
  if (!clks.empty()) {
    // Look for clock slew limits.
    bool is_clk = sta_->clkNetwork()->isIdealClock(pin);
    for (Clock *clk : clks) {
      PathClkOrData clk_data = is_clk ? PathClkOrData::clk : PathClkOrData::data;
      sdc->slewLimit(clk, rf, clk_data, min_max,
		     limit1, exists1);
      if (exists1
	  && (!exists
	      || min_max->compare(limit, limit1))) {
	limit = limit1;
	exists = true;
      }
    }
  }

  if (network->isTopLevelPort(pin)) {
    Port *port = reinterpret_cast<Port*>(lib_port);
    sdc->slewLimit(port, min_max, limit1, exists1);
    if (exists1
	&& (!exists
	    || min_max->compare(limit, limit1))) {
      limit = limit1;
      exists = true;
    }
    InputDrive *drive = sdc->findInputDrive(port);
    if (drive) {
      for (auto rf : RiseFall::range()) {
        const LibertyCell *cell;
        const LibertyPort *from_port;
        float *from_slews;
        const LibertyPort *to_port;
        drive->driveCell(rf, min_max, cell, from_port, from_slews, to_port);
        if (to_port) {
          const LibertyPort *corner_port = to_port->cornerPort(corner, min_max);
          corner_port->slewLimit(min_max, limit1, exists1);
          if (!exists1
              && corner_port->direction()->isAnyOutput()
              && min_max == MinMax::max())
            corner_port->libertyLibrary()->defaultMaxSlew(limit1, exists1);
          if (exists1
              && (!exists
                  || min_max->compare(limit, limit1))) {
            limit = limit1;
            exists = true;
          }
        }
      }
    }
  }
}

void
LocalSta::checkSlew3(const sta::Pin *pin,
                    Vertex *vertex,
                    const sta::LibertyCell *lib_cell,
                    const sta::Corner * corner,
                    const sta::RiseFall *rf,
                    const sta::MinMax *min_max,
                    float limit,
                    PtGraph *pt_graph,
                    // retrun values
                    const sta::Corner *&corner1,
                    const sta::RiseFall *&rf1,
                    float &slew1,
                    float &slack1,
                    float &limit1) const
{
  const DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(min_max);
  PtVertex *pt_vertex = pt_graph->ptVertex(vertex);
  Slew slew = pt_graph->slew(*pt_vertex, rf, dcalc_ap->index());
  float slew2 = delayAsFloat(slew);
  float slack = (min_max == MinMax::max())
    ? limit - slew2 : slew2 - limit;
  if (corner1 == nullptr
      || (slack < slack1
	  // Break ties for the sake of regression stability.
	  || (fuzzyEqual(slack, slack1)
	      && rf->index() < rf1->index()))) {
    corner1 = corner;
    rf1 = rf;
    slew1 = slew;
    slack1 = slack;
    limit1 = limit;
  }
}

float
LocalSta::getLoadCap(PtVertex &drvr_pt_vertex, const sta::Corner *corner,
                     const sta::MinMax *min_max, PtGraph *pt_graph)
{
  sta::DcalcAnalysisPt *dcalc_ap = corner->findDcalcAnalysisPt(min_max);
  const sta::Parasitic *parasitic;
  float max_cap = 0.0;
  for (const RiseFall *rf : RiseFall::range()) {
    float load_cap = 0.0;
    localParasiticLoad(drvr_pt_vertex, rf, dcalc_ap, nullptr,
                       load_cap, parasitic, pt_graph);
    if (max_cap < load_cap)
      max_cap = load_cap;
  }
  return max_cap;
}




}
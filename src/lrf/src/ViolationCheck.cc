



#include "LocalSta.hh"

namespace lrf {



void
LocalSta::localCheckCapacitance(const sta::Pin *pin,
                                const sta::LibertyCell *lib_cell,
                                const sta::Corner * corner,
                                const sta::MinMax *min_max,
                                // retrun values
                                const sta::Corner *&corner1,
                                const sta::RiseFall *&rf1,
                                float &capacitance1,
                                float &limit1,
                                float &slack1) const
{
  if (!lib_cell)
    lib_cell = network_->libertyCell(network_->instance(pin));
  corner1 = nullptr;
  rf1 = nullptr;
  capacitance1 = 0.0;
  limit1 = 0.0;
  slack1 = sta::MinMax::min()->initValue();
  if (corner == nullptr)
    if (lib_cell)
      localCheckCapacitance1(pin, lib_cell, corner, min_max, corner1, rf1, capacitance1,
                      limit1, slack1);
  else {
    for (auto corner : *sta_->corners()) {
      localCheckCapacitance1(pin, lib_cell, corner, min_max, corner1,
                        rf1, capacitance1, limit1, slack1);
    }
  }
}

void
LocalSta::localCheckCapacitance1(const sta::Pin *pin,
                                const sta::LibertyCell *lib_cell,
                             const sta::Corner * corner,
                             const sta::MinMax *min_max,
                             // retrun values
                             const sta::Corner *&corner1,
                             const sta::RiseFall *&rf1,
                             float &capacitance1,
                             float &limit1,
                             float &slack1) const
{
  float limit;
  bool limit_exists;
  findLimit(pin, corner, min_max, limit, limit_exists);
  if (limit_exists) {
    for (auto rf : RiseFall::range()) {
      checkCapacitance(pin, lib_cell, corner, min_max, rf, limit,
		       corner1, rf1, capacitance1, slack1, limit1);
    }
  }
}

void
LocalSta::localFindLimit(const sta::Pin *pin,
                const sta::LibertyCell *lib_cell,
                const sta::Corner *corner,
                const sta::MinMax *min_max,
                // Return values
                float &limit,
                bool &exists) const
{
  const sta::Network *network = sta_->network();
  Sdc *sdc = sta_->sdc();

  // Default to top ("design") limit.
  Cell *top_cell = network->cell(network->topInstance());
  sdc->capacitanceLimit(top_cell, min_max,
			limit, exists);
  
  float limit1;
  bool exists1;
  sta::Port *port = reinterpret_cast<Port*>(lib_cell->findPort(pin));
  if (!port) {
      printf("ERROR: no port found for pin %s\n", network_->pathName(pin));
      throw std::runtime_error("no port found for pin");
    }
  if (network->isTopLevelPort(pin)) {
    sdc->capacitanceLimit(port, min_max, limit1, exists1);
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
          corner_port->capacitanceLimit(min_max, limit1, exists1);
          if (!exists1
              && corner_port->direction()->isAnyOutput()
              && min_max == MinMax::max())
            corner_port->libertyLibrary()->defaultMaxCapacitance(limit1, exists1);
          if (exists1
              && (!exists
                  || min_max->compare(limit, limit1))) {
            limit = limit1;
            exists = true;
          }
        }
      }
    }
  } else {
    sta::Cell *cell = network_->cell(lib_cell);
    sdc->capacitanceLimit(cell, min_max,
			  limit1, exists1);
    if (exists1
	&& (!exists
	    || min_max->compare(limit, limit1))) {
	limit = limit1;
	exists = true;
    }
    LibertyPort *port = network->libertyPort(pin);
    if (port) {
      LibertyPort *corner_port = port->cornerPort(corner, min_max);
      corner_port->capacitanceLimit(min_max, limit1, exists1);
      if (!exists1
	  && port->direction()->isAnyOutput())
	corner_port->libertyLibrary()->defaultMaxCapacitance(limit1, exists1);
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
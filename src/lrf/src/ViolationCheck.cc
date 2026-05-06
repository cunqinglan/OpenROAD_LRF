

#include "LocalSta.hh"
#include "sta/ClkNetwork.hh"
#include "sta/Sdc.hh"
#include "sta/PortDirection.hh"
#include "sta/InputDrive.hh"
#include "sta/Fuzzy.hh"

#include <fstream>
#include <sstream>
#include <unordered_map>

#include "db_sta/dbNetwork.hh"
#include "odb/db.h"
#include "PtGraph.hh"

namespace lrf {

// ─── ML cap_augment_ratio map (process-global) ────────────────────────
// Populated externally before LR via loadMlCapAugmentRatioCsv() / set...().
// Read inside getEffectiveLoadCap() to inflate per-net load cap (only
// inflate, never deflate). Empty = no ML, behave as plain getLoadCap.
namespace {
std::unordered_map<odb::dbNet*, float>& mlCapMap()
{
  static std::unordered_map<odb::dbNet*, float> m;
  return m;
}
}  // namespace

void setMlCapAugmentRatio(odb::dbNet* net, float ratio)
{
  if (net) mlCapMap()[net] = ratio;
}

float getMlCapAugmentRatio(odb::dbNet* net)
{
  auto& m = mlCapMap();
  auto it = m.find(net);
  return (it == m.end()) ? 1.0f : it->second;
}

void clearMlCapAugmentRatio()
{
  mlCapMap().clear();
}

size_t mlCapAugmentRatioSize()
{
  return mlCapMap().size();
}

// ─── ML slew_augment_ratio map (parallel to cap) ──────────────────────
namespace {
std::unordered_map<odb::dbNet*, float>& mlSlewMap()
{
  static std::unordered_map<odb::dbNet*, float> m;
  return m;
}
}  // namespace

void setMlSlewAugmentRatio(odb::dbNet* net, float ratio)
{
  if (net) mlSlewMap()[net] = ratio;
}

float getMlSlewAugmentRatio(odb::dbNet* net)
{
  auto& m = mlSlewMap();
  auto it = m.find(net);
  return (it == m.end()) ? 1.0f : it->second;
}

void clearMlSlewAugmentRatio()
{
  mlSlewMap().clear();
}

size_t mlSlewAugmentRatioSize()
{
  return mlSlewMap().size();
}

// Generic CSV loader used by both cap and slew. Reads header to find "net"
// column + the requested ratio column; populates the given map. Returns
// # rows resolved.
namespace {
size_t loadRatioCsvImpl(odb::dbBlock* block, const std::string& path,
                        const std::string& ratio_col_name,
                        std::unordered_map<odb::dbNet*, float>& target_map,
                        const char* tag)
{
  if (block == nullptr) return 0;
  std::ifstream f(path);
  if (!f.is_open()) {
    printf("[ML] %s: cannot open %s\n", tag, path.c_str());
    return 0;
  }
  std::string line;
  if (!std::getline(f, line)) return 0;
  int net_col = -1, ratio_col = -1;
  {
    std::stringstream ss(line);
    std::string h; int idx = 0;
    while (std::getline(ss, h, ',')) {
      while (!h.empty() && (h.back() == ' ' || h.back() == '\r')) h.pop_back();
      if (h == "net" || h == "net_name") net_col = idx;
      else if (h == ratio_col_name)      ratio_col = idx;
      ++idx;
    }
  }
  if (net_col < 0 || ratio_col < 0) {
    printf("[ML] %s: header missing 'net' and/or '%s'\n",
           tag, ratio_col_name.c_str());
    return 0;
  }
  size_t n_loaded = 0, n_missing = 0;
  while (std::getline(f, line)) {
    std::vector<std::string> cells;
    {
      std::stringstream ss(line);
      std::string c;
      while (std::getline(ss, c, ',')) cells.push_back(c);
    }
    if ((int)cells.size() <= std::max(net_col, ratio_col)) continue;
    const std::string& nm = cells[net_col];
    float r = 1.0f;
    try { r = std::stof(cells[ratio_col]); } catch (...) { continue; }
    odb::dbNet* dbn = block->findNet(nm.c_str());
    if (dbn == nullptr) { ++n_missing; continue; }
    target_map[dbn] = r;
    ++n_loaded;
  }
  printf("[ML] %s: loaded %zu ratios from %s (%zu nets not found)\n",
         tag, n_loaded, path.c_str(), n_missing);
  fflush(stdout);
  return n_loaded;
}
}  // namespace

size_t loadMlSlewAugmentRatioCsv(odb::dbBlock* block, const std::string& path)
{
  return loadRatioCsvImpl(block, path, "slew_ratio", mlSlewMap(),
                          "loadMlSlewAugmentRatioCsv");
}

size_t loadMlCapAugmentRatioCsv(odb::dbBlock* block, const std::string& path)
{
  return loadRatioCsvImpl(block, path, "cap_ratio", mlCapMap(),
                          "loadMlCapAugmentRatioCsv");
}

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

// ML-aware load cap: multiply raw load cap by predicted GRT/placement
// inflation ratio (only inflate, never deflate). Used by legalCheck and
// violationSum so LR sizing decisions see the projected GRT cap.
// Returns plain getLoadCap() result if the ML map is empty.
float
LocalSta::getEffectiveLoadCap(PtVertex &drvr_pt_vertex,
                              const sta::Corner *corner,
                              const sta::MinMax *min_max,
                              PtGraph *pt_graph)
{
  float raw = getLoadCap(drvr_pt_vertex, corner, min_max, pt_graph);
  if (mlCapMap().empty()) return raw;
  sta::Vertex *v = drvr_pt_vertex.vertex();
  if (v == nullptr) return raw;
  const sta::Pin *pin = v->pin();
  if (pin == nullptr) return raw;
  // Flat dbNet — matches how dumpFeatureBundle / EstimateParasitics index
  // nets (handles hierarchical netlists correctly). Goes straight from pin
  // to flat dbNet, bypassing the protected Parasitics::findParasiticNet.
  sta::dbNetwork *db_net_iface = dynamic_cast<sta::dbNetwork *>(network_);
  if (db_net_iface == nullptr) return raw;
  odb::dbNet *db_net = db_net_iface->flatNet(pin);
  if (db_net == nullptr) return raw;
  float ratio = getMlCapAugmentRatio(db_net);
  if (ratio < 1.0f) ratio = 1.0f;  // never deflate
  return raw * ratio;
}

// ML-aware vertex slew: getVertexMaxSlew × max(1.0, slew_ratio) for the
// net the vertex's pin belongs to. Used in legalCheck/violationSum so LR
// sees GRT-projected slew. Equivalent to plain getVertexMaxSlew when no
// slew ratios loaded.
float
LocalSta::getEffectiveVertexMaxSlew(PtGraph *pt_graph, PtVertex &ptv,
                                    sta::DcalcAnalysisPt *dcalc_ap)
{
  float raw = getVertexMaxSlew(pt_graph, ptv, dcalc_ap);
  if (mlSlewMap().empty()) return raw;
  sta::Vertex *v = ptv.vertex();
  if (v == nullptr) return raw;
  const sta::Pin *pin = v->pin();
  if (pin == nullptr) return raw;
  sta::dbNetwork *db_net_iface = dynamic_cast<sta::dbNetwork *>(network_);
  if (db_net_iface == nullptr) return raw;
  odb::dbNet *db_net = db_net_iface->flatNet(pin);
  if (db_net == nullptr) return raw;
  float ratio = getMlSlewAugmentRatio(db_net);
  if (ratio < 1.0f) ratio = 1.0f;
  return raw * ratio;
}




}
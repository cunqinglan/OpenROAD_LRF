

#include "LrSizer.hh"
#include "LrHelper.hh"
#include "ParallelVisitor.hh"
#include "sta/PathExpanded.hh"
#include "sta/Sdc.hh"
#include <vector>
#include <utility>

namespace lrf {

LrSizer::LrSizer(sta::dbSta* sta, LRHelper *lr_helper, 
      ParallelLrVisitor *visitor)
  : lr_helper_(lr_helper),
    visitor_(visitor)
{
  dbStaState::init(sta);
}

LrSizer::~LrSizer()
{
}

void
LrSizer::criticalPathSizing()
{
  // Refer to "Rapid Gate Sizing with fewer Iterations"
  // by Ankur.
  // Step1: identify failing endpoints.
  // Sort by slack and pick the endpoints with slack
  // smaller than tsh (0.1 * T)
  // Step2: for each endpoint, get the min delta LM and 
  // update the critical path LMs.
  // Step3: resize gates on the critical paths. 
  // Sort failing endpoints by slack from resizer.
  const sta::VertexSet* endpoints = sta_->endpoints();
  std::vector<std::pair<sta::Vertex*, sta::Slack>> violating_ends;
  float clock_period = 0.0;
  for (auto *clock : *sdc_->clocks()) {
    float period = clock->period();
    if (period > clock_period) {
      clock_period = period;
      break;
    }
  }
  if (clock_period == 0.0) {
    throw std::runtime_error("LrSizer::criticalPathSizing: found zero clock period");
  }
  // Step 1: Collect all violating endpoints (negative slack).
  for (sta::Vertex* end : *endpoints) {
    const sta::Slack end_slack = sta_->vertexSlack(end, max_);
    if (end_slack < 0.0) {
      violating_ends.emplace_back(end, end_slack);
    }
  }
  if (violating_ends.empty()) {
    // printf("No violating endpoints found (all slacks >= 0). WNS met.\n");
    return;
  }
  // Step 2: Compute per-endpoint threshold.
  // tsh_TNS = para_tsh_discount_ * T is the total TNS budget.
  // Distribute it across violating endpoints to get per-endpoint threshold.
  float tsh_TNS = para_tsh_discount_ * clock_period;
  float tsh = tsh_TNS / violating_ends.size();
  // printf("criticalPathSizing: %zu violating endpoints, clock_period=%.4e, "
         // "tsh_TNS=%.4e, per-endpoint tsh=%.4e\n",
         // violating_ends.size(), clock_period, tsh_TNS, tsh);
  // Filter to only keep endpoints worse than the per-endpoint threshold.
  std::erase_if(violating_ends, [tsh](const auto& p) {
    return p.second >= -tsh;
  });
  if (violating_ends.empty()) {
    // printf("No violating endpoints with slack < %.4e after per-endpoint filtering.\n", -tsh);
    return;
  }
  std::ranges::stable_sort(violating_ends,
                           [](const auto& end_slack1, const auto& end_slack2) {
                             return end_slack1.second < end_slack2.second;
                           });
  
  
  for (const auto& [end, slack] : violating_ends) {
    if (repairCriticalPath(end)) {
      // printf("Resized gates on critical path to endpoint %s with slack %e.\n",
             // end->to_string(graph_).c_str(), slack);
    } else {
      // printf("Failed to resize gates on critical path to endpoint %s with slack %e.\n",
             // end->to_string(graph_).c_str(), slack);
    }
  }
}

bool
LrSizer::repairCriticalPath(sta::Vertex* end)
{
  bool success = false;
  // Step1: update LM on violated path.
  // Step2: resize gates on the violated path.
  // Make sure the lrhelper is of rapid mode
  if (lr_helper_->strategyName().find("Rapid") == std::string::npos) {
    throw std::runtime_error("LrSizer::repairCriticalPath: LRHelper is not of rapid mode, cannot call repairCriticalPath");
  }
  sta::Path *end_path = sta_->vertexWorstSlackPath(end, max_);
  if (end_path == nullptr) {
    throw std::runtime_error("LrSizer::repairCriticalPath: found null critical path for vertex " + end->to_string(graph_));
  }
  success = lr_helper_->updateCriticalPathLms(end_path);
  success = sizeCriticalPathGates(end_path);
  return success;
}

bool
LrSizer::sizeCriticalPathGates(sta::Path* path_end)
{
  sta::PathExpanded expended_path(path_end, sta_);
  size_t path_length = expended_path.size();
  const int start_index = expended_path.startIndex();
  int gates_sized = 0;
  for (size_t i = start_index; i < path_length; ++i) {
    const sta::Path *path = expended_path.path(i);
    const sta::Pin *path_pin = path->pin(sta_);
    if (i > 0 && network_->isDriver(path_pin) &&
        !network_->isTopLevelPort(path_pin)) {
      sta::Instance *inst = network_->instance(path_pin);
      // We don't support reg sizing now.
      if (network_->libertyCell(inst)->hasSequentials()) {
        continue;
      }
      if (singleGateSizing(inst, visitor_)) {
        gates_sized++;
      }
    }
  }
  // printf("Sized %d gates on critical path.\n", gates_sized);

  return gates_sized > 0;
}

bool
LrSizer::singleGateSizing(sta::Instance* inst, ParallelLrVisitor *visitor)
{
  if (inst == nullptr) {
    return false;
  }
  return visitor->singleGateSizing(inst);
}

} // namespace lrf
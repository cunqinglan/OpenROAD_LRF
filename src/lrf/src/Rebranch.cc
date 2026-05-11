// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "Rebranch.hh"

#include "est/SteinerTree.h"

#include "db_sta/dbSta.hh"
#include "db_sta/dbNetwork.hh"
#include "sta/Network.hh"
#include "sta/NetworkClass.hh"
#include "sta/Scene.hh"
#include "sta/Graph.hh"
#include "sta/Scene.hh"
#include "stt/SteinerTreeBuilder.h"
#include "utl/Logger.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lrf {

// NOTE: This is a topology-only implementation.
// - We treat the Steiner tree as an undirected tree (from stt::Tree branches).
// - We rebuild a new tree after applying re-branching on selected sink pins.
// - We do NOT model geometric overlap of Manhattan segments.

namespace {

using SteinerPt = est::SteinerPt;

// Local helper: build an est::SteinerTree using sta->getDbNetwork().
//
// We intentionally keep this inside lrf to avoid touching est module API.
// Logging is avoided (or via printf) to prevent lrf logger conflicts.
static est::SteinerTree*
makeSteinerTreeFromSta(const sta::Pin* drvr_pin,
                       sta::dbSta* sta,
                       stt::SteinerTreeBuilder* stt_builder,
                       const sta::Network* network)
{
  if (drvr_pin == nullptr || sta == nullptr || stt_builder == nullptr || network == nullptr) {
    return nullptr;
  }

  sta::dbNetwork* db_network = sta->getDbNetwork();
  if (db_network == nullptr) {
    return nullptr;
  }

  odb::dbNet* db_net = db_network->findFlatDbNet(drvr_pin);
  if (db_net == nullptr) {
    return nullptr;
  }
  sta::Net* net = db_network->dbToSta(db_net);
  if (net == nullptr) {
    return nullptr;
  }

  // We don't have a stable Logger instance in lrf without potential conflicts.
  // Pass nullptr to SteinerTree; it only uses logger_ for validatePoint errors.
  est::SteinerTree* tree = new est::SteinerTree(drvr_pin, db_network, nullptr);

  // Collect connected pins and locations on the flat net.
  sta::Network* sdc_network = const_cast<sta::Network*>(network->sdcNetwork());

  // connectedPins helper exists in est/EstimateParasitics.cpp (static), not accessible here.
  // So we reconstruct pin locations using dbNetwork API.
  // Use connectedPinIterator on STA net.
  sta::NetConnectedPinIterator* pin_iter = sdc_network->connectedPinIterator(net);
  if (!pin_iter) {
    delete tree;
    return nullptr;
  }

  sta::Vector<est::PinLoc>& pinlocs = tree->pinlocs();
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    if (!pin) {
      continue;
    }
    // Only keep flat pins (iterm/bterm). dbNetwork::location handles those.
    if (!db_network->isPlaced(pin)) {
      // If any pin isn't placed, bail like est does.
      delete pin_iter;
      delete tree;
      return nullptr;
    }
    odb::Point loc = db_network->location(pin);
    pinlocs.push_back(est::PinLoc{pin, loc});
    tree->locAddPin(loc, pin);
  }
  delete pin_iter;

  const int pin_count = pinlocs.size();
  if (pin_count < 2) {
    delete tree;
    return nullptr;
  }

  // Sort pins by location for determinism.
  std::sort(pinlocs.begin(), pinlocs.end(), [](const est::PinLoc& a, const est::PinLoc& b) {
    return a.loc.getX() < b.loc.getX() || (a.loc.getX() == b.loc.getX() && a.loc.getY() < b.loc.getY());
  });

  std::vector<int> x;
  std::vector<int> y;
  x.reserve(pin_count);
  y.reserve(pin_count);
  int drvr_idx = 0;
  for (int i = 0; i < pin_count; i++) {
    const auto& pl = pinlocs[i];
    if (pl.pin == drvr_pin) {
      drvr_idx = i;
    }
    x.push_back(pl.loc.x());
    y.push_back(pl.loc.y());
  }

  stt::Tree ftree = stt_builder->makeSteinerTree(db_net, x, y, drvr_idx);
  tree->setTree(ftree);
  tree->createSteinerPtToPinMap();
  return tree;
}

struct NodeInfo
{
  SteinerPt parent = est::SteinerTree::null_pt;
  int level = 0;
};

static std::vector<std::vector<int>>
buildAdjacents(const est::SteinerTree& tree)
{
  const stt::Tree& ft = const_cast<est::SteinerTree&>(tree).fluteTree();
  const int n = tree.branchCount();
  std::vector<std::vector<int>> adj(n);
  for (int i = 0; i < n; i++) {
    const stt::Branch& b = ft.branch[i];
    const int j = b.n;
    if (j != i) {
      adj[i].push_back(j);
      adj[j].push_back(i);
    }
  }
  return adj;
}

static std::vector<NodeInfo>
computeParentLevel(const std::vector<std::vector<int>>& adj, SteinerPt root)
{
  const int n = static_cast<int>(adj.size());
  std::vector<NodeInfo> info(n);
  if (root == est::SteinerTree::null_pt || root < 0 || root >= n) {
    return info;
  }

  std::vector<char> visited(n, 0);
  std::queue<int> q;
  visited[root] = 1;
  info[root].parent = est::SteinerTree::null_pt;
  info[root].level = 0;
  q.push(root);

  while (!q.empty()) {
    const int u = q.front();
    q.pop();
    for (const int v : adj[u]) {
      if (!visited[v]) {
        visited[v] = 1;
        info[v].parent = u;
        info[v].level = info[u].level + 1;
        q.push(v);
      }
    }
  }

  return info;
}

static bool
isTree(const std::vector<std::vector<int>>& adj, SteinerPt root)
{
  const int n = static_cast<int>(adj.size());
  if (n == 0 || root == est::SteinerTree::null_pt || root < 0 || root >= n) {
    return false;
  }

  // Connected?
  std::vector<char> visited(n, 0);
  std::queue<int> q;
  visited[root] = 1;
  q.push(root);
  int seen = 0;
  while (!q.empty()) {
    const int u = q.front();
    q.pop();
    seen++;
    for (const int v : adj[u]) {
      if (!visited[v]) {
        visited[v] = 1;
        q.push(v);
      }
    }
  }
  if (seen != n) {
    return false;
  }

  // Edge count == n-1?
  long long edges2 = 0;
  for (int i = 0; i < n; i++) {
    edges2 += adj[i].size();
  }
  const long long undirected_edges = edges2 / 2;
  return undirected_edges == n - 1;
}

static SteinerPt
ancestorK(const std::vector<NodeInfo>& info, SteinerPt node, int k)
{
  SteinerPt cur = node;
  while (k > 0 && cur != est::SteinerTree::null_pt) {
    const SteinerPt p = info[cur].parent;
    if (p == est::SteinerTree::null_pt) {
      break;
    }
    cur = p;
    k--;
  }
  return cur;
}

static stt::Tree
orientedTreeFromAdj(const est::SteinerTree& old_tree,
                    const std::vector<std::vector<int>>& adj,
                    SteinerPt root)
{
  const stt::Tree& old_ft
      = const_cast<est::SteinerTree&>(old_tree).fluteTree();

  stt::Tree out;
  const int n = static_cast<int>(adj.size());
  out.deg = n;
  out.length = 0;
  out.branch.resize(n);

  for (int i = 0; i < n; i++) {
    const stt::Branch& b = old_ft.branch[i];
    out.branch[i].x = b.x;
    out.branch[i].y = b.y;
    out.branch[i].n = i;  // placeholder
  }

  const auto info = computeParentLevel(adj, root);
  for (int i = 0; i < n; i++) {
    const int p = info[i].parent;
    out.branch[i].n = (p == est::SteinerTree::null_pt) ? i : p;
  }

  long long wl = 0;
  for (int i = 0; i < n; i++) {
    const int j = out.branch[i].n;
    if (j != i) {
      wl += std::abs(out.branch[i].x - out.branch[j].x)
            + std::abs(out.branch[i].y - out.branch[j].y);
    }
  }
  out.length = static_cast<int>(wl);
  return out;
}

// In-place rebranching: updates the topology stored in `tree`.
// Returns true if any rebranch was applied.
static bool
rebranchTopologyInPlace(est::SteinerTree* tree,
                        const std::unordered_map<const sta::Pin*, double>& sink_criticality,
                        double max_rebranch_ratio)
{
  if (tree == nullptr) {
    return false;
  }
  if (max_rebranch_ratio <= 0.0) {
    return false;
  }

  auto adj = buildAdjacents(*tree);
  const SteinerPt root = tree->drvrPt();
  if (!isTree(adj, root)) {
    return false;
  }

  struct SinkEntry
  {
    SteinerPt pt;
    const sta::Pin* pin;
    double crit = 0.0;
  };

  std::vector<SinkEntry> sinks;
  sinks.reserve(tree->pinCount());

  for (int pt = 0; pt < tree->branchCount(); pt++) {
    const sta::PinSeq* pins = tree->pins(pt);
    if (!pins) {
      continue;
    }
    const sta::Pin* rep_load = nullptr;
    for (const sta::Pin* pin : *pins) {
      if (pin && sink_criticality.find(pin) != sink_criticality.end()) {
        rep_load = pin;
        break;
      }
    }
    if (rep_load) {
      sinks.push_back(SinkEntry{pt, rep_load, sink_criticality.at(rep_load)});
    }
  }

  // As requested: if there are no provided sinks, this is a no-op.
  if (sinks.empty()) {
    return false;
  }

  double max_c = 0.0;
  for (const auto& s : sinks) {
    max_c = std::max(max_c, s.crit);
  }
  if (max_c <= 0.0) {
    return false;
  }
  for (auto& s : sinks) {
    s.crit /= max_c;
  }

  std::sort(sinks.begin(), sinks.end(), [](const SinkEntry& a, const SinkEntry& b) {
    if (a.crit != b.crit) {
      return a.crit > b.crit;
    }
    return a.pt < b.pt;
  });

  auto info = computeParentLevel(adj, root);
  const int max_rebranches
      = static_cast<int>(std::ceil(max_rebranch_ratio * static_cast<double>(sinks.size())));

  int applied = 0;
  for (const auto& s : sinks) {
    if (applied >= max_rebranches) {
      break;
    }
    if (s.crit <= 0.0) {
      break;
    }

    const int p_i = info[s.pt].level;
    if (p_i <= 1) {
      continue;
    }

    const int k_i = static_cast<int>(std::ceil(s.crit * p_i));
    if (k_i <= 0) {
      continue;
    }

    const SteinerPt new_parent = ancestorK(info, s.pt, k_i);
    if (new_parent == est::SteinerTree::null_pt || new_parent == s.pt) {
      continue;
    }
    const SteinerPt old_parent = info[s.pt].parent;
    if (old_parent == est::SteinerTree::null_pt || new_parent == old_parent) {
      continue;
    }

    auto remove_edge = [&](int u, int v) {
      auto& vec = adj[u];
      vec.erase(std::remove(vec.begin(), vec.end(), v), vec.end());
    };

    remove_edge(s.pt, old_parent);
    remove_edge(old_parent, s.pt);
    adj[s.pt].push_back(new_parent);
    adj[new_parent].push_back(s.pt);

    if (!isTree(adj, root)) {
      // Revert.
      remove_edge(s.pt, new_parent);
      remove_edge(new_parent, s.pt);
      adj[s.pt].push_back(old_parent);
      adj[old_parent].push_back(s.pt);
      continue;
    }

    info = computeParentLevel(adj, root);
    applied++;
  }

  if (applied == 0) {
    return false;
  }

  stt::Tree new_stt = orientedTreeFromAdj(*tree, adj, root);
  tree->setTree(new_stt);
  tree->createSteinerPtToPinMap();
  return true;
}

}  // namespace


std::unordered_map<const sta::Pin*, double>
computeSinkCriticality(const sta::Pin* drvr_pin,
                       sta::Scene* corner,
                       sta::dbSta* sta)
{
  std::unordered_map<const sta::Pin*, double> sink_criticality;
  
  if (drvr_pin == nullptr) {
    printf("ERROR: computeSinkCriticality called with null driver pin\n");
    return sink_criticality;
  }

  sta::Network* network = sta->network();
  sta::Term* term = network->term(drvr_pin);
  if (term == nullptr) {
    printf("ERROR: Cannot find term for driver pin %s\n", 
           network->pathName(drvr_pin));
    return sink_criticality;
  }
  
  sta::Net* net = network->net(term);
  if (net == nullptr) {
    printf("ERROR: Cannot find net for driver pin %s\n", 
           network->pathName(drvr_pin));
    return sink_criticality;
  }

  // Determine which corner(s) to use
  if (corner == nullptr) {
    corner = sta->findScene("default");
  }

  if (corner == nullptr) {
    printf("ERROR: No valid corner found for criticality computation\n");
    return sink_criticality;
  }

  // Iterate through all connected pins
  sta::NetConnectedPinIterator* pin_iter = network->connectedPinIterator(net);
  while (pin_iter->hasNext()) {
    const sta::Pin* pin = pin_iter->next();
    
    // Skip driver pins
    if (network->isDriver(pin)) {
      continue;
    }
    
    // Compute criticality as sum of all negative slacks for paths in the specified corner
    double total_negative_slack = 0.0;
    sta::Vertex* vertex = sta->graph()->pinLoadVertex(pin);
    
    if (vertex) {
      // Enumerate all paths ending at this vertex
      sta::VertexPathIterator path_iter(vertex, sta);
      while (path_iter.hasNext()) {
        sta::Path* path = path_iter.next();
        
        // Check if this path belongs to the target corner
        if (path->pathAnalysisPt(sta)->corner() == corner) {
          // Get slack for this specific path
          sta::Slack slack = path->slack(sta);
          
          // Only accumulate negative slack (timing violations)
          if (slack < 0.0) {
            total_negative_slack += (-slack);  // Convert to positive value for accumulation
          }
        }
      }
    }
    
    sink_criticality[pin] = total_negative_slack;
  }
  delete pin_iter;
  
  return sink_criticality;
}

void
printSinkCriticality(const sta::Pin* drvr_pin,
                     const std::unordered_map<const sta::Pin*, double>& sink_criticality,
                     sta::dbSta* sta)
{
  if (drvr_pin == nullptr) {
    printf("ERROR: printSinkCriticality called with null driver pin\n");
    return;
  }

  sta::Network* network = sta->network();
  sta::Term* term = network->term(drvr_pin);
  sta::Net* net = (term != nullptr) ? network->net(term) : nullptr;
  
  printf("\n=== Sink Criticality for net: %s (driver: %s) ===\n",
         net ? network->pathName(net) : "unknown",
         network->pathName(drvr_pin));
  printf("Total sink count: %zu\n", sink_criticality.size());
  
  if (sink_criticality.empty()) {
    printf("No sinks found or all sinks have non-negative slack.\n");
    printf("===================================================\n\n");
    return;
  }
  
  // Find max criticality for normalization
  double max_crit = 0.0;
  for (const auto& entry : sink_criticality) {
    max_crit = std::max(max_crit, entry.second);
  }
  
  // Sort by criticality (descending)
  std::vector<std::pair<const sta::Pin*, double>> sorted_sinks(
      sink_criticality.begin(), sink_criticality.end());
  std::sort(sorted_sinks.begin(), sorted_sinks.end(),
            [](const auto& a, const auto& b) { return a.second > b.second; });
  
  printf("%-60s %15s %15s\n", "Pin Name", "Criticality", "Normalized");
  printf("%-60s %15s %15s\n", "--------", "-----------", "----------");
  
  for (const auto& entry : sorted_sinks) {
    const sta::Pin* pin = entry.first;
    double crit = entry.second;
    double normalized = (max_crit > 0.0) ? (crit / max_crit) : 0.0;
    
    printf("%-60s %15.6f %14.1f%%\n",
           network->pathName(pin),
           crit,
           normalized * 100.0);
  }
  
  printf("\nMax criticality: %.6f\n", max_crit);
  printf("===================================================\n\n");
  fflush(stdout);
}

est::SteinerTree*
makeRebranchedSteinerTree(const sta::Pin* drvr_pin,
                          sta::dbSta* sta,
                          const std::unordered_map<const sta::Pin*, double>& sink_criticality,
                          double max_rebranch_ratio)
{
  if (drvr_pin == nullptr || sta == nullptr) {
    return nullptr;
  }
  if (max_rebranch_ratio <= 0.0) {
    // Treat as no-op: still return a valid tree.
    max_rebranch_ratio = 0.0;
  }

  // 1) Build initial tree shell once.
  // We need a SteinerTreeBuilder for this. We lazily construct it after we
  // have a dbNetwork/db handle (available in the just-built tree).
  static stt::SteinerTreeBuilder* builder = nullptr;
  est::SteinerTree* initial = nullptr;
  if (builder != nullptr) {
    initial = makeSteinerTreeFromSta(drvr_pin, sta, builder, sta->network());
  } else {
    // Temporary builder can't be created without a db pointer, so first build
    // a minimal tree using a locally-created builder based on sta->getDbNetwork().
    // In this repo integration, sta->getDbNetwork() exists.
    sta::dbNetwork* tmp_net = sta->getDbNetwork();
    if (tmp_net == nullptr) {
      return nullptr;
    }
    // dbNetwork exposes block(); use it to reach db.
    odb::dbBlock* block = tmp_net->block();
    if (block == nullptr) {
      return nullptr;
    }
    odb::dbDatabase* db = block->getDb();
    builder = new stt::SteinerTreeBuilder(db, nullptr);
    initial = makeSteinerTreeFromSta(drvr_pin, sta, builder, sta->network());
  }
  
  if (initial == nullptr) {
    return nullptr;
  }

  rebranchTopologyInPlace(initial, sink_criticality, max_rebranch_ratio);
  
  return initial;
}

}  // namespace lrf

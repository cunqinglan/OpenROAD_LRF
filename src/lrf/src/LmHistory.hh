#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "lrf/LrfClass.hh"
#include "sta/Graph.hh"
#include "sta/TimingArc.hh"

namespace lrf {

// A single edge's LM snapshot: stores EdgeId + its LM values.
struct LmRecord {
  sta::EdgeId edge_id;
  std::vector<LMValue> lm_values;  // size = arc_count * ap_count
};

// One frame: all edges' LM records.
typedef std::vector<LmRecord> LmSnapshot;

// LmHistory: record and restore LM snapshots by frame.
// Record uses VertexIterator + VertexOutEdgeIterator to collect edges,
// but each record stores EdgeId for stable identification.
// Restore uses graph->edge(EdgeId) to locate each edge, independent
// of iteration order.
class LmHistory {
public:
  explicit LmHistory(sta::Graph *graph);
  ~LmHistory() = default;

  // Record current LM values of all edges into a new frame.
  // Returns the frame id (0-based index).
  int recordLM();

  // Restore LM values from a given frame.
  // Uses EdgeId to locate each edge in the graph.
  // Returns the number of edges successfully restored.
  // Edges that no longer exist in the graph are silently skipped.
  int restoreLM(int frame_id);

  // Number of recorded frames.
  int frameCount() const;

  // Clear all recorded frames.
  void clear();

  // Remove the last recorded frame.
  void popBack();

  // Save the last recorded frame to a binary file.
  // design_name and vertex_count are stored as metadata for validation on load.
  // Returns true on success.
  bool saveToFile(const std::string &path,
                  const std::string &design_name,
                  uint32_t vertex_count) const;

  // Load a frame from a binary file and append it to history.
  // Validates that design_name and vertex_count match the file header.
  // Returns the frame id on success, -1 on failure.
  int loadFromFile(const std::string &path,
                   const std::string &design_name,
                   uint32_t vertex_count);

  // Update the graph pointer (e.g. after graph rebuild).
  void setGraph(sta::Graph *graph) { graph_ = graph; }

private:
  // Collect all edges' LM data into a snapshot.
  void collectEdgeLMs(LmSnapshot &snapshot);

  // Compute the LM array size for one edge.
  size_t edgeLmCount(const sta::Edge *edge) const;

  sta::Graph *graph_;
  std::vector<LmSnapshot> history_;
};

} // namespace lrf

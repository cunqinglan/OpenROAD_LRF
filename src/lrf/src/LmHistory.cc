
#include "LmHistory.hh"

#include "sta/Graph.hh"
#include "sta/TimingArc.hh"

namespace lrf {
using namespace sta;

LmHistory::LmHistory(Graph *graph)
  : graph_(graph)
{
}

size_t
LmHistory::edgeLmCount(const Edge *edge) const
{
  // LM array layout: arc_index * ap_count + ap_index
  size_t arc_count = edge->timingArcSet()->arcs().size();
  size_t ap_count = graph_->apCount();
  return arc_count * ap_count;
}

void
LmHistory::collectEdgeLMs(LmSnapshot &snapshot)
{
  // Walk all vertices, then all out-edges per vertex.
  // Same traversal pattern as LRHelper::updateAllEdgeLms,
  // but we store EdgeId for each record so restore is order-independent.
  VertexIterator vertex_iter(graph_);
  while (vertex_iter.hasNext()) {
    Vertex *vertex = vertex_iter.next();
    VertexOutEdgeIterator out_edge_iter(vertex, graph_);
    while (out_edge_iter.hasNext()) {
      Edge *edge = out_edge_iter.next();

      // Read LM array from edge
      LMValue *lms = edge->arcLms();
      if (lms == nullptr)
        continue;

      size_t lm_count = edgeLmCount(edge);
      if (lm_count == 0)
        continue;

      LmRecord record;
      record.edge_id = graph_->id(edge);
      record.lm_values.assign(lms, lms + lm_count);
      snapshot.push_back(std::move(record));
    }
  }
}

int
LmHistory::recordLM()
{
  int frame_id = static_cast<int>(history_.size());
  history_.emplace_back();
  LmSnapshot &snapshot = history_.back();
  collectEdgeLMs(snapshot);
  return frame_id;
}

int
LmHistory::restoreLM(int frame_id)
{
  if (frame_id < 0 || frame_id >= static_cast<int>(history_.size()))
    return 0;

  const LmSnapshot &snapshot = history_[frame_id];
  int restored_count = 0;

  for (const LmRecord &record : snapshot) {
    // Use EdgeId to locate the edge — stable regardless of iteration order
    Edge *edge = graph_->edge(record.edge_id);
    if (edge == nullptr)
      continue;

    LMValue *lms = edge->arcLms();
    if (lms == nullptr)
      continue;

    size_t lm_count = edgeLmCount(edge);
    // Safety: only restore if the sizes match (graph structure unchanged for this edge)
    if (record.lm_values.size() != lm_count)
      continue;

    std::copy(record.lm_values.begin(), record.lm_values.end(), lms);
    restored_count++;
  }

  return restored_count;
}

int
LmHistory::frameCount() const
{
  return static_cast<int>(history_.size());
}

void
LmHistory::clear()
{
  history_.clear();
}

void
LmHistory::popBack()
{
  if (!history_.empty())
    history_.pop_back();
}

} // namespace lrf

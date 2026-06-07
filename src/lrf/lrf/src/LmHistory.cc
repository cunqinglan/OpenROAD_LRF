
#include "LmHistory.hh"

#include <cstdio>
#include <cstdint>

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

// Binary format (version "LMS2"):
//   magic:         uint32 = 0x4C4D5332
//   name_len:      uint32
//   design_name:   char[name_len]  (no null terminator)
//   vertex_count:  uint32
//   num_records:   uint32
//   per record:
//     edge_id:     uint32  (EdgeId)
//     lm_count:    uint32
//     lm_values:   float[lm_count]

static constexpr uint32_t LM_FILE_MAGIC = 0x4C4D5332;

bool
LmHistory::saveToFile(const std::string &path,
                      const std::string &design_name,
                      uint32_t vertex_count) const
{
  if (history_.empty()) {
    printf("LmHistory::saveToFile: no frames to save\n");
    return false;
  }

  FILE *fp = fopen(path.c_str(), "wb");
  if (!fp) {
    printf("LmHistory::saveToFile: cannot open %s for writing\n", path.c_str());
    return false;
  }

  const LmSnapshot &snapshot = history_.back();
  uint32_t magic = LM_FILE_MAGIC;
  uint32_t name_len = static_cast<uint32_t>(design_name.size());
  uint32_t num_records = static_cast<uint32_t>(snapshot.size());

  fwrite(&magic, sizeof(uint32_t), 1, fp);
  fwrite(&name_len, sizeof(uint32_t), 1, fp);
  fwrite(design_name.data(), 1, name_len, fp);
  fwrite(&vertex_count, sizeof(uint32_t), 1, fp);
  fwrite(&num_records, sizeof(uint32_t), 1, fp);

  for (const LmRecord &record : snapshot) {
    uint32_t edge_id = static_cast<uint32_t>(record.edge_id);
    uint32_t lm_count = static_cast<uint32_t>(record.lm_values.size());
    fwrite(&edge_id, sizeof(uint32_t), 1, fp);
    fwrite(&lm_count, sizeof(uint32_t), 1, fp);
    fwrite(record.lm_values.data(), sizeof(LMValue), lm_count, fp);
  }

  fclose(fp);
  printf("LmHistory::saveToFile: saved %u edge records to %s "
         "(design=%s, vertices=%u)\n",
         num_records, path.c_str(), design_name.c_str(), vertex_count);
  return true;
}

int
LmHistory::loadFromFile(const std::string &path,
                        const std::string &design_name,
                        uint32_t vertex_count)
{
  FILE *fp = fopen(path.c_str(), "rb");
  if (!fp) {
    printf("LmHistory::loadFromFile: cannot open %s for reading\n", path.c_str());
    return -1;
  }

  uint32_t magic = 0;
  if (fread(&magic, sizeof(uint32_t), 1, fp) != 1 || magic != LM_FILE_MAGIC) {
    printf("LmHistory::loadFromFile: bad magic in %s\n", path.c_str());
    fclose(fp);
    return -1;
  }

  // Read and validate design name
  uint32_t name_len = 0;
  if (fread(&name_len, sizeof(uint32_t), 1, fp) != 1 || name_len > 4096) {
    printf("LmHistory::loadFromFile: bad name_len %u\n", name_len);
    fclose(fp);
    return -1;
  }
  std::string file_design_name(name_len, '\0');
  if (fread(&file_design_name[0], 1, name_len, fp) != name_len) {
    printf("LmHistory::loadFromFile: truncated design name\n");
    fclose(fp);
    return -1;
  }
  if (file_design_name != design_name) {
    printf("LmHistory::loadFromFile: design name mismatch: "
           "file has '%s', current design is '%s'\n",
           file_design_name.c_str(), design_name.c_str());
    fclose(fp);
    return -1;
  }

  // Read and validate vertex count
  uint32_t file_vertex_count = 0;
  if (fread(&file_vertex_count, sizeof(uint32_t), 1, fp) != 1) {
    printf("LmHistory::loadFromFile: failed to read vertex count\n");
    fclose(fp);
    return -1;
  }
  if (file_vertex_count != vertex_count) {
    printf("LmHistory::loadFromFile: vertex count mismatch: "
           "file has %u, current graph has %u\n",
           file_vertex_count, vertex_count);
    fclose(fp);
    return -1;
  }

  uint32_t num_records = 0;
  if (fread(&num_records, sizeof(uint32_t), 1, fp) != 1) {
    printf("LmHistory::loadFromFile: failed to read record count\n");
    fclose(fp);
    return -1;
  }

  int frame_id = static_cast<int>(history_.size());
  history_.emplace_back();
  LmSnapshot &snapshot = history_.back();
  snapshot.reserve(num_records);

  for (uint32_t i = 0; i < num_records; ++i) {
    uint32_t edge_id = 0;
    uint32_t lm_count = 0;
    if (fread(&edge_id, sizeof(uint32_t), 1, fp) != 1 ||
        fread(&lm_count, sizeof(uint32_t), 1, fp) != 1) {
      printf("LmHistory::loadFromFile: truncated file at record %u\n", i);
      history_.pop_back();
      fclose(fp);
      return -1;
    }

    LmRecord record;
    record.edge_id = static_cast<EdgeId>(edge_id);
    record.lm_values.resize(lm_count);
    if (fread(record.lm_values.data(), sizeof(LMValue), lm_count, fp) != lm_count) {
      printf("LmHistory::loadFromFile: truncated LM data at record %u\n", i);
      history_.pop_back();
      fclose(fp);
      return -1;
    }
    snapshot.push_back(std::move(record));
  }

  fclose(fp);
  printf("LmHistory::loadFromFile: loaded %u edge records from %s "
         "(design=%s, vertices=%u, frame %d)\n",
         num_records, path.c_str(), file_design_name.c_str(),
         file_vertex_count, frame_id);
  return frame_id;
}

} // namespace lrf

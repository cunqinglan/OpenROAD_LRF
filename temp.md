# PtGraph / LocalSta Virtual Graph 优化分析

## 一、架构概览

PtGraph 是 STA timing graph (`sta::Graph`) 的局部虚拟镜像，用于对单个 instance 的邻域做 cell swap 评估，避免修改全局 STA 状态。核心热路径为：

```
makePtGraph → [对每个候选 cell: virtualReplaceCell → findLocalDelays → findLocalArrivals → findLocalRequireds → delayLmSum]
```

---

## 二、关键瓶颈和优化建议

### 1. `VertexPtToIdMap` 使用 `std::map` — 应改为 `unordered_map`

**文件**: `LrfClass.hh:53`
```cpp
typedef std::map<const sta::Vertex*, sta::VertexId> VertexPtToIdMap;
```

`vertex_map_` 在 `makePtVertexAndPtEdge`、`makePtInstEdge`、`makePtWireEdge`、`annotateVerticesType`、`pinToPtVertex`、`topVertexLevel` 中被频繁查找。`std::map` 查找是 O(log N)，指针作为 key 完全没有有序性需求，改为 `std::unordered_map` 可将所有查找变为 O(1)。

**估算影响**：一个局部图通常有 10~50 个顶点，单次 map 查找微秒级，但在 `makePtWireEdge` 中每条 wire edge 都查一次，且在多线程 precheck 中执行数万次 `makePtGraph`，累积可观。

---

### 2. `topoSortVertices` 实现有 bug 并且低效

**文件**: `PtGraph.cc:577-592`
```cpp
bool PtGraph::topoSortVertices()
{
  if (sorted_) return sorted_;
  if (graph_made_) {
    size_t n = ...;
    sorted_vertex_ids_.resize(n);
    std::iota(sorted_vertex_ids_.begin(), sorted_vertex_ids_.end(), 1);
    for (auto &pt_vertex : pt_vertices_) {       // ← BUG: 循环体无关 pt_vertex
      if (pt_vertex.type() == PtVertexType::Sentinel) continue;
      std::stable_sort(sorted_vertex_ids_.begin(), sorted_vertex_ids_.end(),
                       PtVertexIdLevelLess(this)); // ← 重复排序 N 次！
    }
    sorted_ = true;
  }
  return sorted_;
}
```

**问题**：`std::stable_sort` 被放在 for 循环内部，但循环变量 `pt_vertex` 根本没被使用。结果是对同一个数组排序了 N 次（N = 顶点数），每次 O(N log N)，总计 O(N² log N)。应该只排一次。

**修复**：将 `std::stable_sort` 移到循环外面，或直接删掉 for 循环。

---

### 3. `initVertexAndEdges` 每次 makePtGraph 都做完整拷贝

**文件**: `PtGraph.cc:873-893`
```cpp
void PtGraph::initVertexAndEdges()
{
  for (PtVertex &pt_vertex : pt_vertices_) {
    pt_vertex.copyInfoFromVertex(ap_count_, slew_rf_count_);  // 拷贝 slew 数组
    initPaths(pt_vertex);                                      // new Path[] 拷贝
  }
  for (PtEdge &pt_edge : pt_edges_) {
    pt_edge.copyInfoFromEdge(ap_count_);  // 拷贝 arc_delays 数组
    pt_edge.timing_arc_set_ = pt_edge.edge()->timingArcSet();
  }
}
```

`copyInfoFromVertex` 做 `slews_.assign(src, src+count)`，`initPaths` 做 `new Path[path_count]` + 逐元素拷贝。对于 precheck 场景（`makePtGraph(inst, false)`），`initVertexAndEdges` 仍然会在 `makeGraph → initVertexAndEdges` 中被调用。

**建议**：
- 对于 precheck 只需要 delay，不需要完整的 arrival/required path 拷贝。考虑增加一个轻量的 `initVertexAndEdgesLite()` 跳过 `initPaths`。
- slew 和 arc_delay 拷贝可以延迟到实际需要时（lazy init）。

---

### 4. `PtVertex::paths_` 使用裸 `new[]`/`delete[]` — 内存碎片化

**文件**: `PtGraph.hh:311`, `PtGraph.cc:462-471`
```cpp
sta::Path *paths_ = nullptr;
// ...
sta::Path *paths = new sta::Path[path_count];
```

每个 PtVertex 单独 `new Path[]`，一个 PtGraph 有 10~50 个顶点就是 10~50 次堆分配。在 precheck 中创建上万个 PtGraph，总共数十万次小堆分配，极度碎片化。

**建议**：使用 arena/pool allocator，在 PtGraph 级别预分配一块连续内存，所有 PtVertex 的 paths 从中分配。或者使用 `std::vector<Path>` 替代裸指针。

---

### 5. `virtualReplaceCell` 每次都调用 `recomputeLocalParasitics`，后者有全局锁

**文件**: `LocalSta.cc:1870`, `LocalParasitics.cc:144`
```cpp
// virtualReplaceCell:
pt_graph->updateTimingArcSets();
recomputeLocalParasitics(pt_graph);    // ← 每次 swap 尝试都执行

// makeLocalPiElmore:
std::lock_guard<std::mutex> lock(g_odb_sta_access_mutex);  // ← 全局锁！
```

`g_odb_sta_access_mutex` 是所有线程共享的全局锁。在 `increAndGetLocalTimingCost` 中，每尝试一个候选 cell 就要经过 `virtualReplaceCell → recomputeLocalParasitics → makeLocalPiElmore → lock(g_odb_sta_access_mutex)`。这是多线程并行的严重串行化点。

**建议**：
- 分析 `reduceToPi` 到底读了哪些全局状态。如果只是读 parasitic network（只读），可以用 shared_lock 替代 exclusive lock。
- 如果 parasitic network 不变（只是 pin capacitance 变化），可以缓存 network 拓扑，只重算 capacitance 部分。

---

### 6. `PtEdge::arc_delays_` 使用 `std::vector<ArcDelay>` — 频繁 resize

**文件**: `PtGraph.hh:229`
```cpp
std::vector<sta::ArcDelay> arc_delays_;
```

`copyInfoFromEdge` 每次都 `arc_delays_.assign(src, src + delay_count)`，触发 vector 内部的 allocate + memcpy。一个 PtGraph 的 edge 数量通常为 20~100，乘以上万个 PtGraph = 数十万次小 vector 分配。

**建议**：和 vertex paths 一样，使用 PtGraph 级别的 arena allocator，或者预分配足够大的 flat 数组。

---

### 7. `collectLocalVertices` 每次都创建新的 `VertexSet`

**文件**: `LocalSta.cc:491-494`
```cpp
PtGraph *LocalSta::makePtGraph(Instance *inst, ...) {
  VertexSet local_vertices(graph_);          // ← 每次新建 set
  collectLocalVertices(inst, local_vertices); // ← 遍历 fanin/fanout
```

`VertexSet` 是 `std::set<Vertex*>`，需要多次堆分配用于树节点。对于反复调用的 precheck 场景，可以：
- 复用一个 thread-local 的 `VertexSet`，每次 clear 而不是重建。
- 或直接使用 `std::vector<Vertex*>` + 后续去重（因为局部顶点数通常很少）。

---

### 8. `makePtGraph` 中每个 PtGraph 都 `push_back` 到 `local_graphs_` + 加锁

**文件**: `LocalSta.cc:516-517`
```cpp
std::lock_guard<std::mutex> lock(pt_graph_vector_mutex_);
local_graphs_.push_back(pt_graph);
```

在 precheck 中对 10000+ 个 instance 每个都加一次锁，且这些 PtGraph 直到 LocalSta 析构才释放。

**建议**：
- precheck 场景下 PtGraph 应该在 `trySwapPrecheck` 结束后立即释放（评估完就不需要了）。
- 可以使用 thread-local PtGraph 池，复用而非每次 new/delete。

---

### 9. `delayLmSum` 遍历所有边，包括 Sentinel

**文件**: `PtGraph.cc:919-951`
```cpp
for (PtEdge &pt_edge : pt_edges_) {
  if (pt_edge.type() == PtEdgeType::Sentinel || !pt_edge.hasBase())
    continue;
  // ...
}
```

每次都从 index 0（Sentinel）开始遍历。对于小图问题不大，但如果有 deleted edges（type 被设为 Sentinel）则会浪费遍历。

**建议**：维护一个 active edge list 或者从 index 1 开始遍历。

---

### 10. `updateTimingArcSets` 的 fallback 搜索效率低

**文件**: `PtGraph.cc:543-570`
```cpp
sta::TimingArcSet *new_arc_set = ref_lib_cell_->findTimingArcSet(ref_arc_set);
if (new_arc_set == nullptr) {
  // Fallback: 遍历 candidates...
  const sta::TimingArcSetSeq &candidates = ref_lib_cell_->timingArcSets(new_from, new_to);
  for (sta::TimingArcSet *candidate : candidates) { ... }
}
```

每次 `virtualReplaceCell` 调用 `updateTimingArcSets` 都要对所有 RefInstEdge 做 arc set 匹配。如果候选 cell 多次来自相同 LibertyCell，这个查找是重复的。

**建议**：缓存 `LibertyCell* → {old_arc_set → new_arc_set}` 映射，同一 cell 只查找一次。

---

## 三、优先级排序

| 优先级 | 问题 | 预期收益 | 复杂度 |
|--------|------|----------|--------|
| **P0** | #5 全局锁 `g_odb_sta_access_mutex` | 多线程吞吐提升数倍 | 中 |
| **P0** | #2 topoSort N 次重复排序 bug | 正确性 + 性能 | 低 |
| **P1** | #8 PtGraph 不释放 + 加锁 push | 内存 + 锁竞争 | 低 |
| **P1** | #4 Path 裸 new[] 碎片化 | 减少堆分配 | 中 |
| **P1** | #6 arc_delays vector 频繁分配 | 减少堆分配 | 中 |
| **P2** | #1 map → unordered_map | 查找加速 | 极低 |
| **P2** | #3 initVertexAndEdges 全量拷贝 | precheck 加速 | 低 |
| **P2** | #7 VertexSet 重复创建 | 减少堆分配 | 低 |
| **P3** | #10 TimingArcSet 查找缓存 | 减少重复查找 | 低 |
| **P3** | #9 delayLmSum 遍历 Sentinel | 微小 | 极低 |

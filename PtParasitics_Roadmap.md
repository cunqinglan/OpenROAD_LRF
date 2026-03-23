# PtParasitics 开发路线图

## 一、背景与动机

### 1.1 当前架构的问题

当前 `LocalParasitics` 继承自 `ConcreteParasitics`（OpenSTA 底层寄生参数类），存在以下核心问题：

| 问题 | 具体表现 | 影响 |
|------|---------|------|
| **全局共享数据结构** | `local_parasitic_network_map_` 直接复制全局 `parasitic_network_map_` 指针（`LocalParasitics.cc:108`），`local_drvr_parasitic_map_` 按 `Pin*` 索引，所有 PtGraph 共享同一份 | 多线程并发写同一个 pin 的 parasitic 时产生 race condition |
| **全局锁瓶颈** | `g_odb_sta_access_mutex`（全局互斥锁）保护 `makeLocalPiElmore`（`LocalParasitics.cc:144`）和 `TaskArranger` 中的 DB 访问 | 并行任务退化为串行，严重削弱多线程性能 |
| **Buffering 不兼容** | `LrRebuffer::buildVirtualBuffer` 插入虚拟 buffer 后原始 parasitic network 失效，只能用 `hasVirtualBuffer` flag 绕过，走 `computeVirtualLoadCap` 估算路径 | 虚拟 buffer 评估精度不足，无法增量更新寄生参数 |
| **Pin-based 查找冲突** | `findLocalParasitic(Pin*, RiseFall*, DcalcAnalysisPt*)` 按全局 Pin 查找，不同 PtGraph 对同一 net 的不同操作会互相覆盖 | cell swap 评估结果不确定 |
| **初始化开销** | `initParasiticMapFromBase()` 在构造时深拷贝全局所有 PiElmore 对象 | 内存冗余、初始化慢 |

### 1.2 目标架构

```
                   ┌──────────────────────────────────────────┐
                   │              LocalSta                    │
                   │  ┌────────┐  ┌────────┐  ┌────────┐    │
                   │  │PtGraph1│  │PtGraph2│  │PtGraph3│    │
                   │  │  ┌───┐ │  │  ┌───┐ │  │  ┌───┐ │    │
                   │  │  │Pt │ │  │  │Pt │ │  │  │Pt │ │    │
                   │  │  │Par│ │  │  │Par│ │  │  │Par│ │    │
                   │  │  └───┘ │  │  └───┘ │  │  └───┘ │    │
                   │  └────────┘  └────────┘  └────────┘    │
                   │     独立        独立         独立         │
                   │   无锁操作    无锁操作     无锁操作       │
                   └──────────────────────────────────────────┘
```

**核心思想**：每个 `PtGraph` 拥有独立的 `PtParasitics` 实例，parasitic 数据以 `PtVertexId` 为索引（而非全局 `Pin*`），生命周期与 PtGraph 完全绑定，彻底消除跨线程共享状态。

---

## 二、架构设计

### 2.1 新类 PtParasitics

```cpp
// PtParasitics.hh
class PtParasitics : public StaState {
public:
  PtParasitics(StaState *state, PtGraph *pt_graph);
  ~PtParasitics();

  // ---- 初始化 ----
  // 从全局 ConcreteParasitics 拷贝与本 PtGraph 相关的 parasitic data
  void initFromGlobal(const ConcreteParasitics *global);
  
  // ---- 查找接口（按 PtVertexId 索引）----
  // 查找 driver vertex 的 reduced PiElmore
  Parasitic *findPiElmore(VertexId pt_drvr_id, 
                          const RiseFall *rf,
                          const DcalcAnalysisPt *ap) const;
  
  // 查找 driver vertex 的 parasitic network (RC tree)
  Parasitic *findParasiticNetwork(VertexId pt_drvr_id,
                                  const ParasiticAnalysisPt *ap) const;

  // ---- 计算/更新接口 ----
  // 对 driver vertex 做 Pi-Elmore reduce
  Parasitic *reduceToPiElmore(VertexId pt_drvr_id,
                              const RiseFall *rf,
                              const Corner *corner,
                              const MinMax *min_max,
                              const ParasiticAnalysisPt *ap);

  // 重算所有 driver vertex 的 parasitic（cell swap 后调用）
  void recomputeAll();
  
  // ---- Virtual Buffer 支持 ----
  // 当插入虚拟 buffer 时，按新拓扑重建子网的 parasitic
  void rebuildForVirtualBuffer(VertexId orig_drvr_id,
                               const VirtualBufferInfo &vinfo);
  
  // 移除虚拟 buffer 后恢复原始 parasitic
  void restoreFromVirtualBuffer(VertexId orig_drvr_id,
                                const VirtualBufferInfo &vinfo);

  // ---- 容量/模型查询 ----
  float capacitance(VertexId pt_drvr_id,
                    const RiseFall *rf,
                    const DcalcAnalysisPt *ap) const;
  bool isPiModel(VertexId pt_drvr_id,
                 const RiseFall *rf,
                 const DcalcAnalysisPt *ap) const;

private:
  PtGraph *pt_graph_;  // 所属的 PtGraph（non-owning）

  // 按 PtVertexId 索引的 PiElmore 数组
  // pi_elmore_map_[pt_drvr_id] = ConcretePiElmore*[ap_rf_count]
  std::vector<ConcretePiElmore**> pi_elmore_map_;
  
  // 按 PtVertexId 索引的 parasitic network
  // network_map_[pt_drvr_id] = ConcreteParasiticNetwork*[ap_count]
  std::vector<ConcreteParasiticNetwork**> network_map_;

  // 虚拟 buffer 备份
  struct VBufBackup {
    ConcretePiElmore **saved_pi;
    ConcreteParasiticNetwork **saved_network;
  };
  std::unordered_map<VertexId, VBufBackup> vbuf_backups_;
};
```

### 2.2 数据索引模式对比

| | 当前 LocalParasitics | 新 PtParasitics |
|---|---|---|
| **索引 key** | `const Pin*` (全局) | `PtVertexId` (PtGraph-local) |
| **存储结构** | `Map<Pin*, ConcreteParasitic**>` (hash map) | `std::vector<...>` (连续数组，O(1) 访问) |
| **作用域** | 全局唯一，所有 PtGraph 共享 | 每个 PtGraph 一个，完全隔离 |
| **线程安全** | 需要 `g_odb_sta_access_mutex` | 无锁（PtGraph 粒度独占） |
| **Network Map** | 浅拷贝全局指针 | 深拷贝相关子网 |

### 2.3 在 PtGraph 中的集成

```cpp
// PtGraph.hh 中新增
class PtGraph {
  // ... existing members ...
  
  PtParasitics *ptParasitics() { return pt_parasitics_.get(); }
  const PtParasitics *ptParasitics() const { return pt_parasitics_.get(); }

private:
  std::unique_ptr<PtParasitics> pt_parasitics_;
};
```

---

## 三、分阶段开发计划

### Phase 1: 基础框架搭建（无功能变更）

**目标**：创建 PtParasitics 类的骨架，通过编译，不改变任何现有行为。

**文件变更**：
- 新增 `PtParasitics.hh` / `PtParasitics.cc`
- 修改 `CMakeLists.txt`（添加新源文件）
- 修改 `PtGraph.hh`（添加 `unique_ptr<PtParasitics>` 成员）

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 1.1 | 定义 `PtParasitics` 类骨架 | 继承 `StaState`；包含构造/析构、空的 `initFromGlobal()`、空的 `findPiElmore()`、空的 `recomputeAll()` |
| 1.2 | 在 PtGraph 中嵌入 | 添加 `std::unique_ptr<PtParasitics> pt_parasitics_` 成员和 accessor；在 `PtGraph` 构造函数中创建 |
| 1.3 | 编译验证 | 确保 `make -j` 通过，无 link error |

**验收标准**：编译通过，现有测试全部通过，无行为变化。

---

### Phase 2: 实现 initFromGlobal — 按 PtGraph 范围拷贝 parasitic

**目标**：PtParasitics 能在 PtGraph 构建时从全局 ConcreteParasitics 拷贝所需的 parasitic data。

**关键设计决策**：

```
initFromGlobal() 流程:
  1. 遍历 PtGraph 中所有 PtVertex
  2. 对 type == RefDriver || RefOutput 的 vertex:
     a. 获取其 Pin* → 在全局 drvr_parasitic_map_ 中查找
     b. 深拷贝 ConcretePiElmore 到 pi_elmore_map_[pt_vertex_id]
  3. 对每个相关 Net:
     a. 在全局 parasitic_network_map_ 中查找
     b. 深拷贝 ConcreteParasiticNetwork 到 network_map_[pt_vertex_id]
```

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 2.1 | 实现 `pi_elmore_map_` 存储 | `std::vector` 按 `vertexCount()` 预分配，每个 driver vertex 位置分配 `ConcretePiElmore*[ap_rf_count]` |
| 2.2 | 实现 `network_map_` 存储 | 类似结构，存 `ConcreteParasiticNetwork*[ap_count]` |
| 2.3 | 实现 PiElmore 深拷贝 | 复用现有 `ParasiticCopyHelper::getCopy(ConcretePiElmore*)` 逻辑 |
| 2.4 | 实现 Network 深拷贝 | `ConcreteParasiticNetwork` 的完整拷贝（nodes + resistors + capacitors）— 这是最复杂的部分，需要重建 node 到 pin 的映射 |
| 2.5 | 在 `PtGraph::createParasiticsNetworks()` 中调用 | 替换当前的 placeholder 为 `pt_parasitics_->initFromGlobal(global_parasitics)` |
| 2.6 | 实现析构 | 正确释放所有深拷贝的 parasitic 对象，避免 double free |

**风险与难点**：
- `ConcreteParasiticNetwork` 深拷贝需要重建内部 node graph 的连接关系
- `ParasiticNode` 内的 `Pin*` 指针仍然引用全局对象（只读，可共享）
- 需要考虑 `ap_count` 变化时的内存布局

**验收标准**：`initFromGlobal()` 能正确拷贝数据，通过 `printLocalParasitics` 验证值一致。

---

### Phase 3: 实现查找接口 — findPiElmore / findParasiticNetwork

**目标**：提供按 `PtVertexId` 的查找接口，与 `LocalParasitics::findLocalParasitic` 功能等价。

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 3.1 | 实现 `findPiElmore()` | 从 `pi_elmore_map_[pt_drvr_id]` 中按 `ap_rf_index` 取值 |
| 3.2 | 实现 `findParasiticNetwork()` | 从 `network_map_[pt_drvr_id]` 中按 `ap->index()` 取值 |
| 3.3 | 实现 `capacitance()` | 从 PiElmore 取 `c1 + c2` |
| 3.4 | 实现 `isPiModel()` | 检查对应位置的 parasitic 类型 |
| 3.5 | 添加 Pin→PtVertexId 转换层 | 在需要从 Pin* 查找的场景提供适配函数，使用 PtGraph 的 `vertex_map_` |

**验收标准**：所有查找接口的单元测试通过。

---

### Phase 4: 实现 reduce 计算 — 独立的 Pi-Elmore 归约

**目标**：PtParasitics 能独立执行 reduce 操作，不依赖全局锁。

**关键变更**：将 `LocalReduceToPiElmore` 的调用从 `LocalParasitics::makeLocalPiElmore`（需要全局锁）迁移到 `PtParasitics::reduceToPiElmore`（无锁）。

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 4.1 | 实现 `reduceToPiElmore()` | 从本地 `network_map_` 获取 parasitic network，调用 `LocalReduceToPiElmore` 做 reduce，结果写入本地 `pi_elmore_map_` |
| 4.2 | 实现 `recomputeAll()` | 遍历 PtGraph 所有 driver vertex，调用 `reduceToPiElmore()` |
| 4.3 | 移除全局锁 | `reduceToPiElmore` 内部不需要 `g_odb_sta_access_mutex`，因为所有数据都是 PtGraph-local 的 |
| 4.4 | 处理 `pinCapacitance` 调用 | `LocalReduceToPi::localPinCapacitance` 需要查 PtGraph 中的 virtual cell cap — 确保通过 `pt_graph_` 指针访问 |

**风险与难点**：
- `LocalReduceToPi` 在 DFS 过程中调用 `parasitics_->pin(node)` 等全局查询 — 这些只读调用是否线程安全需要验证
- `sdc_->pinCapacitance()` 是否有写操作需要审查

**验收标准**：`recomputeAll()` 在多线程场景下结果正确，无 data race（通过 ThreadSanitizer 验证）。

---

### Phase 5: 接入 LocalSta — 替换 LocalParasitics 调用点

**目标**：LocalSta 的所有 parasitic 操作迁移到使用 PtParasitics。

**影响范围**（LocalSta.cc 中的调用点）：

```
Line  54:  local_parasitics_(new LocalParasitics(sta))        → 移除
Line  81:  local_parasitics_->copyState(sta)                  → 移除
Line 671:  local_parasitics_->findLocalParasitic(...)         → pt_graph->ptParasitics()->findPiElmore(...)
Line 1368: local_parasitics_->recomputeLocalParasitics(...)   → pt_graph->ptParasitics()->recomputeAll()
Line 1438: local_parasitics_->findLocalParasitic(...)         → pt_graph->ptParasitics()->findPiElmore(...)
Line 1440: local_parasitics_->isPiModel(...)                  → pt_graph->ptParasitics()->isPiModel(...)
Line 1445: local_parasitics_->capacitance(...)                → pt_graph->ptParasitics()->capacitance(...)
Line 1470-1534: 同上模式                                       → 统一迁移
```

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 5.1 | 修改 `localParasiticLoad()` | 两个重载都改为从 `pt_graph->ptParasitics()` 查询；pin-only 版本需要先做 Pin→PtVertexId 转换 |
| 5.2 | 修改 `recomputeLocalParasitics()` | 改为调用 `pt_graph->ptParasitics()->recomputeAll()` |
| 5.3 | 修改 `seedNoDrvrSlew()` | `findLocalParasitic` 调用替换 |
| 5.4 | 修改 `printLocalParasitics()` | 改用 PtParasitics 接口 |
| 5.5 | 修改 `loadLocalParasitics()` | 改用 PtParasitics 接口 |
| 5.6 | 保留 `LocalParasitics` 作为 fallback | 添加编译开关 `USE_PT_PARASITICS`，先双轨运行对比结果 |
| 5.7 | 对比验证 | 运行所有 test benchmark，对比 PtParasitics 与 LocalParasitics 的计算结果差异 |

**验收标准**：所有 benchmark 的 timing 结果与迁移前差异 < 0.1%。

---

### Phase 6: Virtual Buffer 支持 — 消除 `computeVirtualLoadCap` Workaround

**目标**：当 LrRebuffer 插入虚拟 buffer 时，PtParasitics 能增量更新 parasitic（而非丢弃整个 parasitic 走估算路径）。

**当前 Workaround 流程**：
```
buildVirtualBuffer()
  → setHasVirtualBuffer(true)       // 标记 parasitic 失效
  → updateLocalTiming()
    → localParasiticLoad()
      → if (hasVirtualBuffer) 
          computeVirtualLoadCap()    // 只用 pin cap 求和，精度差
```

**新流程**：
```
buildVirtualBuffer()
  → ptParasitics()->rebuildForVirtualBuffer(drvr_id, vinfo)
    → 1. 备份原始 driver 的 PiElmore 和 Network
    → 2. 将原始 network 按 buffer insertion point 拆分为多段子网
    → 3. 对每段子网做 reduce，得到各自的 PiElmore
    → 4. 虚拟 buffer vertex 的 PiElmore 就是其下游子网的 reduce 结果
  → updateLocalTiming()                // 正常路径，无需特殊处理
  → ptParasitics()->restoreFromVirtualBuffer(drvr_id, vinfo)
    → 从备份恢复
```

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 6.1 | 实现 RC Network 拆分 | 给定 parasitic network 和 buffer insertion point (ParasiticNode)，生成两个子网络：drvr→buffer_input 和 buffer_output→loads |
| 6.2 | 实现 `rebuildForVirtualBuffer()` | 备份 + 拆分 + reduce |
| 6.3 | 实现 `restoreFromVirtualBuffer()` | 从备份恢复 |
| 6.4 | 修改 `LrRebuffer::buildVirtualBuffer()` | 调用 `rebuildForVirtualBuffer()` 而非设 `hasVirtualBuffer` flag |
| 6.5 | 修改 `LrRebuffer::removeVirtualBuffer()` | 调用 `restoreFromVirtualBuffer()` |
| 6.6 | 移除 `computeVirtualLoadCap()` 路径 | `localParasiticLoad()` 中不再需要 `hasVirtualBuffer` 分支 |

**风险与难点**：
- RC network 拆分是计算几何/图论问题，需要确定 buffer insertion point 在 RC tree 中的位置
- 拆分后的子网络 reduce 精度需要与完整网络 reduce 对比验证
- Junction (multi-sink) 场景下拆分逻辑复杂

**验收标准**：
- `evaluateOption()` 中使用增量 parasitic 的 slack 评估结果与 ground truth 差异 < 5%
- 完全移除 `hasVirtualBuffer` flag 和 `computeVirtualLoadCap()` 路径

---

### Phase 7: 清理与性能优化

**目标**：移除旧的 `LocalParasitics`，消除全局锁，优化内存布局。

**具体任务**：

| # | 任务 | 详细说明 |
|---|------|---------|
| 7.1 | 移除 `LocalParasitics` 类 | 删除 `LocalParasitics.hh/.cc`，从 `LocalSta` 中移除 `local_parasitics_` 成员 |
| 7.2 | 移除 `g_odb_sta_access_mutex` | 确认所有对全局 STA/ODB 的写操作已在 `TaskArranger` 层处理，不再需要 parasitic 层的锁 |
| 7.3 | 移除 `ParasiticCopyHelper` | 拷贝逻辑已内化到 `PtParasitics::initFromGlobal()` |
| 7.4 | 内存池优化 | `ConcretePiElmore` 使用 arena allocator，减少小对象分配开销 |
| 7.5 | Cache-friendly 布局 | PiElmore 数据按 driver vertex 排列在连续内存中，提高 reduce 遍历的 cache locality |
| 7.6 | 移除 `USE_PT_PARASITICS` 开关 | 确认全部迁移完成后移除双轨代码 |

**验收标准**：
- 编译无 warning
- ThreadSanitizer 无 data race
- 多线程性能提升 > 20%（通过去除全局锁瓶颈）

---

## 四、依赖关系图

```
Phase 1 (骨架)
    │
    ▼
Phase 2 (initFromGlobal)
    │
    ├──────────────┐
    ▼              ▼
Phase 3 (查找)  Phase 4 (reduce)
    │              │
    └──────┬───────┘
           ▼
    Phase 5 (接入 LocalSta)
           │
           ▼
    Phase 6 (Virtual Buffer)
           │
           ▼
    Phase 7 (清理优化)
```

Phase 3 和 Phase 4 可以并行开发。

---

## 五、关键技术细节

### 5.1 ConcreteParasiticNetwork 深拷贝策略

```
原始 Network:
  nodes_: Map<key, ParasiticNode*>     ← 每个 node 有 pin/net 引用
  resistors_: vector<ParasiticResistor*>
  capacitors_: vector<ParasiticCapacitor*>
  
深拷贝时需要:
  1. 创建所有 node 的副本，建立 old_node→new_node 映射
  2. 创建所有 resistor 的副本，用映射替换内部 node 指针
  3. 创建所有 capacitor 的副本，同上
  4. Pin* 和 Net* 保持原始指针（只读共享安全）
```

### 5.2 `PtVertexId` 到 `Pin*` 的映射

```
PtGraph 已有 vertex_map_: Map<Vertex*, VertexId>
PtVertex 已有 vertex()->pin() 获取 Pin*

新增反向映射（在 PtParasitics 中）:
  pin_to_pt_id_: unordered_map<const Pin*, VertexId>
  在 initFromGlobal() 时构建
```

### 5.3 Virtual Buffer 的 RC Network 拆分示意

```
原始 RC Tree:
  D ─R1─ N1 ─R2─ L1
              ├─R3─ L2
              └─R4─ N2 ─R5─ L3

在 N1 处插入 buffer 后:

子网 1 (driver → buffer input):
  D ─R1─ N1(cap = Cin_buffer)

子网 2 (buffer output → loads):
  Bout ─R2─ L1
        ├─R3─ L2
        └─R4─ N2 ─R5─ L3

两个子网分别做 PiElmore reduce:
  子网1 → PiElmore for D vertex
  子网2 → PiElmore for buffer_out vertex
```

### 5.4 线程安全分析

| 操作 | 当前 | 迁移后 |
|------|------|--------|
| `findLocalParasitic` | 读全局 map（需锁） | 读本地 vector（无锁） |
| `makeLocalPiElmore` | 写全局 map + 读 network（全局锁） | 写本地 vector + 读本地 network（无锁） |
| `recomputeLocalParasitics` | 写全局 map（全局锁） | 写本地 vector（无锁） |
| `initParasiticMapFromBase` | 读全局 map（构造时串行） | 读全局 map（构造时串行，不变） |

---

## 六、测试策略

### 6.1 单元测试

- `PtParasitics::initFromGlobal()` 后与全局 parasitic 值精确比较
- `PtParasitics::reduceToPiElmore()` 与 `LocalParasitics::makeLocalPiElmore()` 结果对比
- `rebuildForVirtualBuffer()` / `restoreFromVirtualBuffer()` 的备份恢复正确性

### 6.2 集成测试

- 在 `TestLrf` 中添加 PtParasitics 的端到端测试
- 双轨运行（Phase 5.6）：同时用 LocalParasitics 和 PtParasitics 计算，assert 结果一致

### 6.3 性能测试

- 单线程：PtParasitics vs LocalParasitics 的查找/reduce 延迟对比
- 多线程：移除全局锁后的并行加速比
- 内存：PtParasitics per-graph 内存开销 vs 全局 LocalParasitics

### 6.4 Benchmark

| Benchmark | 用途 |
|-----------|------|
| aes_nangate45 | 小规模功能验证 |
| aes_asap7 | 中等规模性能测试 |
| aes_sky130hd | 工艺多样性验证 |

---

## 七、风险与缓解

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|---------|
| ConcreteParasiticNetwork 深拷贝实现复杂度高 | 高 | Phase 2 延期 | 先实现浅拷贝 + COW (Copy-on-Write)，后续再做完整深拷贝 |
| `sdc_->pinCapacitance()` 有隐藏的写状态 | 中 | Phase 4 需要额外锁 | 审查 SDC 代码，必要时对 SDC 查询加 read lock |
| RC Network 拆分精度不足 | 中 | Phase 6 评估精度下降 | 对拆分点增加补偿电容项 |
| 内存增长（每个 PtGraph 一份副本）| 低 | 大规模设计 OOM | PtGraph 通常只有 10-50 个 vertex，parasitic 数据极小；可加 lazy init |

---

## 八、预期收益

| 指标 | 当前 | 预期 |
|------|------|------|
| `recomputeLocalParasitics` 锁等待 | 全局锁串行 | 完全消除 |
| `makeLocalPiElmore` 锁等待 | 全局锁串行 | 完全消除 |
| Virtual buffer 评估精度 | `computeVirtualLoadCap` 估算 | 精确 PiElmore reduce |
| 多线程加速比 (parasitic-bound) | ~1x (被锁限制) | ~Nx (N = 线程数) |
| 代码维护性 | `LocalParasitics` 继承 `ConcreteParasitics` (tight coupling) | `PtParasitics` 组合 `StaState` (loose coupling) |

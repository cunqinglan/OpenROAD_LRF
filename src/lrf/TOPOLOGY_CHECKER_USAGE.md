# TopologyChecker 使用说明

## 功能概述

TopologyChecker 是一个用于检测并发环境下拓扑遍历正确性的工具。在多线程并行 gate sizing 过程中，它可以检测以下三种违规情况：

1. **FANIN_NOT_VISITED**: Fanin 节点还没被访问就访问当前节点（违反拓扑顺序）
2. **SIBLING_ORDER_VIOLATION**: 同一个 fanout 的 sibling 节点被乱序访问
3. **CONCURRENT_MODIFICATION**: 同一个节点被多个线程同时修改（并发冲突）

## 使用方法

### 1. 启用拓扑检查

在调用 `visitParallel()` 之前启用检查：

```cpp
TaskArranger* arranger = new TaskArranger(sta);
// ... 构建依赖图 ...

// 启用拓扑检查
arranger->enableTopologyCheck(true);

// 并行访问
arranger->visitParallel(sta, local_sta, resizer, visitor);
```

### 2. 查看违规报告

违规会实时打印到 stdout，格式如下：

```
!!! TOPOLOGY VIOLATION !!!
Type: FANIN_NOT_VISITED
Vertex: _12345_
Thread: 140123456789000
Detail: Fanin _12340_ has not been visited yet

!!! TOPOLOGY VIOLATION !!!
Type: CONCURRENT_MODIFICATION
Vertex: _12346_
Thread: 140123456789001
Detail: Vertex is already being modified by thread 140123456789000
```

### 3. 访问结束后查看汇总

`visitParallel()` 结束时会自动打印违规汇总：

```
=== Topology Violations Summary ===
Total violations: 5

[Violation 1]
Type: FANIN_NOT_VISITED
Vertex: _12345_
Thread: 140123456789000
Detail: Fanin _12340_ has not been visited yet

[Violation 2]
...
====================================
```

也可以手动调用：

```cpp
arranger->printTopologyViolations();
```

### 4. TCL 接口使用

```tcl
# 启用拓扑检查
enable_topology_check true

# 运行并行优化
parallel_gate_sizing ...

# 查看违规（如果需要手动查看）
print_topology_violations
```

## 工作原理

TopologyChecker 通过跟踪三个状态集合来检测违规：

1. **visited_vertices_**: 已经访问过的节点
2. **vertices_being_modified_**: 正在被修改的节点（及其所属线程）
3. **committed_vertices_**: 已经提交到 DB 的节点

在 `runTask()` 中插入了三个检查点：

```cpp
void TaskArranger::runTask(ParallelLrVisitor *visitor, InstVertex* inst_vertex)
{
  // 检查点 1: 访问时检查 fanin 和 sibling 顺序
  if (enable_topology_check_ && topology_checker_) {
    topology_checker_->onVisit(inst_vertex, std::this_thread::get_id());
  }
  
  if (visitor->visit(inst_vertex->inst())) 
  {
    // 检查点 2: 修改前检查是否有并发修改
    if (enable_topology_check_ && topology_checker_) {
      topology_checker_->onBeforeModify(inst_vertex, std::this_thread::get_id());
    }
    
    std::lock_guard<std::mutex> lock_odb(g_odb_sta_access_mutex);
    visitor->applyChangesToDb(resizer_);
    
    // 检查点 3: 修改后标记为已提交
    if (enable_topology_check_ && topology_checker_) {
      topology_checker_->onAfterModify(inst_vertex, std::this_thread::get_id());
    }
  }
  // ...
}
```

## 性能影响

- 启用检查会增加少量开销（主要是 mutex 锁和集合查找）
- 建议仅在调试并发问题时启用
- 正常运行时关闭以获得最佳性能

## 检测到违规后怎么办

### FANIN_NOT_VISITED

这表明任务调度有问题，某个节点在其 fanin 完成前就被调度了。可能原因：

1. 引用计数初始化错误
2. `decreOutRefCount()` 实现有误
3. 依赖边缺失

**解决方法**: 检查 `initVertexRefCounts()` 和边的构建逻辑。

### SIBLING_ORDER_VIOLATION

同一个 fanout 的多个 fanin 被并发访问，但可能有数据竞争。

**解决方法**: 这可能是正常的并发行为，除非导致了 segfault。如果有 segfault，需要在 visitor 中加锁保护共享数据。

### CONCURRENT_MODIFICATION

**这是严重问题！** 两个线程试图同时修改同一个节点。

**解决方法**: 
1. 检查任务调度逻辑，确保没有重复调度
2. 检查 `createTask()` 是否有线程安全问题
3. 可能需要在调度层面加锁

## 示例输出

```
Visit with 8 threads
!!! TOPOLOGY VIOLATION !!!
Type: CONCURRENT_MODIFICATION
Vertex: _45678_
Thread: 139876543210000
Detail: Vertex is already being modified by thread 139876543210001

!!! TOPOLOGY VIOLATION !!!
Type: FANIN_NOT_VISITED
Vertex: _45679_
Thread: 139876543210002
Detail: Fanin _45670_ has not been visited yet

Visitor 0 runtime profile:
...

=== Topology Violations Summary ===
Total violations: 2

[Violation 1]
Type: CONCURRENT_MODIFICATION
Vertex: _45678_
Thread: 139876543210000
Detail: Vertex is already being modified by thread 139876543210001

[Violation 2]
Type: FANIN_NOT_VISITED
Vertex: _45679_
Thread: 139876543210002
Detail: Fanin _45670_ has not been visited yet
====================================
```

## 注意事项

1. **不影响程序执行**: 检测到违规只会打印警告，不会中断程序
2. **线程安全**: 所有检查操作都是线程安全的
3. **内存占用**: 需要存储所有访问过的节点 ID，大规模电路可能占用较多内存
4. **实时输出**: 违规会立即打印，方便实时观察

## 禁用检查

```cpp
arranger->enableTopologyCheck(false);  // 默认就是 false
```

或者不调用 `enableTopologyCheck()`，默认就是禁用状态。

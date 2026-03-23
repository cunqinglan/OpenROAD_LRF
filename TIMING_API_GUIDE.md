# Timing API 命令使用指南

本文档详细说明了 OpenROAD 中 Timing 模块的并行 resize 相关命令的使用方法。

## 概览

Timing 模块提供了多种并行 resize 测试方法，用于评估和优化电路的 timing resizing 性能。这些方法支持不同的算法和配置选项。

## 主推命令

### ⭐ 1. `testParallelResizeByArrayWithPrecheck`

**最推荐使用的命令**，结合了数组式处理和预检查机制。

```python
timing.testParallelResizeByArrayWithPrecheck(
    num_iterations=20000000,    # 迭代次数或数据量
    num_threads=12,              # 并行线程数
    batch_size=3,                # 每批处理的数据大小
    enable_logging=True,         # 是否启用日志记录
    timeout=10,                  # 超时时间（秒）
    lr_mode=args.lr_mode,        # LR resizing 模式
    precheck_threshold=0.3       # 预检查阈值（0.0-1.0）
)
```

**特点：**
- ✅ 通过预检查过滤，减少不必要的计算
- ✅ 数组式批处理，提高缓存命中率
- ✅ 线程安全的并行执行
- ✅ 阈值灵活配置（0.3 表示只处理满足条件 30% 的数据）

**适用场景：** 大规模设计的 timing resizing，追求整体性能

---

### ⭐ 2. `testParallelLrResizing`

**最直接的 LR resizing 测试命令**，使用简化的并行策略。

```python
timing.testParallelLrResizing(
    num_iterations=20000000,    # 迭代次数或数据量
    num_threads=20,              # 并行线程数
    batch_size=6,                # 每批处理的数据大小
    enable_logging=True,         # 是否启用日志记录
    timeout=10,                  # 超时时间（秒）
    lr_mode=args.lr_mode         # LR resizing 模式
)
```

**特点：**
- ✅ 简洁直接的 LR resizing 实现
- ✅ 支持高度并行化（默认 20 线程）
- ✅ 最小化内存开销
- ✅ 快速测试反馈

**适用场景：** 快速性能评估、LR resizing 特性测试

---

## 其他可选命令

### 3. `testParallelResizeByArray`

基础的数组式并行 resize，不含预检查机制。

```python
timing.testParallelResizeByArray(
    num_iterations=20000000,    # 迭代次数或数据量
    num_threads=12,              # 并行线程数
    batch_size=6,                # 每批处理的数据大小
    enable_logging=True,         # 是否启用日志记录
    timeout=20,                  # 超时时间（秒）
    lr_mode=args.lr_mode         # LR resizing 模式
)
```

**特点：**
- 纯数组式批处理
- 无预检查，处理所有数据
- 中等性能和内存占用

---

### 4. `testParallelResizeByArrayWithBuffering`

数组式处理 + 缓冲机制，减少内存分配次数。

```python
timing.testParallelResizeByArrayWithBuffering(
    num_iterations=20000000,    # 迭代次数或数据量
    num_threads=12,              # 并行线程数
    batch_size=3,                # 每批处理的数据大小
    enable_logging=True,         # 是否启用日志记录
    timeout=10,                  # 超时时间（秒）
    lr_mode=args.lr_mode         # LR resizing 模式
)
```

**特点：**
- 数组式 + 缓冲机制
- 减少内存碎片化
- 提高内存访问效率

---

### 5. `testParallelResizingBuffering`

简化版本，仅包含并行 + 缓冲。

```python
timing.testParallelResizingBuffering(
    num_iterations=20000000,    # 迭代次数或数据量
    num_threads=1,               # 并行线程数（可单线程）
    batch_size=6,                # 每批处理的数据大小
    enable_logging=True,         # 是否启用日志记录
    timeout=10,                  # 超时时间（秒）
    lr_mode=args.lr_mode         # LR resizing 模式
)
```

**特点：**
- 核心缓冲优化
- 灵活的线程配置
- 适合基准测试对比

---

## 参数说明

| 参数名 | 类型 | 范围/含义 | 默认值 | 说明 |
|--------|------|---------|--------|------|
| `num_iterations` | int | > 0 | - | 迭代次数或数据量，通常设为 20000000 |
| `num_threads` | int | 1-32 | - | 并行线程数，推荐 1-20 |
| `batch_size` | int | 1-10 | - | 每批处理的数据大小，通常 3-6 |
| `enable_logging` | bool | True/False | True | 是否输出详细日志 |
| `timeout` | int | > 0 | 10 | 超时时间（秒） |
| `lr_mode` | enum | BALANCED/LEFT/RIGHT | - | LR resizing 模式选择 |
| `precheck_threshold` | float | 0.0-1.0 | 0.3 | 预检查阈值（仅用于 WithPrecheck） |

---

## 使用示例

### 示例 1: 基准测试对比

```python
from openroad import Tech, Design, Timing
import time

# 初始化
tech = Tech()
design = Design(tech)
timing = Timing(design)

# 方法A: 主推方案（含预检查）
start = time.time()
timing.testParallelResizeByArrayWithPrecheck(
    20000000, 12, 3, True, 10, "BALANCED", 0.3
)
time_a = time.time() - start

# 方法B: 简洁方案（LR resizing）
start = time.time()
timing.testParallelLrResizing(20000000, 20, 6, True, 10, "BALANCED")
time_b = time.time() - start

print(f"WithPrecheck: {time_a:.2f}s")
print(f"LrResizing: {time_b:.2f}s")
```

### 示例 2: 调试模式

```python
# 启用日志，小规模数据测试
timing.testParallelResizeByArrayWithPrecheck(
    num_iterations=1000000,      # 小规模测试
    num_threads=4,                # 少线程便于调试
    batch_size=3,
    enable_logging=True,          # 启用日志
    timeout=30,
    lr_mode="BALANCED",
    precheck_threshold=0.5        # 较高阈值
)
```

### 示例 3: 性能优化

```python
# 大规模性能测试
timing.testParallelLrResizing(
    num_iterations=100000000,     # 大规模数据
    num_threads=16,                # 充分利用多核
    batch_size=6,
    enable_logging=False,         # 关闭日志减少开销
    timeout=60,
    lr_mode="BALANCED"
)
```

---

## 选择指南

**我应该选择哪个命令？**

| 需求 | 推荐命令 | 理由 |
|------|---------|------|
| 整体性能最优 | `testParallelResizeByArrayWithPrecheck` | 预检查机制过滤低效数据 |
| 快速评估 | `testParallelLrResizing` | 简洁高效，反馈快 |
| 算法研究 | `testParallelResizeByArray` | 纯数组式，便于对比 |
| 内存优化 | `testParallelResizeByArrayWithBuffering` | 缓冲机制减少分配 |
| 基准测试 | `testParallelResizingBuffering` | 稳定可控 |

---

## 常见问题

### Q: `precheck_threshold` 应该设置多少？
**A:**
- `0.1-0.3`: 严格过滤，只处理高优先级数据
- `0.5`: 平衡方案
- `0.7-1.0`: 宽松过滤，处理大部分数据

### Q: 线程数设置多少合适？
**A:** 一般推荐为 CPU 核心数的 50-100%。例如 12 核 CPU，推荐设为 6-12。

### Q: 为什么执行超时了？
**A:**
- 增加 `timeout` 参数值
- 减少 `num_iterations`
- 提高 `num_threads` 加速处理

### Q: 日志输出对性能影响大吗？
**A:** 有一定影响（通常 5-15%），生产环境推荐设 `enable_logging=False`

---

## 性能对比参考

| 命令 | CPU效率 | 内存占用 | 适用规模 | 推荐度 |
|------|--------|---------|---------|--------|
| WithPrecheck | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | 大 | ⭐⭐⭐⭐⭐ |
| LrResizing | ⭐⭐⭐⭐ | ⭐⭐ | 中-大 | ⭐⭐⭐⭐⭐ |
| ResizeByArray | ⭐⭐⭐ | ⭐⭐⭐ | 中 | ⭐⭐⭐ |
| WithBuffering | ⭐⭐⭐⭐ | ⭐⭐ | 中-大 | ⭐⭐⭐ |
| Buffering | ⭐⭐⭐ | ⭐ | 小-中 | ⭐⭐ |

---

## 更新日志

- **v1.0** (2026-03): 初版文档，记录 5 个主要命令

# Bug Report: `trySwapPrecheck` 重复调用局部 STA 导致性能瓶颈

## 文件位置

`src/lrf/src/ParallelVisitor.cc`，函数 `ParallelLrVisitor::trySwapPrecheck`（约第 332 行）

## 实测 Profile（10 线程，10911 个 combinational instances）

```
equiv_cell_check:      24.02 s   ← 占总时间 99%
equiv_cell_count:      6844      ← 累计候选 cell 数（所有线程之和）
pt_graph_construction: 0.18 s
precheck total:        24.20 s
```

## 问题描述

函数内对每个 instance 的候选 cell 进行了**两遍完整局部 STA 评估**：

**Pass 1**（收集原始代价，约第 396～432 行）：
```cpp
for (size_t i = 0; i < candidates.size(); i++) {
    // ... legalCheckBeforeSwap ...
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, arc_delay_calc_, cand).delay_lm_sum;  // 完整局部STA
    // ... legalCheckAfterSwap ...
    if (cand == ori_cell) { ori_cost = cost; ori_slack = slack; }
    // 其他候选的 cost 结果被丢弃！
}
```

**Pass 2**（寻找最优 cell，约第 438～469 行）：
```cpp
for (size_t i = 0; i < candidates.size(); i++) {
    // ... legalCheckBeforeSwap（重复） ...
    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, arc_delay_calc_, cand).delay_lm_sum;  // 完整局部STA（重复！）
    // ... legalCheckAfterSwap（重复） ...
    if (cost < best_cost ...) best_cost = cost;
}
```

`increAndGetLocalTimingCost` 内部执行：
```
virtualReplaceCell → findLocalDelays → findLocalArrivals → findLocalRequireds
```
这是完整的局部时序传播。**6844 个候选被执行了约 2×6844 ≈ 13688 次**，Pass 1 对非 ori_cell 的结果全部白算。

另外，leakage 查表是 O(N) 线性扫描（每个候选遍历整个 `full_equiv_cells`）：
```cpp
for (size_t j = 0; j < full_equiv_cells->size(); j++) {
    if ((*full_equiv_cells)[j] == cand) { leakage = ...; break; }
}
```

## 修复方案

将两遍 pass 合并为一遍，第一遍遍历时把每个候选的 `{cost, slack}` 缓存进向量，第二遍只在向量内比较，不再重复调用 STA。同时预建 `unordered_map` 替代 leakage 的 O(N) 扫描。

**将约第 391～469 行的两段 for 循环替换为以下代码：**

```cpp
  // Single-pass evaluation: cache each candidate's result to avoid running STA twice.
  auto t_eval_start = std::chrono::high_resolution_clock::now();

  // Build O(1) leakage lookup to replace the O(N) linear scan per candidate.
  std::unordered_map<sta::LibertyCell*, float> leakage_cache;
  if (cell_info && full_equiv_cells) {
    for (size_t j = 0; j < full_equiv_cells->size(); j++)
      leakage_cache[(*full_equiv_cells)[j]] = cell_info->cell_leakages[j];
  }

  struct CandResult {
    float cost  = std::numeric_limits<float>::max();
    sta::Slack slack = 0.0f;
  };
  std::vector<CandResult> cand_results(candidates.size());

  float ori_cost  = std::numeric_limits<float>::max();
  float ori_slack = 0.0f;

  for (size_t i = 0; i < candidates.size(); i++) {
    sta::LibertyCell *cand = candidates[i];

    if (!local_sta_->legalCheckBeforeSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float leakage = 0.0f;
    auto lk_it = leakage_cache.find(cand);
    if (lk_it != leakage_cache.end())
      leakage = lk_it->second;

    float delay_lm_sum = local_sta_->increAndGetLocalTimingCost(
        pt_graph, arc_delay_calc_, cand).delay_lm_sum;

    if (!local_sta_->legalCheckAfterSwap(inst, cand, nullptr, nullptr, pt_graph)
        && cand != ori_cell)
      continue;

    float cost       = swapCost(delay_lm_sum, leakage);
    sta::Slack slack = local_sta_->localSlackAroundRef(pt_graph);
    cand_results[i]  = {cost, slack};

    if (cand == ori_cell) {
      ori_cost  = cost;
      ori_slack = slack;
    }
  }

  // Derive best cost from cached results — no second STA pass needed.
  if (ori_cost == std::numeric_limits<float>::max()) {
    // pt_graph is owned by local_sta_->local_graphs_, do NOT delete here
    return 0.0f;
  }

  float best_cost = ori_cost;
  for (size_t i = 0; i < candidates.size(); i++) {
    if (candidates[i] == ori_cell)
      continue;
    const CandResult &r = cand_results[i];
    if (r.cost == std::numeric_limits<float>::max())
      continue;  // was skipped (illegal)
    // Slack protection: same as trySwapByArray
    if (r.cost < best_cost && r.slack >= ori_slack * slack_margin_)
      best_cost = r.cost;
  }
```

## 预期收益

| 指标 | 优化前 | 优化后 |
|------|--------|--------|
| `increAndGetLocalTimingCost` 调用次数 | ~13688 | ~6844 |
| leakage 查表复杂度 | O(N) per candidate | O(1) per candidate |
| `equiv_cell_check` 时间（估算） | 24.0 s | ~12 s |

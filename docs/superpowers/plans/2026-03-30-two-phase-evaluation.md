# Two-Phase Solution Evaluation with ECO-Based RepairSetup

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Re-rank top-K solutions from Phase 1 (fork-based raw slack) using localized repairSetup in the parent process with ECO undo, so the best solution accounts for sizing/buffering/pin-swap potential.

**Architecture:** After Phase 1 ranks all N solutions by raw slack, Phase 2 takes the top-K (configurable, default 5) and evaluates them sequentially in the parent: `beginEco` → `InsertAbcMapSolution` → `performIncreDpl` → `networkChanged` → `updateTiming` → per-endpoint `repairSetup` → measure post-repair slack → `undoEco`. The solution with the best post-repair slack is permanently applied.

**Tech Stack:** ODB ECO journaling (`beginEco`/`endEco`/`undoEco`), Resizer `repairSetup(const sta::Pin*)`, existing `forkEvaluateSolutions` for Phase 1.

---

### Task 1: Add `top_k` parameter to RemapConfig

**Files:**
- Modify: `src/rmp/include/rmp/RemapConfig.hh`
- Modify: `src/rmp/src/rmp.tcl` (add `-top_k` flag to `position_driven_remap`)

- [ ] **Step 1: Add field to RemapConfig**

In `src/rmp/include/rmp/RemapConfig.hh`, add after the `max_candidates` field:

```cpp
  // Number of top solutions from Phase 1 to re-evaluate with repairSetup
  // in Phase 2. Set to 0 to skip Phase 2 (raw slack ranking only).
  int top_k = 5;
```

- [ ] **Step 2: Wire up the Tcl parameter**

In `src/rmp/src/rmp.tcl`, add `-top_k` to the `position_driven_remap` command definition and pass it through to the config struct. Follow the existing pattern used by `-max_solutions` and other integer parameters.

- [ ] **Step 3: Build and verify**

Run: `source /opt/openEuler/gcc-toolset-12/enable && cmake --build build --target openroad -j$(nproc)`
Expected: Clean build, no errors.

- [ ] **Step 4: Commit**

```bash
git add src/rmp/include/rmp/RemapConfig.hh src/rmp/src/rmp.tcl
git commit -m "rmp: add top_k parameter for two-phase evaluation"
```

---

### Task 2: Extract `getCutFanoutEndpoints` helper

The code to collect cut output pins and find fanout endpoints exists in `evaluateSolution` (lines ~2215-2237). Extract it into a reusable method since Phase 2 needs the same logic in the parent.

**Files:**
- Modify: `src/rmp/src/position_driven.hh`
- Modify: `src/rmp/src/position_driven.cc`

- [ ] **Step 1: Declare the helper in the header**

In `src/rmp/src/position_driven.hh`, add inside `class PositionDrivenStrategy`:

```cpp
  // Collect fanout endpoints of the cut's primary outputs.
  // Returns the set of timing endpoint pins reachable from the cut outputs.
  sta::PinSet getCutFanoutEndpoints(
      cut::LogicCut& candidate_cut,
      sta::dbSta* sta,
      sta::dbNetwork* network);
```

- [ ] **Step 2: Implement the helper**

In `src/rmp/src/position_driven.cc`, add the method (extract from `evaluateSolution` lines ~2215-2237):

```cpp
sta::PinSet PositionDrivenStrategy::getCutFanoutEndpoints(
    cut::LogicCut& candidate_cut,
    sta::dbSta* sta,
    sta::dbNetwork* network)
{
  sta::PinSeq cut_output_pins;
  for (sta::Net* output_net : candidate_cut.primary_outputs()) {
    sta::NetPinIterator* pin_iter = network->pinIterator(output_net);
    while (pin_iter->hasNext()) {
      const sta::Pin* pin = pin_iter->next();
      if (network->direction(pin)->isAnyOutput()) {
        cut_output_pins.push_back(pin);
        break;
      }
    }
    delete pin_iter;
  }

  return sta->findFanoutPins(
      &cut_output_pins,
      /*flat=*/true,
      /*endpoints_only=*/true,
      /*inst_levels=*/-1,
      /*pin_levels=*/-1,
      /*thru_disabled=*/false,
      /*thru_constants=*/false);
}
```

- [ ] **Step 3: Update evaluateSolution to use the helper**

Replace the duplicated code in `evaluateSolution` (the `cut_output_pins` block and `findFanoutPins` call) with:

```cpp
  sta::PinSet fanout_endpoints = getCutFanoutEndpoints(candidate_cut, sta, network);
```

- [ ] **Step 4: Build and verify**

Run: `source /opt/openEuler/gcc-toolset-12/enable && cmake --build build --target openroad -j$(nproc)`
Expected: Clean build. Behavior unchanged.

- [ ] **Step 5: Commit**

```bash
git add src/rmp/src/position_driven.hh src/rmp/src/position_driven.cc
git commit -m "rmp: extract getCutFanoutEndpoints helper for reuse"
```

---

### Task 3: Extract `getWorstCutSlack` helper

Similarly, extracting worst-slack computation from `evaluateSolution` so Phase 2 can reuse it.

**Files:**
- Modify: `src/rmp/src/position_driven.hh`
- Modify: `src/rmp/src/position_driven.cc`

- [ ] **Step 1: Declare the helper**

In `src/rmp/src/position_driven.hh`, add:

```cpp
  // Find worst slack among a set of endpoint pins.
  sta::Slack getWorstSlackFromEndpoints(
      const sta::PinSet& fanout_endpoints,
      sta::dbSta* sta);
```

- [ ] **Step 2: Implement the helper**

```cpp
sta::Slack PositionDrivenStrategy::getWorstSlackFromEndpoints(
    const sta::PinSet& fanout_endpoints,
    sta::dbSta* sta)
{
  sta::Slack worst_slack = std::numeric_limits<sta::Slack>::infinity();
  sta::Graph* graph = sta->ensureGraph();
  for (const sta::Pin* pin : fanout_endpoints) {
    sta::Vertex* vertex = nullptr;
    sta::Vertex* bidir = nullptr;
    graph->pinVertices(pin, vertex, bidir);
    if (!vertex) continue;
    sta::Slack slack = sta->vertexSlack(vertex, sta::MinMax::max());
    if (slack < worst_slack) worst_slack = slack;
  }
  return worst_slack;
}
```

- [ ] **Step 3: Update evaluateSolution to use both helpers**

Replace the endpoint collection + slack loop in `evaluateSolution` with:

```cpp
  sta::PinSet fanout_endpoints = getCutFanoutEndpoints(candidate_cut, sta, network);
  int endpoint_count = fanout_endpoints.size();
  sta::Slack worst_slack = getWorstSlackFromEndpoints(fanout_endpoints, sta);
```

- [ ] **Step 4: Build and verify**

Run: `source /opt/openEuler/gcc-toolset-12/enable && cmake --build build --target openroad -j$(nproc)`

- [ ] **Step 5: Commit**

```bash
git add src/rmp/src/position_driven.hh src/rmp/src/position_driven.cc
git commit -m "rmp: extract getWorstSlackFromEndpoints helper"
```

---

### Task 4: Implement `reEvaluateWithRepair` — the Phase 2 core

This is the main new method. It takes a single solution, applies it with ECO wrapping, runs localized repairSetup, measures post-repair slack, then undoes everything.

**Files:**
- Modify: `src/rmp/src/position_driven.hh`
- Modify: `src/rmp/src/position_driven.cc`

- [ ] **Step 1: Declare the method**

In `src/rmp/src/position_driven.hh`, add:

```cpp
  // Phase 2: Re-evaluate a single solution with localized repairSetup
  // in the parent process. Uses ECO journaling so changes can be undone.
  // Returns post-repair worst slack, or lowest() on failure.
  sta::Slack reEvaluateWithRepair(
      abc::Map_MappingSolution_t* pSolution,
      abc::Map_Man_t* pMan,
      abc::Abc_Ntk_t* pOriginalNetwork,
      cut::LogicCut& candidate_cut,
      SeqRemapper& remapper);
```

- [ ] **Step 2: Implement the method**

In `src/rmp/src/position_driven.cc`, add:

```cpp
sta::Slack PositionDrivenStrategy::reEvaluateWithRepair(
    abc::Map_MappingSolution_t* pSolution,
    abc::Map_Man_t* pMan,
    abc::Abc_Ntk_t* pOriginalNetwork,
    cut::LogicCut& candidate_cut,
    SeqRemapper& remapper)
{
  sta::dbSta* sta = remapper.getSta();
  sta::dbNetwork* network = sta->getDbNetwork();
  odb::dbBlock* block = remapper.getDb()->getChip()->getBlock();
  rsz::Resizer* resizer = remapper.getResizer();

  // Start ECO recording so we can undo all changes.
  odb::dbDatabase::beginEco(block);

  try {
    // Suppress ABC stdout.
    int saved_stdout = -1;
    if (!verbose_) {
      fflush(stdout);
      saved_stdout = dup(STDOUT_FILENO);
      int devnull = open("/dev/null", O_WRONLY);
      if (devnull >= 0) {
        dup2(devnull, STDOUT_FILENO);
        close(devnull);
      }
    }

    candidate_cut.InsertAbcMapSolution(
        pSolution, pMan, pOriginalNetwork,
        *remapper.getAbcLibrary(), network, sta,
        remapper.getNameGenerator(), logger_);

    if (saved_stdout >= 0) {
      fflush(stdout);
      dup2(saved_stdout, STDOUT_FILENO);
      close(saved_stdout);
    }

    // Place new instances and update STA.
    remapper.performIncreDpl(candidate_cut, remapper.getDpl());
    sta->networkChanged();
    sta->updateTiming(false);

    // Localized repairSetup on cut-affected endpoints.
    sta::PinSet fanout_endpoints = getCutFanoutEndpoints(
        candidate_cut, sta, network);

    for (const sta::Pin* pin : fanout_endpoints) {
      sta::Vertex* vertex = sta->ensureGraph()->pinLoadVertex(pin);
      if (!vertex) continue;
      sta::Slack ep_slack = sta->vertexSlack(vertex, sta::MinMax::max());
      if (ep_slack < 0) {
        resizer->repairSetup(pin);
      }
    }

    // Re-measure after repair.
    sta->updateTiming(false);
    sta::PinSet post_endpoints = getCutFanoutEndpoints(
        candidate_cut, sta, network);
    sta::Slack post_slack = getWorstSlackFromEndpoints(post_endpoints, sta);

    odb::dbDatabase::endEco(block);
    return post_slack;

  } catch (const std::exception& e) {
    logger_->warn(utl::RES, 450,
        "Phase 2 re-evaluation failed: {}", e.what());
    odb::dbDatabase::endEco(block);
    odb::dbDatabase::undoEco(block);
    sta->networkChanged();
    sta->updateTiming(false);
    return std::numeric_limits<sta::Slack>::lowest();
  }
}
```

- [ ] **Step 3: Build and verify**

Run: `source /opt/openEuler/gcc-toolset-12/enable && cmake --build build --target openroad -j$(nproc)`
Expected: Clean build. Method is not called yet.

- [ ] **Step 4: Commit**

```bash
git add src/rmp/src/position_driven.hh src/rmp/src/position_driven.cc
git commit -m "rmp: implement reEvaluateWithRepair for Phase 2"
```

---

### Task 5: Integrate Phase 2 into `remapOneCut`

Replace the current "apply best solution" block in `remapOneCut` with the two-phase logic: Phase 1 ranks by raw slack, Phase 2 re-ranks the top-K with repairSetup, then the winner is permanently applied.

**Files:**
- Modify: `src/rmp/src/position_driven.cc`

- [ ] **Step 1: Replace the "Apply the best solution" block**

In `remapOneCut`, find the block starting at `// Apply the best solution.` (currently ~line 1546). Replace the entire block from there through `abc::Abc_NtkMapEnumFreeStore(pMan);` with the following two-phase logic:

```cpp
  // ========== Phase 2: Re-evaluate top-K with repairSetup ==========
  // Sort results by slack (best first), collect top-K with their solutions.
  struct RankedSolution {
    int solution_index;
    abc::Map_MappingSolution_t* pSolution;
    sta::Slack raw_slack;
  };
  std::vector<RankedSolution> ranked;
  ranked.reserve(evaluated_count);
  for (auto& res : results) {
    if (res.success) {
      ranked.push_back({res.solution_index, res.pSolution, res.slack});
    }
  }
  // Also include UCT results if any were collected.
  // (UCT results are already in evaluated_count and best_slack tracking above.)
  // ranked already has all successful results from both phases.

  std::sort(ranked.begin(), ranked.end(),
            [](const RankedSolution& a, const RankedSolution& b) {
              return a.raw_slack > b.raw_slack;  // best (highest) slack first
            });

  const int top_k = config_.top_k;
  odb::dbBlock* block = remapper.getDb()->getChip()->getBlock();

  abc::Map_MappingSolution_t* pSolutionFinal = nullptr;
  sta::Slack final_best_slack = std::numeric_limits<sta::Slack>::lowest();
  int final_best_index = -1;

  if (top_k > 0 && static_cast<int>(ranked.size()) > 1) {
    int k = std::min(top_k, static_cast<int>(ranked.size()));
    logger_->info(utl::RES, 451,
        "Phase 2: re-evaluating top {} solutions with repairSetup.", k);

    for (int ri = 0; ri < k; ++ri) {
      auto& cand = ranked[ri];
      logger_->info(utl::RES, 452,
          "Phase 2 candidate {}/{}: solution {} (raw slack={:.4e})",
          ri + 1, k, cand.solution_index + 1, cand.raw_slack);

      sta::Slack repaired_slack = reEvaluateWithRepair(
          cand.pSolution, map_man, logic_network.get(),
          candidate_cut, remapper);

      // Undo the ECO (reEvaluateWithRepair left it on the stack via endEco).
      odb::dbDatabase::undoEco(block);
      sta->networkChanged();
      sta->updateTiming(false);

      logger_->info(utl::RES, 453,
          "Phase 2 candidate {}: post-repair slack={:.4e}",
          ri + 1, repaired_slack);

      if (repaired_slack > final_best_slack) {
        final_best_slack = repaired_slack;
        pSolutionFinal = cand.pSolution;
        final_best_index = cand.solution_index;
      }
    }

    logger_->info(utl::RES, 454,
        "Phase 2 winner: solution {} (post-repair slack={:.4e}, raw slack={:.4e})",
        final_best_index + 1, final_best_slack,
        ranked[0].raw_slack);
  } else {
    // Phase 2 disabled or only one solution: use Phase 1 winner.
    if (!ranked.empty()) {
      pSolutionFinal = ranked[0].pSolution;
      final_best_slack = ranked[0].raw_slack;
      final_best_index = ranked[0].solution_index;
    }
  }

  // ========== Permanently apply the final best solution ==========
  bool applied = false;
  if (pSolutionFinal) {
    logger_->info(utl::RES, 346,
        "Best solution found (index {}) with worst slack = {:.4e}",
        final_best_index + 1, final_best_slack);

    int saved_stdout2 = -1;
    if (!verbose_) {
      fflush(stdout);
      saved_stdout2 = dup(STDOUT_FILENO);
      int devnull = open("/dev/null", O_WRONLY);
      if (devnull >= 0) {
        dup2(devnull, STDOUT_FILENO);
        close(devnull);
      }
    }

    candidate_cut.InsertAbcMapSolution(
        pSolutionFinal, map_man, logic_network.get(),
        *remapper.getAbcLibrary(), network, sta,
        remapper.getNameGenerator(), logger_);

    if (saved_stdout2 >= 0) {
      fflush(stdout);
      dup2(saved_stdout2, STDOUT_FILENO);
      close(saved_stdout2);
    }

    remapper.performIncreDpl(candidate_cut, remapper.getDpl());
    logger_->info(utl::RES, 364, "Best solution permanently applied.");
    applied = true;
  } else {
    logger_->warn(utl::RES, 361, "No valid solution found to apply.");
  }

  abc::Abc_NtkMapEnumFreeStore(pMan);
  return applied;
```

**Important:** This replaces ALL the code from `// Apply the best solution.` through `abc::Abc_NtkMapEnumFreeStore(pMan); return applied;`. The existing Phase 1 result collection loops (enumeration + UCT) remain untouched above this block. You need to merge results from both loops into the `ranked` vector — the simplest way is to collect ALL successful results from `results` (enumeration) and `round_results` (UCT) into a single vector before sorting. Adjust the UCT loop to also push results into a shared vector, or collect them separately and merge before the sort.

- [ ] **Step 2: Handle UCT results in the ranking**

The current code tracks `pSolutionBest` / `best_slack` across both enumeration and UCT loops. For Phase 2, we need all successful results in one vector. The cleanest approach: after the UCT loop ends, collect all UCT results the same way as enumeration results. Add a `std::vector<RankedSolution> all_ranked;` before the enumeration loop and push successful results from both the enumeration results and UCT round_results into it. Then use `all_ranked` instead of `ranked` in Phase 2.

- [ ] **Step 3: Build and test**

Run: `source /opt/openEuler/gcc-toolset-12/enable && cmake --build build --target openroad -j$(nproc)`

Then run the fpu test:
```bash
cd /home/jzj/physyn/testremap
DESIGN_NAME="fpu" PERCENT=1 /home/jzj/physyn/OpenROAD/build/bin/openroad testscript_input_fp.tcl > /tmp/fpu_phase2.log 2>/tmp/fpu_phase2.err
```

Verify in the log:
- `grep "Phase 2" /tmp/fpu_phase2.log` should show re-evaluation messages
- `grep "Child reap summary" /tmp/fpu_phase2.log` should show all clean exits
- `grep "child failed" /tmp/fpu_phase2.log` should return 0 matches
- No crashes in `/tmp/fpu_phase2.err`

- [ ] **Step 4: Commit**

```bash
git add src/rmp/src/position_driven.cc
git commit -m "rmp: integrate two-phase evaluation with ECO-based repairSetup"
```

---

### Task 6: Verify and clean up diagnostic instrumentation

The earlier debugging session added step markers (`g_child_step`) and signal diagnostics to `child_fatal_handler`. These are useful for ongoing development but add noise. Make them conditional on verbose mode or keep them permanently (they're lightweight and only fire on crashes).

**Files:**
- Modify: `src/rmp/src/position_driven.cc`

- [ ] **Step 1: Review diagnostic code**

The `g_child_step` markers and enhanced `child_fatal_handler` are lightweight (a few `volatile sig_atomic_t` writes per child). The handler only fires on crashes. The parent-side logging of failure diagnostics (`"child failed: [SIG6 step=1]"`) is useful for debugging. **Keep all of it** — it has near-zero overhead in the happy path and is invaluable when things go wrong.

- [ ] **Step 2: Remove `trySizeUpCutInstances` from evaluateSolution (optional)**

Now that Phase 2 handles proper sizing/buffering, the lightweight size-up in children is less critical. However, it still improves raw slack ranking quality at minimal cost. **Keep it** for now — it helps Phase 1's ranking correlate better with Phase 2's results.

- [ ] **Step 3: Full test run**

Run the full test script with all 3 designs:
```bash
cd /home/jzj/physyn/testremap
bash testremap.sh
```

Check all logs for:
- Phase 2 messages in each design
- Zero child failures
- No crashes

- [ ] **Step 4: Commit**

```bash
git commit -m "rmp: finalize two-phase evaluation, keep diagnostics"
```

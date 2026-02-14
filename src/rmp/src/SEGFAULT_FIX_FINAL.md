# Segmentation Fault Fix - Final Version

## Problem History

### Issue 1: Signal 11 (SIGSEGV) - Network State Corruption
**Stack trace frame #2**: `sta::dbNetwork::connect()` → `odb::dbInst::getITerm()`
- **Cause**: Destructive network modification in evaluation loop
- **Symptom**: Accessing deleted instances during connection
- **Attempt 1**: Save/restore with manual deletion → **FAILED** (crashed in instance deletion)
- **Attempt 2**: Copy LogicCut for each evaluation → **FAILED** (see Issue 2)

### Issue 2: Signal 6 (SIGABRT) - Vector Corruption
**Error**: `Assertion '__n < this->size()' failed` in `std::vector::operator[]`
**Sequence**:
1. Solution 1 evaluated successfully
2. Solution 2 starts → "Deleted existing logic cut"
3. **CRASH**: Vector out of bounds

- **Cause**: Copying `LogicCut` creates shallow shared state
- **Problem**: `DeleteExistingLogicCut()` invalidates vectors that copied LogicCuts still reference
- **Result**: Vector out-of-bounds when next solution accesses corrupted state

## Final Solution: Extract Fresh Cuts

**Key Insight**: LogicCut's copy semantics don't support our use case. Don't copy - extract fresh!

### Implementation

```cpp
for (int i = 0; i < num_solutions; ++i) {
    // ❌ BAD: Copy creates shared state
    // cut::LogicCut eval_cut = candidate_cut;
    
    // ✅ GOOD: Extract fresh cut from scratch  
    cut::LogicCut eval_cut = remapper.extractBottleneck(*this);
    
    // Evaluate on independent cut
    sta::Slack slack = evaluateSolution(..., eval_cut, ...);
    
    // eval_cut destructor cleans up safely
}

// Apply best solution to original candidate_cut
candidate_cut.InsertAbcMapSolution(pSolutionBest, ...);
```

## Why This Works

1. **Independent state**: Each extracted LogicCut has its own vectors/data structures
2. **No sharing**: No shared references between LogicCuts
3. **Safe cleanup**: Each eval_cut destructor cleans up only its own resources
4. **Clean extraction**: Each solution gets the same fresh starting point

## Trade-offs

**Pros**:
- ✅ No crashes - each cut is independent
- ✅ Simple - no manual state management
- ✅ Safe - RAII handles all cleanup
- ✅ Correct - each solution evaluated on identical network state

**Cons**:
- ⚠️ Performance: Extracting cuts repeatedly might be slower than copying
- ⚠️ Assumption: All solutions work on the same bottleneck cut

## Files Modified

- **position_driven.cc** (lines ~365-385):
  - Changed from `cut::LogicCut eval_cut = candidate_cut;`
  - To: `cut::LogicCut eval_cut = remapper.extractBottleneck(*this);`
  - Uses actual `num_solutions` from ABC (not hardcoded 5)

## Testing

Should see log output:
```
[INFO RES-0348] Found 20 solutions to evaluate.
[INFO RES-0344] Evaluating solution 1/20...
[INFO CUT-0054] Inserting mapped ABC network...
[INFO RES-0345] Solution evaluated: Worst Slack = X.XXXX
[INFO RES-0344] Evaluating solution 2/20...
...
[INFO RES-0351] Evaluation complete: 20 solutions evaluated.
[INFO RES-0346] Best solution found with worst slack = X.XXXX
```

No crashes should occur.

## Alternative Approaches Considered

1. **Manual state restoration**: Too complex, database internals not accessible
2. **Shallow copy + cleanup**: LogicCut doesn't support this pattern
3. **Evaluate single solution**: Defeats purpose of enumeration
4. **Deep copy LogicCut**: Would require modifying LogicCut class

The fresh extraction approach is the simplest solution that works with existing code architecture.

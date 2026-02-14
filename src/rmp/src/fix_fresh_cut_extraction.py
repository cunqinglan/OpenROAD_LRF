#!/usr/bin/env python3
"""
Fix: Extract a fresh LogicCut for each solution evaluation instead of copying.
Copying LogicCut causes shared state issues when cuts are deleted/recreated.
"""

with open('position_driven.cc', 'r') as f:
    content = f.read()

import re

# Find the evaluation loop and replace it
old_loop = r'''  abc::Map_MappingSolution_t\* pSolutionBest = nullptr;
  sta::Slack best_slack = std::numeric_limits<sta::Slack>::lowest\(\);
  int evaluated_count = 0;

  for \(int i = 0; i < num_solutions; \+\+i\) \{
    abc::Map_MappingSolution_t\* pSolution =
        abc::Map_MappingGetSolution\(map_man, i\);
    
    if \(pSolution == nullptr\) \{
      logger_->warn\(utl::RES, 349, "Solution \{\} is NULL, skipping\.", i \+ 1\);
      continue;
    \}
    
    logger_->info\(
        utl::RES, 344, "Evaluating solution \{\}/\{\}\.\.\.", 
        i \+ 1, num_solutions\);

    // Create a COPY of candidate_cut for evaluation \(don't modify original\)
    cut::LogicCut eval_cut = candidate_cut;
    
    // Evaluate this solution on the copy
    sta::Slack slack = evaluateSolution\(
        pSolution,
        map_man,
        logic_network\.get\(\),
        eval_cut,  // Pass copy instead of original
        remapper\);

    // Track the best solution \(least negative slack\)
    if \(slack > best_slack\) \{
      best_slack = slack;
      pSolutionBest = pSolution;
    \}
    
    evaluated_count\+\+;
    // eval_cut goes out of scope here, cleaning up temporary network changes
  \}'''

new_loop = '''  abc::Map_MappingSolution_t* pSolutionBest = nullptr;
  sta::Slack best_slack = std::numeric_limits<sta::Slack>::lowest();
  int evaluated_count = 0;

  for (int i = 0; i < num_solutions; ++i) {
    abc::Map_MappingSolution_t* pSolution =
        abc::Map_MappingGetSolution(map_man, i);
    
    if (pSolution == nullptr) {
      logger_->warn(utl::RES, 349, "Solution {} is NULL, skipping.", i + 1);
      continue;
    }
    
    logger_->info(
        utl::RES, 344, "Evaluating solution {}/{}...", 
        i + 1, num_solutions);

    // Extract a FRESH LogicCut for this solution evaluation
    // Don't copy candidate_cut - extract from scratch to avoid shared state
    cut::LogicCut eval_cut = remapper.extractBottleneck(*this);
    
    // Evaluate this solution on the fresh cut
    sta::Slack slack = evaluateSolution(
        pSolution,
        map_man,
        logic_network.get(),
        eval_cut,
        remapper);

    // Track the best solution (least negative slack)
    if (slack > best_slack) {
      best_slack = slack;
      pSolutionBest = pSolution;
    }
    
    evaluated_count++;
    // eval_cut destructor cleans up
  }'''

content = re.sub(old_loop, new_loop, content, flags=re.DOTALL)

with open('position_driven.cc', 'w') as f:
    f.write(content)

print("✓ Fixed: Extract fresh LogicCut for each evaluation")
print("✓ Removed copying of candidate_cut")
print("✓ Each solution gets a clean cut from remapper.extractBottleneck()")
print("\nThis prevents shared state issues that caused vector corruption.")

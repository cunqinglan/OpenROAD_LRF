#!/usr/bin/env python3
"""
Better fix: Don't modify candidate_cut during evaluation.
Create temporary copies for evaluation, only modify candidate_cut for best solution.
"""

with open('position_driven.cc', 'r') as f:
    content = f.read()

# Remove the restoreOriginalCut function - we won't need it
import re

# Find and remove the restore function
restore_pattern = r'// Helper function to restore the original cut state.*?current_cut = original_cut;\n}\n\n'
content = re.sub(restore_pattern, '', content, flags=re.DOTALL)

# Now update the evaluation loop to use temporary copies
old_loop = r'''  // Save the original cut state before evaluation
  cut::LogicCut original_cut = candidate_cut;
  
  abc::Map_MappingSolution_t\* pSolutionBest = nullptr;
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

    // Evaluate this solution and get its worst slack
    sta::Slack slack = evaluateSolution\(
        pSolution,
        map_man,
        logic_network\.get\(\),
        candidate_cut,
        remapper\);

    // Track the best solution \(least negative slack\)
    if \(slack > best_slack\) \{
      best_slack = slack;
      pSolutionBest = pSolution;
    \}
    
    evaluated_count\+\+;
    
    // Restore original network state for next evaluation
    // Don't restore after the last iteration
    if \(i < num_solutions - 1\) \{
      restoreOriginalCut\(original_cut, candidate_cut\);
    \}
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

    // Create a COPY of candidate_cut for evaluation (don't modify original)
    cut::LogicCut eval_cut = candidate_cut;
    
    // Evaluate this solution on the copy
    sta::Slack slack = evaluateSolution(
        pSolution,
        map_man,
        logic_network.get(),
        eval_cut,  // Pass copy instead of original
        remapper);

    // Track the best solution (least negative slack)
    if (slack > best_slack) {
      best_slack = slack;
      pSolutionBest = pSolution;
    }
    
    evaluated_count++;
    // eval_cut goes out of scope here, cleaning up temporary network changes
  }'''

content = re.sub(old_loop, new_loop, content, flags=re.DOTALL)

# Update the "apply best solution" section to remove the restore call
old_apply = r'''  // Apply the best solution permanently
  if \(pSolutionBest\) \{
    remapper\.getLogger\(\)->info\(
        utl::RES, 346,
        "Best solution found with worst slack = \{:.4f\}", best_slack\);
    
    // First restore to original state
    restoreOriginalCut\(original_cut, candidate_cut\);
    
    // Now apply the best solution permanently
    candidate_cut\.InsertAbcMapSolution\('''

new_apply = '''  // Apply the best solution permanently
  if (pSolutionBest) {
    remapper.getLogger()->info(
        utl::RES, 346,
        "Best solution found with worst slack = {:.4f}", best_slack);
    
    // Apply the best solution to the original candidate_cut
    candidate_cut.InsertAbcMapSolution('''

content = re.sub(old_apply, new_apply, content, flags=re.DOTALL)

with open('position_driven.cc', 'w') as f:
    f.write(content)

print("✓ Removed restoreOriginalCut function")
print("✓ Updated evaluation loop to use temporary copies")
print("✓ Each solution evaluated on eval_cut (copy of candidate_cut)")
print("✓ Original candidate_cut only modified for best solution")
print("\nThis approach prevents network corruption by never modifying")
print("the original cut during evaluation.")

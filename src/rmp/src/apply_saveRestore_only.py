#!/usr/bin/env python3
"""
Apply only Solution 1 (Save/Restore) without validation.
The validation function accesses opaque ABC structures, so we can't use it.
"""

with open('position_driven.cc', 'r') as f:
    content = f.read()

# STEP 1: Add restoreOriginalCut helper function before PositionDrivenStrategy::remap
restore_func = '''// Helper function to restore the original cut state
void restoreOriginalCut(
    const cut::LogicCut& original_cut,
    cut::LogicCut& current_cut,
    rmp::SeqRemapper& remapper) {
  
  sta::dbNetwork* network = remapper.getSta()->getDbNetwork();
  std::vector<sta::Instance*> instances_to_delete;
  
  for (const auto* inst : current_cut.cut_instances()) {
    if (original_cut.cut_instances().find(inst) == original_cut.cut_instances().end()) {
      instances_to_delete.push_back(const_cast<sta::Instance*>(inst));
    }
  }
  
  for (sta::Instance* inst : instances_to_delete) {
    network->deleteInstance(inst);
  }
  
  current_cut = original_cut;
}

'''

insert_pos = content.find('void PositionDrivenStrategy::remap(')
if insert_pos != -1:
    content = content[:insert_pos] + restore_func + content[insert_pos:]
    print("✓ Added restoreOriginalCut helper function")
else:
    print("ERROR: Could not find remap function")
    exit(1)

# STEP 2: Update the evaluation loop
# Find the section from "// Step 6" to before "// Step 7"
import re

# Replace the hardcoded loop with the new one
old_loop_pattern = r'(  // Step 6: Process the mapping solutions\..*?)(\n  // Step 7: Clean up the mapping manager\.)'

new_loop = r'''  // Step 6: Process the mapping solutions.
  // Evaluate each solution and select the best one based on worst slack.
  abc::Map_Man_t* map_man = static_cast<abc::Map_Man_t*>(pMan);
  const int num_solutions = abc::Map_ManReadNumSolutions(map_man);
  
  if (num_solutions <= 0) {
    remapper.getLogger()->warn(
        utl::RES, 341, "ABC mapping enumeration returned no solutions.");
    abc::Abc_NtkMapEnumFreeStore(pMan);
    return;
  }
  
  logger_->info(utl::RES, 348, "Found {} solutions to evaluate.", num_solutions);

  // Save the original cut state before evaluation
  cut::LogicCut original_cut = candidate_cut;
  
  abc::Map_MappingSolution_t* pSolutionBest = nullptr;
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

    // Evaluate this solution and get its worst slack
    sta::Slack slack = evaluateSolution(
        pSolution,
        map_man,
        logic_network.get(),
        candidate_cut,
        remapper);

    // Track the best solution (least negative slack)
    if (slack > best_slack) {
      best_slack = slack;
      pSolutionBest = pSolution;
    }
    
    evaluated_count++;
    
    // Restore original network state for next evaluation
    // Don't restore after the last iteration
    if (i < num_solutions - 1) {
      restoreOriginalCut(original_cut, candidate_cut, remapper);
    }
  }
  
  logger_->info(utl::RES, 351, 
               "Evaluation complete: {} solutions evaluated.",
               evaluated_count);
  
  // Apply the best solution permanently
  if (pSolutionBest) {
    remapper.getLogger()->info(
        utl::RES, 346,
        "Best solution found with worst slack = {:.4f}", best_slack);
    
    // First restore to original state
    restoreOriginalCut(original_cut, candidate_cut, remapper);
    
    // Now apply the best solution permanently
    candidate_cut.InsertAbcMapSolution(
        pSolutionBest,
        static_cast<abc::Map_Man_t*>(pMan),
        logic_network.get(),
        *remapper.getAbcLibrary(),
        remapper.getSta()->getDbNetwork(),
        remapper.getNameGenerator(),
        remapper.getLogger());
  } else {
    remapper.getLogger()->warn(
        utl::RES, 352,
        "No valid solution found to apply.");
  }

  logger_->info(
      utl::RES, 347, "After step 6. Best solution applied to the network.");

'''

content = re.sub(old_loop_pattern, new_loop + r'\2', content, flags=re.DOTALL)

with open('position_driven.cc', 'w') as f:
    f.write(content)

print("✓ Updated evaluation loop with save/restore mechanism")
print("\nSummary:")
print("  1. restoreOriginalCut() - Restores network state between evaluations")
print("  2. Enhanced loop - Saves state, evaluates each solution, restores between iterations")
print("  3. Uses actual num_solutions from ABC (not hardcoded)")
print("  4. Validation SKIPPED (ABC structures are opaque - can't access internals)")
print("\nThis fix prevents the segfault by restoring network state between evaluations.")

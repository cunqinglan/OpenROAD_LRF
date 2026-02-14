#!/usr/bin/env python3

with open('position_driven.cc', 'r') as f:
    content = f.read()

# Find and replace the evaluation loop
# Start marker: "// Step 6: Process the mapping solutions."
# End marker: before "// Step 7: Clean up the mapping manager."

start_marker = "  // Step 6: Process the mapping solutions."
end_marker = "  // Step 7: Clean up the mapping manager."

start_idx = content.find(start_marker)
end_idx = content.find(end_marker)

if start_idx == -1 or end_idx == -1:
    print("ERROR: Could not find markers")
    exit(1)

new_loop = '''  // Step 6: Process the mapping solutions.
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
  int valid_solutions = 0;
  int invalid_solutions = 0;

  for (int i = 0; i < num_solutions; ++i) {
    abc::Map_MappingSolution_t* pSolution =
        abc::Map_MappingGetSolution(map_man, i);
    
    if (pSolution == nullptr) {
      logger_->warn(utl::RES, 349, "Solution {} is NULL, skipping.", i + 1);
      continue;
    }
    
    // Validate the solution before evaluating
    if (!IsValidMappingSolution(map_man, pSolution)) {
      logger_->warn(utl::RES, 350, 
                   "Solution {}/{} is invalid (missing cuts for referenced nodes), skipping.",
                   i + 1, num_solutions);
      invalid_solutions++;
      continue;
    }
    
    logger_->info(
        utl::RES, 344, "Evaluating valid solution {}/{} (total valid: {})...", 
        i + 1, num_solutions, valid_solutions + 1);

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
    
    valid_solutions++;
    
    // Restore original network state for next evaluation
    // Don't restore after the last iteration
    if (i < num_solutions - 1) {
      restoreOriginalCut(original_cut, candidate_cut, remapper);
    }
  }
  
  logger_->info(utl::RES, 351, 
               "Evaluation complete: {} valid, {} invalid solutions.",
               valid_solutions, invalid_solutions);
  
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

# Replace the section
new_content = content[:start_idx] + new_loop + content[end_idx:]

with open('position_driven.cc', 'w') as f:
    f.write(new_content)

print("✓ Step 3: Updated evaluation loop with validation and state restoration")
print("✓ All three changes applied successfully!")
print("\nSummary of changes:")
print("  1. IsValidMappingSolution() - validates solutions before evaluation")
print("  2. restoreOriginalCut() - restores network state between evaluations")
print("  3. Enhanced loop - saves state, validates, evaluates, and restores for each solution")
print("  4. Using actual num_solutions from ABC (not hardcoded)")

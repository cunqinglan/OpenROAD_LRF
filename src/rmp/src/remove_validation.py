#!/usr/bin/env python3
# Remove the validation function that accesses opaque ABC structures

with open('position_driven.cc', 'r') as f:
    content = f.read()

# Remove the IsValidMappingSolution function
import re

# Find and remove the validation function
pattern = r'// Helper function to validate.*?return true;\n}\n\n'
content = re.sub(pattern, '', content, flags=re.DOTALL)

# Remove the validation call in the loop
# Replace the validation check section with direct evaluation
old_section = r'''    // Validate the solution before evaluating
    if \(!IsValidMappingSolution\(map_man, pSolution\)\) \{
      logger_->warn\(utl::RES, 350, 
                   "Solution \{\}/\{\} is invalid \(missing cuts for referenced nodes\), skipping\.",
                   i \+ 1, num_solutions\);
      invalid_solutions\+\+;
      continue;
    \}
    
    logger_->info\(
        utl::RES, 344, "Evaluating valid solution \{\}/\{\} \(total valid: \{\}\)\.\.\.", 
        i \+ 1, num_solutions, valid_solutions \+ 1\);'''

new_section = '''    logger_->info(
        utl::RES, 344, "Evaluating solution {}/{}...", 
        i + 1, num_solutions);'''

content = re.sub(old_section, new_section, content)

# Update the final log message
content = content.replace(
    '"Evaluation complete: {} valid, {} invalid solutions.",',
    '"Evaluation complete: {} evaluated, {} skipped (NULL).",'
)

with open('position_driven.cc', 'w') as f:
    f.write(content)

print("✓ Removed validation function and calls (ABC structures are opaque)")
print("✓ Kept save/restore mechanism (the critical fix)")

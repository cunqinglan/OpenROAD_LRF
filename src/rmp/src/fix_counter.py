#!/usr/bin/env python3
with open('position_driven.cc', 'r') as f:
    content = f.read()

# Fix the NULL check to increment invalid_solutions
content = content.replace(
    '''    if (pSolution == nullptr) {
      logger_->warn(utl::RES, 349, "Solution {} is NULL, skipping.", i + 1);
      continue;
    }''',
    '''    if (pSolution == nullptr) {
      logger_->warn(utl::RES, 349, "Solution {} is NULL, skipping.", i + 1);
      invalid_solutions++;
      continue;
    }'''
)

with open('position_driven.cc', 'w') as f:
    f.write(content)

print("✓ Fixed invalid_solutions counter")

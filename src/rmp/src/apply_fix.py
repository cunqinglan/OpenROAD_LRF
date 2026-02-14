#!/usr/bin/env python3
import re

# Read the original file
with open('position_driven.cc', 'r') as f:
    lines = f.readlines()

# Step 1: Uncomment validation function (find lines starting with /* before "Helper function")
in_comment_block = False
new_lines = []
for i, line in enumerate(lines):
    if '/*' in line and i < 100 and 'Helper function to validate' in lines[i+1] if i+1 < len(lines) else False:
        in_comment_block = True
        new_lines.append(line.replace('/*', '//'))  # Keep first line but as comment
        continue
    elif '*/' in line and in_comment_block:
        in_comment_block = False
        continue  # Skip the closing */
    elif in_comment_block:
        # Remove leading // or * from commented lines
        cleaned = line.lstrip('/').lstrip('*').lstrip()
        if cleaned:
            new_lines.append(cleaned)
    else:
        new_lines.append(line)

# Write intermediate result
with open('position_driven.cc', 'w') as f:
    f.writelines(new_lines)

print("Step 1: Uncommented validation function")

# Step 2: Find line number of "void PositionDrivenStrategy::remap(" and insert restore function before it
with open('position_driven.cc', 'r') as f:
    content = f.read()

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
    with open('position_driven.cc', 'w') as f:
        f.write(content)
    print("Step 2: Added restoreOriginalCut function")

print("✓ Fixes applied successfully!")
print("Note: Step 3 (updating evaluation loop) requires manual editing due to complexity")

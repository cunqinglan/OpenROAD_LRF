#!/usr/bin/env python3
"""
Fix restoreOriginalCut to use simple copy instead of manual deletion.
Manual instance deletion causes crashes due to complex network dependencies.
"""

with open('position_driven.cc', 'r') as f:
    content = f.read()

# Replace the complex restoreOriginalCut with a simpler version
old_restore = '''// Helper function to restore the original cut state
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
}'''

new_restore = '''// Helper function to restore the original cut state
// Simply copy the original cut - let the LogicCut destructor handle cleanup
void restoreOriginalCut(
    const cut::LogicCut& original_cut,
    cut::LogicCut& current_cut) {
  
  // Direct assignment - LogicCut's assignment operator handles the details
  current_cut = original_cut;
}'''

content = content.replace(old_restore, new_restore)

# Update the function calls to match new signature (remove remapper parameter)
content = content.replace(
    'restoreOriginalCut(original_cut, candidate_cut, remapper);',
    'restoreOriginalCut(original_cut, candidate_cut);'
)

with open('position_driven.cc', 'w') as f:
    f.write(content)

print("✓ Simplified restoreOriginalCut() function")
print("✓ Removed manual instance deletion (causes crashes)")
print("✓ Now uses LogicCut's copy assignment operator")
print("\nThe LogicCut class handles resource management internally.")

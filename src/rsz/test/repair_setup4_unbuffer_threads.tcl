# Test unbuffer with multi-threading (batch mode).
# Verifies no crash (use-after-free) when UnbufferMove runs with -threads 2.
source "helpers.tcl"
set repair_args [list -sequence "unbuffer" -threads 2]
source "repair_setup4.tcl"

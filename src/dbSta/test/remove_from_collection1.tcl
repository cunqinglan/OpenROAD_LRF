# remove_from_collection: set difference / intersection over STA collections,
# with Synopsys-style warnings for unresolved names and object class mismatches.
# Reuses the get_ports1 fixture (Nangate45 + get_ports1.v).
source "helpers.tcl"

read_lef Nangate45/Nangate45.lef
read_liberty Nangate45/Nangate45_typ.lib
read_verilog get_ports1.v
link_design top

# Print sorted full names on one line so output is order-independent.
proc show { label objs } {
  set names {}
  foreach obj $objs {
    lappend names [get_full_name $obj]
  }
  puts "$label: [lsort $names]"
}

set ports [get_ports {clk top_in_single top_out_single}]
show "base" $ports

# 1. Difference, object_spec given as objects.
show "diff-by-object" [remove_from_collection $ports [get_ports clk]]

# 2. Difference, object_spec given as a name (looked up as Port).
show "diff-by-name" [remove_from_collection $ports top_in_single]

# 3. Name that exists in the design but is not in the base collection:
#    silent no-op, no warning (matches Synopsys).
show "diff-not-in-base" \
  [remove_from_collection [get_ports {clk top_in_single}] top_out_single]

# 4. Intersection.
show "intersect" \
  [remove_from_collection $ports [get_ports {clk top_out_single}] -intersect]

# 5. Warning: unresolved name in object_spec (STA-2066). Result is unchanged.
show "warn-missing" [remove_from_collection $ports no_such_port]

# 6. Warning: object class mismatch (STA-2067). Removing a Clock object from a
#    Port collection. Result is unchanged.
create_clock -name clk -period 1.0 [get_ports clk]
show "warn-mismatch" [remove_from_collection $ports [get_clocks clk]]

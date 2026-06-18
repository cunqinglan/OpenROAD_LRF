# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

namespace eval sta {
define_cmd_args "report_cell_usage" { \
  [-verbose] [module_inst] [-file file] [-stage stage]}

proc report_cell_usage { args } {
  parse_key_args "highlight_path" args keys {-file -stage} \
    flags {-verbose} 0

  check_argc_eq0or1 "report_cell_usage" $args

  if { [ord::get_db_block] == "NULL" } {
    sta_error 1001 "No design block found."
  }

  set module [[ord::get_db_block] getTopModule]
  if { $args != "" } {
    set modinst [[ord::get_db_block] findModInst [lindex $args 0]]
    if { $modinst == "NULL" } {
      sta_error 1002 "Unable to find $args"
    }
    set module [$modinst getMaster]
  }
  set verbose [info exists flags(-verbose)]
  set file_name ""
  if { [info exists keys(-file)] } {
    set file_name $keys(-file)
  }
  set stage_name ""
  if { [info exists keys(-stage)] } {
    set stage_name $keys(-stage)
  }

  report_cell_usage_cmd $module $verbose $file_name $stage_name
}

define_cmd_args "report_timing_histogram" \
  {[-num_bins num_bins] [-bin_size bin_size] [-setup|-hold]}

proc report_timing_histogram { args } {
  parse_key_args "report_timing_histogram" args \
    keys {-num_bins -bin_size} \
    flags {-setup -hold}

  check_argc_eq0 "report_timing_histogram" $args

  if { [info exists flags(-setup)] && [info exists flags(-hold)] } {
    utl::error STA 7 "Both -setup and -hold cannot be specified"
  }

  if { [info exists keys(-num_bins)] && [info exists keys(-bin_size)] } {
    utl::error STA 73 "Both -num_bins and -bin_size cannot be specified"
  }

  set num_bins 10
  if { [info exists keys(-num_bins)] } {
    set num_bins $keys(-num_bins)
  }

  set bin_size 0.0
  if { [info exists keys(-bin_size)] } {
    set bin_size $keys(-bin_size)
    if { $bin_size <= 0 } {
      utl::error STA 74 "-bin_size must be a positive value"
    }
  }

  set min_max max
  if { [info exists flags(-hold)] } {
    set min_max min
  }

  report_timing_histogram_cmd $num_bins $min_max $bin_size
}

define_cmd_args "report_logic_depth_histogram" { \
  [-num_bins num_bins] [-exclude_buffers] [-exclude_inverters]}

proc report_logic_depth_histogram { args } {
  parse_key_args "report_logic_depth_histogram" args keys \
    {-num_bins} flags {-exclude_buffers -exclude_inverters}

  check_argc_eq0 "report_logic_depth_histogram" $args

  set num_bins 10
  if { [info exists keys(-num_bins)] } {
    set num_bins $keys(-num_bins)
  }

  set exclude_buffers false
  if { [info exists flags(-exclude_buffers)] } {
    set exclude_buffers true
  }

  set exclude_inverters false
  if { [info exists flags(-exclude_inverters)] } {
    set exclude_inverters true
  }

  report_logic_depth_histogram_cmd $num_bins $exclude_buffers $exclude_inverters
}

# redefine sta::sta_warn/error to call utl::warn/error
proc sta_error { id msg } {
  utl::error STA $id $msg
}

proc sta_warn { id msg } {
  utl::warn STA $id $msg
}

define_cmd_args "replace_hier_module" {instance module}
proc replace_hier_module { instance module } {
  set design [get_hier_module $module]
  if { $design != "NULL" } {
    set modinst [[ord::get_db_block] findModInst $instance]
    if { $modinst == "NULL" } {
      sta_error 1003 "Unable to find $instance"
    }
    replace_hier_module_cmd $modinst $design
    return 1
  }
  return 0
}
interp alias {} replace_design {} replace_hier_module

define_cmd_args "get_hier_module" {design_name}
proc get_hier_module { arg } {
  if { [llength $arg] > 1 } {
    sta_error 200 "module must be a single module."
  }

  set block [ord::get_db_block]
  if { $block == "NULL" } {
    sta_error 202 "database block cannot be found."
  }

  set design [$block findModule $arg]
  if { $design == "NULL" } {
    set child_block [$block findChild $arg]
    if { $child_block != "NULL" } {
      set design [$child_block findModule $arg]
    }
  }

  if { $design == "NULL" } {
    sta_error 201 "module $arg cannot be found."
  }

  return $design
}
interp alias {} get_design {} get_hier_module

define_cmd_args "check_axioms" {}
proc check_axioms { args } {
  check_argc_eq0 "check_axioms" $args
  check_axioms_cmd
}

proc endpoint_path_count { } {
  return [endpoint_count]
}

define_cmd_args "check_ip" {
  [-master master_name]
  [-all]
  [-max_polygons count]
  [-verbose]
}

proc check_ip { args } {
  parse_key_args "check_ip" args \
    keys {-master -max_polygons} \
    flags {-all -verbose}

  set master_name ""
  if { [info exists keys(-master)] } {
    set master_name $keys(-master)
  }

  set check_all [info exists flags(-all)]

  if { !$check_all && $master_name eq "" } {
    utl::error CHK 7 "Must specify either -master or -all"
  }

  set max_polygons 10000
  if { [info exists keys(-max_polygons)] } {
    set max_polygons $keys(-max_polygons)
    sta::check_positive_integer "-max_polygons" $max_polygons
  }

  set verbose [info exists flags(-verbose)]

  return [sta::check_ip_cmd $master_name $check_all $max_polygons $verbose]
}

################################################################
#
# remove_from_collection
#
# Synopsys-compatible collection command. Returns a new collection that is
# "collection" with the objects in "object_spec" removed (set difference), or
# the intersection of the two when -intersect is given. Object names in
# object_spec are looked up using the object class of the base collection
# (exact names, not patterns), mirroring delete_from_list. Unlike
# delete_from_list, names that cannot be resolved (or objects whose class does
# not match the base collection) emit a warning, matching dc_shell behavior.
#
# Note: OpenSTA represents collections as plain Tcl lists of object handles, so
# the result is a Tcl list (use llength / get_full_name), not an opaque
# Synopsys collection object.

define_cmd_args "remove_from_collection" {collection object_spec [-intersect]}

proc remove_from_collection { args } {
  parse_key_args "remove_from_collection" args keys {} flags {-intersect}
  check_argc_eq2 "remove_from_collection" $args
  set base [lindex $args 0]
  set spec [lindex $args 1]

  # Empty base: object class cannot be inferred, so the result is empty.
  if { $base eq {} } {
    return {}
  }

  set base0 [lindex $base 0]
  set base_is_objects [is_object $base0]
  set base_type [expr { $base_is_objects ? [object_type $base0] : "" }]

  # Normalize object_spec to objects of the base collection's class, warning on
  # names that do not resolve or objects whose class differs from the base.
  set spec_objs {}
  foreach obj $spec {
    if { $base_is_objects && ![is_object $obj] } {
      set resolved [remove_from_collection_resolve $base_type $obj]
      if { $resolved eq {} || $resolved eq "NULL" } {
        utl::warn STA 2066 "remove_from_collection: $base_type '$obj' not found."
        continue
      }
      lappend spec_objs $resolved
    } elseif { $base_is_objects && [is_object $obj] \
               && [object_type $obj] ne $base_type } {
      set obj_class [object_type $obj]
      utl::warn STA 2067 \
        "remove_from_collection: object class '$obj_class' does not match '$base_type'; ignored."
      continue
    } else {
      lappend spec_objs $obj
    }
  }

  # The set operation reuses delete_from_list. Elements of spec_objs are already
  # objects, so delete_from_list removes them by identity without reconverting.
  if { [info exists flags(-intersect)] } {
    # Intersection: base & spec == base - (base - spec).
    return [delete_from_list $base [delete_from_list $base $spec_objs]]
  }
  # Default: set difference base - spec.
  return [delete_from_list $base $spec_objs]
}

# Resolve an object name to an object of the given class, mirroring the type
# dispatch in delete_objects_from_list_cmd. Unknown types fall back to the raw
# name (handled downstream) rather than warning.
proc remove_from_collection_resolve { base_type name } {
  switch -- $base_type {
    Clock { return [find_clock $name] }
    Port {
      set top [top_instance]
      return [[$top cell] find_port $name]
    }
    Pin { return [find_pin $name] }
    Instance { return [find_instance $name] }
    Net { return [find_net $name] }
    LibertyLibrary { return [find_liberty $name] }
    LibertyCell { return [find_liberty_cell $name] }
    LibertyPort { return [get_lib_pins $name] }
    default { return $name }
  }
}

# namespace
}

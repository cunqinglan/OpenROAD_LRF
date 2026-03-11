set DESIGN "NV_NVDLA_partition_m"
set LEF_DIR "/home/jzj/OpenROAD-flow-scripts/flow/platforms/asap7/lef"
#set LIB_DIR "/app/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM"
set BASE_DIR "/home/jzj/physyn/OpenROAD/testremap/designs"
set PLATFORM_DIR "/home/jzj/OpenROAD-flow-scripts/flow/platforms/asap7"
set LEF_DIR "$PLATFORM_DIR/lef"
set LIB_DIR "$BASE_DIR/asap7_all.lib"
set VERILOG_PATH "$BASE_DIR/$DESIGN/EDA_files/$DESIGN.v"
set DEF_PATH "$BASE_DIR/$DESIGN/EDA_files/${DESIGN}_fp.def"
set SDC_PATH "$BASE_DIR/$DESIGN/EDA_files/$DESIGN.sdc"

# Read technology LEF
set tech_lef "$LEF_DIR/asap7_tech_1x_201209.lef"
puts "Reading technology LEF: $tech_lef"
read_lef $tech_lef

# Read verilog netlist
puts "Reading verilog: $VERILOG_PATH"
read_verilog $VERILOG_PATH

read_liberty $LIB_DIR

# Read all other LEF files
set other_lefs [glob -nocomplain $LEF_DIR/*.lef]
foreach lef_file $other_lefs {
    if {$lef_file == $tech_lef} {
        continue
    }
    puts "Reading LEF: $lef_file"
    read_lef $lef_file
}

# Read all liberty files
#set lib_files [glob -nocomplain $LIB_DIR/*.lib]
#foreach lib_file $lib_files {
    #puts "Reading LIB: $lib_file"
    #read_liberty $lib_file
#}

# Link design
link_design $DESIGN

read_def -floorplan_initialize $DEF_PATH
read_sdc $SDC_PATH

source "$PLATFORM_DIR/setRC.tcl"

global_placement -routability_driven -init_density_penalty 0.05 -initial_place_max_iter 10
detailed_placement

estimate_parasitics -placement

puts "=====timing report before remap======"

# Check if clocks are defined
report_clocks

# Check if timing graph has any endpoints
report_checks -path_delay max -unconstrained

# Check what libs are loaded
list_libs

# Check design has instances
report_design_area

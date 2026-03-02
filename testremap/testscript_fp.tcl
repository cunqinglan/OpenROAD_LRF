set DESIGN "c432"
set LEF_DIR "/app/OpenROAD-flow-scripts/flow/platforms/asap7/lef"
#set LIB_DIR "/app/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM"
set LIB_DIR "/app/designs/asap7_all.lib"
set BASE_DIR "/app/designs"
set VERILOG_PATH "$BASE_DIR/$DESIGN/${DESIGN}_mapped.v"
#set DEF_PATH "$BASE_DIR/$DESIGN/EDA_files/${DESIGN}_fp.def"
set SDC_PATH "/app/constraint.sdc"

# Read technology LEF
set tech_lef "$LEF_DIR/asap7_tech_1x_201209.lef"
puts "Reading technology LEF: $tech_lef"
read_lef $tech_lef

# Read verilog netlist
puts "Reading verilog: $VERILOG_PATH"
read_verilog $VERILOG_PATH

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
read_liberty $LIB_DIR

# Link design
link_design $DESIGN

read_sdc $SDC_PATH

initialize_floorplan \
    -utilization 50.0 \
    -aspect_ratio 1.0 \
    -core_space 0.0 \
    -site asap7sc7p5t

#read_def -floorplan_initialize $DEF_PATH

global_placement \
    -skip_io \
    -density 0.6 \
    -pad_left 0 \
    -pad_right 0
#detailed_placement

puts "=====timing report before remap======"

report_checks -path_delay max

position_driven_remap

puts "=====timing report after remap======"

report_checks -path_delay max

exit
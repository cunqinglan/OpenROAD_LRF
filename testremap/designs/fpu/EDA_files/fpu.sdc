set_cmd_units -time ps -capacitance fF
set_max_fanout 16.000 [current_design]
create_clock -name clk -period 3680.0 [get_ports clk]
set_input_delay  -max -clock [get_clocks "clk"] -add_delay 368.0 [all_inputs -no_clocks]
set_output_delay -max -clock [get_clocks "clk"] -add_delay 368.0 [all_outputs]
set_input_delay  -min -clock [get_clocks "clk"] -add_delay 184.0 [all_inputs -no_clocks]
set_output_delay -min -clock [get_clocks "clk"] -add_delay 184.0 [all_outputs]

set_max_transition 36.800000000000004 [all_outputs]
set_input_transition -max 184.0 [all_inputs]
set_input_transition -min 36.800000000000004 [all_inputs]

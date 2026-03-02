set_cmd_units -time ps -capacitance fF

set_max_fanout 16.000 [current_design]
create_clock -name clk -period 320.0 [get_ports clk]
set_input_delay  -max -clock [get_clocks "clk"] -add_delay 26.0 [all_inputs -no_clocks]
set_output_delay -max -clock [get_clocks "clk"] -add_delay 26.0 [all_outputs]
set_input_delay  -min -clock [get_clocks "clk"] -add_delay 13.0 [all_inputs -no_clocks]
set_output_delay -min -clock [get_clocks "clk"] -add_delay 13.0 [all_outputs]

set_max_transition 2.6 [all_outputs]
set_input_transition -max 13.0 [all_inputs]
set_input_transition -min 2.6 [all_inputs]

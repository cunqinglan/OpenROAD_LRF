# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

# Restructuring could be done targeting area or timing.
#
# Argument Description
# target:            "area"|"timing". In area mode focus is area reduction and
#                    timing may degrade. In timing mode delay would be reduced
#                    but area may increase.
# slack_threshold: specifies slack value below which timing paths need to be
#                  analyzed for restructuring
# depth_threshold: specifies the path depth above which a timing path would be
#                  considered for restructuring
# tielo_port:      specifies port name of tie low cell in format <cell_name>/<port_name>
# tielo_port:      specifies port name of tie high cell in format <cell_name>/<port_name>
# work_dir:        Name of working directory for temporary files.
#                  If not provided run directory would be used
#
# Note that for delay mode slack_threshold and depth_threshold are both considered together.
# Even if slack_threshold is violated, path may not be considered for re-synthesis unless
# depth_threshold is violated as well.

sta::define_cmd_args "restructure" { \
                                      [-slack_threshold slack]\
                                      [-depth_threshold depth]\
                                      [-target area|timing]\
                                      [-abc_logfile logfile]\
                                      [-liberty_file liberty_file]\
                                      [-tielo_port tielow_port]\
                                      [-tiehi_port tiehigh_port]\
                                      [-split_large_inputs split_large_inputs]\
                                      [-work_dir workdir_name]
                                    }

proc restructure { args } {
  sta::parse_key_args "restructure" args \
    keys {-slack_threshold -depth_threshold -target -liberty_file -abc_logfile\
          -tielo_port -tiehi_port -work_dir -split_large_inputs} \
    flags {}

  set slack_threshold_value 0
  set depth_threshold_value 16
  set target "area"
  set workdir_name "."
  set abc_logfile ""
  set split_large_inputs_value 0

  if { [info exists keys(-slack_threshold)] } {
    set slack_threshold_value $keys(-slack_threshold)
  }

  if { [info exists keys(-depth_threshold)] } {
    set depth_threshold_value $keys(-depth_threshold)
  }

  if { [info exists keys(-target)] } {
    set target $keys(-target)
  }

  if { [info exists keys(-abc_logfile)] } {
    set abc_logfile $keys(-abc_logfile)
  }

  if { [info exists keys(-liberty_file)] } {
    set liberty_file_name $keys(-liberty_file)
  } else {
    utl::error RMP 1 "Missing argument -liberty_file"
  }

  if { [info exists keys(-tielo_port)] } {
    set loport $keys(-tielo_port)
    if { ![sta::is_object $loport] } {
      set loport [sta::get_lib_pins $keys(-tielo_port)]
      if { [llength $loport] > 1 } {
        # multiple libraries match the lib port arg; use any
        set loport [lindex $loport 0]
      }
    }
    if { $loport != "" } {
      rmp::set_tielo_port_cmd $loport
    }
  } else {
    utl::warn RMP 7 "-tielo_port not specified"
  }

  if { [info exists keys(-tiehi_port)] } {
    set hiport $keys(-tiehi_port)
    if { ![sta::is_object $hiport] } {
      set hiport [sta::get_lib_pins $keys(-tiehi_port)]
      if { [llength $hiport] > 1 } {
        # multiple libraries match the lib port arg; use any
        set hiport [lindex $hiport 0]
      }
    }
    if { $hiport != "" } {
      rmp::set_tiehi_port_cmd $hiport
    }
  } else {
    utl::warn RMP 8 "-tiehi_port not specified"
  }

  if { [info exists keys(-work_dir)] } {
    set workdir_name $keys(-work_dir)
  }

  if { [info exists keys(-split_large_inputs)] } {
    set split_large_inputs_value $keys(-split_large_inputs)
  }
  rmp::set_split_large_inputs $split_large_inputs_value

  rmp::restructure_cmd $liberty_file_name $target $slack_threshold_value \
    $depth_threshold_value $workdir_name $abc_logfile
}

sta::define_cmd_args "resynth" {[-corner corner] [-split_large_inputs split_large_inputs]}

proc resynth { args } {
  sta::parse_key_args "resynth" args \
    keys {-corner -split_large_inputs} \
    flags {}
  set corner [sta::parse_corner keys]
  if { [info exists keys(-split_large_inputs)] } {
    rmp::set_split_large_inputs $keys(-split_large_inputs)
  } else {
    rmp::set_split_large_inputs 0
  }
  rmp::resynth_cmd $corner
}

sta::define_cmd_args "resynth_annealing" {
                                            [-corner corner]
                                            [-slack_threshold slack_threshold]
                                            [-seed seed]
                                            [-temp temp]
                                            [-iters iters]
                                            [-revert_after revert_after]
                                            [-initial_ops initial_ops]
                                            [-split_large_inputs split_large_inputs]
                                          }

proc resynth_annealing { args } {
  sta::parse_key_args "resynth_annealing" args \
    keys {-corner -iters -revert_after -seed -temp -initial_ops -slack_threshold -split_large_inputs} \
    flags {}

  set corner [sta::parse_corner keys]
  if { [info exists keys(-slack_threshold)] } {
    rmp::set_slack_threshold $keys(-slack_threshold)
  }
  if { [info exists keys(-seed)] } {
    rmp::set_annealing_seed $keys(-seed)
  }
  if { [info exists keys(-temp)] } {
    rmp::set_annealing_temp $keys(-temp)
  }
  if { [info exists keys(-iters)] } {
    rmp::set_annealing_iters $keys(-iters)
  }
  if { [info exists keys(-revert_after)] } {
    rmp::set_annealing_revert_after $keys(-revert_after)
  }
  if { [info exists keys(-initial_ops)] } {
    rmp::set_annealing_initial_ops $keys(-initial_ops)
  }
  if { [info exists keys(-split_large_inputs)] } {
    rmp::set_split_large_inputs $keys(-split_large_inputs)
  } else {
    rmp::set_split_large_inputs 0
  }

  rmp::resynth_annealing_cmd $corner
}

sta::define_cmd_args "position_driven_remap" {
  [-corner corner]
  [-percentage percentage]
  [-max_percentage max_percentage]
  [-slack_threshold slack_threshold]
  [-detailed_placement]
  [-verbose]
  [-max_vertices_per_endpoint n]
  [-max_cut_instances n]
  [-max_cut_pis n]
  [-max_solutions n]
  [-uct_batch_size n]
  [-uct_rounds n]
  [-uct_c c]
  [-max_candidates n]
  [-top_k n]
  [-use_beam_search]
}

proc position_driven_remap { args } {
  sta::parse_key_args "position_driven_remap" args \
    keys {-corner -percentage -max_percentage -slack_threshold
          -max_vertices_per_endpoint -max_cut_instances -max_cut_pis
          -max_solutions -uct_batch_size -uct_rounds -uct_c -max_candidates
          -top_k} \
    flags {-detailed_placement -verbose -use_beam_search}
  set corner [sta::parse_corner keys]

  # Defaults: -1.0 signals "not set" for percentage/max_percentage;
  # has_threshold=0 signals that -slack_threshold was not provided.
  set percentage     -1.0
  set max_percentage -1.0
  set slack_threshold 0.0
  set has_threshold   0
  set run_dpl [info exists flags(-detailed_placement)]
  set verbose [info exists flags(-verbose)]

  if { [info exists keys(-percentage)] } {
    set percentage $keys(-percentage)
  }
  if { [info exists keys(-max_percentage)] } {
    set max_percentage $keys(-max_percentage)
  }
  if { [info exists keys(-slack_threshold)] } {
    set slack_threshold $keys(-slack_threshold)
    set has_threshold 1
  }

  # RemapConfig defaults (must match RemapConfig struct defaults in RemapConfig.hh)
  set max_vertices_per_endpoint 5
  set max_cut_instances         10
  set max_cut_pis               20
  set max_solutions             80
  set uct_batch_size            20
  set uct_rounds                5
  set uct_c                     1.414
  set max_candidates            100
  set top_k                     5
  set use_beam_search [info exists flags(-use_beam_search)]

  if { [info exists keys(-max_vertices_per_endpoint)] } {
    set max_vertices_per_endpoint $keys(-max_vertices_per_endpoint)
  }
  if { [info exists keys(-max_cut_instances)] } {
    set max_cut_instances $keys(-max_cut_instances)
  }
  if { [info exists keys(-max_cut_pis)] } {
    set max_cut_pis $keys(-max_cut_pis)
  }
  if { [info exists keys(-max_solutions)] } {
    set max_solutions $keys(-max_solutions)
  }
  if { [info exists keys(-uct_batch_size)] } {
    set uct_batch_size $keys(-uct_batch_size)
  }
  if { [info exists keys(-uct_rounds)] } {
    set uct_rounds $keys(-uct_rounds)
  }
  if { [info exists keys(-uct_c)] } {
    set uct_c $keys(-uct_c)
  }
  if { [info exists keys(-max_candidates)] } {
    set max_candidates $keys(-max_candidates)
  }
  if { [info exists keys(-top_k)] } {
    set top_k $keys(-top_k)
  }

  rmp::position_driven_remap_cmd $corner $percentage $max_percentage \
      $slack_threshold $has_threshold $run_dpl $verbose \
      $max_vertices_per_endpoint $max_cut_instances $max_cut_pis \
      $max_solutions $uct_batch_size $uct_rounds $uct_c $max_candidates \
      $use_beam_search $top_k
}

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

add_library(jsonout
    jsonout.cpp
    jsonout_impl.cpp
)

target_include_directories(jsonout
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include
        ${TCL_INCLUDE_PATH}
)
target_link_libraries(jsonout
    db
    utl_lib
)

set_target_properties(jsonout
  PROPERTIES
    # python requirement
    POSITION_INDEPENDENT_CODE ON
)


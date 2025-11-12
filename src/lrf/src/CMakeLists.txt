# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2019-2025, The OpenROAD Authors

include("openroad")

swig_lib(NAME      lrf
         NAMESPACE lrf
         I_FILE    LRFW.i
         SWIG_INCLUDES ${OPENSTA_HOME}
         SCRIPTS   LRFW.tcl
)

add_library(lrf_lib
    LrHelper.cc
    LrfMgr.cc
    LrSizer.cc
)

target_sources(lrf
  PRIVATE
    # Graphics.cc
    MakeLrf.cc
)

target_include_directories(lrf
  PUBLIC
    ../include
  PRIVATE
    # A side-effect of including OpenSTA swig files
    ${OPENSTA_HOME}
    # ${OPENSTA_HOME}/include
    ${OPENSTA_HOME}/include/sta
    .
)

target_include_directories(lrf_lib
  PUBLIC
    ../include
  PRIVATE
    ${OPENSTA_HOME}
    # ${OPENSTA_HOME}/include
    ${OPENSTA_HOME}/include/sta
    .
)

target_link_libraries(lrf_lib
  PUBLIC
    est_lib
    stt_lib
    odb
    OpenSTA
    dbSta_lib
    grt_lib
    utl_lib
)

target_link_libraries(lrf
  PUBLIC
    lrf_lib
    est
    stt
    odb
    OpenSTA
    dbSta
    grt
    gui
    utl_lib
)

messages(
  TARGET lrf
  OUTPUT_DIR ..
)

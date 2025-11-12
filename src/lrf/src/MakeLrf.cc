// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "lrf/MakeLrf.h"

#include "odb/db.h"
#include "lrf/LrfMgr.hh"
#include "tcl.h"
#include "utl/decode.h"

extern "C" {
extern int Lrf_Init(Tcl_Interp* interp);
}

namespace lrf {
extern const char* lrf_tcl_inits[];

void initLrf(Tcl_Interp* tcl_interp)
{
  // Define swig TCL commands.
  Lrf_Init(tcl_interp);
  // Eval encoded sta TCL sources.
  utl::evalTclInit(tcl_interp, lrf::lrf_tcl_inits);
}

}  // namespace lrf

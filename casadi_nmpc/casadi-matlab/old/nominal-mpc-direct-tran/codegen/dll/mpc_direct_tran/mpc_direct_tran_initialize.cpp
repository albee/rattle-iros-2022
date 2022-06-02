//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_direct_tran_initialize.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "mpc_direct_tran_initialize.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : void
// Return Type  : void
//
void mpc_direct_tran_initialize()
{
  rt_InitInfAndNaN();
  isInitialized_libmpc_direct_tran = true;
}

//
// File trailer for mpc_direct_tran_initialize.cpp
//
// [EOF]
//

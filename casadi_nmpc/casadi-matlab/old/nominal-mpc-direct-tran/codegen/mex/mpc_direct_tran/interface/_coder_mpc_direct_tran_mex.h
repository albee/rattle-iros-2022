//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  _coder_mpc_direct_tran_mex.h
//
//  Code generation for function '_coder_mpc_direct_tran_mex'
//


#pragma once

// Include files
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "mex.h"
#include "emlrt.h"
#include "covrt.h"
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mpc_direct_tran_mexFunction(mpc_direct_tranStackData
  *SD, int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[6]);
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs,
  const mxArray *prhs[]);
emlrtCTX mexFunctionCreateRootTLS();

// End of code generation (_coder_mpc_direct_tran_mex.h)

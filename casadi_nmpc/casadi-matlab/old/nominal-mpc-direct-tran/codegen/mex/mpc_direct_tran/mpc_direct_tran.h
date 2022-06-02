//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mpc_direct_tran.h
//
//  Code generation for function 'mpc_direct_tran'
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
CODEGEN_EXPORT_SYM void mpc_direct_tran(const emlrtStack *sp, const real_T
  x_cur[6], real_T t_idx, real_T dt, real_T N, real_T Nf, const real_T
  x_ref_hist[60000], real_T u_t_idx[3]);

// End of code generation (mpc_direct_tran.h)

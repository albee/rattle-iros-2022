//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFval.h
//
//  Code generation for function 'computeFval'
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
real_T computeFval(const emlrtStack *sp, const d_struct_T *obj, coder::array<
                   real_T, 2U> &workspace, const coder::array<real_T, 2U> &H,
                   const coder::array<real_T, 1U> &f, const coder::array<real_T,
                   1U> &x);

// End of code generation (computeFval.h)

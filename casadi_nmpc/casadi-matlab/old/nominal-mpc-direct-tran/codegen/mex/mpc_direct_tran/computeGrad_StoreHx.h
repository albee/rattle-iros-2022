//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeGrad_StoreHx.h
//
//  Code generation for function 'computeGrad_StoreHx'
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
void computeGrad_StoreHx(const emlrtStack *sp, d_struct_T *obj, const coder::
  array<real_T, 2U> &H, const coder::array<real_T, 1U> &f, const coder::array<
  real_T, 1U> &x);

// End of code generation (computeGrad_StoreHx.h)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  maxConstraintViolation.h
//
//  Code generation for function 'maxConstraintViolation'
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
real_T b_maxConstraintViolation(const emlrtStack *sp, g_struct_T *obj, const
  coder::array<real_T, 2U> &x, int32_T ix0);
real_T c_maxConstraintViolation(const emlrtStack *sp, g_struct_T *obj, const
  coder::array<real_T, 1U> &x);
real_T maxConstraintViolation(const emlrtStack *sp, g_struct_T *obj, const coder::
  array<real_T, 2U> &x);

// End of code generation (maxConstraintViolation.h)

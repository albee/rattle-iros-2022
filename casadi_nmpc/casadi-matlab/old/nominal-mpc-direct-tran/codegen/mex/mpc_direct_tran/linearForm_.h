//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  linearForm_.h
//
//  Code generation for function 'linearForm_'
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
void linearForm_(const emlrtStack *sp, boolean_T obj_hasLinear, int32_T obj_nvar,
                 coder::array<real_T, 2U> &workspace, const coder::array<real_T,
                 2U> &H, const coder::array<real_T, 1U> &f, const coder::array<
                 real_T, 1U> &x);

// End of code generation (linearForm_.h)

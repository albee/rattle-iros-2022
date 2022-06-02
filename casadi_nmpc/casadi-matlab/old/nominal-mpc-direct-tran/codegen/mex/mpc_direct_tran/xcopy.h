//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xcopy.h
//
//  Code generation for function 'xcopy'
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
void b_xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 2U> &x,
             int32_T ix0, coder::array<real_T, 2U> &y, int32_T iy0);
void c_xcopy(const emlrtStack *sp, int32_T n, coder::array<real_T, 1U> &y);
void d_xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 2U> &x,
             int32_T ix0, coder::array<real_T, 2U> &y, int32_T iy0);
void e_xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 2U> &x,
             coder::array<real_T, 2U> &y);
void xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 1U> &x,
           coder::array<real_T, 2U> &y);

// End of code generation (xcopy.h)

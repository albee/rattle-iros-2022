//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgemm.h
//
//  Code generation for function 'xgemm'
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
void b_xgemm(const emlrtStack *sp, int32_T m, int32_T n, int32_T k, const coder::
             array<real_T, 2U> &A, int32_T ia0, int32_T lda, const coder::array<
             real_T, 2U> &B, int32_T ldb, coder::array<real_T, 2U> &C, int32_T
             ldc);
void xgemm(const emlrtStack *sp, int32_T m, int32_T n, int32_T k, const coder::
           array<real_T, 2U> &A, int32_T lda, const coder::array<real_T, 2U> &B,
           int32_T ib0, int32_T ldb, coder::array<real_T, 2U> &C, int32_T ldc);

// End of code generation (xgemm.h)

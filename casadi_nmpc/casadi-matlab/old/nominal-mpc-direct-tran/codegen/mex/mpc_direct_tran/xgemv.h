//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgemv.h
//
//  Code generation for function 'xgemv'
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
void b_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 2U> &x,
             int32_T ix0, coder::array<real_T, 1U> &y);
void c_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 1U> &y);
void d_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 1U> &y);
void e_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T ia0, int32_T lda, const coder::array<real_T,
             2U> &x, coder::array<real_T, 1U> &y);
void f_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 2U> &y);
void g_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 2U> &y, int32_T iy0);
void xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<real_T,
           2U> &A, int32_T lda, const coder::array<real_T, 2U> &x, coder::array<
           real_T, 1U> &y);

// End of code generation (xgemv.h)

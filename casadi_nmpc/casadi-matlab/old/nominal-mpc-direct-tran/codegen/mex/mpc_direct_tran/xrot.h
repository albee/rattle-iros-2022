//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xrot.h
//
//  Code generation for function 'xrot'
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
void b_xrot(const emlrtStack *sp, int32_T n, coder::array<real_T, 2U> &x,
            int32_T ix0, int32_T incx, int32_T iy0, int32_T incy, real_T c,
            real_T s);
void xrot(const emlrtStack *sp, int32_T n, coder::array<real_T, 2U> &x, int32_T
          ix0, int32_T iy0, real_T c, real_T s);

// End of code generation (xrot.h)

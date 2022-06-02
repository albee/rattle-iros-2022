//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgeqrf.h
//
//  Code generation for function 'xgeqrf'
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
void xgeqrf(const emlrtStack *sp, coder::array<real_T, 2U> &A, int32_T m,
            int32_T n, coder::array<real_T, 1U> &tau);

// End of code generation (xgeqrf.h)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factor.h
//
//  Code generation for function 'factor'
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
void factor(const emlrtStack *sp, f_struct_T *obj, const coder::array<real_T, 2U>
            &A, int32_T ndims, int32_T ldA);

// End of code generation (factor.h)

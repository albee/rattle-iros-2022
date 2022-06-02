//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  eye.h
//
//  Code generation for function 'eye'
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
void b_eye(const emlrtStack *sp, real_T varargin_1, coder::array<real_T, 2U>
           &b_I);
void eye(real_T b_I[36]);

// End of code generation (eye.h)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  repmat.h
//
//  Code generation for function 'repmat'
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
void b_repmat(const emlrtStack *sp, real_T varargin_1, coder::array<real_T, 1U>
              &b);
void repmat(const emlrtStack *sp, real_T varargin_1, coder::array<real_T, 1U> &b);

// End of code generation (repmat.h)

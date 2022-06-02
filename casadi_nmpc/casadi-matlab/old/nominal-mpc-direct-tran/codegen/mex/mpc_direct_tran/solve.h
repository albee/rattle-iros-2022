//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  solve.h
//
//  Code generation for function 'solve'
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
void solve(const emlrtStack *sp, const f_struct_T *obj, coder::array<real_T, 1U>
           &rhs);

// End of code generation (solve.h)

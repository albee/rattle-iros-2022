//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  diag.h
//
//  Code generation for function 'diag'
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
void diag(const emlrtStack *sp, const coder::array<real_T, 2U> &v, coder::array<
          real_T, 2U> &d);

// End of code generation (diag.h)

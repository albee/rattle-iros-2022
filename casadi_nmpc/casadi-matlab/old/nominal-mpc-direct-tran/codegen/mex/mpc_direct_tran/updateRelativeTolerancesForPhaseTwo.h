//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  updateRelativeTolerancesForPhaseTwo.h
//
//  Code generation for function 'updateRelativeTolerancesForPhaseTwo'
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
void c_updateRelativeTolerancesForPh(const emlrtStack *sp, real_T *tol, const
  coder::array<real_T, 2U> &H, const coder::array<real_T, 1U> &f);

// End of code generation (updateRelativeTolerancesForPhaseTwo.h)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  checkStoppingAndUpdateFval.h
//
//  Code generation for function 'checkStoppingAndUpdateFval'
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
void checkStoppingAndUpdateFval(const emlrtStack *sp, int32_T *activeSetChangeID,
  const coder::array<real_T, 1U> &f, b_struct_T *solution, c_struct_T *memspace,
  const d_struct_T *objective, g_struct_T *workingset, e_struct_T *qrmanager,
  real_T options_ObjectiveLimit, int32_T runTimeOptions_MaxIterations, real_T
  c_runTimeOptions_ConstrRelTolFa, boolean_T updateFval);

// End of code generation (checkStoppingAndUpdateFval.h)

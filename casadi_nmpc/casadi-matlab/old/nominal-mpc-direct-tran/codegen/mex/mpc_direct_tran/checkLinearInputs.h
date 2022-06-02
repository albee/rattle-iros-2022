//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  checkLinearInputs.h
//
//  Code generation for function 'checkLinearInputs'
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
void checkLinearInputs(const emlrtStack *sp, const coder::array<real_T, 1U> &x0,
  const coder::array<real_T, 2U> &Aineq, const coder::array<real_T, 1U> &bineq,
  const coder::array<real_T, 2U> &Aeq, const coder::array<real_T, 1U> &beq);

// End of code generation (checkLinearInputs.h)

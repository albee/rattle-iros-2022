//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFirstOrderOpt.h
//
//  Code generation for function 'computeFirstOrderOpt'
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
void computeFirstOrderOpt(const emlrtStack *sp, b_struct_T *solution, const
  d_struct_T *objective, int32_T workingset_nVar, int32_T workingset_ldA, const
  coder::array<real_T, 2U> &workingset_ATwset, int32_T workingset_nActiveConstr,
  coder::array<real_T, 2U> &workspace);

// End of code generation (computeFirstOrderOpt.h)

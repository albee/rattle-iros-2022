//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleratiotest.h
//
//  Code generation for function 'feasibleratiotest'
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
void feasibleratiotest(const emlrtStack *sp, const coder::array<real_T, 1U>
  &solution_xstar, const coder::array<real_T, 1U> &solution_searchDir, coder::
  array<real_T, 2U> &workspace, int32_T workingset_nVar, int32_T workingset_ldA,
  const coder::array<real_T, 2U> &workingset_Aineq, const coder::array<real_T,
  2U> &workingset_bineq, const coder::array<real_T, 1U> &workingset_lb, const
  coder::array<int32_T, 1U> &workingset_indexLB, const int32_T workingset_sizes
  [5], const int32_T workingset_isActiveIdx[6], const coder::array<boolean_T, 1U>
  &workingset_isActiveConstr, const int32_T workingset_nWConstr[5], boolean_T
  isPhaseOne, real_T *alpha, boolean_T *newBlocking, int32_T *constrType,
  int32_T *constrIdx);

// End of code generation (feasibleratiotest.h)

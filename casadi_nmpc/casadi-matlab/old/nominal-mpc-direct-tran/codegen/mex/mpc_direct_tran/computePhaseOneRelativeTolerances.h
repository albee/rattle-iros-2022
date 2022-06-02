//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computePhaseOneRelativeTolerances.h
//
//  Code generation for function 'computePhaseOneRelativeTolerances'
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
real_T c_computePhaseOneRelativeTolera(const emlrtStack *sp, int32_T
  workingset_nVarOrig, const coder::array<real_T, 2U> &workingset_Aineq, const
  coder::array<real_T, 2U> &workingset_Aeq, const int32_T workingset_sizes[5]);

// End of code generation (computePhaseOneRelativeTolerances.h)

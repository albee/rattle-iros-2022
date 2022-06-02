//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleX0ForWorkingSet.h
//
//  Code generation for function 'feasibleX0ForWorkingSet'
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
boolean_T feasibleX0ForWorkingSet(const emlrtStack *sp, coder::array<real_T, 2U>
  &workspace, coder::array<real_T, 1U> &xCurrent, g_struct_T *workingset,
  e_struct_T *qrmanager);

// End of code generation (feasibleX0ForWorkingSet.h)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  iterate.h
//
//  Code generation for function 'iterate'
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
void iterate(const emlrtStack *sp, const coder::array<real_T, 2U> &H, const
             coder::array<real_T, 1U> &f, b_struct_T *solution, c_struct_T
             *memspace, g_struct_T *workingset, e_struct_T *qrmanager,
             f_struct_T *cholmanager, d_struct_T *objective, real_T
             options_ObjectiveLimit, real_T options_StepTolerance, int32_T
             runTimeOptions_MaxIterations, real_T
             c_runTimeOptions_ConstrRelTolFa, real_T
             runTimeOptions_ProbRelTolFactor, boolean_T
             runTimeOptions_RemainFeasible);

// End of code generation (iterate.h)

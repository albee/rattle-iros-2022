//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  phaseone.h
//
//  Code generation for function 'phaseone'
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
void phaseone(const emlrtStack *sp, const coder::array<real_T, 2U> &H, const
              coder::array<real_T, 1U> &f, b_struct_T *solution, c_struct_T
              *memspace, g_struct_T *workingset, e_struct_T *qrmanager,
              f_struct_T *cholmanager, d_struct_T *objective, h_struct_T
              *options, const struct_T *runTimeOptions);

// End of code generation (phaseone.h)

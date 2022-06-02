//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  compute_lambda.h
//
//  Code generation for function 'compute_lambda'
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
void compute_lambda(const emlrtStack *sp, coder::array<real_T, 2U> &workspace,
                    b_struct_T *solution, const d_struct_T *objective, const
                    e_struct_T *qrmanager);

// End of code generation (compute_lambda.h)

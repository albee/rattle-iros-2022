//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  compute_deltax.h
//
//  Code generation for function 'compute_deltax'
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
void compute_deltax(const emlrtStack *sp, const coder::array<real_T, 2U> &H,
                    b_struct_T *solution, c_struct_T *memspace, const e_struct_T
                    *qrmanager, f_struct_T *cholmanager, const d_struct_T
                    *objective);

// End of code generation (compute_deltax.h)

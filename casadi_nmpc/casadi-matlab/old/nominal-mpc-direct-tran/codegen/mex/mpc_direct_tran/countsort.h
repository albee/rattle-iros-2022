//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  countsort.h
//
//  Code generation for function 'countsort'
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
void countsort(const emlrtStack *sp, coder::array<int32_T, 1U> &x, int32_T xLen,
               coder::array<int32_T, 1U> &workspace, int32_T xMin, int32_T xMax);

// End of code generation (countsort.h)

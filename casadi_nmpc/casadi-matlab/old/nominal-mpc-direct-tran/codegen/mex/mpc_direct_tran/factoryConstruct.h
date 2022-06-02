//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factoryConstruct.h
//
//  Code generation for function 'factoryConstruct'
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
void factoryConstruct(const emlrtStack *sp, int32_T mIneq, int32_T mLinIneq,
                      const coder::array<real_T, 2U> &Aineq, const coder::array<
                      real_T, 1U> &bineq, int32_T mEq, int32_T mLinEq, const
                      coder::array<real_T, 2U> &Aeq, const coder::array<real_T,
                      1U> &beq, int32_T nVar, int32_T nVarMax, int32_T
                      mConstrMax, g_struct_T *obj);

// End of code generation (factoryConstruct.h)

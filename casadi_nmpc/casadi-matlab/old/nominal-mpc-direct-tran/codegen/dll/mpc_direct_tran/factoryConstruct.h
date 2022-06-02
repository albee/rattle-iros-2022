//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factoryConstruct.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef FACTORYCONSTRUCT_H
#define FACTORYCONSTRUCT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void factoryConstruct(int mIneq, int mLinIneq, const coder::array<double,
  2U> &Aineq, const coder::array<double, 1U> &bineq, int mEq, int mLinEq, const
  coder::array<double, 2U> &Aeq, const coder::array<double, 1U> &beq, int nVar,
  int nVarMax, int mConstrMax, g_struct_T *obj);

#endif

//
// File trailer for factoryConstruct.h
//
// [EOF]
//

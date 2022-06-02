//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFirstOrderOpt.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef COMPUTEFIRSTORDEROPT_H
#define COMPUTEFIRSTORDEROPT_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void computeFirstOrderOpt(b_struct_T *solution, const d_struct_T
  *objective, int workingset_nVar, int workingset_ldA, const coder::array<double,
  2U> &workingset_ATwset, int workingset_nActiveConstr, coder::array<double, 2U>
  &workspace);

#endif

//
// File trailer for computeFirstOrderOpt.h
//
// [EOF]
//

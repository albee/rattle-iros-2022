//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ratiotest.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef RATIOTEST_H
#define RATIOTEST_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void ratiotest(const coder::array<double, 1U> &solution_xstar, const
                      coder::array<double, 1U> &solution_searchDir, coder::array<
                      double, 2U> &workspace, int workingset_nVar, int
                      workingset_ldA, const coder::array<double, 2U>
                      &workingset_Aineq, const coder::array<double, 2U>
                      &workingset_bineq, const coder::array<double, 1U>
                      &workingset_lb, const coder::array<int, 1U>
                      &workingset_indexLB, const int workingset_sizes[5], const
                      int workingset_isActiveIdx[6], const coder::array<
                      boolean_T, 1U> &workingset_isActiveConstr, const int
                      workingset_nWConstr[5], double *toldelta, double *alpha,
                      boolean_T *newBlocking, int *constrType, int *constrIdx);

#endif

//
// File trailer for ratiotest.h
//
// [EOF]
//

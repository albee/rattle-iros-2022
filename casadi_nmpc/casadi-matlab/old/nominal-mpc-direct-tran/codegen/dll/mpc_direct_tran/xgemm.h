//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemm.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef XGEMM_H
#define XGEMM_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void b_xgemm(int m, int n, int k, const coder::array<double, 2U> &A, int
                    ia0, int lda, const coder::array<double, 2U> &B, int ldb,
                    coder::array<double, 2U> &C, int ldc);
extern void xgemm(int m, int n, int k, const coder::array<double, 2U> &A, int
                  lda, const coder::array<double, 2U> &B, int ib0, int ldb,
                  coder::array<double, 2U> &C, int ldc);

#endif

//
// File trailer for xgemm.h
//
// [EOF]
//

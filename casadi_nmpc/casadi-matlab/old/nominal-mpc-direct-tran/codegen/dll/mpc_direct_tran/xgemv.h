//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef XGEMV_H
#define XGEMV_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void b_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda,
                    const coder::array<double, 2U> &x, int ix0, coder::array<
                    double, 1U> &y);
extern void c_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda,
                    const coder::array<double, 1U> &x, coder::array<double, 1U>
                    &y);
extern void d_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda,
                    const coder::array<double, 1U> &x, coder::array<double, 2U>
                    &y);
extern void e_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda,
                    const coder::array<double, 1U> &x, coder::array<double, 2U>
                    &y, int iy0);
extern void xgemv(int m, int n, const coder::array<double, 2U> &A, int lda,
                  const coder::array<double, 2U> &x, coder::array<double, 1U> &y);

#endif

//
// File trailer for xgemv.h
//
// [EOF]
//

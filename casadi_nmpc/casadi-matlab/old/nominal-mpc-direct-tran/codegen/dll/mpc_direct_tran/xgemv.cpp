//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemv.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "xgemv.h"
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                const coder::array<double, 2U> *A
//                int lda
//                const coder::array<double, 2U> *x
//                int ix0
//                coder::array<double, 1U> *y
// Return Type  : void
//
void b_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda, const
             coder::array<double, 2U> &x, int ix0, coder::array<double, 1U> &y)
{
  int iac;
  double c;
  if (n != 0) {
    int iy;
    int i;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }

    iy = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int ix;
      int i1;
      ix = ix0;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ix - 1];
        ix++;
      }

      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const coder::array<double, 2U> *A
//                int lda
//                const coder::array<double, 1U> *x
//                coder::array<double, 1U> *y
// Return Type  : void
//
void c_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda, const
             coder::array<double, 1U> &x, coder::array<double, 1U> &y)
{
  int iac;
  double c;
  if (n != 0) {
    int iy;
    int i;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }

    iy = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int ix;
      int i1;
      ix = 0;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ix];
        ix++;
      }

      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const coder::array<double, 2U> *A
//                int lda
//                const coder::array<double, 1U> *x
//                coder::array<double, 2U> *y
// Return Type  : void
//
void d_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda, const
             coder::array<double, 1U> &x, coder::array<double, 2U> &y)
{
  int iac;
  double c;
  if (n != 0) {
    int iy;
    int i;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }

    iy = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int ix;
      int i1;
      ix = 0;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ix];
        ix++;
      }

      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const coder::array<double, 2U> *A
//                int lda
//                const coder::array<double, 1U> *x
//                coder::array<double, 2U> *y
//                int iy0
// Return Type  : void
//
void e_xgemv(int m, int n, const coder::array<double, 2U> &A, int lda, const
             coder::array<double, 1U> &x, coder::array<double, 2U> &y, int iy0)
{
  int iac;
  double c;
  if (n != 0) {
    int iyend;
    int iy;
    iyend = (iy0 + n) - 1;
    for (iy = iy0; iy <= iyend; iy++) {
      y[iy - 1] = 0.0;
    }

    iy = iy0 - 1;
    iyend = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= iyend : iac <= iyend; iac += lda) {
      int ix;
      int i;
      ix = 0;
      c = 0.0;
      i = (iac + m) - 1;
      for (int ia = iac; ia <= i; ia++) {
        c += A[ia - 1] * x[ix];
        ix++;
      }

      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// Arguments    : int m
//                int n
//                const coder::array<double, 2U> *A
//                int lda
//                const coder::array<double, 2U> *x
//                coder::array<double, 1U> *y
// Return Type  : void
//
void xgemv(int m, int n, const coder::array<double, 2U> &A, int lda, const coder::
           array<double, 2U> &x, coder::array<double, 1U> &y)
{
  int iac;
  double c;
  if (n != 0) {
    int iy;
    int i;
    for (iy = 0; iy < n; iy++) {
      y[iy] = -y[iy];
    }

    iy = 0;
    i = lda * (n - 1) + 1;
    for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int ix;
      int i1;
      ix = 0;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia = iac; ia <= i1; ia++) {
        c += A[ia - 1] * x[ix];
        ix++;
      }

      y[iy] = y[iy] + c;
      iy++;
    }
  }
}

//
// File trailer for xgemv.cpp
//
// [EOF]
//

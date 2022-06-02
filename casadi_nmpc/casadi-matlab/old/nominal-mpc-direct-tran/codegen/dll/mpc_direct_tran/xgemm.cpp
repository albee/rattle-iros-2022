//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgemm.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "xgemm.h"
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                int k
//                const coder::array<double, 2U> *A
//                int ia0
//                int lda
//                const coder::array<double, 2U> *B
//                int ldb
//                coder::array<double, 2U> *C
//                int ldc
// Return Type  : void
//
void b_xgemm(int m, int n, int k, const coder::array<double, 2U> &A, int ia0,
             int lda, const coder::array<double, 2U> &B, int ldb, coder::array<
             double, 2U> &C, int ldc)
{
  int cr;
  int br;
  int ar;
  double temp;
  if ((m != 0) && (n != 0)) {
    int lastColC;
    int i;
    int i1;
    int ic;
    lastColC = ldc * (n - 1);
    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        C[ic - 1] = 0.0;
      }
    }

    br = -1;
    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        temp = 0.0;
        for (int w = 0; w < k; w++) {
          int b_w;
          b_w = w + 1;
          temp += A[(b_w + ar) - 2] * B[b_w + br];
        }

        C[ic - 1] = C[ic - 1] + temp;
        ar += lda;
      }

      br += ldb;
    }
  }
}

//
// Arguments    : int m
//                int n
//                int k
//                const coder::array<double, 2U> *A
//                int lda
//                const coder::array<double, 2U> *B
//                int ib0
//                int ldb
//                coder::array<double, 2U> *C
//                int ldc
// Return Type  : void
//
void xgemm(int m, int n, int k, const coder::array<double, 2U> &A, int lda,
           const coder::array<double, 2U> &B, int ib0, int ldb, coder::array<
           double, 2U> &C, int ldc)
{
  int br;
  int cr;
  int ar;
  if ((m != 0) && (n != 0)) {
    int lastColC;
    int i;
    int i1;
    int ic;
    br = ib0;
    lastColC = ldc * (n - 1);
    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      i = cr + 1;
      i1 = cr + m;
      for (ic = i; ic <= i1; ic++) {
        C[ic - 1] = 0.0;
      }
    }

    for (cr = 0; ldc < 0 ? cr >= lastColC : cr <= lastColC; cr += ldc) {
      ar = -1;
      i = br + k;
      for (int ib = br; ib < i; ib++) {
        int ia;
        int i2;
        ia = ar;
        i1 = cr + 1;
        i2 = cr + m;
        for (ic = i1; ic <= i2; ic++) {
          ia++;
          C[ic - 1] = C[ic - 1] + B[ib - 1] * A[ia];
        }

        ar += lda;
      }

      br += ldb;
    }
  }
}

//
// File trailer for xgemm.cpp
//
// [EOF]
//

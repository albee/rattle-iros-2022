//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgerc.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "xgerc.h"
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                double alpha1
//                int ix0
//                const coder::array<double, 1U> *y
//                coder::array<double, 2U> *A
//                int ia0
//                int lda
// Return Type  : void
//
void xgerc(int m, int n, double alpha1, int ix0, const coder::array<double, 1U>
           &y, coder::array<double, 2U> &A, int ia0, int lda)
{
  int jA;
  if (!(alpha1 == 0.0)) {
    int jy;
    jA = ia0;
    jy = 0;
    for (int j = 0; j < n; j++) {
      if (y[jy] != 0.0) {
        double temp;
        int ix;
        int i;
        temp = y[jy] * alpha1;
        ix = ix0;
        i = m + jA;
        for (int ijA = jA; ijA < i; ijA++) {
          A[ijA - 1] = A[ijA - 1] + A[ix - 1] * temp;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

//
// File trailer for xgerc.cpp
//
// [EOF]
//

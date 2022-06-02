//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xzlarf.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "xzlarf.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xgerc.h"

// Function Definitions

//
// Arguments    : int m
//                int n
//                int iv0
//                double tau
//                coder::array<double, 2U> *C
//                int ic0
//                int ldc
//                coder::array<double, 1U> *work
// Return Type  : void
//
void xzlarf(int m, int n, int iv0, double tau, coder::array<double, 2U> &C, int
            ic0, int ldc, coder::array<double, 1U> &work)
{
  int lastv;
  int lastc;
  int i;
  int iac;
  int ia;
  double c;
  if (tau != 0.0) {
    boolean_T exitg2;
    lastv = m;
    i = iv0 + m;
    while ((lastv > 0) && (C[i - 2] == 0.0)) {
      lastv--;
      i--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      int exitg1;
      i = ic0 + (lastc - 1) * ldc;
      ia = i;
      do {
        exitg1 = 0;
        if (ia <= (i + lastv) - 1) {
          if (C[ia - 1] != 0.0) {
            exitg1 = 1;
          } else {
            ia++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    if (lastc != 0) {
      int iy;
      for (iy = 0; iy < lastc; iy++) {
        work[iy] = 0.0;
      }

      iy = 0;
      i = ic0 + ldc * (lastc - 1);
      for (iac = ic0; ldc < 0 ? iac >= i : iac <= i; iac += ldc) {
        int ix;
        int b_i;
        ix = iv0;
        c = 0.0;
        b_i = (iac + lastv) - 1;
        for (ia = iac; ia <= b_i; ia++) {
          c += C[ia - 1] * C[ix - 1];
          ix++;
        }

        work[iy] = work[iy] + c;
        iy++;
      }
    }

    xgerc(lastv, lastc, -tau, iv0, work, C, ic0, ldc);
  }
}

//
// File trailer for xzlarf.cpp
//
// [EOF]
//

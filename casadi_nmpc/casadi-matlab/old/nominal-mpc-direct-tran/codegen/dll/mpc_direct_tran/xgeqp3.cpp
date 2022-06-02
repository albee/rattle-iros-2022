//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: xgeqp3.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "xgeqp3.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xzgeqp3.h"
#include "xzlarf.h"
#include "xzlarfg.h"
#include <cmath>

// Function Definitions

//
// Arguments    : coder::array<double, 2U> *A
//                int m
//                int n
//                coder::array<int, 1U> *jpvt
//                coder::array<double, 1U> *tau
// Return Type  : void
//
void xgeqp3(coder::array<double, 2U> &A, int m, int n, coder::array<int, 1U>
            &jpvt, coder::array<double, 1U> &tau)
{
  int ma;
  int pvt;
  int minmana;
  int minmn_tmp;
  int i;
  coder::array<double, 1U> work;
  double temp;
  coder::array<double, 1U> vn1;
  coder::array<double, 1U> vn2;
  ma = A.size(0);
  pvt = A.size(0);
  minmana = A.size(1);
  if (pvt < minmana) {
    minmana = pvt;
  }

  if (m < n) {
    minmn_tmp = m;
  } else {
    minmn_tmp = n;
  }

  tau.set_size(minmana);
  for (i = 0; i < minmana; i++) {
    tau[i] = 0.0;
  }

  if (minmn_tmp < 1) {
    for (int ii = 0; ii < n; ii++) {
      jpvt[ii] = ii + 1;
    }
  } else {
    int ii;
    int ix;
    int iy;
    int k;
    minmana = 0;
    for (ii = 0; ii < n; ii++) {
      if (jpvt[ii] != 0) {
        minmana++;
        if (ii + 1 != minmana) {
          ix = ii * ma;
          iy = (minmana - 1) * ma;
          for (k = 0; k < m; k++) {
            temp = A[ix];
            A[ix] = A[iy];
            A[iy] = temp;
            ix++;
            iy++;
          }

          jpvt[ii] = jpvt[minmana - 1];
          jpvt[minmana - 1] = ii + 1;
        } else {
          jpvt[ii] = ii + 1;
        }
      } else {
        jpvt[ii] = ii + 1;
      }
    }

    if (minmana >= minmn_tmp) {
      minmana = minmn_tmp;
    }

    qrf(A, m, n, minmana, tau);
    if (minmana < minmn_tmp) {
      double d;
      ma = A.size(0);
      work.set_size(A.size(1));
      pvt = A.size(1);
      for (i = 0; i < pvt; i++) {
        work[i] = 0.0;
      }

      vn1.set_size(A.size(1));
      pvt = A.size(1);
      for (i = 0; i < pvt; i++) {
        vn1[i] = 0.0;
      }

      vn2.set_size(A.size(1));
      pvt = A.size(1);
      for (i = 0; i < pvt; i++) {
        vn2[i] = 0.0;
      }

      i = minmana + 1;
      for (ii = i; ii <= n; ii++) {
        d = xnrm2(m - minmana, A, (minmana + (ii - 1) * ma) + 1);
        vn1[ii - 1] = d;
        vn2[ii - 1] = d;
      }

      i = minmana + 1;
      for (int b_i = i; b_i <= minmn_tmp; b_i++) {
        int ip1;
        int nmi;
        int mmi;
        double s;
        ip1 = b_i + 1;
        iy = (b_i - 1) * ma;
        ii = (iy + b_i) - 1;
        nmi = (n - b_i) + 1;
        mmi = m - b_i;
        if (nmi < 1) {
          minmana = -2;
        } else {
          minmana = -1;
          if (nmi > 1) {
            ix = b_i;
            temp = std::abs(vn1[b_i - 1]);
            for (k = 2; k <= nmi; k++) {
              ix++;
              s = std::abs(vn1[ix - 1]);
              if (s > temp) {
                minmana = k - 2;
                temp = s;
              }
            }
          }
        }

        pvt = b_i + minmana;
        if (pvt + 1 != b_i) {
          ix = pvt * ma;
          for (k = 0; k < m; k++) {
            temp = A[ix];
            A[ix] = A[iy];
            A[iy] = temp;
            ix++;
            iy++;
          }

          minmana = jpvt[pvt];
          jpvt[pvt] = jpvt[b_i - 1];
          jpvt[b_i - 1] = minmana;
          vn1[pvt] = vn1[b_i - 1];
          vn2[pvt] = vn2[b_i - 1];
        }

        if (b_i < m) {
          temp = A[ii];
          d = xzlarfg(mmi + 1, &temp, A, ii + 2);
          tau[b_i - 1] = d;
          A[ii] = temp;
        } else {
          d = 0.0;
          tau[b_i - 1] = 0.0;
        }

        if (b_i < n) {
          temp = A[ii];
          A[ii] = 1.0;
          xzlarf(mmi + 1, nmi - 1, ii + 1, d, A, (ii + ma) + 1, ma, work);
          A[ii] = temp;
        }

        for (ii = ip1; ii <= n; ii++) {
          pvt = b_i + (ii - 1) * ma;
          d = vn1[ii - 1];
          if (d != 0.0) {
            temp = std::abs(A[pvt - 1]) / d;
            temp = 1.0 - temp * temp;
            if (temp < 0.0) {
              temp = 0.0;
            }

            s = d / vn2[ii - 1];
            s = temp * (s * s);
            if (s <= 1.4901161193847656E-8) {
              if (b_i < m) {
                d = xnrm2(mmi, A, pvt + 1);
                vn1[ii - 1] = d;
                vn2[ii - 1] = d;
              } else {
                vn1[ii - 1] = 0.0;
                vn2[ii - 1] = 0.0;
              }
            } else {
              vn1[ii - 1] = d * std::sqrt(temp);
            }
          }
        }
      }
    }
  }
}

//
// File trailer for xgeqp3.cpp
//
// [EOF]
//

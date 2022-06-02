//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeQ_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "computeQ_.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : e_struct_T *obj
//                int nrows
// Return Type  : void
//
void computeQ_(e_struct_T *obj, int nrows)
{
  int i;
  int idx;
  int m;
  int iQR0;
  int ix;
  int n;
  int lda;
  int i1;
  coder::array<double, 1U> work;
  int jA;
  coder::array<double, 2U> A;
  double c;
  i = obj->minRowCol;
  for (idx = 0; idx < i; idx++) {
    iQR0 = obj->ldq * idx + idx;
    n = obj->mrows - idx;
    for (ix = 0; ix <= n - 2; ix++) {
      i1 = (iQR0 + ix) + 1;
      obj->Q[i1] = obj->QR[i1];
    }
  }

  m = obj->mrows;
  ix = obj->minRowCol;
  lda = obj->ldq;
  if (nrows >= 1) {
    int itau;
    int ia;
    int b_i;
    i = nrows - 1;
    for (idx = ix; idx <= i; idx++) {
      ia = idx * lda;
      i1 = m - 1;
      for (b_i = 0; b_i <= i1; b_i++) {
        obj->Q[ia + b_i] = 0.0;
      }

      obj->Q[ia + idx] = 1.0;
    }

    itau = obj->minRowCol - 1;
    iQR0 = obj->Q.size(1);
    work.set_size(iQR0);
    for (i = 0; i < iQR0; i++) {
      work[i] = 0.0;
    }

    for (b_i = obj->minRowCol; b_i >= 1; b_i--) {
      int iaii;
      iaii = b_i + (b_i - 1) * lda;
      if (b_i < nrows) {
        int lastv;
        int lastc;
        obj->Q[iaii - 1] = 1.0;
        jA = iaii + lda;
        A.set_size(obj->Q.size(0), obj->Q.size(1));
        iQR0 = obj->Q.size(0) * obj->Q.size(1);
        for (i = 0; i < iQR0; i++) {
          A[i] = obj->Q[i];
        }

        if (obj->tau[itau] != 0.0) {
          boolean_T exitg2;
          lastv = (m - b_i) + 1;
          iQR0 = (iaii + m) - b_i;
          while ((lastv > 0) && (obj->Q[iQR0 - 1] == 0.0)) {
            lastv--;
            iQR0--;
          }

          lastc = nrows - b_i;
          exitg2 = false;
          while ((!exitg2) && (lastc > 0)) {
            int exitg1;
            iQR0 = jA + (lastc - 1) * lda;
            ia = iQR0;
            do {
              exitg1 = 0;
              if (ia <= (iQR0 + lastv) - 1) {
                if (obj->Q[ia - 1] != 0.0) {
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
            for (iQR0 = 0; iQR0 < lastc; iQR0++) {
              work[iQR0] = 0.0;
            }

            iQR0 = 0;
            i = jA + lda * (lastc - 1);
            for (n = jA; lda < 0 ? n >= i : n <= i; n += lda) {
              ix = iaii;
              c = 0.0;
              i1 = (n + lastv) - 1;
              for (ia = n; ia <= i1; ia++) {
                c += obj->Q[ia - 1] * obj->Q[ix - 1];
                ix++;
              }

              work[iQR0] = work[iQR0] + c;
              iQR0++;
            }
          }

          c = -obj->tau[itau];
          if (!(c == 0.0)) {
            iQR0 = 0;
            for (idx = 0; idx < lastc; idx++) {
              if (work[iQR0] != 0.0) {
                double temp;
                temp = work[iQR0] * c;
                ix = iaii;
                i = lastv + jA;
                for (n = jA; n < i; n++) {
                  A[n - 1] = A[n - 1] + A[ix - 1] * temp;
                  ix++;
                }
              }

              iQR0++;
              jA += lda;
            }
          }
        }

        obj->Q.set_size(A.size(0), A.size(1));
        iQR0 = A.size(0) * A.size(1);
        for (i = 0; i < iQR0; i++) {
          obj->Q[i] = A[i];
        }
      }

      if (b_i < m) {
        iQR0 = iaii + 1;
        i = (iaii + m) - b_i;
        for (ix = iQR0; ix <= i; ix++) {
          obj->Q[ix - 1] = -obj->tau[itau] * obj->Q[ix - 1];
        }
      }

      obj->Q[iaii - 1] = 1.0 - obj->tau[itau];
      for (idx = 0; idx <= b_i - 2; idx++) {
        obj->Q[(iaii - idx) - 2] = 0.0;
      }

      itau--;
    }
  }
}

//
// File trailer for computeQ_.cpp
//
// [EOF]
//

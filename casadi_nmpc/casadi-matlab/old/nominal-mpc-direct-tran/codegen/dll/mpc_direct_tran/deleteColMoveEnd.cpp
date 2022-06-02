//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: deleteColMoveEnd.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "deleteColMoveEnd.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"
#include "xrotg.h"

// Function Definitions

//
// Arguments    : e_struct_T *obj
//                int idx
// Return Type  : void
//
void deleteColMoveEnd(e_struct_T *obj, int idx)
{
  int i;
  int QRk0;
  double temp;
  double c;
  double s;
  coder::array<double, 2U> x;
  if (obj->usedPivoting) {
    i = 1;
    while ((i <= obj->ncols) && (obj->jpvt[i - 1] != idx)) {
      i++;
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    int b_i;
    int k;
    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    b_i = obj->minRowCol;
    for (k = 0; k < b_i; k++) {
      obj->QR[k + obj->QR.size(0) * (idx - 1)] = obj->QR[k + obj->QR.size(0) *
        (obj->ncols - 1)];
    }

    obj->ncols--;
    QRk0 = obj->mrows;
    i = obj->ncols;
    if (QRk0 < i) {
      i = QRk0;
    }

    obj->minRowCol = i;
    if (idx < obj->mrows) {
      int endIdx;
      int n;
      int b_k;
      QRk0 = obj->mrows - 1;
      endIdx = obj->ncols;
      if (QRk0 < endIdx) {
        endIdx = QRk0;
      }

      for (k = endIdx; k >= idx; k--) {
        temp = obj->QR[k + obj->QR.size(0) * (idx - 1)];
        xrotg(&obj->QR[(k + obj->QR.size(0) * (idx - 1)) - 1], &temp, &c, &s);
        obj->QR[k + obj->QR.size(0) * (idx - 1)] = temp;
        obj->QR[k + obj->QR.size(0) * (k - 1)] = 0.0;
        QRk0 = k + obj->ldq * idx;
        n = obj->ncols - idx;
        x.set_size(obj->QR.size(0), obj->QR.size(1));
        i = obj->QR.size(0) * obj->QR.size(1);
        for (b_i = 0; b_i < i; b_i++) {
          x[b_i] = obj->QR[b_i];
        }

        if (n >= 1) {
          i = QRk0 - 1;
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * x[i] + s * x[QRk0];
            x[QRk0] = c * x[QRk0] - s * x[i];
            x[i] = temp;
            QRk0 += obj->ldq;
            i += obj->ldq;
          }
        }

        obj->QR.set_size(x.size(0), x.size(1));
        i = x.size(0) * x.size(1);
        for (b_i = 0; b_i < i; b_i++) {
          obj->QR[b_i] = x[b_i];
        }

        QRk0 = obj->ldq * (k - 1);
        n = obj->mrows;
        x.set_size(obj->Q.size(0), obj->Q.size(1));
        i = obj->Q.size(0) * obj->Q.size(1);
        for (b_i = 0; b_i < i; b_i++) {
          x[b_i] = obj->Q[b_i];
        }

        if (obj->mrows >= 1) {
          i = obj->ldq + QRk0;
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * x[QRk0] + s * x[i];
            x[i] = c * x[i] - s * x[QRk0];
            x[QRk0] = temp;
            i++;
            QRk0++;
          }
        }

        obj->Q.set_size(x.size(0), x.size(1));
        i = x.size(0) * x.size(1);
        for (b_i = 0; b_i < i; b_i++) {
          obj->Q[b_i] = x[b_i];
        }
      }

      b_i = idx + 1;
      for (k = b_i; k <= endIdx; k++) {
        temp = obj->QR[k + obj->QR.size(0) * (k - 1)];
        xrotg(&obj->QR[(k + obj->QR.size(0) * (k - 1)) - 1], &temp, &c, &s);
        obj->QR[k + obj->QR.size(0) * (k - 1)] = temp;
        QRk0 = k * (obj->ldq + 1);
        n = obj->ncols - k;
        x.set_size(obj->QR.size(0), obj->QR.size(1));
        i = obj->QR.size(0) * obj->QR.size(1);
        for (b_k = 0; b_k < i; b_k++) {
          x[b_k] = obj->QR[b_k];
        }

        if (n >= 1) {
          i = QRk0 - 1;
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * x[i] + s * x[QRk0];
            x[QRk0] = c * x[QRk0] - s * x[i];
            x[i] = temp;
            QRk0 += obj->ldq;
            i += obj->ldq;
          }
        }

        obj->QR.set_size(x.size(0), x.size(1));
        i = x.size(0) * x.size(1);
        for (b_k = 0; b_k < i; b_k++) {
          obj->QR[b_k] = x[b_k];
        }

        QRk0 = obj->ldq * (k - 1);
        n = obj->mrows;
        x.set_size(obj->Q.size(0), obj->Q.size(1));
        i = obj->Q.size(0) * obj->Q.size(1);
        for (b_k = 0; b_k < i; b_k++) {
          x[b_k] = obj->Q[b_k];
        }

        if (obj->mrows >= 1) {
          i = obj->ldq + QRk0;
          for (b_k = 0; b_k < n; b_k++) {
            temp = c * x[QRk0] + s * x[i];
            x[i] = c * x[i] - s * x[QRk0];
            x[QRk0] = temp;
            i++;
            QRk0++;
          }
        }

        obj->Q.set_size(x.size(0), x.size(1));
        i = x.size(0) * x.size(1);
        for (b_k = 0; b_k < i; b_k++) {
          obj->Q[b_k] = x[b_k];
        }
      }
    }
  }
}

//
// File trailer for deleteColMoveEnd.cpp
//
// [EOF]
//

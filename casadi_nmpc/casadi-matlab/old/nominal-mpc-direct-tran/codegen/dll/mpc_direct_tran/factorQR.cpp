//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factorQR.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "factorQR.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"

// Function Definitions

//
// Arguments    : e_struct_T *obj
//                const coder::array<double, 2U> *A
//                int mrows
//                int ncols
// Return Type  : void
//
void factorQR(e_struct_T *obj, const coder::array<double, 2U> &A, int mrows, int
              ncols)
{
  int idx;
  boolean_T guard1 = false;
  int ix0;
  int minmana;
  int b_k;
  coder::array<double, 2U> b_A;
  coder::array<double, 1U> tau;
  idx = mrows * ncols;
  guard1 = false;
  if (idx > 0) {
    for (idx = 0; idx < ncols; idx++) {
      ix0 = A.size(0) * idx;
      minmana = obj->ldq * idx;
      for (int k = 0; k < mrows; k++) {
        b_k = k + 1;
        obj->QR[(minmana + b_k) - 1] = A[(ix0 + b_k) - 1];
      }
    }

    guard1 = true;
  } else if (idx == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    for (idx = 0; idx < ncols; idx++) {
      obj->jpvt[idx] = idx + 1;
    }

    if (mrows < ncols) {
      idx = mrows;
    } else {
      idx = ncols;
    }

    obj->minRowCol = idx;
    b_A.set_size(obj->QR.size(0), obj->QR.size(1));
    ix0 = obj->QR.size(0) * obj->QR.size(1);
    for (b_k = 0; b_k < ix0; b_k++) {
      b_A[b_k] = obj->QR[b_k];
    }

    ix0 = obj->QR.size(0);
    minmana = obj->QR.size(1);
    if (ix0 < minmana) {
      minmana = ix0;
    }

    tau.set_size(minmana);
    for (b_k = 0; b_k < minmana; b_k++) {
      tau[b_k] = 0.0;
    }

    if (idx >= 1) {
      qrf(b_A, mrows, ncols, idx, tau);
    }

    obj->QR.set_size(b_A.size(0), b_A.size(1));
    ix0 = b_A.size(0) * b_A.size(1);
    for (idx = 0; idx < ix0; idx++) {
      obj->QR[idx] = b_A[idx];
    }

    obj->tau.set_size(tau.size(0));
    ix0 = tau.size(0);
    for (idx = 0; idx < ix0; idx++) {
      obj->tau[idx] = tau[idx];
    }
  }
}

//
// File trailer for factorQR.cpp
//
// [EOF]
//

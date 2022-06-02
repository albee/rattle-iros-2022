//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factor.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "factor.h"
#include "computeFval.h"
#include "fullColLDL2_.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : f_struct_T *obj
//                const coder::array<double, 2U> *A
//                int ndims
//                int ldA
// Return Type  : void
//
void factor(f_struct_T *obj, const coder::array<double, 2U> &A, int ndims, int
            ldA)
{
  double SCALED_REG_PRIMAL;
  int LDimSizeP1_tmp;
  int idx;
  int A_maxDiag_idx;
  int ix;
  double smax;
  int k;
  double s;
  SCALED_REG_PRIMAL = 1.4901161193847656E-6 * static_cast<double>(ndims);
  LDimSizeP1_tmp = obj->ldm + 1;
  obj->ndims = ndims;
  if (A.size(1) != 0) {
    for (idx = 0; idx < ndims; idx++) {
      A_maxDiag_idx = ldA * idx;
      ix = obj->ldm * idx;
      for (k = 0; k < ndims; k++) {
        int b_k;
        b_k = k + 1;
        obj->FMat[(ix + b_k) - 1] = A[(A_maxDiag_idx + b_k) - 1];
      }
    }
  }

  if (ndims < 1) {
    A_maxDiag_idx = 0;
  } else {
    A_maxDiag_idx = 1;
    if (ndims > 1) {
      ix = 0;
      smax = std::abs(obj->FMat[0]);
      for (k = 2; k <= ndims; k++) {
        ix += LDimSizeP1_tmp;
        s = std::abs(obj->FMat[ix]);
        if (s > smax) {
          A_maxDiag_idx = k;
          smax = s;
        }
      }
    }
  }

  smax = std::abs(obj->FMat[A_maxDiag_idx * (obj->ldm + 1) - 1]) *
    2.2204460492503131E-16;
  s = std::abs(SCALED_REG_PRIMAL);
  if (smax > s) {
    s = smax;
  }

  obj->regTol_ = s;
  if (ndims > 128) {
    boolean_T exitg1;
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k < ndims)) {
      A_maxDiag_idx = LDimSizeP1_tmp * k + 1;
      ix = ndims - k;
      if (k + 48 <= ndims) {
        partialColLDL3_(obj, A_maxDiag_idx, ix, SCALED_REG_PRIMAL);
        k += 48;
      } else {
        fullColLDL2_(obj, A_maxDiag_idx, ix, SCALED_REG_PRIMAL);
        exitg1 = true;
      }
    }
  } else {
    fullColLDL2_(obj, 1, ndims, SCALED_REG_PRIMAL);
  }

  if (obj->ConvexCheck) {
    idx = 0;
    int exitg2;
    do {
      exitg2 = 0;
      if (idx <= ndims - 1) {
        if (obj->FMat[idx + obj->FMat.size(0) * idx] <= 0.0) {
          obj->info = -idx - 1;
          exitg2 = 1;
        } else {
          idx++;
        }
      } else {
        obj->ConvexCheck = false;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }
}

//
// File trailer for factor.cpp
//
// [EOF]
//

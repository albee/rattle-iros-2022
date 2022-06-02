//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: partialColLDL3_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "partialColLDL3_.h"
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : f_struct_T *obj
//                int LD_offset
//                int NColsRemain
//                double REG_PRIMAL
// Return Type  : void
//
void partialColLDL3_(f_struct_T *obj, int LD_offset, int NColsRemain, double
                     REG_PRIMAL)
{
  int LDimSizeP1;
  int k;
  int i;
  int subRows;
  int j;
  int lastColC;
  int LD_diagOffset;
  int offsetColK;
  int b_k;
  int i1;
  int lda;
  int ix;
  int iy;
  int ia;
  LDimSizeP1 = obj->ldm + 1;
  for (k = 0; k < 48; k++) {
    double a;
    subRows = (NColsRemain - k) - 1;
    lastColC = LDimSizeP1 * k;
    LD_diagOffset = (LD_offset + lastColC) - 1;
    for (b_k = 0; b_k <= subRows; b_k++) {
      obj->workspace_[lastColC + b_k] = obj->FMat[LD_diagOffset + b_k];
    }

    offsetColK = obj->ldm * k;
    for (b_k = 0; b_k < NColsRemain; b_k++) {
      obj->workspace2_[b_k] = obj->workspace_[offsetColK + b_k];
    }

    lda = obj->ldm;
    if ((NColsRemain != 0) && (k != 0)) {
      ix = LD_offset + k;
      i = obj->ldm * (k - 1) + 1;
      for (b_k = 1; lda < 0 ? b_k >= i : b_k <= i; b_k += lda) {
        iy = 0;
        i1 = (b_k + NColsRemain) - 1;
        for (ia = b_k; ia <= i1; ia++) {
          obj->workspace2_[iy] = obj->workspace2_[iy] + obj->workspace_[ia - 1] *
            -obj->FMat[ix - 1];
          iy++;
        }

        ix += obj->ldm;
      }
    }

    for (b_k = 0; b_k < NColsRemain; b_k++) {
      obj->workspace_[offsetColK + b_k] = obj->workspace2_[b_k];
    }

    for (b_k = 0; b_k <= subRows; b_k++) {
      obj->FMat[LD_diagOffset + b_k] = obj->workspace_[lastColC + b_k];
    }

    if (std::abs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] = obj->FMat[LD_diagOffset] + REG_PRIMAL;
    }

    a = 1.0 / obj->FMat[LD_diagOffset];
    lda = LD_diagOffset + 2;
    i = (LD_diagOffset + subRows) + 1;
    for (b_k = lda; b_k <= i; b_k++) {
      obj->FMat[b_k - 1] = a * obj->FMat[b_k - 1];
    }
  }

  i = NColsRemain - 1;
  for (j = 48; j <= i; j += 48) {
    int subBlockSize;
    int ia0;
    int m;
    int i2;
    int i3;
    offsetColK = NColsRemain - j;
    if (48 < offsetColK) {
      subBlockSize = 48;
    } else {
      subBlockSize = offsetColK;
    }

    ia0 = j + subBlockSize;
    i1 = ia0 - 1;
    for (k = j; k <= i1; k++) {
      m = ia0 - k;
      subRows = (LD_offset + LDimSizeP1 * k) - 1;
      lda = LD_offset + k;
      for (b_k = 0; b_k < 48; b_k++) {
        obj->workspace2_[b_k] = obj->FMat[(lda + b_k * obj->ldm) - 1];
      }

      lastColC = k + 1;
      lda = obj->ldm;
      if (m != 0) {
        ix = 0;
        i2 = (k + obj->ldm * 47) + 1;
        for (b_k = lastColC; lda < 0 ? b_k >= i2 : b_k <= i2; b_k += lda) {
          iy = subRows;
          i3 = (b_k + m) - 1;
          for (ia = b_k; ia <= i3; ia++) {
            obj->FMat[iy] = obj->FMat[iy] + obj->workspace_[ia - 1] *
              -obj->workspace2_[ix];
            iy++;
          }

          ix++;
        }
      }
    }

    if (ia0 < NColsRemain) {
      int ic0;
      m = offsetColK - subBlockSize;
      ic0 = ((LD_offset + subBlockSize) + LDimSizeP1 * j) - 1;
      for (offsetColK = 0; offsetColK < 48; offsetColK++) {
        lastColC = offsetColK * obj->ldm;
        lda = (LD_offset + j) + lastColC;
        for (k = 0; k < subBlockSize; k++) {
          b_k = k + 1;
          obj->workspace2_[(lastColC + b_k) - 1] = obj->FMat[(lda + b_k) - 2];
        }
      }

      LD_diagOffset = obj->ldm;
      lda = obj->ldm;
      if ((m != 0) && (subBlockSize != 0)) {
        lastColC = ic0 + obj->ldm * (subBlockSize - 1);
        offsetColK = 0;
        for (b_k = ic0; lda < 0 ? b_k >= lastColC : b_k <= lastColC; b_k += lda)
        {
          subRows = ia0 - 1;
          offsetColK++;
          i1 = offsetColK + LD_diagOffset * 47;
          for (ix = offsetColK; LD_diagOffset < 0 ? ix >= i1 : ix <= i1; ix +=
               LD_diagOffset) {
            ia = subRows;
            i2 = b_k + 1;
            i3 = b_k + m;
            for (iy = i2; iy <= i3; iy++) {
              ia++;
              obj->FMat[iy - 1] = obj->FMat[iy - 1] + -obj->workspace2_[ix - 1] *
                obj->workspace_[ia];
            }

            subRows += obj->ldm;
          }
        }
      }
    }
  }
}

//
// File trailer for partialColLDL3_.cpp
//
// [EOF]
//

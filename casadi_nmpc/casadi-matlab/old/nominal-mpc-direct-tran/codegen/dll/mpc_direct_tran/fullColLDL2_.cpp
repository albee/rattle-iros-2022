//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: fullColLDL2_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "fullColLDL2_.h"
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
void fullColLDL2_(f_struct_T *obj, int LD_offset, int NColsRemain, double
                  REG_PRIMAL)
{
  int LDimSizeP1;
  int jA;
  LDimSizeP1 = obj->ldm;
  for (int k = 0; k < NColsRemain; k++) {
    int LD_diagOffset;
    double alpha1;
    int subMatrixDim;
    int offset1;
    int i;
    LD_diagOffset = (LD_offset + (LDimSizeP1 + 1) * k) - 1;
    if (std::abs(obj->FMat[LD_diagOffset]) <= obj->regTol_) {
      obj->FMat[LD_diagOffset] = obj->FMat[LD_diagOffset] + REG_PRIMAL;
    }

    alpha1 = -1.0 / obj->FMat[LD_diagOffset];
    subMatrixDim = NColsRemain - k;
    offset1 = LD_diagOffset + 2;
    for (jA = 0; jA <= subMatrixDim - 2; jA++) {
      obj->workspace_[jA] = obj->FMat[(LD_diagOffset + jA) + 1];
    }

    if (!(alpha1 == 0.0)) {
      int jy;
      jA = LD_diagOffset + LDimSizeP1;
      jy = 0;
      for (int j = 0; j <= subMatrixDim - 2; j++) {
        if (obj->workspace_[jy] != 0.0) {
          double temp;
          int ix;
          int i1;
          temp = obj->workspace_[jy] * alpha1;
          ix = 0;
          i = jA + 2;
          i1 = subMatrixDim + jA;
          for (int ijA = i; ijA <= i1; ijA++) {
            obj->FMat[ijA - 1] = obj->FMat[ijA - 1] + obj->workspace_[ix] * temp;
            ix++;
          }
        }

        jy++;
        jA += obj->ldm;
      }
    }

    alpha1 = 1.0 / obj->FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i; jA++) {
      obj->FMat[jA - 1] = alpha1 * obj->FMat[jA - 1];
    }
  }

  jA = (LD_offset + (obj->ldm + 1) * (NColsRemain - 1)) - 1;
  if (std::abs(obj->FMat[jA]) <= obj->regTol_) {
    obj->FMat[jA] = obj->FMat[jA] + REG_PRIMAL;
  }
}

//
// File trailer for fullColLDL2_.cpp
//
// [EOF]
//

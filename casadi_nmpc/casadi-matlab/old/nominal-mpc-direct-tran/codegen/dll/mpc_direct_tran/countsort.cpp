//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: countsort.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "countsort.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : coder::array<int, 1U> *x
//                int xLen
//                coder::array<int, 1U> *workspace
//                int xMin
//                int xMax
// Return Type  : void
//
void countsort(coder::array<int, 1U> &x, int xLen, coder::array<int, 1U>
               &workspace, int xMin, int xMax)
{
  if ((xLen > 1) && (xMax > xMin)) {
    int idxStart;
    int idx;
    int maxOffset;
    int idxEnd;
    idxStart = xMax - xMin;
    for (idx = 0; idx <= idxStart; idx++) {
      workspace[idx] = 0;
    }

    maxOffset = idxStart - 1;
    for (idx = 0; idx < xLen; idx++) {
      idxStart = x[idx] - xMin;
      workspace[idxStart] = workspace[idxStart] + 1;
    }

    for (idx = 2; idx <= maxOffset + 2; idx++) {
      workspace[idx - 1] = workspace[idx - 1] + workspace[idx - 2];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    for (idx = 0; idx <= maxOffset; idx++) {
      for (int idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        x[idxFill - 1] = idx + xMin;
      }

      idxStart = workspace[idx] + 1;
      idxEnd = workspace[idx + 1];
    }

    for (idx = idxStart; idx <= idxEnd; idx++) {
      x[idx - 1] = xMax;
    }
  }
}

//
// File trailer for countsort.cpp
//
// [EOF]
//

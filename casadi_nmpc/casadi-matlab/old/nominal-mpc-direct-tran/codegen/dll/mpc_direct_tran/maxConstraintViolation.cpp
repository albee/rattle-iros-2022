//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: maxConstraintViolation.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "maxConstraintViolation.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <cmath>

// Function Definitions

//
// Arguments    : g_struct_T *obj
//                const coder::array<double, 1U> *x
// Return Type  : double
//
double maxConstraintViolation(g_struct_T *obj, const coder::array<double, 1U> &x)
{
  double v;
  int mLB;
  int idx;
  double u1;
  mLB = obj->sizes[3];
  switch (obj->probType) {
   case 2:
    {
      int mIneq;
      int mEq;
      int offsetEq2;
      v = 0.0;
      mIneq = obj->sizes[2] - 1;
      mEq = obj->sizes[1] - 1;
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        for (offsetEq2 = 0; offsetEq2 <= mIneq; offsetEq2++) {
          obj->maxConstrWorkspace[offsetEq2] = obj->bineq[offsetEq2];
        }

        c_xgemv(obj->nVarOrig, obj->sizes[2], obj->Aineq, obj->ldA, x,
                obj->maxConstrWorkspace);
        for (idx = 0; idx <= mIneq; idx++) {
          obj->maxConstrWorkspace[idx] = obj->maxConstrWorkspace[idx] - x
            [obj->nVarOrig + idx];
          if ((!(v > obj->maxConstrWorkspace[idx])) && (!rtIsNaN
               (obj->maxConstrWorkspace[idx]))) {
            v = obj->maxConstrWorkspace[idx];
          }
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        for (offsetEq2 = 0; offsetEq2 <= mEq; offsetEq2++) {
          obj->maxConstrWorkspace[offsetEq2] = obj->beq[offsetEq2];
        }

        c_xgemv(obj->nVarOrig, obj->sizes[1], obj->Aeq, obj->ldA, x,
                obj->maxConstrWorkspace);
        mIneq = obj->nVarOrig + obj->sizes[2];
        offsetEq2 = mIneq + obj->sizes[1];
        for (idx = 0; idx <= mEq; idx++) {
          obj->maxConstrWorkspace[idx] = (obj->maxConstrWorkspace[idx] - x[mIneq
            + idx]) + x[offsetEq2 + idx];
          u1 = std::abs(obj->maxConstrWorkspace[idx]);
          if ((!(v > u1)) && (!rtIsNaN(u1))) {
            v = u1;
          }
        }
      }
    }
    break;

   default:
    {
      int mIneq;
      int mEq;
      int offsetEq2;
      v = 0.0;
      mIneq = obj->sizes[2] - 1;
      mEq = obj->sizes[1] - 1;
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        for (offsetEq2 = 0; offsetEq2 <= mIneq; offsetEq2++) {
          obj->maxConstrWorkspace[offsetEq2] = obj->bineq[offsetEq2];
        }

        c_xgemv(obj->nVar, obj->sizes[2], obj->Aineq, obj->ldA, x,
                obj->maxConstrWorkspace);
        for (idx = 0; idx <= mIneq; idx++) {
          if ((!(v > obj->maxConstrWorkspace[idx])) && (!rtIsNaN
               (obj->maxConstrWorkspace[idx]))) {
            v = obj->maxConstrWorkspace[idx];
          }
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        for (offsetEq2 = 0; offsetEq2 <= mEq; offsetEq2++) {
          obj->maxConstrWorkspace[offsetEq2] = obj->beq[offsetEq2];
        }

        c_xgemv(obj->nVar, obj->sizes[1], obj->Aeq, obj->ldA, x,
                obj->maxConstrWorkspace);
        for (idx = 0; idx <= mEq; idx++) {
          u1 = std::abs(obj->maxConstrWorkspace[idx]);
          if ((!(v > u1)) && (!rtIsNaN(u1))) {
            v = u1;
          }
        }
      }
    }
    break;
  }

  if (obj->sizes[3] > 0) {
    for (idx = 0; idx < mLB; idx++) {
      u1 = -x[obj->indexLB[idx] - 1] - obj->lb[obj->indexLB[idx] - 1];
      if ((!(v > u1)) && (!rtIsNaN(u1))) {
        v = u1;
      }
    }
  }

  return v;
}

//
// File trailer for maxConstraintViolation.cpp
//
// [EOF]
//

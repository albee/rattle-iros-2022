//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: factoryConstruct.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "factoryConstruct.h"
#include "RemoveDependentEq_.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : int mIneq
//                int mLinIneq
//                const coder::array<double, 2U> *Aineq
//                const coder::array<double, 1U> *bineq
//                int mEq
//                int mLinEq
//                const coder::array<double, 2U> *Aeq
//                const coder::array<double, 1U> *beq
//                int nVar
//                int nVarMax
//                int mConstrMax
//                g_struct_T *obj
// Return Type  : void
//
void factoryConstruct(int mIneq, int mLinIneq, const coder::array<double, 2U>
                      &Aineq, const coder::array<double, 1U> &bineq, int mEq,
                      int mLinEq, const coder::array<double, 2U> &Aeq, const
                      coder::array<double, 1U> &beq, int nVar, int nVarMax, int
                      mConstrMax, g_struct_T *obj)
{
  int obj_tmp;
  coder::array<double, 1U> r;
  int i;
  int b_obj_tmp[5];
  int c_obj_tmp[6];
  int k;
  coder::array<double, 2U> y;
  obj_tmp = mIneq + mEq;
  obj->mConstr = obj_tmp;
  obj->mConstrOrig = obj_tmp;
  obj->mConstrMax = mConstrMax;
  obj->nVar = nVar;
  obj->nVarOrig = nVar;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  if (mIneq > 0) {
    obj->Aineq.set_size(nVarMax, mIneq);
    r.set_size(mIneq);
    for (i = 0; i < mIneq; i++) {
      r[i] = 1.7976931348623157E+308;
    }

    obj->bineq.set_size(r.size(0), 1);
  } else {
    obj->Aineq.set_size(0, 0);
    obj->bineq.set_size(0, 0);
  }

  if (mEq > 0) {
    obj->Aeq.set_size(nVarMax, mEq);
    r.set_size(mEq);
    for (i = 0; i < mEq; i++) {
      r[i] = 1.7976931348623157E+308;
    }

    obj->beq.set_size(r.size(0), 1);
  } else {
    obj->Aeq.set_size(0, 0);
    obj->beq.set_size(0, 0);
  }

  obj->lb.set_size(nVarMax);
  obj->ub.set_size(nVarMax);
  obj->indexLB.set_size(nVarMax);
  obj->indexUB.set_size(nVarMax);
  obj->indexFixed.set_size(nVarMax);
  obj->mEqRemoved = 0;
  obj->indexEqRemoved.set_size(mEq);
  obj->ATwset.set_size(nVarMax, mConstrMax);
  obj->bwset.set_size(mConstrMax);
  obj->nActiveConstr = 0;
  obj->maxConstrWorkspace.set_size(mConstrMax);
  b_obj_tmp[0] = 0;
  b_obj_tmp[1] = mEq;
  b_obj_tmp[2] = mIneq;
  b_obj_tmp[3] = 0;
  b_obj_tmp[4] = 0;
  obj->sizesPhaseOne[0] = 0;
  obj->sizesPhaseOne[1] = mEq;
  obj->sizesPhaseOne[2] = mIneq;
  obj->sizesPhaseOne[3] = 1;
  obj->sizesPhaseOne[4] = 0;
  obj->sizesRegularized[0] = 0;
  obj->sizesRegularized[1] = mEq;
  obj->sizesRegularized[2] = mIneq;
  i = mIneq + (mEq << 1);
  obj->sizesRegularized[3] = i;
  obj->sizesRegularized[4] = 0;
  obj->sizesRegPhaseOne[0] = 0;
  obj->sizesRegPhaseOne[1] = mEq;
  obj->sizesRegPhaseOne[2] = mIneq;
  obj_tmp = i + 1;
  obj->sizesRegPhaseOne[3] = obj_tmp;
  obj->sizesRegPhaseOne[4] = 0;
  c_obj_tmp[0] = 1;
  c_obj_tmp[1] = 0;
  c_obj_tmp[2] = mEq;
  c_obj_tmp[3] = mIneq;
  c_obj_tmp[4] = 0;
  c_obj_tmp[5] = 0;
  for (k = 0; k < 5; k++) {
    obj->sizes[k] = b_obj_tmp[k];
    obj->sizesNormal[k] = b_obj_tmp[k];
    c_obj_tmp[k + 1] += c_obj_tmp[k];
  }

  for (k = 0; k < 6; k++) {
    obj->isActiveIdx[k] = c_obj_tmp[k];
    obj->isActiveIdxNormal[k] = c_obj_tmp[k];
  }

  c_obj_tmp[0] = 1;
  c_obj_tmp[1] = 0;
  c_obj_tmp[2] = mEq;
  c_obj_tmp[3] = mIneq;
  c_obj_tmp[4] = 1;
  c_obj_tmp[5] = 0;
  for (k = 0; k < 5; k++) {
    c_obj_tmp[k + 1] += c_obj_tmp[k];
  }

  for (k = 0; k < 6; k++) {
    obj->isActiveIdxPhaseOne[k] = c_obj_tmp[k];
  }

  c_obj_tmp[0] = 1;
  c_obj_tmp[1] = 0;
  c_obj_tmp[2] = mEq;
  c_obj_tmp[3] = mIneq;
  c_obj_tmp[4] = i;
  c_obj_tmp[5] = 0;
  for (k = 0; k < 5; k++) {
    c_obj_tmp[k + 1] += c_obj_tmp[k];
  }

  for (k = 0; k < 6; k++) {
    obj->isActiveIdxRegularized[k] = c_obj_tmp[k];
  }

  c_obj_tmp[0] = 1;
  c_obj_tmp[1] = 0;
  c_obj_tmp[2] = mEq;
  c_obj_tmp[3] = mIneq;
  c_obj_tmp[4] = obj_tmp;
  c_obj_tmp[5] = 0;
  for (k = 0; k < 5; k++) {
    c_obj_tmp[k + 1] += c_obj_tmp[k];
  }

  for (k = 0; k < 6; k++) {
    obj->isActiveIdxRegPhaseOne[k] = c_obj_tmp[k];
  }

  obj->isActiveConstr.set_size(mConstrMax);
  obj->Wid.set_size(mConstrMax);
  obj->Wlocalidx.set_size(mConstrMax);
  for (k = 0; k < 5; k++) {
    obj->nWConstr[k] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
  if (mIneq > 0) {
    for (obj_tmp = 0; obj_tmp < mLinIneq; obj_tmp++) {
      for (k = 0; k < nVar; k++) {
        obj->Aineq[k + obj->Aineq.size(0) * obj_tmp] = Aineq[obj_tmp +
          Aineq.size(0) * k];
      }
    }

    if (bineq.size(0) != 0) {
      y.set_size(obj->bineq.size(0), obj->bineq.size(1));
      obj_tmp = obj->bineq.size(0) * obj->bineq.size(1);
      for (i = 0; i < obj_tmp; i++) {
        y[i] = obj->bineq[i];
      }

      for (k = 0; k < mLinIneq; k++) {
        y[k] = bineq[k];
      }

      obj->bineq.set_size(y.size(0), y.size(1));
      obj_tmp = y.size(0) * y.size(1);
      for (i = 0; i < obj_tmp; i++) {
        obj->bineq[i] = y[i];
      }
    }
  }

  if (mEq > 0) {
    for (obj_tmp = 0; obj_tmp < mLinEq; obj_tmp++) {
      for (k = 0; k < nVar; k++) {
        obj->Aeq[k + obj->Aeq.size(0) * obj_tmp] = Aeq[obj_tmp + Aeq.size(0) * k];
      }
    }

    if (beq.size(0) != 0) {
      y.set_size(obj->beq.size(0), obj->beq.size(1));
      obj_tmp = obj->beq.size(0) * obj->beq.size(1);
      for (i = 0; i < obj_tmp; i++) {
        y[i] = obj->beq[i];
      }

      for (k = 0; k < mLinEq; k++) {
        y[k] = beq[k];
      }

      obj->beq.set_size(y.size(0), y.size(1));
      obj_tmp = y.size(0) * y.size(1);
      for (i = 0; i < obj_tmp; i++) {
        obj->beq[i] = y[i];
      }
    }
  }
}

//
// File trailer for factoryConstruct.cpp
//
// [EOF]
//

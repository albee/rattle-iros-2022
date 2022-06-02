//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factoryConstruct.cpp
//
//  Code generation for function 'factoryConstruct'
//


// Include files
#include "factoryConstruct.h"
#include "RemoveDependentEq_.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo qb_emlrtRSI = { 1,  // lineNo
  "factoryConstruct",                  // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/factoryConstruct.p"// pathName 
};

static emlrtBCInfo p_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "factoryConstruct",                  // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/factoryConstruct.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo kb_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "factoryConstruct",                  // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/factoryConstruct.p"// pName 
};

// Function Definitions
void factoryConstruct(const emlrtStack *sp, int32_T mIneq, int32_T mLinIneq,
                      const coder::array<real_T, 2U> &Aineq, const coder::array<
                      real_T, 1U> &bineq, int32_T mEq, int32_T mLinEq, const
                      coder::array<real_T, 2U> &Aeq, const coder::array<real_T,
                      1U> &beq, int32_T nVar, int32_T nVarMax, int32_T
                      mConstrMax, g_struct_T *obj)
{
  int32_T obj_tmp;
  coder::array<real_T, 1U> r;
  int32_T idx_col;
  int32_T b_obj_tmp[5];
  int32_T c_obj_tmp[6];
  int32_T k;
  boolean_T overflow;
  boolean_T b_overflow;
  boolean_T c_overflow;
  coder::array<real_T, 2U> r1;
  int32_T idx_row;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  obj_tmp = mIneq + mEq;
  obj->mConstr = obj_tmp;
  obj->mConstrOrig = obj_tmp;
  obj->mConstrMax = mConstrMax;
  obj->nVar = nVar;
  obj->nVarOrig = nVar;
  obj->nVarMax = nVarMax;
  obj->ldA = nVarMax;
  if (mIneq > 0) {
    obj->Aineq.set_size((&kb_emlrtRTEI), sp, nVarMax, mIneq);
    r.set_size((&kb_emlrtRTEI), sp, mIneq);
    for (idx_col = 0; idx_col < mIneq; idx_col++) {
      r[idx_col] = 1.7976931348623157E+308;
    }

    obj->bineq.set_size((&kb_emlrtRTEI), sp, r.size(0), 1);
  } else {
    obj->Aineq.set_size((&kb_emlrtRTEI), sp, 0, 0);
    obj->bineq.set_size((&kb_emlrtRTEI), sp, 0, 0);
  }

  if (mEq > 0) {
    obj->Aeq.set_size((&kb_emlrtRTEI), sp, nVarMax, mEq);
    r.set_size((&kb_emlrtRTEI), sp, mEq);
    for (idx_col = 0; idx_col < mEq; idx_col++) {
      r[idx_col] = 1.7976931348623157E+308;
    }

    obj->beq.set_size((&kb_emlrtRTEI), sp, r.size(0), 1);
  } else {
    obj->Aeq.set_size((&kb_emlrtRTEI), sp, 0, 0);
    obj->beq.set_size((&kb_emlrtRTEI), sp, 0, 0);
  }

  obj->lb.set_size((&kb_emlrtRTEI), sp, nVarMax);
  obj->ub.set_size((&kb_emlrtRTEI), sp, nVarMax);
  obj->indexLB.set_size((&kb_emlrtRTEI), sp, nVarMax);
  obj->indexUB.set_size((&kb_emlrtRTEI), sp, nVarMax);
  obj->indexFixed.set_size((&kb_emlrtRTEI), sp, nVarMax);
  obj->mEqRemoved = 0;
  obj->indexEqRemoved.set_size((&kb_emlrtRTEI), sp, mEq);
  obj->ATwset.set_size((&kb_emlrtRTEI), sp, nVarMax, mConstrMax);
  obj->bwset.set_size((&kb_emlrtRTEI), sp, mConstrMax);
  obj->nActiveConstr = 0;
  obj->maxConstrWorkspace.set_size((&kb_emlrtRTEI), sp, mConstrMax);
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
  idx_col = mIneq + (mEq << 1);
  obj->sizesRegularized[3] = idx_col;
  obj->sizesRegularized[4] = 0;
  obj->sizesRegPhaseOne[0] = 0;
  obj->sizesRegPhaseOne[1] = mEq;
  obj->sizesRegPhaseOne[2] = mIneq;
  obj_tmp = idx_col + 1;
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
  c_obj_tmp[4] = idx_col;
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

  obj->isActiveConstr.set_size((&kb_emlrtRTEI), sp, mConstrMax);
  obj->Wid.set_size((&kb_emlrtRTEI), sp, mConstrMax);
  obj->Wlocalidx.set_size((&kb_emlrtRTEI), sp, mConstrMax);
  for (k = 0; k < 5; k++) {
    obj->nWConstr[k] = 0;
  }

  obj->probType = 3;
  obj->SLACK0 = 1.0E-5;
  if (mIneq > 0) {
    st.site = &qb_emlrtRSI;
    overflow = ((1 <= mLinIneq) && (mLinIneq > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    if (0 <= mLinIneq - 1) {
      b_overflow = ((1 <= nVar) && (nVar > 2147483646));
    }

    for (idx_col = 0; idx_col < mLinIneq; idx_col++) {
      obj_tmp = idx_col + 1;
      st.site = &qb_emlrtRSI;
      if (b_overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idx_row = 0; idx_row < nVar; idx_row++) {
        k = idx_row + 1;
        if ((k < 1) || (k > obj->Aineq.size(0))) {
          emlrtDynamicBoundsCheckR2012b(k, 1, obj->Aineq.size(0), &p_emlrtBCI,
            sp);
        }

        if ((obj_tmp < 1) || (obj_tmp > obj->Aineq.size(1))) {
          emlrtDynamicBoundsCheckR2012b(obj_tmp, 1, obj->Aineq.size(1),
            &p_emlrtBCI, sp);
        }

        if (obj_tmp > Aineq.size(0)) {
          emlrtDynamicBoundsCheckR2012b(obj_tmp, 1, Aineq.size(0), &p_emlrtBCI,
            sp);
        }

        if (k > Aineq.size(1)) {
          emlrtDynamicBoundsCheckR2012b(k, 1, Aineq.size(1), &p_emlrtBCI, sp);
        }

        obj->Aineq[(obj_tmp + obj->Aineq.size(1) * (k - 1)) - 1] = Aineq[(k +
          Aineq.size(1) * (obj_tmp - 1)) - 1];
      }
    }

    if (bineq.size(0) != 0) {
      r1.set_size((&kb_emlrtRTEI), sp, obj->bineq.size(0), obj->bineq.size(1));
      obj_tmp = obj->bineq.size(1) * obj->bineq.size(0);
      for (idx_col = 0; idx_col < obj_tmp; idx_col++) {
        r1[idx_col] = obj->bineq[idx_col];
      }

      st.site = &qb_emlrtRSI;
      xcopy(&st, mLinIneq, bineq, r1);
      obj->bineq.set_size((&kb_emlrtRTEI), sp, r1.size(0), r1.size(1));
      obj_tmp = r1.size(1) * r1.size(0);
      for (idx_col = 0; idx_col < obj_tmp; idx_col++) {
        obj->bineq[idx_col] = r1[idx_col];
      }
    }
  }

  if (mEq > 0) {
    st.site = &qb_emlrtRSI;
    overflow = ((1 <= mLinEq) && (mLinEq > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    if (0 <= mLinEq - 1) {
      c_overflow = ((1 <= nVar) && (nVar > 2147483646));
    }

    for (idx_col = 0; idx_col < mLinEq; idx_col++) {
      obj_tmp = idx_col + 1;
      st.site = &qb_emlrtRSI;
      if (c_overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idx_row = 0; idx_row < nVar; idx_row++) {
        k = idx_row + 1;
        if ((k < 1) || (k > obj->Aeq.size(0))) {
          emlrtDynamicBoundsCheckR2012b(k, 1, obj->Aeq.size(0), &p_emlrtBCI, sp);
        }

        if ((obj_tmp < 1) || (obj_tmp > obj->Aeq.size(1))) {
          emlrtDynamicBoundsCheckR2012b(obj_tmp, 1, obj->Aeq.size(1),
            &p_emlrtBCI, sp);
        }

        if (obj_tmp > Aeq.size(0)) {
          emlrtDynamicBoundsCheckR2012b(obj_tmp, 1, Aeq.size(0), &p_emlrtBCI, sp);
        }

        if (k > Aeq.size(1)) {
          emlrtDynamicBoundsCheckR2012b(k, 1, Aeq.size(1), &p_emlrtBCI, sp);
        }

        obj->Aeq[(obj_tmp + obj->Aeq.size(1) * (k - 1)) - 1] = Aeq[(k + Aeq.size
          (1) * (obj_tmp - 1)) - 1];
      }
    }

    if (beq.size(0) != 0) {
      r1.set_size((&kb_emlrtRTEI), sp, obj->beq.size(0), obj->beq.size(1));
      obj_tmp = obj->beq.size(1) * obj->beq.size(0);
      for (idx_col = 0; idx_col < obj_tmp; idx_col++) {
        r1[idx_col] = obj->beq[idx_col];
      }

      st.site = &qb_emlrtRSI;
      xcopy(&st, mLinEq, beq, r1);
      obj->beq.set_size((&kb_emlrtRTEI), sp, r1.size(0), r1.size(1));
      obj_tmp = r1.size(1) * r1.size(0);
      for (idx_col = 0; idx_col < obj_tmp; idx_col++) {
        obj->beq[idx_col] = r1[idx_col];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (factoryConstruct.cpp)

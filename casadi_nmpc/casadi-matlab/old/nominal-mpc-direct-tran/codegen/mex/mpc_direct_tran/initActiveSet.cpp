//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  initActiveSet.cpp
//
//  Code generation for function 'initActiveSet'
//


// Include files
#include "initActiveSet.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo rb_emlrtRSI = { 1,  // lineNo
  "initActiveSet",                     // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/initActiveSet.p"// pathName 
};

static emlrtBCInfo r_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "initActiveSet",                     // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/initActiveSet.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void initActiveSet(const emlrtStack *sp, g_struct_T *obj)
{
  int32_T i;
  int32_T idxFillStart;
  int32_T b;
  boolean_T overflow;
  int32_T idx;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &rb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  obj->nVar = obj->nVarOrig;
  obj->mConstr = obj->mConstrOrig;
  for (i = 0; i < 5; i++) {
    obj->sizes[i] = obj->sizesNormal[i];
  }

  for (i = 0; i < 6; i++) {
    obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
  }

  obj->probType = 3;
  idxFillStart = obj->isActiveIdx[2];
  b = obj->mConstrMax;
  st.site = &rb_emlrtRSI;
  overflow = ((obj->isActiveIdx[2] <= obj->mConstrMax) && (obj->mConstrMax >
    2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = idxFillStart; idx <= b; idx++) {
    i = obj->isActiveConstr.size(0);
    if ((idx < 1) || (idx > i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &r_emlrtBCI, sp);
    }

    obj->isActiveConstr[idx - 1] = false;
  }

  obj->nWConstr[0] = 0;
  obj->nWConstr[1] = obj->sizes[1];
  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[1];
  st.site = &rb_emlrtRSI;
  idxFillStart = obj->sizes[1];
  st.site = &rb_emlrtRSI;
  overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (int32_T idx_local = 0; idx_local < idxFillStart; idx_local++) {
    b = idx_local + 1;
    i = obj->Wid.size(0);
    if ((b < 1) || (b > i)) {
      emlrtDynamicBoundsCheckR2012b(b, 1, i, &r_emlrtBCI, sp);
    }

    obj->Wid[b - 1] = 2;
    i = obj->Wlocalidx.size(0);
    if (b > i) {
      emlrtDynamicBoundsCheckR2012b(b, 1, i, &r_emlrtBCI, sp);
    }

    obj->Wlocalidx[b - 1] = b;
    i = obj->isActiveConstr.size(0);
    if (b > i) {
      emlrtDynamicBoundsCheckR2012b(b, 1, i, &r_emlrtBCI, sp);
    }

    obj->isActiveConstr[b - 1] = true;
    idx = obj->ldA * (b - 1);
    i = obj->nVar - 1;
    for (int32_T b_i = 0; b_i <= i; b_i++) {
      int32_T i1;
      int32_T i2;
      int32_T i3;
      i1 = obj->ATwset.size(0) * obj->ATwset.size(1);
      i2 = idx + b_i;
      i3 = i2 + 1;
      if ((i3 < 1) || (i3 > i1)) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, i1, &r_emlrtBCI, sp);
      }

      i1 = obj->Aeq.size(0) * obj->Aeq.size(1);
      i3 = i2 + 1;
      if ((i3 < 1) || (i3 > i1)) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, i1, &r_emlrtBCI, sp);
      }

      i1 = obj->ATwset.size(0);
      i3 = obj->Aeq.size(0);
      obj->ATwset[i2 % i1 * obj->ATwset.size(1) + i2 / i1] = obj->Aeq[i2 % i3 *
        obj->Aeq.size(1) + i2 / i3];
    }

    i = obj->beq.size(0) * obj->beq.size(1);
    if (b > i) {
      emlrtDynamicBoundsCheckR2012b(b, 1, i, &r_emlrtBCI, sp);
    }

    i = obj->bwset.size(0);
    if (b > i) {
      emlrtDynamicBoundsCheckR2012b(b, 1, i, &r_emlrtBCI, sp);
    }

    obj->bwset[b - 1] = obj->beq[b - 1];
  }
}

// End of code generation (initActiveSet.cpp)

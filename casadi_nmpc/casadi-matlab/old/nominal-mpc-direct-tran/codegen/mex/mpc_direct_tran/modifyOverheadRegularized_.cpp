//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  modifyOverheadRegularized_.cpp
//
//  Code generation for function 'modifyOverheadRegularized_'
//


// Include files
#include "modifyOverheadRegularized_.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo ub_emlrtRSI = { 1,  // lineNo
  "modifyOverheadRegularized_",        // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/modifyOverheadRegularized_.p"// pathName 
};

static emlrtBCInfo t_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "modifyOverheadRegularized_",        // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/modifyOverheadRegularized_.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void modifyOverheadRegularized_(const emlrtStack *sp, g_struct_T *obj)
{
  int32_T mIneq;
  int32_T mEq;
  int32_T offsetIneq_tmp_tmp;
  int32_T offsetEq1_tmp_tmp;
  int32_T offsetEq2;
  boolean_T overflow;
  int32_T idx_col;
  int32_T idxGlobalColStart;
  int32_T b_idx_col;
  int32_T b;
  int32_T a;
  int32_T idx_row;
  boolean_T b_overflow;
  int32_T b_a;
  int32_T i;
  int32_T c_a;
  int32_T d_a;
  int32_T i1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  mIneq = obj->sizes[2];
  mEq = obj->sizes[1];
  offsetIneq_tmp_tmp = obj->nVarOrig + 1;
  offsetEq1_tmp_tmp = obj->nVarOrig + obj->sizes[2];
  offsetEq2 = offsetEq1_tmp_tmp + obj->sizes[1];
  st.site = &ub_emlrtRSI;
  st.site = &ub_emlrtRSI;
  overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx_col = 0; idx_col < mIneq; idx_col++) {
    b_idx_col = idx_col + 1;
    b = (offsetIneq_tmp_tmp + b_idx_col) - 2;
    st.site = &ub_emlrtRSI;
    overflow = ((offsetIneq_tmp_tmp <= b) && (b > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = offsetIneq_tmp_tmp; idx_row <= b; idx_row++) {
      i = obj->Aineq.size(1);
      if ((b_idx_col < 1) || (b_idx_col > i)) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aineq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aineq[(b_idx_col + obj->Aineq.size(1) * (idx_row - 1)) - 1] = 0.0;
    }

    i = obj->Aineq.size(1);
    if ((b_idx_col < 1) || (b_idx_col > i)) {
      emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
    }

    i = obj->Aineq.size(0);
    d_a = offsetIneq_tmp_tmp + b_idx_col;
    i1 = d_a - 1;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
    }

    obj->Aineq[(b_idx_col + obj->Aineq.size(1) * (i1 - 1)) - 1] = -1.0;
    b = obj->nVar;
    st.site = &ub_emlrtRSI;
    overflow = ((d_a <= obj->nVar) && (obj->nVar > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = d_a; idx_row <= b; idx_row++) {
      i = obj->Aineq.size(1);
      if (b_idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aineq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aineq[(b_idx_col + obj->Aineq.size(1) * (idx_row - 1)) - 1] = 0.0;
    }
  }

  idxGlobalColStart = obj->isActiveIdx[1] - 1;
  st.site = &ub_emlrtRSI;
  overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  if (0 <= mEq - 1) {
    a = offsetIneq_tmp_tmp;
    b_overflow = ((offsetIneq_tmp_tmp <= offsetEq1_tmp_tmp) &&
                  (offsetEq1_tmp_tmp > 2147483646));
    b_a = offsetEq1_tmp_tmp + 1;
    c_a = offsetEq2 + 1;
  }

  for (idx_col = 0; idx_col < mEq; idx_col++) {
    b_idx_col = idx_col + 1;
    st.site = &ub_emlrtRSI;
    if (b_overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = a; idx_row <= offsetEq1_tmp_tmp; idx_row++) {
      i = obj->Aeq.size(1);
      if ((b_idx_col < 1) || (b_idx_col > i)) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aeq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (idx_row - 1)) - 1] = 0.0;
      i = obj->ATwset.size(1);
      i1 = idxGlobalColStart + b_idx_col;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->ATwset.size(0);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->ATwset[(i1 + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
    }

    d_a = offsetEq1_tmp_tmp + b_idx_col;
    b = d_a - 1;
    st.site = &ub_emlrtRSI;
    overflow = ((offsetEq1_tmp_tmp + 1 <= b) && (b > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = b_a; idx_row <= b; idx_row++) {
      i = obj->Aeq.size(1);
      if ((b_idx_col < 1) || (b_idx_col > i)) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aeq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (idx_row - 1)) - 1] = 0.0;
      i = obj->ATwset.size(1);
      i1 = idxGlobalColStart + b_idx_col;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->ATwset.size(0);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->ATwset[(i1 + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
    }

    i = obj->Aeq.size(1);
    if ((b_idx_col < 1) || (b_idx_col > i)) {
      emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
    }

    i = obj->Aeq.size(0);
    if ((d_a < 1) || (d_a > i)) {
      emlrtDynamicBoundsCheckR2012b(d_a, 1, i, &t_emlrtBCI, sp);
    }

    obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (d_a - 1)) - 1] = -1.0;
    i = obj->ATwset.size(1);
    i1 = idxGlobalColStart + b_idx_col;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if (d_a > i) {
      emlrtDynamicBoundsCheckR2012b(d_a, 1, i, &t_emlrtBCI, sp);
    }

    obj->ATwset[(i1 + obj->ATwset.size(1) * (d_a - 1)) - 1] = -1.0;
    d_a++;
    st.site = &ub_emlrtRSI;
    overflow = ((d_a <= offsetEq2) && (offsetEq2 > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = d_a; idx_row <= offsetEq2; idx_row++) {
      i = obj->Aeq.size(1);
      if (b_idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aeq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (idx_row - 1)) - 1] = 0.0;
      i = obj->ATwset.size(1);
      i1 = idxGlobalColStart + b_idx_col;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->ATwset.size(0);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->ATwset[(i1 + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
    }

    d_a = offsetEq2 + b_idx_col;
    b = d_a - 1;
    st.site = &ub_emlrtRSI;
    overflow = ((offsetEq2 + 1 <= b) && (b > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = c_a; idx_row <= b; idx_row++) {
      i = obj->Aeq.size(1);
      if (b_idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aeq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (idx_row - 1)) - 1] = 0.0;
      i = obj->ATwset.size(1);
      i1 = idxGlobalColStart + b_idx_col;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->ATwset.size(0);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->ATwset[(i1 + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
    }

    i = obj->Aeq.size(1);
    if (b_idx_col > i) {
      emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
    }

    i = obj->Aeq.size(0);
    if ((d_a < 1) || (d_a > i)) {
      emlrtDynamicBoundsCheckR2012b(d_a, 1, i, &t_emlrtBCI, sp);
    }

    obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (d_a - 1)) - 1] = 1.0;
    i = obj->ATwset.size(1);
    i1 = idxGlobalColStart + b_idx_col;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if (d_a > i) {
      emlrtDynamicBoundsCheckR2012b(d_a, 1, i, &t_emlrtBCI, sp);
    }

    obj->ATwset[(i1 + obj->ATwset.size(1) * (d_a - 1)) - 1] = 1.0;
    d_a++;
    b = obj->nVar;
    st.site = &ub_emlrtRSI;
    overflow = ((d_a <= obj->nVar) && (obj->nVar > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = d_a; idx_row <= b; idx_row++) {
      i = obj->Aeq.size(1);
      if (b_idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->Aeq.size(0);
      if ((idx_row < 1) || (idx_row > i)) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->Aeq[(b_idx_col + obj->Aeq.size(1) * (idx_row - 1)) - 1] = 0.0;
      i = obj->ATwset.size(1);
      i1 = idxGlobalColStart + b_idx_col;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->ATwset.size(0);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
      }

      obj->ATwset[(i1 + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
    }
  }

  mIneq = obj->nVarOrig;
  b = obj->sizesRegularized[3];
  st.site = &ub_emlrtRSI;
  overflow = ((1 <= obj->sizesRegularized[3]) && (obj->sizesRegularized[3] >
    2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idxGlobalColStart = 1; idxGlobalColStart <= b; idxGlobalColStart++) {
    mIneq++;
    i = obj->indexLB.size(0);
    if (idxGlobalColStart > i) {
      emlrtDynamicBoundsCheckR2012b(idxGlobalColStart, 1, i, &t_emlrtBCI, sp);
    }

    obj->indexLB[idxGlobalColStart - 1] = mIneq;
  }

  d_a = obj->nVarOrig + 1;
  b = (obj->nVarOrig + obj->sizes[2]) + (obj->sizes[1] << 1);
  st.site = &ub_emlrtRSI;
  overflow = ((obj->nVarOrig + 1 <= b) && (b > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idxGlobalColStart = d_a; idxGlobalColStart <= b; idxGlobalColStart++) {
    i = obj->lb.size(0);
    if ((idxGlobalColStart < 1) || (idxGlobalColStart > i)) {
      emlrtDynamicBoundsCheckR2012b(idxGlobalColStart, 1, i, &t_emlrtBCI, sp);
    }

    obj->lb[idxGlobalColStart - 1] = 0.0;
  }

  idxGlobalColStart = obj->isActiveIdx[2];
  b = obj->nActiveConstr;
  st.site = &ub_emlrtRSI;
  overflow = ((obj->isActiveIdx[2] <= obj->nActiveConstr) && (obj->nActiveConstr
    > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx_col = idxGlobalColStart; idx_col <= b; idx_col++) {
    i = obj->Wid.size(0);
    if ((idx_col < 1) || (idx_col > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &t_emlrtBCI, sp);
    }

    switch (obj->Wid[idx_col - 1]) {
     case 3:
      i = obj->Wlocalidx.size(0);
      if (idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &t_emlrtBCI, sp);
      }

      mIneq = obj->Wlocalidx[idx_col - 1];
      d_a = offsetIneq_tmp_tmp + mIneq;
      mIneq = d_a - 2;
      st.site = &ub_emlrtRSI;
      overflow = ((offsetIneq_tmp_tmp <= mIneq) && (mIneq > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idx_row = offsetIneq_tmp_tmp; idx_row <= mIneq; idx_row++) {
        i = obj->ATwset.size(1);
        if (idx_col > i) {
          emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &t_emlrtBCI, sp);
        }

        i = obj->ATwset.size(0);
        if ((idx_row < 1) || (idx_row > i)) {
          emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
        }

        obj->ATwset[(idx_col + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
      }

      i = obj->ATwset.size(1);
      if (idx_col > i) {
        emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &t_emlrtBCI, sp);
      }

      i = obj->ATwset.size(0);
      i1 = d_a - 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &t_emlrtBCI, sp);
      }

      obj->ATwset[(idx_col + obj->ATwset.size(1) * (i1 - 1)) - 1] = -1.0;
      mIneq = obj->nVar;
      st.site = &ub_emlrtRSI;
      overflow = ((d_a <= obj->nVar) && (obj->nVar > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idx_row = d_a; idx_row <= mIneq; idx_row++) {
        i = obj->ATwset.size(1);
        if (idx_col > i) {
          emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &t_emlrtBCI, sp);
        }

        i = obj->ATwset.size(0);
        if ((idx_row < 1) || (idx_row > i)) {
          emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
        }

        obj->ATwset[(idx_col + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
      }
      break;

     default:
      mIneq = obj->nVar;
      st.site = &ub_emlrtRSI;
      overflow = ((offsetIneq_tmp_tmp <= obj->nVar) && (obj->nVar > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idx_row = offsetIneq_tmp_tmp; idx_row <= mIneq; idx_row++) {
        i = obj->ATwset.size(1);
        if (idx_col > i) {
          emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &t_emlrtBCI, sp);
        }

        i = obj->ATwset.size(0);
        if ((idx_row < 1) || (idx_row > i)) {
          emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &t_emlrtBCI, sp);
        }

        obj->ATwset[(idx_col + obj->ATwset.size(1) * (idx_row - 1)) - 1] = 0.0;
      }
      break;
    }
  }
}

// End of code generation (modifyOverheadRegularized_.cpp)

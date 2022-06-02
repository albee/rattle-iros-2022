//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  RemoveDependentEq_.cpp
//
//  Code generation for function 'RemoveDependentEq_'
//


// Include files
#include "RemoveDependentEq_.h"
#include "blas.h"
#include "computeFval.h"
#include "computeQ_.h"
#include "countsort.h"
#include "eml_int_forloop_overflow_check.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "moveConstraint_.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"

// Variable Definitions
static emlrtRSInfo dc_emlrtRSI = { 1,  // lineNo
  "RemoveDependentEq_",                // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/RemoveDependentEq_.p"// pathName 
};

static emlrtRSInfo ec_emlrtRSI = { 1,  // lineNo
  "ComputeNumDependentEq_",            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/ComputeNumDependentEq_.p"// pathName 
};

static emlrtRSInfo ld_emlrtRSI = { 1,  // lineNo
  "IndexOfDependentEq_",               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/IndexOfDependentEq_.p"// pathName 
};

static emlrtRSInfo nd_emlrtRSI = { 1,  // lineNo
  "removeEqConstr",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeEqConstr.p"// pathName 
};

static emlrtBCInfo y_emlrtBCI = { 1,   // iFirst
  6,                                   // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeEqConstr",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeEqConstr.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo ab_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "ComputeNumDependentEq_",            // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/ComputeNumDependentEq_.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo bb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "RemoveDependentEq_",                // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/RemoveDependentEq_.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo cb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "IndexOfDependentEq_",               // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/IndexOfDependentEq_.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo db_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeEqConstr",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeEqConstr.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo eb_emlrtBCI = { 1,  // iFirst
  5,                                   // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeEqConstr",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeEqConstr.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo nb_emlrtRTEI = { 99,// lineNo
  26,                                  // colNo
  "xdot",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xdot.m"// pName 
};

static emlrtRTEInfo ob_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "RemoveDependentEq_",                // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/RemoveDependentEq_.p"// pName 
};

// Function Definitions
int32_T RemoveDependentEq_(const emlrtStack *sp, c_struct_T *memspace,
  g_struct_T *workingset, e_struct_T *qrmanager)
{
  int32_T nDepInd;
  int32_T nVar;
  int32_T mWorkingFixed;
  int32_T mTotalWorkingEq_tmp_tmp;
  coder::array<int32_T, 1U> r;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  nVar = workingset->nVar;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[1] + workingset->nWConstr[0];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    boolean_T overflow;
    int32_T idx_row;
    int32_T idx_local;
    int32_T i;
    int32_T i1;
    real_T tol;
    int32_T i2;
    boolean_T exitg1;
    st.site = &dc_emlrtRSI;
    if (mTotalWorkingEq_tmp_tmp > 2147483646) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    overflow = ((1 <= nVar) && (nVar > 2147483646));
    for (idx_row = 0; idx_row < mTotalWorkingEq_tmp_tmp; idx_row++) {
      st.site = &dc_emlrtRSI;
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (int32_T idx_col = 0; idx_col < nVar; idx_col++) {
        idx_local = idx_col + 1;
        i = workingset->ATwset.size(1);
        i1 = idx_row + 1;
        if (i1 > i) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &bb_emlrtBCI, sp);
        }

        i = workingset->ATwset.size(0);
        if ((idx_local < 1) || (idx_local > i)) {
          emlrtDynamicBoundsCheckR2012b(idx_local, 1, i, &bb_emlrtBCI, sp);
        }

        i = qrmanager->QR.size(1);
        if (idx_local > i) {
          emlrtDynamicBoundsCheckR2012b(idx_local, 1, i, &bb_emlrtBCI, sp);
        }

        i = qrmanager->QR.size(0);
        i2 = idx_row + 1;
        if (i2 > i) {
          emlrtDynamicBoundsCheckR2012b(i2, 1, i, &bb_emlrtBCI, sp);
        }

        qrmanager->QR[(idx_local + qrmanager->QR.size(1) * (i2 - 1)) - 1] =
          workingset->ATwset[(i1 + workingset->ATwset.size(1) * (idx_local - 1))
          - 1];
      }
    }

    st.site = &dc_emlrtRSI;
    idx_local = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    nDepInd = muIntScalarMax_sint32(0, idx_local);
    b_st.site = &ec_emlrtRSI;
    overflow = ((1 <= workingset->nVar) && (workingset->nVar > 2147483646));
    if (overflow) {
      c_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&c_st);
    }

    for (idx_row = 0; idx_row < nVar; idx_row++) {
      i = qrmanager->jpvt.size(0);
      i1 = idx_row + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ab_emlrtBCI, &st);
      }

      qrmanager->jpvt[i1 - 1] = 0;
    }

    b_st.site = &ec_emlrtRSI;
    qrmanager->usedPivoting = true;
    qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
    qrmanager->ncols = workingset->nVar;
    qrmanager->minRowCol = muIntScalarMin_sint32(mTotalWorkingEq_tmp_tmp,
      workingset->nVar);
    c_st.site = &fc_emlrtRSI;
    xgeqp3(&c_st, qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
           qrmanager->jpvt, qrmanager->tau);
    tol = 100.0 * static_cast<real_T>(workingset->nVar) * 2.2204460492503131E-16;
    idx_row = muIntScalarMin_sint32(workingset->nVar, mTotalWorkingEq_tmp_tmp);
    exitg1 = false;
    while ((!exitg1) && (idx_row > 0)) {
      i = qrmanager->QR.size(1);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &ab_emlrtBCI, &st);
      }

      i = qrmanager->QR.size(0);
      if (idx_row > i) {
        emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &ab_emlrtBCI, &st);
      }

      if (muDoubleScalarAbs(qrmanager->QR[(idx_row + qrmanager->QR.size(1) *
            (idx_row - 1)) - 1]) < tol) {
        idx_row--;
        nDepInd++;
      } else {
        exitg1 = true;
      }
    }

    if (nDepInd > 0) {
      b_st.site = &ec_emlrtRSI;
      c_st.site = &dd_emlrtRSI;
      computeQ_(&c_st, qrmanager, mTotalWorkingEq_tmp_tmp);
      b_st.site = &ec_emlrtRSI;
      if (nDepInd > 2147483646) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      idx_row = 0;
      exitg1 = false;
      while ((!exitg1) && (idx_row <= nDepInd - 1)) {
        real_T x;
        b_st.site = &ec_emlrtRSI;
        n_t = (ptrdiff_t)mTotalWorkingEq_tmp_tmp;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        r1.set_size((&nb_emlrtRTEI), (&b_st), qrmanager->Q.size(1),
                    qrmanager->Q.size(0));
        idx_local = qrmanager->Q.size(1);
        for (i = 0; i < idx_local; i++) {
          nVar = qrmanager->Q.size(0);
          for (i1 = 0; i1 < nVar; i1++) {
            r1[i1 + r1.size(1) * i] = qrmanager->Q[i + qrmanager->Q.size(1) * i1];
          }
        }

        x = ddot(&n_t, &r1[qrmanager->ldq * ((mTotalWorkingEq_tmp_tmp - idx_row)
                  - 1)], &incx_t, &(workingset->bwset.data())[0], &incy_t);
        if (muDoubleScalarAbs(x) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx_row++;
        }
      }
    }

    if (nDepInd > 0) {
      st.site = &dc_emlrtRSI;
      r.set_size((&ob_emlrtRTEI), (&st), memspace->workspace_int.size(0));
      idx_local = memspace->workspace_int.size(0);
      for (i = 0; i < idx_local; i++) {
        r[i] = memspace->workspace_int[i];
      }

      b_st.site = &ld_emlrtRSI;
      overflow = ((1 <= workingset->nWConstr[0]) && (workingset->nWConstr[0] >
        2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (idx_row = 0; idx_row < mWorkingFixed; idx_row++) {
        i = qrmanager->jpvt.size(0);
        i1 = idx_row + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &cb_emlrtBCI, &st);
        }

        qrmanager->jpvt[i1 - 1] = 1;
      }

      idx_local = workingset->nWConstr[0] + 1;
      b_st.site = &ld_emlrtRSI;
      for (idx_row = idx_local; idx_row <= mTotalWorkingEq_tmp_tmp; idx_row++) {
        i = qrmanager->jpvt.size(0);
        if ((idx_row < 1) || (idx_row > i)) {
          emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &cb_emlrtBCI, &st);
        }

        qrmanager->jpvt[idx_row - 1] = 0;
      }

      b_st.site = &ld_emlrtRSI;
      factorQRE(&b_st, qrmanager, workingset->ATwset, workingset->nVar,
                mTotalWorkingEq_tmp_tmp);
      b_st.site = &ld_emlrtRSI;
      for (idx_row = 0; idx_row < nDepInd; idx_row++) {
        i = qrmanager->jpvt.size(0);
        i1 = ((mTotalWorkingEq_tmp_tmp - nDepInd) + idx_row) + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &cb_emlrtBCI, &st);
        }

        i = idx_row + 1;
        if (i > r.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, r.size(0), &cb_emlrtBCI, &st);
        }

        r[i - 1] = qrmanager->jpvt[i1 - 1];
      }

      memspace->workspace_int.set_size((&ob_emlrtRTEI), sp, r.size(0));
      idx_local = r.size(0);
      for (i = 0; i < idx_local; i++) {
        memspace->workspace_int[i] = r[i];
      }

      st.site = &dc_emlrtRSI;
      countsort(&st, memspace->workspace_int, nDepInd, memspace->workspace_sort,
                1, mTotalWorkingEq_tmp_tmp);
      for (idx_row = nDepInd; idx_row >= 1; idx_row--) {
        st.site = &dc_emlrtRSI;
        i = memspace->workspace_int.size(0);
        if (idx_row > i) {
          emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &bb_emlrtBCI, &st);
        }

        i = workingset->nWConstr[0] + workingset->nWConstr[1];
        if (i != 0) {
          i1 = memspace->workspace_int[idx_row - 1];
          if (i1 <= i) {
            if ((workingset->nActiveConstr == i) || (i1 == i)) {
              workingset->mEqRemoved++;
              i = workingset->Wlocalidx.size(0);
              if ((i1 < 1) || (i1 > i)) {
                emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx_row -
                  1], 1, i, &db_emlrtBCI, &st);
              }

              i = workingset->indexEqRemoved.size(0);
              if ((workingset->mEqRemoved < 1) || (workingset->mEqRemoved > i))
              {
                emlrtDynamicBoundsCheckR2012b(workingset->mEqRemoved, 1, i,
                  &db_emlrtBCI, &st);
              }

              i = memspace->workspace_int[idx_row - 1] - 1;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx[i];
              b_st.site = &nd_emlrtRSI;
              i2 = workingset->Wid.size(0);
              if (i1 > i2) {
                emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx_row -
                  1], 1, i2, &x_emlrtBCI, &b_st);
              }

              mWorkingFixed = workingset->Wid[i];
              i2 = workingset->Wlocalidx.size(0);
              if (i1 > i2) {
                emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx_row -
                  1], 1, i2, &x_emlrtBCI, &b_st);
              }

              i1 = workingset->Wid[i];
              if ((i1 < 1) || (i1 > 6)) {
                emlrtDynamicBoundsCheckR2012b(i1, 1, 6, &w_emlrtBCI, &b_st);
              }

              i1 = workingset->isActiveConstr.size(0);
              i = (workingset->isActiveIdx[workingset->Wid[i] - 1] +
                   workingset->Wlocalidx[i]) - 1;
              if ((i < 1) || (i > i1)) {
                emlrtDynamicBoundsCheckR2012b(i, 1, i1, &x_emlrtBCI, &b_st);
              }

              workingset->isActiveConstr[i - 1] = false;
              c_st.site = &od_emlrtRSI;
              moveConstraint_(&c_st, workingset, workingset->nActiveConstr,
                              memspace->workspace_int[idx_row - 1]);
              workingset->nActiveConstr--;
              if ((mWorkingFixed < 1) || (mWorkingFixed > 5)) {
                emlrtDynamicBoundsCheckR2012b(mWorkingFixed, 1, 5, &fb_emlrtBCI,
                  &b_st);
              }

              workingset->nWConstr[mWorkingFixed - 1]--;
            } else {
              workingset->mEqRemoved++;
              i2 = workingset->Wid.size(0);
              if ((i1 < 1) || (i1 > i2)) {
                emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx_row -
                  1], 1, i2, &db_emlrtBCI, &st);
              }

              nVar = i1 - 1;
              mWorkingFixed = workingset->Wid[nVar];
              i2 = workingset->Wlocalidx.size(0);
              if (i1 > i2) {
                emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx_row -
                  1], 1, i2, &db_emlrtBCI, &st);
              }

              idx_local = workingset->Wlocalidx[nVar];
              i1 = workingset->indexEqRemoved.size(0);
              if ((workingset->mEqRemoved < 1) || (workingset->mEqRemoved > i1))
              {
                emlrtDynamicBoundsCheckR2012b(workingset->mEqRemoved, 1, i1,
                  &db_emlrtBCI, &st);
              }

              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx[nVar];
              if ((mWorkingFixed < 1) || (mWorkingFixed > 6)) {
                emlrtDynamicBoundsCheckR2012b(mWorkingFixed, 1, 6, &y_emlrtBCI,
                  &st);
              }

              i1 = workingset->isActiveConstr.size(0);
              i2 = (workingset->isActiveIdx[mWorkingFixed - 1] + idx_local) - 1;
              if ((i2 < 1) || (i2 > i1)) {
                emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &db_emlrtBCI, &st);
              }

              workingset->isActiveConstr[i2 - 1] = false;
              b_st.site = &nd_emlrtRSI;
              moveConstraint_(&b_st, workingset, i, memspace->
                              workspace_int[idx_row - 1]);
              b_st.site = &nd_emlrtRSI;
              moveConstraint_(&b_st, workingset, workingset->nActiveConstr, i);
              workingset->nActiveConstr--;
              if (mWorkingFixed > 5) {
                emlrtDynamicBoundsCheckR2012b(6, 1, 5, &eb_emlrtBCI, &st);
              }

              workingset->nWConstr[mWorkingFixed - 1]--;
            }
          }
        }
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return nDepInd;
}

// End of code generation (RemoveDependentEq_.cpp)

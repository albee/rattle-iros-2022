//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  RemoveDependentIneq_.cpp
//
//  Code generation for function 'RemoveDependentIneq_'
//


// Include files
#include "RemoveDependentIneq_.h"
#include "computeFval.h"
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
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo qd_emlrtRSI = { 1,  // lineNo
  "RemoveDependentIneq_",              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/RemoveDependentIneq_.p"// pathName 
};

static emlrtBCInfo ib_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "RemoveDependentIneq_",              // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/RemoveDependentIneq_.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void RemoveDependentIneq_(const emlrtStack *sp, g_struct_T *workingset,
  e_struct_T *qrmanager, c_struct_T *memspace)
{
  int32_T nActiveConstr;
  int32_T nFixedConstr;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nActiveConstr = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[1] + workingset->nWConstr[0];
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    real_T tol;
    boolean_T overflow;
    int32_T idx;
    int32_T a;
    int32_T i;
    int32_T i1;
    int32_T nDepIneq;
    tol = 100.0 * static_cast<real_T>(workingset->nVar) * 2.2204460492503131E-16;
    st.site = &qd_emlrtRSI;
    overflow = ((1 <= nFixedConstr) && (nFixedConstr > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < nFixedConstr; idx++) {
      i = qrmanager->jpvt.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ib_emlrtBCI, sp);
      }

      qrmanager->jpvt[i1 - 1] = 1;
    }

    a = nFixedConstr + 1;
    st.site = &qd_emlrtRSI;
    overflow = ((nFixedConstr + 1 <= workingset->nActiveConstr) &&
                (workingset->nActiveConstr > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = a; idx <= nActiveConstr; idx++) {
      i = qrmanager->jpvt.size(0);
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
      }

      qrmanager->jpvt[idx - 1] = 0;
    }

    st.site = &qd_emlrtRSI;
    factorQRE(&st, qrmanager, workingset->ATwset, workingset->nVar,
              workingset->nActiveConstr);
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx--) {
      nDepIneq++;
      i = qrmanager->jpvt.size(0);
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
      }

      i = memspace->workspace_int.size(0);
      if ((nDepIneq < 1) || (nDepIneq > i)) {
        emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &ib_emlrtBCI, sp);
      }

      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx - 1];
    }

    if (idx <= workingset->nVar) {
      boolean_T exitg1;
      exitg1 = false;
      while ((!exitg1) && (idx > nFixedConstr)) {
        i = qrmanager->QR.size(1);
        if ((idx < 1) || (idx > i)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
        }

        i = qrmanager->QR.size(0);
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
        }

        if (muDoubleScalarAbs(qrmanager->QR[(idx + qrmanager->QR.size(1) * (idx
               - 1)) - 1]) < tol) {
          nDepIneq++;
          i = qrmanager->jpvt.size(0);
          if (idx > i) {
            emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
          }

          i = memspace->workspace_int.size(0);
          if ((nDepIneq < 1) || (nDepIneq > i)) {
            emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &ib_emlrtBCI, sp);
          }

          memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx - 1];
          idx--;
        } else {
          exitg1 = true;
        }
      }
    }

    st.site = &qd_emlrtRSI;
    countsort(&st, memspace->workspace_int, nDepIneq, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      st.site = &qd_emlrtRSI;
      i = memspace->workspace_int.size(0);
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, &st);
      }

      i = workingset->Wid.size(0);
      i1 = memspace->workspace_int[idx - 1];
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx - 1], 1, i,
          &x_emlrtBCI, &st);
      }

      a = i1 - 1;
      nActiveConstr = workingset->Wid[a];
      i = workingset->Wlocalidx.size(0);
      if (i1 > i) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx - 1], 1, i,
          &x_emlrtBCI, &st);
      }

      i = workingset->Wid[a];
      if ((i < 1) || (i > 6)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 6, &w_emlrtBCI, &st);
      }

      i = workingset->isActiveConstr.size(0);
      i1 = (workingset->isActiveIdx[workingset->Wid[a] - 1] +
            workingset->Wlocalidx[a]) - 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &x_emlrtBCI, &st);
      }

      workingset->isActiveConstr[i1 - 1] = false;
      b_st.site = &od_emlrtRSI;
      moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                      memspace->workspace_int[idx - 1]);
      workingset->nActiveConstr--;
      if ((nActiveConstr < 1) || (nActiveConstr > 5)) {
        emlrtDynamicBoundsCheckR2012b(nActiveConstr, 1, 5, &fb_emlrtBCI, &st);
      }

      workingset->nWConstr[nActiveConstr - 1]--;
    }
  }
}

void b_RemoveDependentIneq_(const emlrtStack *sp, g_struct_T *workingset,
  e_struct_T *qrmanager, c_struct_T *memspace)
{
  int32_T nActiveConstr;
  int32_T nFixedConstr;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nActiveConstr = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[1] + workingset->nWConstr[0];
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    real_T tol;
    boolean_T overflow;
    int32_T idx;
    int32_T a;
    int32_T i;
    int32_T i1;
    int32_T nDepIneq;
    tol = 1000.0 * static_cast<real_T>(workingset->nVar) *
      2.2204460492503131E-16;
    st.site = &qd_emlrtRSI;
    overflow = ((1 <= nFixedConstr) && (nFixedConstr > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < nFixedConstr; idx++) {
      i = qrmanager->jpvt.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &ib_emlrtBCI, sp);
      }

      qrmanager->jpvt[i1 - 1] = 1;
    }

    a = nFixedConstr + 1;
    st.site = &qd_emlrtRSI;
    overflow = ((nFixedConstr + 1 <= workingset->nActiveConstr) &&
                (workingset->nActiveConstr > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = a; idx <= nActiveConstr; idx++) {
      i = qrmanager->jpvt.size(0);
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
      }

      qrmanager->jpvt[idx - 1] = 0;
    }

    st.site = &qd_emlrtRSI;
    factorQRE(&st, qrmanager, workingset->ATwset, workingset->nVar,
              workingset->nActiveConstr);
    nDepIneq = 0;
    for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx--) {
      nDepIneq++;
      i = qrmanager->jpvt.size(0);
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
      }

      i = memspace->workspace_int.size(0);
      if ((nDepIneq < 1) || (nDepIneq > i)) {
        emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &ib_emlrtBCI, sp);
      }

      memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx - 1];
    }

    if (idx <= workingset->nVar) {
      boolean_T exitg1;
      exitg1 = false;
      while ((!exitg1) && (idx > nFixedConstr)) {
        i = qrmanager->QR.size(1);
        if ((idx < 1) || (idx > i)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
        }

        i = qrmanager->QR.size(0);
        if (idx > i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
        }

        if (muDoubleScalarAbs(qrmanager->QR[(idx + qrmanager->QR.size(1) * (idx
               - 1)) - 1]) < tol) {
          nDepIneq++;
          i = qrmanager->jpvt.size(0);
          if (idx > i) {
            emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, sp);
          }

          i = memspace->workspace_int.size(0);
          if ((nDepIneq < 1) || (nDepIneq > i)) {
            emlrtDynamicBoundsCheckR2012b(nDepIneq, 1, i, &ib_emlrtBCI, sp);
          }

          memspace->workspace_int[nDepIneq - 1] = qrmanager->jpvt[idx - 1];
          idx--;
        } else {
          exitg1 = true;
        }
      }
    }

    st.site = &qd_emlrtRSI;
    countsort(&st, memspace->workspace_int, nDepIneq, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      st.site = &qd_emlrtRSI;
      i = memspace->workspace_int.size(0);
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &ib_emlrtBCI, &st);
      }

      i = workingset->Wid.size(0);
      i1 = memspace->workspace_int[idx - 1];
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx - 1], 1, i,
          &x_emlrtBCI, &st);
      }

      a = i1 - 1;
      nActiveConstr = workingset->Wid[a];
      i = workingset->Wlocalidx.size(0);
      if (i1 > i) {
        emlrtDynamicBoundsCheckR2012b(memspace->workspace_int[idx - 1], 1, i,
          &x_emlrtBCI, &st);
      }

      i = workingset->Wid[a];
      if ((i < 1) || (i > 6)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, 6, &w_emlrtBCI, &st);
      }

      i = workingset->isActiveConstr.size(0);
      i1 = (workingset->isActiveIdx[workingset->Wid[a] - 1] +
            workingset->Wlocalidx[a]) - 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &x_emlrtBCI, &st);
      }

      workingset->isActiveConstr[i1 - 1] = false;
      b_st.site = &od_emlrtRSI;
      moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                      memspace->workspace_int[idx - 1]);
      workingset->nActiveConstr--;
      if ((nActiveConstr < 1) || (nActiveConstr > 5)) {
        emlrtDynamicBoundsCheckR2012b(nActiveConstr, 1, 5, &fb_emlrtBCI, &st);
      }

      workingset->nWConstr[nActiveConstr - 1]--;
    }
  }
}

// End of code generation (RemoveDependentIneq_.cpp)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  feasibleX0ForWorkingSet.cpp
//
//  Code generation for function 'feasibleX0ForWorkingSet'
//


// Include files
#include "feasibleX0ForWorkingSet.h"
#include "blas.h"
#include "computeFval.h"
#include "computeQ_.h"
#include "eml_int_forloop_overflow_check.h"
#include "factorQR.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xgeqrf.h"

// Variable Definitions
static emlrtRSInfo rd_emlrtRSI = { 1,  // lineNo
  "feasibleX0ForWorkingSet",           // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/feasibleX0ForWorkingSet.p"// pathName 
};

static emlrtRSInfo fe_emlrtRSI = { 86, // lineNo
  "xgemm",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemm.m"// pathName 
};

static emlrtRSInfo ge_emlrtRSI = { 77, // lineNo
  "xtrsm",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsm.m"// pathName 
};

static emlrtRSInfo he_emlrtRSI = { 76, // lineNo
  "xtrsm",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsm.m"// pathName 
};

static emlrtRSInfo ie_emlrtRSI = { 1,  // lineNo
  "computeTallQ",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/computeTallQ.p"// pathName 
};

static emlrtBCInfo jb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "feasibleX0ForWorkingSet",           // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/feasibleX0ForWorkingSet.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo yb_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "feasibleX0ForWorkingSet",           // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/feasibleX0ForWorkingSet.p"// pName 
};

static emlrtRTEInfo ac_emlrtRTEI = { 171,// lineNo
  28,                                  // colNo
  "xtrsm",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsm.m"// pName 
};

static emlrtRTEInfo bc_emlrtRTEI = { 173,// lineNo
  27,                                  // colNo
  "xtrsm",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsm.m"// pName 
};

static emlrtRTEInfo fc_emlrtRTEI = { 150,// lineNo
  26,                                  // colNo
  "xaxpy",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xaxpy.m"// pName 
};

// Function Definitions
boolean_T feasibleX0ForWorkingSet(const emlrtStack *sp, coder::array<real_T, 2U>
  &workspace, coder::array<real_T, 1U> &xCurrent, g_struct_T *workingset,
  e_struct_T *qrmanager)
{
  boolean_T nonDegenerateWset;
  int32_T mWConstr;
  int32_T nVar;
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  boolean_T b_overflow;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  char_T TRANSA1;
  coder::array<real_T, 2U> r1;
  char_T UPLO1;
  char_T SIDE1;
  ptrdiff_t ldc_t;
  coder::array<real_T, 2U> r2;
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
  mWConstr = workingset->nActiveConstr;
  nVar = workingset->nVar;
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    boolean_T overflow;
    int32_T idx;
    int32_T b_idx;
    int32_T i;
    int32_T idx_row;
    st.site = &rd_emlrtRSI;
    overflow = ((1 <= workingset->nActiveConstr) && (workingset->nActiveConstr >
      2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < mWConstr; idx++) {
      b_idx = idx + 1;
      i = workingset->bwset.size(0);
      if ((b_idx < 1) || (b_idx > i)) {
        emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &jb_emlrtBCI, sp);
      }

      i = workspace.size(0);
      if (b_idx > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &jb_emlrtBCI, sp);
      }

      alpha1 = workingset->bwset[b_idx - 1];
      workspace[workspace.size(1) * (b_idx - 1)] = alpha1;
      i = workingset->bwset.size(0);
      if (b_idx > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &jb_emlrtBCI, sp);
      }

      i = workspace.size(0);
      if (b_idx > i) {
        emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &jb_emlrtBCI, sp);
      }

      workspace[workspace.size(1) * (b_idx - 1) + 1] = alpha1;
    }

    st.site = &rd_emlrtRSI;
    if (workingset->nActiveConstr >= 1) {
      b_st.site = &yc_emlrtRSI;
      b_st.site = &ad_emlrtRSI;
      alpha1 = -1.0;
      beta1 = 1.0;
      TRANSA = 'T';
      m_t = (ptrdiff_t)workingset->nVar;
      n_t = (ptrdiff_t)workingset->nActiveConstr;
      lda_t = (ptrdiff_t)workingset->ATwset.size(0);
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      r.set_size((&wb_emlrtRTEI), (&b_st), workingset->ATwset.size(1),
                 workingset->ATwset.size(0));
      b_idx = workingset->ATwset.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = workingset->ATwset.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          r[idx_row + r.size(1) * i] = workingset->ATwset[i +
            workingset->ATwset.size(1) * idx_row];
        }
      }

      r1.set_size((&xb_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
      b_idx = workspace.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = workspace.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          r1[idx_row + r1.size(1) * i] = workspace[i + workspace.size(1) *
            idx_row];
        }
      }

      dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t,
            &(xCurrent.data())[0], &incx_t, &beta1, &(r1.data())[0], &incy_t);
      workspace.set_size((&yb_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
      b_idx = r1.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = r1.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          workspace[idx_row + workspace.size(1) * i] = r1[i + r1.size(1) *
            idx_row];
        }
      }
    }

    if (workingset->nActiveConstr >= workingset->nVar) {
      int32_T ldw;
      st.site = &rd_emlrtRSI;
      overflow = ((1 <= workingset->nVar) && (workingset->nVar > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      if (0 <= nVar - 1) {
        b_overflow = ((1 <= mWConstr) && (mWConstr > 2147483646));
      }

      for (idx = 0; idx < nVar; idx++) {
        b_idx = idx + 1;
        st.site = &rd_emlrtRSI;
        if (b_overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (idx_row = 0; idx_row < mWConstr; idx_row++) {
          ldw = idx_row + 1;
          i = workingset->ATwset.size(1);
          if ((ldw < 1) || (ldw > i)) {
            emlrtDynamicBoundsCheckR2012b(ldw, 1, i, &jb_emlrtBCI, sp);
          }

          i = workingset->ATwset.size(0);
          if ((b_idx < 1) || (b_idx > i)) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &jb_emlrtBCI, sp);
          }

          i = qrmanager->QR.size(1);
          if (b_idx > i) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &jb_emlrtBCI, sp);
          }

          i = qrmanager->QR.size(0);
          if (ldw > i) {
            emlrtDynamicBoundsCheckR2012b(ldw, 1, i, &jb_emlrtBCI, sp);
          }

          qrmanager->QR[(b_idx + qrmanager->QR.size(1) * (ldw - 1)) - 1] =
            workingset->ATwset[(ldw + workingset->ATwset.size(1) * (b_idx - 1))
            - 1];
        }
      }

      st.site = &rd_emlrtRSI;
      qrmanager->usedPivoting = false;
      qrmanager->mrows = workingset->nActiveConstr;
      qrmanager->ncols = workingset->nVar;
      ldw = workingset->nVar;
      b_st.site = &sd_emlrtRSI;
      overflow = ((1 <= workingset->nVar) && (workingset->nVar > 2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (idx = 0; idx < ldw; idx++) {
        b_idx = idx + 1;
        i = qrmanager->jpvt.size(0);
        if ((b_idx < 1) || (b_idx > i)) {
          emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &kb_emlrtBCI, &st);
        }

        qrmanager->jpvt[b_idx - 1] = b_idx;
      }

      qrmanager->minRowCol = muIntScalarMin_sint32(workingset->nActiveConstr,
        workingset->nVar);
      b_st.site = &sd_emlrtRSI;
      xgeqrf(&b_st, qrmanager->QR, workingset->nActiveConstr, workingset->nVar,
             qrmanager->tau);
      st.site = &rd_emlrtRSI;
      b_st.site = &dd_emlrtRSI;
      computeQ_(&b_st, qrmanager, workingset->nActiveConstr);
      ldw = workspace.size(0);
      st.site = &rd_emlrtRSI;
      b_st.site = &fe_emlrtRSI;
      b_st.site = &ee_emlrtRSI;
      alpha1 = 1.0;
      beta1 = 0.0;
      TRANSA = 'N';
      TRANSA1 = 'T';
      m_t = (ptrdiff_t)workingset->nVar;
      n_t = (ptrdiff_t)2;
      incy_t = (ptrdiff_t)workingset->nActiveConstr;
      lda_t = (ptrdiff_t)qrmanager->ldq;
      incx_t = (ptrdiff_t)workspace.size(0);
      ldc_t = (ptrdiff_t)workspace.size(0);
      r.set_size((&cc_emlrtRTEI), (&b_st), qrmanager->Q.size(1),
                 qrmanager->Q.size(0));
      b_idx = qrmanager->Q.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = qrmanager->Q.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          r[idx_row + r.size(1) * i] = qrmanager->Q[i + qrmanager->Q.size(1) *
            idx_row];
        }
      }

      r1.set_size((&dc_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
      b_idx = workspace.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = workspace.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          r1[idx_row + r1.size(1) * i] = workspace[i + workspace.size(1) *
            idx_row];
        }
      }

      r2.set_size((&ec_emlrtRTEI), (&b_st), r1.size(0), r1.size(1));
      b_idx = r1.size(1) * r1.size(0);
      for (i = 0; i < b_idx; i++) {
        r2[i] = r1[i];
      }

      dgemm(&TRANSA1, &TRANSA, &m_t, &n_t, &incy_t, &alpha1, &(r.data())[0],
            &lda_t, &(r1.data())[0], &incx_t, &beta1, &(r2.data())[0], &ldc_t);
      workspace.set_size((&yb_emlrtRTEI), (&b_st), r2.size(1), r2.size(0));
      b_idx = r2.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = r2.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          workspace[idx_row + workspace.size(1) * i] = r2[i + r2.size(1) *
            idx_row];
        }
      }

      st.site = &rd_emlrtRSI;
      b_st.site = &he_emlrtRSI;
      b_st.site = &ge_emlrtRSI;
      alpha1 = 1.0;
      TRANSA = 'N';
      TRANSA1 = 'N';
      UPLO1 = 'U';
      SIDE1 = 'L';
      m_t = (ptrdiff_t)workingset->nVar;
      n_t = (ptrdiff_t)2;
      lda_t = (ptrdiff_t)qrmanager->ldq;
      incx_t = (ptrdiff_t)ldw;
      r.set_size((&ac_emlrtRTEI), (&b_st), qrmanager->QR.size(1),
                 qrmanager->QR.size(0));
      b_idx = qrmanager->QR.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = qrmanager->QR.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          r[idx_row + r.size(1) * i] = qrmanager->QR[i + qrmanager->QR.size(1) *
            idx_row];
        }
      }

      r1.set_size((&bc_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
      b_idx = workspace.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = workspace.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          r1[idx_row + r1.size(1) * i] = workspace[i + workspace.size(1) *
            idx_row];
        }
      }

      dtrsm(&SIDE1, &UPLO1, &TRANSA1, &TRANSA, &m_t, &n_t, &alpha1, &(r.data())
            [0], &lda_t, &(r1.data())[0], &incx_t);
      workspace.set_size((&yb_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
      b_idx = r1.size(1);
      for (i = 0; i < b_idx; i++) {
        idx = r1.size(0);
        for (idx_row = 0; idx_row < idx; idx_row++) {
          workspace[idx_row + workspace.size(1) * i] = r1[i + r1.size(1) *
            idx_row];
        }
      }
    } else {
      int32_T ldw;
      st.site = &rd_emlrtRSI;
      factorQR(&st, qrmanager, workingset->ATwset, workingset->nVar,
               workingset->nActiveConstr);
      st.site = &rd_emlrtRSI;
      b_st.site = &ie_emlrtRSI;
      computeQ_(&b_st, qrmanager, qrmanager->minRowCol);
      ldw = workspace.size(0);
      st.site = &rd_emlrtRSI;
      if (workingset->nActiveConstr >= 1) {
        b_st.site = &he_emlrtRSI;
        b_st.site = &ge_emlrtRSI;
        alpha1 = 1.0;
        TRANSA = 'N';
        TRANSA1 = 'T';
        UPLO1 = 'U';
        SIDE1 = 'L';
        m_t = (ptrdiff_t)workingset->nActiveConstr;
        n_t = (ptrdiff_t)2;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)workspace.size(0);
        r.set_size((&ac_emlrtRTEI), (&b_st), qrmanager->QR.size(1),
                   qrmanager->QR.size(0));
        b_idx = qrmanager->QR.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = qrmanager->QR.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            r[idx_row + r.size(1) * i] = qrmanager->QR[i + qrmanager->QR.size(1)
              * idx_row];
          }
        }

        r1.set_size((&bc_emlrtRTEI), (&b_st), workspace.size(1), workspace.size
                    (0));
        b_idx = workspace.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = workspace.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            r1[idx_row + r1.size(1) * i] = workspace[i + workspace.size(1) *
              idx_row];
          }
        }

        dtrsm(&SIDE1, &UPLO1, &TRANSA1, &TRANSA, &m_t, &n_t, &alpha1, &(r.data())
              [0], &lda_t, &(r1.data())[0], &incx_t);
        workspace.set_size((&yb_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
        b_idx = r1.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = r1.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            workspace[idx_row + workspace.size(1) * i] = r1[i + r1.size(1) *
              idx_row];
          }
        }
      }

      st.site = &rd_emlrtRSI;
      if (workingset->nActiveConstr >= 1) {
        b_st.site = &fe_emlrtRSI;
        b_st.site = &ee_emlrtRSI;
        alpha1 = 1.0;
        beta1 = 0.0;
        TRANSA = 'N';
        TRANSA1 = 'N';
        m_t = (ptrdiff_t)workingset->nVar;
        n_t = (ptrdiff_t)2;
        incy_t = (ptrdiff_t)workingset->nActiveConstr;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)ldw;
        ldc_t = (ptrdiff_t)ldw;
        r.set_size((&cc_emlrtRTEI), (&b_st), qrmanager->Q.size(1),
                   qrmanager->Q.size(0));
        b_idx = qrmanager->Q.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = qrmanager->Q.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            r[idx_row + r.size(1) * i] = qrmanager->Q[i + qrmanager->Q.size(1) *
              idx_row];
          }
        }

        r1.set_size((&dc_emlrtRTEI), (&b_st), workspace.size(1), workspace.size
                    (0));
        b_idx = workspace.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = workspace.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            r1[idx_row + r1.size(1) * i] = workspace[i + workspace.size(1) *
              idx_row];
          }
        }

        r2.set_size((&ec_emlrtRTEI), (&b_st), r1.size(0), r1.size(1));
        b_idx = r1.size(1) * r1.size(0);
        for (i = 0; i < b_idx; i++) {
          r2[i] = r1[i];
        }

        dgemm(&TRANSA1, &TRANSA, &m_t, &n_t, &incy_t, &alpha1, &(r.data())[0],
              &lda_t, &(r1.data())[0], &incx_t, &beta1, &(r2.data())[0], &ldc_t);
        workspace.set_size((&yb_emlrtRTEI), (&b_st), r2.size(1), r2.size(0));
        b_idx = r2.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = r2.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            workspace[idx_row + workspace.size(1) * i] = r2[i + r2.size(1) *
              idx_row];
          }
        }
      }
    }

    st.site = &rd_emlrtRSI;
    overflow = ((1 <= workingset->nVar) && (workingset->nVar > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    idx = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        i = workspace.size(0);
        idx_row = idx + 1;
        if ((idx_row < 1) || (idx_row > i)) {
          emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &jb_emlrtBCI, sp);
        }

        alpha1 = workspace[workspace.size(1) * (idx_row - 1)];
        if (muDoubleScalarIsInf(alpha1) || muDoubleScalarIsNaN(alpha1)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          i = workspace.size(0);
          idx_row = idx + 1;
          if ((idx_row < 1) || (idx_row > i)) {
            emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &jb_emlrtBCI, sp);
          }

          alpha1 = workspace[workspace.size(1) * (idx_row - 1) + 1];
          if (muDoubleScalarIsInf(alpha1) || muDoubleScalarIsNaN(alpha1)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        st.site = &rd_emlrtRSI;
        b_st.site = &ke_emlrtRSI;
        b_st.site = &je_emlrtRSI;
        alpha1 = 1.0;
        n_t = (ptrdiff_t)nVar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        r.set_size((&fc_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
        b_idx = workspace.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = workspace.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            r[idx_row + r.size(1) * i] = workspace[i + workspace.size(1) *
              idx_row];
          }
        }

        daxpy(&n_t, &alpha1, &(xCurrent.data())[0], &incx_t, &(r.data())[0],
              &incy_t);
        workspace.set_size((&yb_emlrtRTEI), (&b_st), r.size(1), r.size(0));
        b_idx = r.size(1);
        for (i = 0; i < b_idx; i++) {
          idx = r.size(0);
          for (idx_row = 0; idx_row < idx; idx_row++) {
            workspace[idx_row + workspace.size(1) * i] = r[i + r.size(1) *
              idx_row];
          }
        }

        st.site = &rd_emlrtRSI;
        alpha1 = maxConstraintViolation(&st, workingset, workspace);
        st.site = &rd_emlrtRSI;
        beta1 = b_maxConstraintViolation(&st, workingset, workspace,
          workspace.size(0) + 1);
        if ((alpha1 <= 2.2204460492503131E-16) || (alpha1 < beta1)) {
          st.site = &rd_emlrtRSI;
          if (nVar >= 1) {
            b_st.site = &ob_emlrtRSI;
            b_st.site = &nb_emlrtRSI;
            n_t = (ptrdiff_t)nVar;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            r.set_size((&vb_emlrtRTEI), (&b_st), workspace.size(1),
                       workspace.size(0));
            b_idx = workspace.size(1);
            for (i = 0; i < b_idx; i++) {
              idx = workspace.size(0);
              for (idx_row = 0; idx_row < idx; idx_row++) {
                r[idx_row + r.size(1) * i] = workspace[i + workspace.size(1) *
                  idx_row];
              }
            }

            dcopy(&n_t, &(r.data())[0], &incx_t, &(xCurrent.data())[0], &incy_t);
          }
        } else {
          st.site = &rd_emlrtRSI;
          b_st.site = &ob_emlrtRSI;
          b_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)nVar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&b_st), workspace.size(1), workspace.size
                     (0));
          b_idx = workspace.size(1);
          for (i = 0; i < b_idx; i++) {
            idx = workspace.size(0);
            for (idx_row = 0; idx_row < idx; idx_row++) {
              r[idx_row + r.size(1) * i] = workspace[i + workspace.size(1) *
                idx_row];
            }
          }

          dcopy(&n_t, &r[workspace.size(0)], &incx_t, &(xCurrent.data())[0],
                &incy_t);
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return nonDegenerateWset;
}

// End of code generation (feasibleX0ForWorkingSet.cpp)

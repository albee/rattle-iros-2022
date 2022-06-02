//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  compute_lambda.cpp
//
//  Code generation for function 'compute_lambda'
//


// Include files
#include "compute_lambda.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo nf_emlrtRSI = { 1,  // lineNo
  "compute_lambda",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/compute_lambda.p"// pathName 
};

static emlrtBCInfo kc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "compute_lambda",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/compute_lambda.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo lc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "isNonDegenerate",                   // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/isNonDegenerate.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo ad_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "compute_lambda",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/compute_lambda.p"// pName 
};

// Function Definitions
void compute_lambda(const emlrtStack *sp, coder::array<real_T, 2U> &workspace,
                    b_struct_T *solution, const d_struct_T *objective, const
                    e_struct_T *qrmanager)
{
  int32_T nActiveConstr;
  real_T tol;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  char_T TRANSA1;
  ptrdiff_t n_t;
  char_T UPLO1;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  nActiveConstr = qrmanager->ncols;
  if (qrmanager->ncols > 0) {
    boolean_T nonDegenerate;
    int32_T idx;
    int32_T i;
    tol = 100.0 * static_cast<real_T>(qrmanager->mrows) * 2.2204460492503131E-16;
    st.site = &nf_emlrtRSI;
    if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
      nonDegenerate = true;
    } else {
      nonDegenerate = false;
    }

    if (nonDegenerate) {
      boolean_T guard1 = false;
      idx = qrmanager->ncols;
      guard1 = false;
      if (qrmanager->mrows < qrmanager->ncols) {
        boolean_T exitg3;
        exitg3 = false;
        while ((!exitg3) && (idx > qrmanager->mrows)) {
          i = qrmanager->QR.size(1);
          if ((idx < 1) || (idx > i)) {
            emlrtDynamicBoundsCheckR2012b(idx, 1, i, &lc_emlrtBCI, &st);
          }

          i = qrmanager->QR.size(0);
          if ((qrmanager->mrows < 1) || (qrmanager->mrows > i)) {
            emlrtDynamicBoundsCheckR2012b(qrmanager->mrows, 1, i, &lc_emlrtBCI,
              &st);
          }

          if (muDoubleScalarAbs(qrmanager->QR[(idx + qrmanager->QR.size(1) *
                (qrmanager->mrows - 1)) - 1]) >= tol) {
            idx--;
          } else {
            exitg3 = true;
          }
        }

        nonDegenerate = (idx == qrmanager->mrows);
        if (nonDegenerate) {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        int32_T exitg2;
        do {
          exitg2 = 0;
          if (idx >= 1) {
            i = qrmanager->QR.size(1);
            if (idx > i) {
              emlrtDynamicBoundsCheckR2012b(idx, 1, i, &lc_emlrtBCI, &st);
            }

            i = qrmanager->QR.size(0);
            if (idx > i) {
              emlrtDynamicBoundsCheckR2012b(idx, 1, i, &lc_emlrtBCI, &st);
            }

            if (muDoubleScalarAbs(qrmanager->QR[(idx + qrmanager->QR.size(1) *
                  (idx - 1)) - 1]) >= tol) {
              idx--;
            } else {
              nonDegenerate = (idx == 0);
              exitg2 = 1;
            }
          } else {
            nonDegenerate = (idx == 0);
            exitg2 = 1;
          }
        } while (exitg2 == 0);
      }
    }

    if (!nonDegenerate) {
      solution->state = -7;
    } else {
      int32_T loop_ub;
      int32_T i1;
      st.site = &nf_emlrtRSI;
      if (qrmanager->mrows >= 1) {
        b_st.site = &ad_emlrtRSI;
        tol = 1.0;
        beta1 = 0.0;
        TRANSA = 'T';
        m_t = (ptrdiff_t)qrmanager->mrows;
        n_t = (ptrdiff_t)qrmanager->ncols;
        lda_t = (ptrdiff_t)qrmanager->ldq;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        r.set_size((&wb_emlrtRTEI), (&b_st), qrmanager->Q.size(1),
                   qrmanager->Q.size(0));
        loop_ub = qrmanager->Q.size(1);
        for (i = 0; i < loop_ub; i++) {
          idx = qrmanager->Q.size(0);
          for (i1 = 0; i1 < idx; i1++) {
            r[i1 + r.size(1) * i] = qrmanager->Q[i + qrmanager->Q.size(1) * i1];
          }
        }

        r1.set_size((&xb_emlrtRTEI), (&b_st), workspace.size(1), workspace.size
                    (0));
        loop_ub = workspace.size(1);
        for (i = 0; i < loop_ub; i++) {
          idx = workspace.size(0);
          for (i1 = 0; i1 < idx; i1++) {
            r1[i1 + r1.size(1) * i] = workspace[i + workspace.size(1) * i1];
          }
        }

        dgemv(&TRANSA, &m_t, &n_t, &tol, &(r.data())[0], &lda_t, &(((coder::
                 array<real_T, 1U> *)&objective->grad)->data())[0], &incx_t,
              &beta1, &(r1.data())[0], &incy_t);
        workspace.set_size((&ad_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
        loop_ub = r1.size(1);
        for (i = 0; i < loop_ub; i++) {
          idx = r1.size(0);
          for (i1 = 0; i1 < idx; i1++) {
            workspace[i1 + workspace.size(1) * i] = r1[i + r1.size(1) * i1];
          }
        }
      }

      st.site = &nf_emlrtRSI;
      b_st.site = &jf_emlrtRSI;
      TRANSA = 'N';
      TRANSA1 = 'N';
      UPLO1 = 'U';
      n_t = (ptrdiff_t)qrmanager->ncols;
      lda_t = (ptrdiff_t)qrmanager->ldq;
      incx_t = (ptrdiff_t)1;
      r.set_size((&pc_emlrtRTEI), (&b_st), qrmanager->QR.size(1),
                 qrmanager->QR.size(0));
      loop_ub = qrmanager->QR.size(1);
      for (i = 0; i < loop_ub; i++) {
        idx = qrmanager->QR.size(0);
        for (i1 = 0; i1 < idx; i1++) {
          r[i1 + r.size(1) * i] = qrmanager->QR[i + qrmanager->QR.size(1) * i1];
        }
      }

      r1.set_size((&qc_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
      loop_ub = workspace.size(1);
      for (i = 0; i < loop_ub; i++) {
        idx = workspace.size(0);
        for (i1 = 0; i1 < idx; i1++) {
          r1[i1 + r1.size(1) * i] = workspace[i + workspace.size(1) * i1];
        }
      }

      dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &(r.data())[0], &lda_t, &(r1.data())
            [0], &incx_t);
      workspace.set_size((&ad_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
      loop_ub = r1.size(1);
      for (i = 0; i < loop_ub; i++) {
        idx = r1.size(0);
        for (i1 = 0; i1 < idx; i1++) {
          workspace[i1 + workspace.size(1) * i] = r1[i + r1.size(1) * i1];
        }
      }

      st.site = &nf_emlrtRSI;
      nonDegenerate = ((1 <= qrmanager->ncols) && (qrmanager->ncols > 2147483646));
      if (nonDegenerate) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idx = 0; idx < nActiveConstr; idx++) {
        i = workspace.size(0) * workspace.size(1);
        i1 = idx + 1;
        if ((i1 < 1) || (i1 > i)) {
          emlrtDynamicBoundsCheckR2012b(i1, 1, i, &kc_emlrtBCI, sp);
        }

        i = workspace.size(0);
        i1 = solution->lambda.size(0);
        loop_ub = idx + 1;
        if ((loop_ub < 1) || (loop_ub > i1)) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 1, i1, &kc_emlrtBCI, sp);
        }

        solution->lambda[loop_ub - 1] = -workspace[idx % i * workspace.size(1) +
          idx / i];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (compute_lambda.cpp)

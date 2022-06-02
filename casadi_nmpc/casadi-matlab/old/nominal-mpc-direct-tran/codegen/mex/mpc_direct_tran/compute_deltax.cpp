//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  compute_deltax.cpp
//
//  Code generation for function 'compute_deltax'
//


// Include files
#include "compute_deltax.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "factor.h"
#include "fullColLDL2_.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "partialColLDL3_.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xcopy.h"
#include "xgemm.h"
#include "xgemv.h"

// Variable Definitions
static emlrtRSInfo wc_emlrtRSI = { 44, // lineNo
  "xscal",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xscal.m"// pathName 
};

static emlrtRSInfo ef_emlrtRSI = { 1,  // lineNo
  "compute_deltax",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/compute_deltax.p"// pathName 
};

static emlrtRSInfo lf_emlrtRSI = { 1,  // lineNo
  "computeProjectedHessian",           // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/computeProjectedHessian.p"// pathName 
};

static emlrtRSInfo mf_emlrtRSI = { 1,  // lineNo
  "computeProjectedHessian_regularized",// fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+fminconsqp/+step/+relaxed/computeProjectedHessian_regularized.p"// pathName 
};

static emlrtBCInfo gc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "compute_deltax",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/compute_deltax.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo hc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "computeProjectedHessian_regularized",// fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+fminconsqp/+step/+relaxed/computeProjectedHessian_regularized.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo oc_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "compute_deltax",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/compute_deltax.p"// pName 
};

static emlrtRTEInfo rc_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "solve",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/solve.p"// pName 
};

// Function Definitions
void compute_deltax(const emlrtStack *sp, const coder::array<real_T, 2U> &H,
                    b_struct_T *solution, c_struct_T *memspace, const e_struct_T
                    *qrmanager, f_struct_T *cholmanager, const d_struct_T
                    *objective)
{
  int32_T nVar;
  int32_T mNull_tmp;
  int32_T k;
  int32_T i1;
  real_T SCALED_REG_PRIMAL;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  coder::array<real_T, 2U> r;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t lda_t;
  ptrdiff_t incy_t;
  char_T TRANSA1;
  char_T UPLO1;
  coder::array<real_T, 2U> r1;
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
  nVar = qrmanager->mrows;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    boolean_T overflow;
    st.site = &ef_emlrtRSI;
    overflow = ((1 <= qrmanager->mrows) && (qrmanager->mrows > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (int32_T idx = 0; idx < nVar; idx++) {
      int32_T i;
      i = solution->searchDir.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &gc_emlrtBCI, sp);
      }

      solution->searchDir[i1 - 1] = 0.0;
    }
  } else {
    boolean_T overflow;
    int32_T idx;
    int32_T i;
    st.site = &ef_emlrtRSI;
    overflow = ((1 <= qrmanager->mrows) && (qrmanager->mrows > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < nVar; idx++) {
      k = idx + 1;
      i = objective->grad.size(0);
      if ((k < 1) || (k > i)) {
        emlrtDynamicBoundsCheckR2012b(k, 1, i, &gc_emlrtBCI, sp);
      }

      i = solution->searchDir.size(0);
      if (k > i) {
        emlrtDynamicBoundsCheckR2012b(k, 1, i, &gc_emlrtBCI, sp);
      }

      solution->searchDir[k - 1] = -objective->grad[k - 1];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        st.site = &ef_emlrtRSI;
        factor(&st, cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          st.site = &ef_emlrtRSI;
          solve(&st, cholmanager, solution->searchDir);
        }
        break;

       case 4:
        st.site = &ef_emlrtRSI;
        factor(&st, cholmanager, H, objective->nvar, objective->nvar);
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          st.site = &ef_emlrtRSI;
          solve(&st, cholmanager, solution->searchDir);
          st.site = &ef_emlrtRSI;
          k = qrmanager->mrows - objective->nvar;
          if (k >= 1) {
            b_st.site = &wc_emlrtRSI;
            b_st.site = &xc_emlrtRSI;
            SCALED_REG_PRIMAL = rtInf;
            n_t = (ptrdiff_t)k;
            incx_t = (ptrdiff_t)1;
            dscal(&n_t, &SCALED_REG_PRIMAL, &solution->searchDir[objective->nvar],
                  &incx_t);
          }
        }
        break;
      }
    } else {
      int32_T nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager->ldq * qrmanager->ncols + 1;
      switch (objective->objtype) {
       case 5:
        {
          st.site = &ef_emlrtRSI;
          if (mNull_tmp > 2147483646) {
            b_st.site = &u_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }

          for (idx = 0; idx < mNull_tmp; idx++) {
            int32_T idx_col;
            i = memspace->workspace_double.size(0) *
              memspace->workspace_double.size(1);
            i1 = idx + 1;
            if (i1 > i) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, i, &gc_emlrtBCI, sp);
            }

            i = memspace->workspace_double.size(0);
            i1 = qrmanager->Q.size(1);
            idx_col = (qrmanager->ncols + idx) + 1;
            if ((idx_col < 1) || (idx_col > i1)) {
              emlrtDynamicBoundsCheckR2012b(idx_col, 1, i1, &gc_emlrtBCI, sp);
            }

            i1 = qrmanager->Q.size(0);
            if ((nVar < 1) || (nVar > i1)) {
              emlrtDynamicBoundsCheckR2012b(nVar, 1, i1, &gc_emlrtBCI, sp);
            }

            memspace->workspace_double[idx % i * memspace->workspace_double.size
              (1) + idx / i] = -qrmanager->Q[(idx_col + qrmanager->Q.size(1) *
              (nVar - 1)) - 1];
          }

          st.site = &ef_emlrtRSI;
          e_xgemv(&st, qrmanager->mrows, mNull_tmp, qrmanager->Q,
                  nullStartIdx_tmp, qrmanager->ldq, memspace->workspace_double,
                  solution->searchDir);
        }
        break;

       default:
        {
          int32_T nVars;
          int32_T idx_col;
          switch (objective->objtype) {
           case 3:
            st.site = &ef_emlrtRSI;
            nVar = memspace->workspace_double.size(0);
            b_st.site = &lf_emlrtRSI;
            xgemm(&b_st, qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                  qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                  qrmanager->ldq, memspace->workspace_double,
                  memspace->workspace_double.size(0));
            b_st.site = &lf_emlrtRSI;
            b_xgemm(&b_st, mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                    nullStartIdx_tmp, qrmanager->ldq, memspace->workspace_double,
                    nVar, cholmanager->FMat, cholmanager->ldm);
            break;

           default:
            k = objective->nvar + 1;
            st.site = &ef_emlrtRSI;
            nVars = qrmanager->mrows;
            nVar = memspace->workspace_double.size(0);
            b_st.site = &mf_emlrtRSI;
            xgemm(&b_st, objective->nvar, mNull_tmp, objective->nvar, H,
                  objective->nvar, qrmanager->Q, nullStartIdx_tmp,
                  qrmanager->ldq, memspace->workspace_double,
                  memspace->workspace_double.size(0));
            b_st.site = &mf_emlrtRSI;
            if (mNull_tmp > 2147483646) {
              c_st.site = &u_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }

            overflow = ((k <= nVars) && (nVars > 2147483646));
            for (idx = 0; idx < mNull_tmp; idx++) {
              idx_col = idx + 1;
              b_st.site = &mf_emlrtRSI;
              if (overflow) {
                c_st.site = &u_emlrtRSI;
                check_forloop_overflow_error(&c_st);
              }

              for (int32_T idx_row = k; idx_row <= nVars; idx_row++) {
                i = qrmanager->Q.size(1);
                i1 = idx_col + qrmanager->ncols;
                if ((i1 < 1) || (i1 > i)) {
                  emlrtDynamicBoundsCheckR2012b(i1, 1, i, &hc_emlrtBCI, &st);
                }

                i = qrmanager->Q.size(0);
                if ((idx_row < 1) || (idx_row > i)) {
                  emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &hc_emlrtBCI, &st);
                }

                i = memspace->workspace_double.size(1);
                if (idx_col > i) {
                  emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &hc_emlrtBCI, &st);
                }

                i = memspace->workspace_double.size(0);
                if (idx_row > i) {
                  emlrtDynamicBoundsCheckR2012b(idx_row, 1, i, &hc_emlrtBCI, &st);
                }

                memspace->workspace_double[(idx_col +
                  memspace->workspace_double.size(1) * (idx_row - 1)) - 1] = 0.0
                  * qrmanager->Q[(i1 + qrmanager->Q.size(1) * (idx_row - 1)) - 1];
              }
            }

            b_st.site = &mf_emlrtRSI;
            b_xgemm(&b_st, mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                    nullStartIdx_tmp, qrmanager->ldq, memspace->workspace_double,
                    nVar, cholmanager->FMat, cholmanager->ldm);
            break;
          }

          st.site = &ef_emlrtRSI;
          SCALED_REG_PRIMAL = 1.4901161193847656E-6 * static_cast<real_T>
            (mNull_tmp);
          idx_col = cholmanager->ldm + 1;
          cholmanager->ndims = mNull_tmp;
          b_st.site = &ff_emlrtRSI;
          n_t = (ptrdiff_t)mNull_tmp;
          incx_t = (ptrdiff_t)(cholmanager->ldm + 1);
          r.set_size((&ib_emlrtRTEI), (&b_st), cholmanager->FMat.size(1),
                     cholmanager->FMat.size(0));
          k = cholmanager->FMat.size(1);
          for (i = 0; i < k; i++) {
            nVar = cholmanager->FMat.size(0);
            for (i1 = 0; i1 < nVar; i1++) {
              r[i1 + r.size(1) * i] = cholmanager->FMat[i +
                cholmanager->FMat.size(1) * i1];
            }
          }

          n_t = idamax(&n_t, &(r.data())[0], &incx_t);
          i = cholmanager->FMat.size(0) * cholmanager->FMat.size(1);
          i1 = (int32_T)n_t * (cholmanager->ldm + 1);
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &fc_emlrtBCI, &st);
          }

          i = (int32_T)n_t * (cholmanager->ldm + 1) - 1;
          i1 = cholmanager->FMat.size(0);
          cholmanager->regTol_ = muDoubleScalarMax(muDoubleScalarAbs
            (cholmanager->FMat[i % i1 * cholmanager->FMat.size(1) + i / i1]) *
            2.2204460492503131E-16, SCALED_REG_PRIMAL);
          if (mNull_tmp > 128) {
            boolean_T exitg1;
            k = 0;
            exitg1 = false;
            while ((!exitg1) && (k < mNull_tmp)) {
              nVar = idx_col * k + 1;
              nVars = mNull_tmp - k;
              if (k + 48 <= mNull_tmp) {
                b_st.site = &ff_emlrtRSI;
                partialColLDL3_(&b_st, cholmanager, nVar, nVars,
                                SCALED_REG_PRIMAL);
                k += 48;
              } else {
                b_st.site = &ff_emlrtRSI;
                fullColLDL2_(&b_st, cholmanager, nVar, nVars, SCALED_REG_PRIMAL);
                exitg1 = true;
              }
            }
          } else {
            b_st.site = &ff_emlrtRSI;
            fullColLDL2_(&b_st, cholmanager, 1, mNull_tmp, SCALED_REG_PRIMAL);
          }

          if (cholmanager->ConvexCheck) {
            b_st.site = &ff_emlrtRSI;
            if (mNull_tmp > 2147483646) {
              c_st.site = &u_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }

            idx = 0;
            int32_T exitg2;
            do {
              exitg2 = 0;
              if (idx <= mNull_tmp - 1) {
                i = cholmanager->FMat.size(1);
                i1 = idx + 1;
                if (i1 > i) {
                  emlrtDynamicBoundsCheckR2012b(i1, 1, i, &fc_emlrtBCI, &st);
                }

                i = cholmanager->FMat.size(0);
                idx_col = idx + 1;
                if (idx_col > i) {
                  emlrtDynamicBoundsCheckR2012b(idx_col, 1, i, &fc_emlrtBCI, &st);
                }

                if (cholmanager->FMat[(i1 + cholmanager->FMat.size(1) * (idx_col
                      - 1)) - 1] <= 0.0) {
                  cholmanager->info = -(idx + 1);
                  exitg2 = 1;
                } else {
                  idx++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }

          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            boolean_T b;
            boolean_T b1;
            st.site = &ef_emlrtRSI;
            if (qrmanager->mrows >= 1) {
              b_st.site = &yc_emlrtRSI;
              b_st.site = &ad_emlrtRSI;
              SCALED_REG_PRIMAL = -1.0;
              beta1 = 0.0;
              TRANSA = 'T';
              m_t = (ptrdiff_t)qrmanager->mrows;
              n_t = (ptrdiff_t)mNull_tmp;
              lda_t = (ptrdiff_t)qrmanager->ldq;
              incx_t = (ptrdiff_t)1;
              incy_t = (ptrdiff_t)1;
              r.set_size((&wb_emlrtRTEI), (&b_st), qrmanager->Q.size(1),
                         qrmanager->Q.size(0));
              k = qrmanager->Q.size(1);
              for (i = 0; i < k; i++) {
                nVar = qrmanager->Q.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r[i1 + r.size(1) * i] = qrmanager->Q[i + qrmanager->Q.size(1) *
                    i1];
                }
              }

              r1.set_size((&xb_emlrtRTEI), (&b_st),
                          memspace->workspace_double.size(1),
                          memspace->workspace_double.size(0));
              k = memspace->workspace_double.size(1);
              for (i = 0; i < k; i++) {
                nVar = memspace->workspace_double.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r1[i1 + r1.size(1) * i] = memspace->workspace_double[i +
                    memspace->workspace_double.size(1) * i1];
                }
              }

              dgemv(&TRANSA, &m_t, &n_t, &SCALED_REG_PRIMAL, &r[nullStartIdx_tmp
                    - 1], &lda_t, &(((coder::array<real_T, 1U> *)
                      &objective->grad)->data())[0], &incx_t, &beta1, &(r1.data())
                    [0], &incy_t);
              memspace->workspace_double.set_size((&oc_emlrtRTEI), (&b_st),
                r1.size(1), r1.size(0));
              k = r1.size(1);
              for (i = 0; i < k; i++) {
                nVar = r1.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  memspace->workspace_double[i1 +
                    memspace->workspace_double.size(1) * i] = r1[i + r1.size(1) *
                    i1];
                }
              }
            }

            st.site = &ef_emlrtRSI;
            r.set_size((&oc_emlrtRTEI), (&st), memspace->workspace_double.size(0),
                       memspace->workspace_double.size(1));
            k = memspace->workspace_double.size(1) *
              memspace->workspace_double.size(0);
            for (i = 0; i < k; i++) {
              r[i] = memspace->workspace_double[i];
            }

            b_st.site = &if_emlrtRSI;
            if (cholmanager->ndims >= 1) {
              c_st.site = &jf_emlrtRSI;
              TRANSA = 'U';
              TRANSA1 = 'N';
              UPLO1 = 'L';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              r.set_size((&pc_emlrtRTEI), (&c_st), cholmanager->FMat.size(1),
                         cholmanager->FMat.size(0));
              k = cholmanager->FMat.size(1);
              for (i = 0; i < k; i++) {
                nVar = cholmanager->FMat.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r[i1 + r.size(1) * i] = cholmanager->FMat[i +
                    cholmanager->FMat.size(1) * i1];
                }
              }

              r1.set_size((&qc_emlrtRTEI), (&c_st),
                          memspace->workspace_double.size(1),
                          memspace->workspace_double.size(0));
              k = memspace->workspace_double.size(1);
              for (i = 0; i < k; i++) {
                nVar = memspace->workspace_double.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r1[i1 + r1.size(1) * i] = memspace->workspace_double[i +
                    memspace->workspace_double.size(1) * i1];
                }
              }

              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &(r.data())[0], &lda_t,
                    &(r1.data())[0], &incx_t);
              r.set_size((&rc_emlrtRTEI), (&c_st), r1.size(1), r1.size(0));
              k = r1.size(1);
              for (i = 0; i < k; i++) {
                nVar = r1.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r[i1 + r.size(1) * i] = r1[i + r1.size(1) * i1];
                }
              }
            }

            k = cholmanager->ndims;
            b_st.site = &if_emlrtRSI;
            overflow = ((1 <= cholmanager->ndims) && (cholmanager->ndims >
              2147483646));
            if (overflow) {
              c_st.site = &u_emlrtRSI;
              check_forloop_overflow_error(&c_st);
            }

            b = true;
            b1 = ((r.size(1) <= 0) || (r.size(0) <= 0));
            i = r.size(1) * r.size(0);
            i1 = 0;
            for (idx = 0; idx < k; idx++) {
              if (b1 || (idx >= i)) {
                i1 = 0;
                b = true;
              } else if (b) {
                b = false;
                i1 = idx % r.size(0) * r.size(1) + idx / r.size(0);
              } else {
                idx_col = r.size(1) * r.size(0) - 1;
                if (i1 > MAX_int32_T - r.size(1)) {
                  i1 = idx % r.size(0) * r.size(1) + idx / r.size(0);
                } else {
                  i1 += r.size(1);
                  if (i1 > idx_col) {
                    i1 -= idx_col;
                  }
                }
              }

              idx_col = r.size(0) * r.size(1);
              nVars = idx + 1;
              if ((nVars < 1) || (nVars > idx_col)) {
                emlrtDynamicBoundsCheckR2012b(nVars, 1, idx_col, &ec_emlrtBCI,
                  &st);
              }

              idx_col = r.size(0) * r.size(1);
              nVars = idx + 1;
              if ((nVars < 1) || (nVars > idx_col)) {
                emlrtDynamicBoundsCheckR2012b(nVars, 1, idx_col, &ec_emlrtBCI,
                  &st);
              }

              idx_col = cholmanager->FMat.size(1);
              nVars = idx + 1;
              if ((nVars < 1) || (nVars > idx_col)) {
                emlrtDynamicBoundsCheckR2012b(nVars, 1, idx_col, &ec_emlrtBCI,
                  &st);
              }

              idx_col = cholmanager->FMat.size(0);
              nVar = idx + 1;
              if ((nVar < 1) || (nVar > idx_col)) {
                emlrtDynamicBoundsCheckR2012b(nVar, 1, idx_col, &ec_emlrtBCI,
                  &st);
              }

              r[i1] = r[i1] / cholmanager->FMat[(nVars + cholmanager->FMat.size
                (1) * (nVar - 1)) - 1];
            }

            b_st.site = &if_emlrtRSI;
            if (cholmanager->ndims >= 1) {
              c_st.site = &jf_emlrtRSI;
              TRANSA = 'U';
              TRANSA1 = 'T';
              UPLO1 = 'L';
              n_t = (ptrdiff_t)cholmanager->ndims;
              lda_t = (ptrdiff_t)cholmanager->ldm;
              incx_t = (ptrdiff_t)1;
              r1.set_size((&pc_emlrtRTEI), (&c_st), cholmanager->FMat.size(1),
                          cholmanager->FMat.size(0));
              k = cholmanager->FMat.size(1);
              for (i = 0; i < k; i++) {
                nVar = cholmanager->FMat.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r1[i1 + r1.size(1) * i] = cholmanager->FMat[i +
                    cholmanager->FMat.size(1) * i1];
                }
              }

              r2.set_size((&qc_emlrtRTEI), (&c_st), r.size(1), r.size(0));
              k = r.size(1);
              for (i = 0; i < k; i++) {
                nVar = r.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r2[i1 + r2.size(1) * i] = r[i + r.size(1) * i1];
                }
              }

              dtrsv(&UPLO1, &TRANSA1, &TRANSA, &n_t, &(r1.data())[0], &lda_t,
                    &(r2.data())[0], &incx_t);
              r.set_size((&rc_emlrtRTEI), (&c_st), r2.size(1), r2.size(0));
              k = r2.size(1);
              for (i = 0; i < k; i++) {
                nVar = r2.size(0);
                for (i1 = 0; i1 < nVar; i1++) {
                  r[i1 + r.size(1) * i] = r2[i + r2.size(1) * i1];
                }
              }
            }

            memspace->workspace_double.set_size((&oc_emlrtRTEI), sp, r.size(0),
              r.size(1));
            k = r.size(1) * r.size(0);
            for (i = 0; i < k; i++) {
              memspace->workspace_double[i] = r[i];
            }

            st.site = &ef_emlrtRSI;
            e_xgemv(&st, qrmanager->mrows, mNull_tmp, qrmanager->Q,
                    nullStartIdx_tmp, qrmanager->ldq, r, solution->searchDir);
          }
        }
        break;
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (compute_deltax.cpp)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  iterate.cpp
//
//  Code generation for function 'iterate'
//


// Include files
#include "iterate.h"
#include "addBoundToActiveSetMatrix_.h"
#include "blas.h"
#include "checkStoppingAndUpdateFval.h"
#include "computeFval.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "compute_lambda.h"
#include "deleteColMoveEnd.h"
#include "eml_int_forloop_overflow_check.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "maxConstraintViolation.h"
#include "moveConstraint_.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "ratiotest.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xrot.h"

// Variable Definitions
static emlrtRSInfo vc_emlrtRSI = { 35, // lineNo
  "xnrm2",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xnrm2.m"// pathName 
};

static emlrtRSInfo te_emlrtRSI = { 1,  // lineNo
  "iterate",                           // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/iterate.p"// pathName 
};

static emlrtRSInfo xe_emlrtRSI = { 1,  // lineNo
  "squareQ_appendCol",                 // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/squareQ_appendCol.p"// pathName 
};

static emlrtRSInfo of_emlrtRSI = { 1,  // lineNo
  "find_neg_lambda",                   // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/find_neg_lambda.p"// pathName 
};

static emlrtRSInfo rf_emlrtRSI = { 1,  // lineNo
  "addAineqConstr",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addAineqConstr.p"// pathName 
};

static emlrtRSInfo sf_emlrtRSI = { 1,  // lineNo
  "addLBConstr",                       // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addLBConstr.p"// pathName 
};

static emlrtRSInfo uf_emlrtRSI = { 1,  // lineNo
  "addUBConstr",                       // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addUBConstr.p"// pathName 
};

static emlrtRSInfo vf_emlrtRSI = { 1,  // lineNo
  "checkUnboundedOrIllPosed",          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/checkUnboundedOrIllPosed.p"// pathName 
};

static emlrtBCInfo vb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "iterate",                           // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/iterate.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo wb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "squareQ_appendCol",                 // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/squareQ_appendCol.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo yb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "find_neg_lambda",                   // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/find_neg_lambda.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo ac_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "addAineqConstr",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addAineqConstr.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo mc_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "iterate",                           // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/iterate.p"// pName 
};

// Function Definitions
void iterate(const emlrtStack *sp, const coder::array<real_T, 2U> &H, const
             coder::array<real_T, 1U> &f, b_struct_T *solution, c_struct_T
             *memspace, g_struct_T *workingset, e_struct_T *qrmanager,
             f_struct_T *cholmanager, d_struct_T *objective, real_T
             options_ObjectiveLimit, real_T options_StepTolerance, int32_T
             runTimeOptions_MaxIterations, real_T
             c_runTimeOptions_ConstrRelTolFa, real_T
             runTimeOptions_ProbRelTolFactor, boolean_T
             runTimeOptions_RemainFeasible)
{
  boolean_T subProblemChanged;
  boolean_T updateFval;
  int32_T activeSetChangeID;
  int32_T TYPE;
  real_T tolDelta;
  int32_T nVar;
  int32_T globalActiveConstrIdx;
  int32_T Qk0;
  real_T normDelta;
  real_T minLambda;
  int32_T localActiveConstrIdx;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  char_T TRANSA;
  ptrdiff_t m_t;
  real_T a;
  boolean_T newBlocking;
  ptrdiff_t lda_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  coder::array<real_T, 2U> r2;
  real_T s;
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
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  st.site = &te_emlrtRSI;
  computeGrad_StoreHx(&st, objective, H, f, solution->xstar);
  st.site = &te_emlrtRSI;
  solution->fstar = computeFval_ReuseHx(&st, objective,
    memspace->workspace_double, f, solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  st.site = &te_emlrtRSI;
  c_xcopy(&st, workingset->mConstrMax, solution->lambda);
  int32_T exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      boolean_T guard1 = false;
      int32_T i;
      int32_T idx;
      int32_T i1;
      guard1 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          st.site = &te_emlrtRSI;
          Qk0 = qrmanager->ncols + 1;
          qrmanager->minRowCol = muIntScalarMin_sint32(qrmanager->mrows, Qk0);
          b_st.site = &xe_emlrtRSI;
          if (qrmanager->mrows >= 1) {
            c_st.site = &ad_emlrtRSI;
            normDelta = 1.0;
            minLambda = 0.0;
            TRANSA = 'T';
            m_t = (ptrdiff_t)qrmanager->mrows;
            n_t = (ptrdiff_t)qrmanager->mrows;
            lda_t = (ptrdiff_t)qrmanager->ldq;
            incx_t = (ptrdiff_t)1;
            incy_t = (ptrdiff_t)1;
            r.set_size((&wb_emlrtRTEI), (&c_st), qrmanager->Q.size(1),
                       qrmanager->Q.size(0));
            Qk0 = qrmanager->Q.size(1);
            for (i = 0; i < Qk0; i++) {
              localActiveConstrIdx = qrmanager->Q.size(0);
              for (i1 = 0; i1 < localActiveConstrIdx; i1++) {
                r[i1 + r.size(1) * i] = qrmanager->Q[i + qrmanager->Q.size(1) *
                  i1];
              }
            }

            r1.set_size((&jc_emlrtRTEI), (&c_st), workingset->ATwset.size(1),
                        workingset->ATwset.size(0));
            Qk0 = workingset->ATwset.size(1);
            for (i = 0; i < Qk0; i++) {
              localActiveConstrIdx = workingset->ATwset.size(0);
              for (i1 = 0; i1 < localActiveConstrIdx; i1++) {
                r1[i1 + r1.size(1) * i] = workingset->ATwset[i +
                  workingset->ATwset.size(1) * i1];
              }
            }

            r2.set_size((&xb_emlrtRTEI), (&c_st), qrmanager->QR.size(1),
                        qrmanager->QR.size(0));
            Qk0 = qrmanager->QR.size(1);
            for (i = 0; i < Qk0; i++) {
              localActiveConstrIdx = qrmanager->QR.size(0);
              for (i1 = 0; i1 < localActiveConstrIdx; i1++) {
                r2[i1 + r2.size(1) * i] = qrmanager->QR[i + qrmanager->QR.size(1)
                  * i1];
              }
            }

            dgemv(&TRANSA, &m_t, &n_t, &normDelta, &(r.data())[0], &lda_t,
                  &r1[workingset->ldA * (workingset->nActiveConstr - 1)],
                  &incx_t, &minLambda, &r2[qrmanager->ldq * qrmanager->ncols],
                  &incy_t);
            qrmanager->QR.set_size((&mc_emlrtRTEI), (&c_st), r2.size(1), r2.size
              (0));
            Qk0 = r2.size(1);
            for (i = 0; i < Qk0; i++) {
              localActiveConstrIdx = r2.size(0);
              for (i1 = 0; i1 < localActiveConstrIdx; i1++) {
                qrmanager->QR[i1 + qrmanager->QR.size(1) * i] = r2[i + r2.size(1)
                  * i1];
              }
            }
          }

          qrmanager->ncols++;
          i = qrmanager->jpvt.size(0);
          if ((qrmanager->ncols < 1) || (qrmanager->ncols > i)) {
            emlrtDynamicBoundsCheckR2012b(qrmanager->ncols, 1, i, &wb_emlrtBCI,
              &st);
          }

          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows; idx > qrmanager->ncols; idx--) {
            b_st.site = &xe_emlrtRSI;
            i = qrmanager->QR.size(1);
            if ((qrmanager->ncols < 1) || (qrmanager->ncols > i)) {
              emlrtDynamicBoundsCheckR2012b(qrmanager->ncols, 1, i, &wb_emlrtBCI,
                &b_st);
            }

            i = qrmanager->QR.size(0);
            i1 = idx - 1;
            if ((i1 < 1) || (i1 > i)) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, i, &wb_emlrtBCI, &b_st);
            }

            a = qrmanager->QR[(qrmanager->ncols + qrmanager->QR.size(1) * (i1 -
              1)) - 1];
            i = qrmanager->QR.size(1);
            if ((qrmanager->ncols < 1) || (qrmanager->ncols > i)) {
              emlrtDynamicBoundsCheckR2012b(qrmanager->ncols, 1, i, &wb_emlrtBCI,
                &b_st);
            }

            i = qrmanager->QR.size(0);
            if ((idx < 1) || (idx > i)) {
              emlrtDynamicBoundsCheckR2012b(idx, 1, i, &wb_emlrtBCI, &b_st);
            }

            normDelta = qrmanager->QR[(qrmanager->ncols + qrmanager->QR.size(1) *
              (idx - 1)) - 1];
            minLambda = 0.0;
            s = 0.0;
            drotg(&a, &normDelta, &minLambda, &s);
            i = qrmanager->QR.size(1);
            if ((qrmanager->ncols < 1) || (qrmanager->ncols > i)) {
              emlrtDynamicBoundsCheckR2012b(qrmanager->ncols, 1, i, &wb_emlrtBCI,
                &b_st);
            }

            i = qrmanager->QR.size(0);
            i1 = idx - 1;
            if ((i1 < 1) || (i1 > i)) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, i, &wb_emlrtBCI, &b_st);
            }

            qrmanager->QR[(qrmanager->ncols + qrmanager->QR.size(1) * (i1 - 1))
              - 1] = a;
            i = qrmanager->QR.size(1);
            if ((qrmanager->ncols < 1) || (qrmanager->ncols > i)) {
              emlrtDynamicBoundsCheckR2012b(qrmanager->ncols, 1, i, &wb_emlrtBCI,
                &b_st);
            }

            i = qrmanager->QR.size(0);
            if (idx > i) {
              emlrtDynamicBoundsCheckR2012b(idx, 1, i, &wb_emlrtBCI, &b_st);
            }

            qrmanager->QR[(qrmanager->ncols + qrmanager->QR.size(1) * (idx - 1))
              - 1] = normDelta;
            Qk0 = qrmanager->ldq * (idx - 2) + 1;
            b_st.site = &xe_emlrtRSI;
            xrot(&b_st, qrmanager->mrows, qrmanager->Q, Qk0, qrmanager->ldq +
                 Qk0, minLambda, s);
          }
          break;

         case -1:
          st.site = &te_emlrtRSI;
          deleteColMoveEnd(&st, qrmanager, globalActiveConstrIdx);
          break;

         default:
          st.site = &te_emlrtRSI;
          factorQR(&st, qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr);
          st.site = &te_emlrtRSI;
          b_st.site = &dd_emlrtRSI;
          computeQ_(&b_st, qrmanager, qrmanager->mrows);
          break;
        }

        st.site = &te_emlrtRSI;
        compute_deltax(&st, H, solution, memspace, qrmanager, cholmanager,
                       objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          st.site = &te_emlrtRSI;
          if (nVar < 1) {
            normDelta = 0.0;
          } else {
            b_st.site = &vc_emlrtRSI;
            n_t = (ptrdiff_t)nVar;
            incx_t = (ptrdiff_t)1;
            normDelta = dnrm2(&n_t, &(solution->searchDir.data())[0], &incx_t);
          }

          if ((normDelta < options_StepTolerance) || (workingset->nActiveConstr >=
               nVar)) {
            guard1 = true;
          } else {
            updateFval = (TYPE == 5);
            if (updateFval || runTimeOptions_RemainFeasible) {
              st.site = &te_emlrtRSI;
              feasibleratiotest(&st, solution->xstar, solution->searchDir,
                                memspace->workspace_double, workingset->nVar,
                                workingset->ldA, workingset->Aineq,
                                workingset->bineq, workingset->lb,
                                workingset->indexLB, workingset->sizes,
                                workingset->isActiveIdx,
                                workingset->isActiveConstr, workingset->nWConstr,
                                updateFval, &a, &newBlocking, &Qk0,
                                &localActiveConstrIdx);
            } else {
              st.site = &te_emlrtRSI;
              ratiotest(&st, solution->xstar, solution->searchDir,
                        memspace->workspace_double, workingset->nVar,
                        workingset->ldA, workingset->Aineq, workingset->bineq,
                        workingset->lb, workingset->indexLB, workingset->sizes,
                        workingset->isActiveIdx, workingset->isActiveConstr,
                        workingset->nWConstr, &tolDelta, &a, &newBlocking, &Qk0,
                        &localActiveConstrIdx);
            }

            if (newBlocking) {
              switch (Qk0) {
               case 3:
                st.site = &te_emlrtRSI;
                b_st.site = &rf_emlrtRSI;
                workingset->nWConstr[2]++;
                i = workingset->isActiveConstr.size(0);
                i1 = (workingset->isActiveIdx[2] + localActiveConstrIdx) - 1;
                if ((i1 < 1) || (i1 > i)) {
                  emlrtDynamicBoundsCheckR2012b(i1, 1, i, &xb_emlrtBCI, &b_st);
                }

                workingset->isActiveConstr[i1 - 1] = true;
                workingset->nActiveConstr++;
                i = workingset->Wid.size(0);
                if ((workingset->nActiveConstr < 1) ||
                    (workingset->nActiveConstr > i)) {
                  emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                    &xb_emlrtBCI, &b_st);
                }

                workingset->Wid[workingset->nActiveConstr - 1] = 3;
                i = workingset->Wlocalidx.size(0);
                if ((workingset->nActiveConstr < 1) ||
                    (workingset->nActiveConstr > i)) {
                  emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                    &xb_emlrtBCI, &b_st);
                }

                workingset->Wlocalidx[workingset->nActiveConstr - 1] =
                  localActiveConstrIdx;
                b_st.site = &rf_emlrtRSI;
                b_xcopy(&b_st, workingset->nVar, workingset->Aineq,
                        workingset->ldA * (localActiveConstrIdx - 1) + 1,
                        workingset->ATwset, workingset->ldA *
                        (workingset->nActiveConstr - 1) + 1);
                i = workingset->bineq.size(0) * workingset->bineq.size(1);
                if ((localActiveConstrIdx < 1) || (localActiveConstrIdx > i)) {
                  emlrtDynamicBoundsCheckR2012b(localActiveConstrIdx, 1, i,
                    &ac_emlrtBCI, &st);
                }

                i = workingset->bwset.size(0);
                if ((workingset->nActiveConstr < 1) ||
                    (workingset->nActiveConstr > i)) {
                  emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
                    &ac_emlrtBCI, &st);
                }

                workingset->bwset[workingset->nActiveConstr - 1] =
                  workingset->bineq[localActiveConstrIdx - 1];
                break;

               case 4:
                st.site = &te_emlrtRSI;
                b_st.site = &sf_emlrtRSI;
                addBoundToActiveSetMatrix_(&b_st, workingset, 4,
                  localActiveConstrIdx);
                break;

               default:
                st.site = &te_emlrtRSI;
                b_st.site = &uf_emlrtRSI;
                addBoundToActiveSetMatrix_(&b_st, workingset, 5,
                  localActiveConstrIdx);
                break;
              }

              activeSetChangeID = 1;
            } else {
              st.site = &te_emlrtRSI;
              if (objective->objtype == 5) {
                b_st.site = &vf_emlrtRSI;
                if (objective->nvar < 1) {
                  normDelta = 0.0;
                } else {
                  n_t = (ptrdiff_t)objective->nvar;
                  incx_t = (ptrdiff_t)1;
                  normDelta = dnrm2(&n_t, &(solution->searchDir.data())[0],
                                    &incx_t);
                }

                if (normDelta > 100.0 * static_cast<real_T>(objective->nvar) *
                    1.4901161193847656E-8) {
                  solution->state = 3;
                } else {
                  solution->state = 4;
                }
              }

              subProblemChanged = false;
              if (workingset->nActiveConstr == 0) {
                solution->state = 1;
              }
            }

            st.site = &te_emlrtRSI;
            if (nVar >= 1) {
              b_st.site = &ke_emlrtRSI;
              b_st.site = &je_emlrtRSI;
              n_t = (ptrdiff_t)nVar;
              incx_t = (ptrdiff_t)1;
              incy_t = (ptrdiff_t)1;
              daxpy(&n_t, &a, &(solution->searchDir.data())[0], &incx_t,
                    &(solution->xstar.data())[0], &incy_t);
            }

            st.site = &te_emlrtRSI;
            computeGrad_StoreHx(&st, objective, H, f, solution->xstar);
            updateFval = true;
            st.site = &te_emlrtRSI;
            checkStoppingAndUpdateFval(&st, &activeSetChangeID, f, solution,
              memspace, objective, workingset, qrmanager, options_ObjectiveLimit,
              runTimeOptions_MaxIterations, c_runTimeOptions_ConstrRelTolFa,
              updateFval);
          }
        }
      } else {
        st.site = &te_emlrtRSI;
        c_xcopy(&st, nVar, solution->searchDir);
        guard1 = true;
      }

      if (guard1) {
        st.site = &te_emlrtRSI;
        compute_lambda(&st, memspace->workspace_double, solution, objective,
                       qrmanager);
        if (solution->state != -7) {
          int32_T idxMinLambda;
          st.site = &te_emlrtRSI;
          idxMinLambda = 0;
          minLambda = 0.0 * runTimeOptions_ProbRelTolFactor * static_cast<real_T>
            (TYPE != 5);
          Qk0 = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          localActiveConstrIdx = workingset->nActiveConstr;
          b_st.site = &of_emlrtRSI;
          updateFval = ((Qk0 <= workingset->nActiveConstr) &&
                        (workingset->nActiveConstr > 2147483646));
          if (updateFval) {
            c_st.site = &u_emlrtRSI;
            check_forloop_overflow_error(&c_st);
          }

          for (idx = Qk0; idx <= localActiveConstrIdx; idx++) {
            i = solution->lambda.size(0);
            if ((idx < 1) || (idx > i)) {
              emlrtDynamicBoundsCheckR2012b(idx, 1, i, &yb_emlrtBCI, &st);
            }

            normDelta = solution->lambda[idx - 1];
            if (normDelta < minLambda) {
              i = solution->lambda.size(0);
              if (idx > i) {
                emlrtDynamicBoundsCheckR2012b(idx, 1, i, &yb_emlrtBCI, &st);
              }

              minLambda = normDelta;
              idxMinLambda = idx;
            }
          }

          if (idxMinLambda == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = idxMinLambda;
            subProblemChanged = true;
            i = workingset->Wid.size(0);
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &vb_emlrtBCI, sp);
            }

            i = workingset->Wlocalidx.size(0);
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &vb_emlrtBCI, sp);
            }

            st.site = &te_emlrtRSI;
            i = workingset->Wid.size(0);
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &x_emlrtBCI, &st);
            }

            Qk0 = workingset->Wid[idxMinLambda - 1];
            i = workingset->Wlocalidx.size(0);
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &x_emlrtBCI, &st);
            }

            if ((Qk0 < 1) || (Qk0 > 6)) {
              emlrtDynamicBoundsCheckR2012b(workingset->Wid[idxMinLambda - 1], 1,
                6, &w_emlrtBCI, &st);
            }

            i = workingset->isActiveConstr.size(0);
            i1 = (workingset->isActiveIdx[Qk0 - 1] + workingset->
                  Wlocalidx[idxMinLambda - 1]) - 1;
            if ((i1 < 1) || (i1 > i)) {
              emlrtDynamicBoundsCheckR2012b(i1, 1, i, &x_emlrtBCI, &st);
            }

            workingset->isActiveConstr[i1 - 1] = false;
            b_st.site = &od_emlrtRSI;
            moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                            idxMinLambda);
            workingset->nActiveConstr--;
            if (Qk0 > 5) {
              emlrtDynamicBoundsCheckR2012b(6, 1, 5, &fb_emlrtBCI, &st);
            }

            workingset->nWConstr[Qk0 - 1]--;
            i = solution->lambda.size(0);
            if (idxMinLambda > i) {
              emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &vb_emlrtBCI, sp);
            }

            solution->lambda[idxMinLambda - 1] = 0.0;
          }
        } else {
          int32_T idxMinLambda;
          idxMinLambda = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          i = workingset->Wid.size(0);
          if ((workingset->nActiveConstr < 1) || (workingset->nActiveConstr > i))
          {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
              &vb_emlrtBCI, sp);
          }

          i = workingset->Wlocalidx.size(0);
          if ((workingset->nActiveConstr < 1) || (workingset->nActiveConstr > i))
          {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
              &vb_emlrtBCI, sp);
          }

          st.site = &te_emlrtRSI;
          i = workingset->Wid.size(0);
          if ((workingset->nActiveConstr < 1) || (workingset->nActiveConstr > i))
          {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
              &x_emlrtBCI, &st);
          }

          Qk0 = workingset->nActiveConstr - 1;
          localActiveConstrIdx = workingset->Wid[Qk0];
          i = workingset->Wlocalidx.size(0);
          if ((workingset->nActiveConstr < 1) || (workingset->nActiveConstr > i))
          {
            emlrtDynamicBoundsCheckR2012b(workingset->nActiveConstr, 1, i,
              &x_emlrtBCI, &st);
          }

          if ((workingset->Wid[Qk0] < 1) || (workingset->Wid[Qk0] > 6)) {
            emlrtDynamicBoundsCheckR2012b(workingset->Wid
              [workingset->nActiveConstr - 1], 1, 6, &w_emlrtBCI, &st);
          }

          i = workingset->isActiveConstr.size(0);
          i1 = (workingset->isActiveIdx[workingset->Wid[Qk0] - 1] +
                workingset->Wlocalidx[Qk0]) - 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &x_emlrtBCI, &st);
          }

          workingset->isActiveConstr[i1 - 1] = false;
          b_st.site = &od_emlrtRSI;
          moveConstraint_(&b_st, workingset, workingset->nActiveConstr,
                          workingset->nActiveConstr);
          workingset->nActiveConstr--;
          if ((localActiveConstrIdx < 1) || (localActiveConstrIdx > 5)) {
            emlrtDynamicBoundsCheckR2012b(localActiveConstrIdx, 1, 5,
              &fb_emlrtBCI, &st);
          }

          workingset->nWConstr[localActiveConstrIdx - 1]--;
          i = solution->lambda.size(0);
          if ((idxMinLambda < 1) || (idxMinLambda > i)) {
            emlrtDynamicBoundsCheckR2012b(idxMinLambda, 1, i, &vb_emlrtBCI, sp);
          }

          solution->lambda[idxMinLambda - 1] = 0.0;
        }

        updateFval = false;
        st.site = &te_emlrtRSI;
        checkStoppingAndUpdateFval(&st, &activeSetChangeID, f, solution,
          memspace, objective, workingset, qrmanager, options_ObjectiveLimit,
          runTimeOptions_MaxIterations, c_runTimeOptions_ConstrRelTolFa,
          updateFval);
      }
    } else {
      if (!updateFval) {
        st.site = &te_emlrtRSI;
        solution->fstar = computeFval_ReuseHx(&st, objective,
          memspace->workspace_double, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (iterate.cpp)

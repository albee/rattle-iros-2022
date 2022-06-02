//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  driver.cpp
//
//  Code generation for function 'driver'
//


// Include files
#include "driver.h"
#include "PresolveWorkingSet.h"
#include "blas.h"
#include "computeFirstOrderOpt.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "phaseone.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "validateattributes.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo yb_emlrtRSI = { 1,  // lineNo
  "snap_bounds",                       // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/snap_bounds.p"// pathName 
};

static emlrtRSInfo bc_emlrtRSI = { 1,  // lineNo
  "driver",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/driver.p"// pathName 
};

static emlrtBCInfo v_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "snap_bounds",                       // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/snap_bounds.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void driver(const emlrtStack *sp, const coder::array<real_T, 2U> &H, const coder::
            array<real_T, 1U> &f, b_struct_T *solution, c_struct_T *memspace,
            g_struct_T *workingset, e_struct_T *qrmanager, f_struct_T
            *cholmanager, d_struct_T *objective, struct_T runTimeOptions)
{
  int32_T nVar;
  int32_T b;
  boolean_T overflow;
  h_struct_T options;
  static const char_T b_cv[128] = { '\x00', '\x01', '\x02', '\x03', '\x04',
    '\x05', '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d',
    '\x0e', '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16',
    '\x17', '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ',
    '!', '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>',
    '?', '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\',
    ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k',
    'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '{', '|', '}', '~', '\x7f' };

  static const char_T cv1[8] = { 'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g' };

  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bc_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  validateattributes(&st, H);
  solution->iterations = 0;
  runTimeOptions.RemainFeasible = true;
  nVar = workingset->nVar;
  st.site = &bc_emlrtRSI;
  b_st.site = &yb_emlrtRSI;
  b = workingset->sizes[3];
  b_st.site = &yb_emlrtRSI;
  overflow = ((1 <= workingset->sizes[3]) && (workingset->sizes[3] > 2147483646));
  if (overflow) {
    c_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }

  for (int32_T idx = 0; idx < b; idx++) {
    int32_T i;
    int32_T i1;
    i = workingset->isActiveConstr.size(0);
    i1 = workingset->isActiveIdx[3] + idx;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &v_emlrtBCI, &st);
    }

    if (workingset->isActiveConstr[i1 - 1]) {
      i = workingset->indexLB.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &v_emlrtBCI, &st);
      }

      i = workingset->lb.size(0);
      if ((workingset->indexLB[idx] < 1) || (workingset->indexLB[idx] > i)) {
        emlrtDynamicBoundsCheckR2012b(workingset->indexLB[idx], 1, i,
          &v_emlrtBCI, &st);
      }

      i = solution->xstar.size(0);
      if ((workingset->indexLB[idx] < 1) || (workingset->indexLB[idx] > i)) {
        emlrtDynamicBoundsCheckR2012b(workingset->indexLB[idx], 1, i,
          &v_emlrtBCI, &st);
      }

      solution->xstar[workingset->indexLB[idx] - 1] = -workingset->lb
        [workingset->indexLB[idx] - 1];
    }
  }

  b_st.site = &yb_emlrtRSI;
  st.site = &bc_emlrtRSI;
  PresolveWorkingSet(&st, solution, memspace, workingset, qrmanager, &options);
  if (solution->state >= 0) {
    real_T maxConstr_new;
    boolean_T guard1 = false;
    solution->iterations = 0;
    st.site = &bc_emlrtRSI;
    solution->maxConstr = c_maxConstraintViolation(&st, workingset,
      solution->xstar);
    maxConstr_new = 1.0E-8 * runTimeOptions.ConstrRelTolFactor;
    guard1 = false;
    if (solution->maxConstr > maxConstr_new) {
      st.site = &bc_emlrtRSI;
      phaseone(&st, H, f, solution, memspace, workingset, qrmanager, cholmanager,
               objective, &options, &runTimeOptions);
      if (solution->state != 0) {
        st.site = &bc_emlrtRSI;
        solution->maxConstr = c_maxConstraintViolation(&st, workingset,
          solution->xstar);
        if (solution->maxConstr > maxConstr_new) {
          st.site = &bc_emlrtRSI;
          c_xcopy(&st, workingset->mConstrMax, solution->lambda);
          st.site = &bc_emlrtRSI;
          solution->fstar = computeFval(&st, objective,
            memspace->workspace_double, H, f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            st.site = &bc_emlrtRSI;
            if (nVar >= 1) {
              b_st.site = &ob_emlrtRSI;
              b_st.site = &nb_emlrtRSI;
              n_t = (ptrdiff_t)nVar;
              incx_t = (ptrdiff_t)1;
              incy_t = (ptrdiff_t)1;
              dcopy(&n_t, &(solution->xstar.data())[0], &incx_t,
                    &(solution->searchDir.data())[0], &incy_t);
            }

            st.site = &bc_emlrtRSI;
            b_PresolveWorkingSet(&st, solution, memspace, workingset, qrmanager);
            st.site = &bc_emlrtRSI;
            maxConstr_new = c_maxConstraintViolation(&st, workingset,
              solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              st.site = &bc_emlrtRSI;
              if (nVar >= 1) {
                b_st.site = &ob_emlrtRSI;
                b_st.site = &nb_emlrtRSI;
                n_t = (ptrdiff_t)nVar;
                incx_t = (ptrdiff_t)1;
                incy_t = (ptrdiff_t)1;
                dcopy(&n_t, &(solution->searchDir.data())[0], &incx_t,
                      &(solution->xstar.data())[0], &incy_t);
              }
            }
          }

          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      st.site = &bc_emlrtRSI;
      iterate(&st, H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.ObjectiveLimit, options.StepTolerance,
              runTimeOptions.MaxIterations, runTimeOptions.ConstrRelTolFactor,
              runTimeOptions.ProbRelTolFactor, true);
      overflow = false;
      nVar = 0;
      int32_T exitg1;
      do {
        exitg1 = 0;
        if (nVar < 8) {
          if (b_cv[static_cast<uint8_T>(options.SolverName[nVar])] != b_cv[
              static_cast<int32_T>(cv1[nVar])]) {
            exitg1 = 1;
          } else {
            nVar++;
          }
        } else {
          overflow = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      if (overflow && (solution->state != -6)) {
        st.site = &bc_emlrtRSI;
        solution->maxConstr = c_maxConstraintViolation(&st, workingset,
          solution->xstar);
        st.site = &bc_emlrtRSI;
        computeFirstOrderOpt(&st, solution, objective, workingset->nVar,
                             workingset->ldA, workingset->ATwset,
                             workingset->nActiveConstr,
                             memspace->workspace_double);
        runTimeOptions.RemainFeasible = false;
        while ((solution->iterations < runTimeOptions.MaxIterations) &&
               ((solution->state == -7) || ((solution->state == 1) &&
                 ((solution->maxConstr > 1.0E-8 *
                   runTimeOptions.ConstrRelTolFactor) ||
                  (solution->firstorderopt > 1.0E-8 *
                   runTimeOptions.ProbRelTolFactor))))) {
          st.site = &bc_emlrtRSI;
          feasibleX0ForWorkingSet(&st, memspace->workspace_double,
            solution->xstar, workingset, qrmanager);
          st.site = &bc_emlrtRSI;
          b_PresolveWorkingSet(&st, solution, memspace, workingset, qrmanager);
          st.site = &bc_emlrtRSI;
          phaseone(&st, H, f, solution, memspace, workingset, qrmanager,
                   cholmanager, objective, &options, &runTimeOptions);
          st.site = &bc_emlrtRSI;
          iterate(&st, H, f, solution, memspace, workingset, qrmanager,
                  cholmanager, objective, options.ObjectiveLimit,
                  options.StepTolerance, runTimeOptions.MaxIterations,
                  runTimeOptions.ConstrRelTolFactor,
                  runTimeOptions.ProbRelTolFactor, false);
          st.site = &bc_emlrtRSI;
          solution->maxConstr = c_maxConstraintViolation(&st, workingset,
            solution->xstar);
          st.site = &bc_emlrtRSI;
          computeFirstOrderOpt(&st, solution, objective, workingset->nVar,
                               workingset->ldA, workingset->ATwset,
                               workingset->nActiveConstr,
                               memspace->workspace_double);
        }
      }
    }
  }
}

// End of code generation (driver.cpp)

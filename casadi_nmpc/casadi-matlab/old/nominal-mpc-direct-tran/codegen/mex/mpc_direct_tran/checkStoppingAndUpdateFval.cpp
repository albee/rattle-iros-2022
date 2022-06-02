//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  checkStoppingAndUpdateFval.cpp
//
//  Code generation for function 'checkStoppingAndUpdateFval'
//


// Include files
#include "checkStoppingAndUpdateFval.h"
#include "blas.h"
#include "computeFval_ReuseHx.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo wf_emlrtRSI = { 1,  // lineNo
  "checkStoppingAndUpdateFval",        // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/checkStoppingAndUpdateFval.p"// pathName 
};

static emlrtBCInfo pc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "checkStoppingAndUpdateFval",        // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/checkStoppingAndUpdateFval.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void checkStoppingAndUpdateFval(const emlrtStack *sp, int32_T *activeSetChangeID,
  const coder::array<real_T, 1U> &f, b_struct_T *solution, c_struct_T *memspace,
  const d_struct_T *objective, g_struct_T *workingset, e_struct_T *qrmanager,
  real_T options_ObjectiveLimit, int32_T runTimeOptions_MaxIterations, real_T
  c_runTimeOptions_ConstrRelTolFa, boolean_T updateFval)
{
  int32_T nVar;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  solution->iterations++;
  nVar = objective->nvar;
  if ((solution->iterations >= runTimeOptions_MaxIterations) &&
      ((solution->state != 1) || (objective->objtype == 5))) {
    solution->state = 0;
  }

  if (solution->iterations - solution->iterations / 50 * 50 == 0) {
    st.site = &wf_emlrtRSI;
    solution->maxConstr = c_maxConstraintViolation(&st, workingset,
      solution->xstar);
    if (solution->maxConstr > 1.0E-8 * c_runTimeOptions_ConstrRelTolFa) {
      boolean_T overflow;
      real_T constrViolation_new;
      st.site = &wf_emlrtRSI;
      if (objective->nvar >= 1) {
        n_t = (ptrdiff_t)objective->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        dcopy(&n_t, &(solution->xstar.data())[0], &incx_t,
              &(solution->searchDir.data())[0], &incy_t);
      }

      st.site = &wf_emlrtRSI;
      overflow = feasibleX0ForWorkingSet(&st, memspace->workspace_double,
        solution->searchDir, workingset, qrmanager);
      if ((!overflow) && (solution->state != 0)) {
        solution->state = -2;
      }

      *activeSetChangeID = 0;
      st.site = &wf_emlrtRSI;
      constrViolation_new = c_maxConstraintViolation(&st, workingset,
        solution->searchDir);
      if (constrViolation_new < solution->maxConstr) {
        st.site = &wf_emlrtRSI;
        overflow = ((1 <= objective->nvar) && (objective->nvar > 2147483646));
        if (overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (int32_T idx = 0; idx < nVar; idx++) {
          int32_T b_idx;
          int32_T i;
          b_idx = idx + 1;
          i = solution->searchDir.size(0);
          if ((b_idx < 1) || (b_idx > i)) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &pc_emlrtBCI, sp);
          }

          i = solution->xstar.size(0);
          if (b_idx > i) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &pc_emlrtBCI, sp);
          }

          solution->xstar[b_idx - 1] = solution->searchDir[b_idx - 1];
        }

        solution->maxConstr = constrViolation_new;
      }
    }
  }

  if (updateFval) {
    st.site = &wf_emlrtRSI;
    solution->fstar = computeFval_ReuseHx(&st, objective,
      memspace->workspace_double, f, solution->xstar);
    if ((solution->fstar < options_ObjectiveLimit) && ((solution->state != 0) ||
         (objective->objtype != 5))) {
      solution->state = 2;
    }
  }
}

// End of code generation (checkStoppingAndUpdateFval.cpp)

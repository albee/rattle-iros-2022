//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computePhaseOneRelativeTolerances.cpp
//
//  Code generation for function 'computePhaseOneRelativeTolerances'
//


// Include files
#include "computePhaseOneRelativeTolerances.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo vb_emlrtRSI = { 1,  // lineNo
  "computePhaseOneRelativeTolerances", // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/computePhaseOneRelativeTolerances.p"// pathName 
};

static emlrtBCInfo q_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "computePhaseOneRelativeTolerances", // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/computePhaseOneRelativeTolerances.p",// pName 
  0                                    // checkKind
};

// Function Definitions
real_T c_computePhaseOneRelativeTolera(const emlrtStack *sp, int32_T
  workingset_nVarOrig, const coder::array<real_T, 2U> &workingset_Aineq, const
  coder::array<real_T, 2U> &workingset_Aeq, const int32_T workingset_sizes[5])
{
  real_T tol;
  int32_T b;
  boolean_T overflow;
  boolean_T b_overflow;
  int32_T idx_col;
  real_T colSum;
  int32_T idx_row;
  boolean_T c_overflow;
  int32_T i;
  int32_T i1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  tol = 1.0;
  b = workingset_sizes[1];
  st.site = &vb_emlrtRSI;
  overflow = ((1 <= workingset_sizes[1]) && (workingset_sizes[1] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  if (0 <= workingset_sizes[1] - 1) {
    b_overflow = ((1 <= workingset_nVarOrig) && (workingset_nVarOrig >
      2147483646));
  }

  for (idx_col = 0; idx_col < b; idx_col++) {
    colSum = 0.0;
    st.site = &vb_emlrtRSI;
    if (b_overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = 0; idx_row < workingset_nVarOrig; idx_row++) {
      i = idx_col + 1;
      if ((i < 1) || (i > workingset_Aeq.size(1))) {
        emlrtDynamicBoundsCheckR2012b(i, 1, workingset_Aeq.size(1), &q_emlrtBCI,
          sp);
      }

      i1 = idx_row + 1;
      if ((i1 < 1) || (i1 > workingset_Aeq.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, workingset_Aeq.size(0), &q_emlrtBCI,
          sp);
      }

      colSum += muDoubleScalarAbs(workingset_Aeq[(i + workingset_Aeq.size(1) *
        (i1 - 1)) - 1]);
    }

    tol = muDoubleScalarMax(tol, colSum);
  }

  b = workingset_sizes[2];
  st.site = &vb_emlrtRSI;
  overflow = ((1 <= workingset_sizes[2]) && (workingset_sizes[2] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  if (0 <= workingset_sizes[2] - 1) {
    c_overflow = ((1 <= workingset_nVarOrig) && (workingset_nVarOrig >
      2147483646));
  }

  for (idx_col = 0; idx_col < b; idx_col++) {
    colSum = 0.0;
    st.site = &vb_emlrtRSI;
    if (c_overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx_row = 0; idx_row < workingset_nVarOrig; idx_row++) {
      i = idx_col + 1;
      if ((i < 1) || (i > workingset_Aineq.size(1))) {
        emlrtDynamicBoundsCheckR2012b(i, 1, workingset_Aineq.size(1),
          &q_emlrtBCI, sp);
      }

      i1 = idx_row + 1;
      if ((i1 < 1) || (i1 > workingset_Aineq.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, workingset_Aineq.size(0),
          &q_emlrtBCI, sp);
      }

      colSum += muDoubleScalarAbs(workingset_Aineq[(i + workingset_Aineq.size(1)
        * (i1 - 1)) - 1]);
    }

    tol = muDoubleScalarMax(tol, colSum);
  }

  return tol;
}

// End of code generation (computePhaseOneRelativeTolerances.cpp)

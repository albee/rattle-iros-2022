//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  updateRelativeTolerancesForPhaseTwo.cpp
//
//  Code generation for function 'updateRelativeTolerancesForPhaseTwo'
//


// Include files
#include "updateRelativeTolerancesForPhaseTwo.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "validateattributes.h"

// Variable Definitions
static emlrtRSInfo wb_emlrtRSI = { 1,  // lineNo
  "updateRelativeTolerancesForPhaseTwo",// fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/updateRelativeTolerancesForPhaseTwo.p"// pathName 
};

static emlrtBCInfo u_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "updateRelativeTolerancesForPhaseTwo",// fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+stopping/updateRelativeTolerancesForPhaseTwo.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void c_updateRelativeTolerancesForPh(const emlrtStack *sp, real_T *tol, const
  coder::array<real_T, 2U> &H, const coder::array<real_T, 1U> &f)
{
  real_T H_infnrm;
  real_T f_infnrm;
  int32_T b_b;
  boolean_T b_overflow;
  real_T colSum;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &wb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  validateattributes(&st, H);
  H_infnrm = 0.0;
  f_infnrm = 0.0;
  if (f.size(0) != 0) {
    int32_T b;
    boolean_T overflow;
    b = H.size(1);
    st.site = &wb_emlrtRSI;
    overflow = ((1 <= H.size(1)) && (H.size(1) > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    if (0 <= H.size(1) - 1) {
      b_b = H.size(0);
      b_overflow = ((1 <= H.size(0)) && (H.size(0) > 2147483646));
    }

    for (int32_T idx_col = 0; idx_col < b; idx_col++) {
      int32_T i;
      colSum = 0.0;
      st.site = &wb_emlrtRSI;
      if (b_overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (int32_T idx_row = 0; idx_row < b_b; idx_row++) {
        i = idx_col + 1;
        if ((i < 1) || (i > H.size(1))) {
          emlrtDynamicBoundsCheckR2012b(i, 1, H.size(1), &u_emlrtBCI, sp);
        }

        i = idx_row + 1;
        if ((i < 1) || (i > H.size(0))) {
          emlrtDynamicBoundsCheckR2012b(i, 1, H.size(0), &u_emlrtBCI, sp);
        }

        colSum += muDoubleScalarAbs(H[idx_col + H.size(1) * idx_row]);
      }

      H_infnrm = muDoubleScalarMax(H_infnrm, colSum);
      i = idx_col + 1;
      if ((i < 1) || (i > f.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i, 1, f.size(0), &u_emlrtBCI, sp);
      }

      f_infnrm = muDoubleScalarMax(f_infnrm, muDoubleScalarAbs(f[idx_col]));
    }
  } else {
    int32_T b;
    boolean_T overflow;
    b = H.size(1);
    st.site = &wb_emlrtRSI;
    overflow = ((1 <= H.size(1)) && (H.size(1) > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    if (0 <= H.size(1) - 1) {
      b_b = H.size(0);
      b_overflow = ((1 <= H.size(0)) && (H.size(0) > 2147483646));
    }

    for (int32_T idx_col = 0; idx_col < b; idx_col++) {
      colSum = 0.0;
      st.site = &wb_emlrtRSI;
      if (b_overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (int32_T idx_row = 0; idx_row < b_b; idx_row++) {
        int32_T i;
        i = idx_col + 1;
        if ((i < 1) || (i > H.size(1))) {
          emlrtDynamicBoundsCheckR2012b(i, 1, H.size(1), &u_emlrtBCI, sp);
        }

        i = idx_row + 1;
        if ((i < 1) || (i > H.size(0))) {
          emlrtDynamicBoundsCheckR2012b(i, 1, H.size(0), &u_emlrtBCI, sp);
        }

        colSum += muDoubleScalarAbs(H[idx_col + H.size(1) * idx_row]);
      }

      H_infnrm = muDoubleScalarMax(H_infnrm, colSum);
    }
  }

  *tol = muDoubleScalarMax(1.0, muDoubleScalarMax(muDoubleScalarMax(*tol,
    f_infnrm), H_infnrm));
}

// End of code generation (updateRelativeTolerancesForPhaseTwo.cpp)

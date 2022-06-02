//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factorQR.cpp
//
//  Code generation for function 'factorQR'
//


// Include files
#include "factorQR.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgeqrf.h"

// Function Definitions
void factorQR(const emlrtStack *sp, e_struct_T *obj, const coder::array<real_T,
              2U> &A, int32_T mrows, int32_T ncols)
{
  int32_T i;
  boolean_T guard1 = false;
  boolean_T overflow;
  int32_T idx;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    st.site = &sd_emlrtRSI;
    overflow = ((1 <= ncols) && (ncols > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < ncols; idx++) {
      st.site = &sd_emlrtRSI;
      b_xcopy(&st, mrows, A, A.size(0) * idx + 1, obj->QR, obj->ldq * idx + 1);
    }

    guard1 = true;
  } else if (i == 0) {
    obj->mrows = mrows;
    obj->ncols = ncols;
    obj->minRowCol = 0;
  } else {
    guard1 = true;
  }

  if (guard1) {
    obj->usedPivoting = false;
    obj->mrows = mrows;
    obj->ncols = ncols;
    st.site = &sd_emlrtRSI;
    overflow = ((1 <= ncols) && (ncols > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < ncols; idx++) {
      int32_T b_idx;
      b_idx = idx + 1;
      i = obj->jpvt.size(0);
      if ((b_idx < 1) || (b_idx > i)) {
        emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &kb_emlrtBCI, sp);
      }

      obj->jpvt[b_idx - 1] = b_idx;
    }

    obj->minRowCol = muIntScalarMin_sint32(mrows, ncols);
    st.site = &sd_emlrtRSI;
    xgeqrf(&st, obj->QR, mrows, ncols, obj->tau);
  }
}

// End of code generation (factorQR.cpp)

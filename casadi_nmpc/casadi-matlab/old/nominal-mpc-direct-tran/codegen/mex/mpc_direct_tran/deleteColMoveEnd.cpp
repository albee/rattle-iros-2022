//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  deleteColMoveEnd.cpp
//
//  Code generation for function 'deleteColMoveEnd'
//


// Include files
#include "deleteColMoveEnd.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "xrot.h"

// Variable Definitions
static emlrtRSInfo df_emlrtRSI = { 1,  // lineNo
  "deleteColMoveEnd",                  // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/deleteColMoveEnd.p"// pathName 
};

static emlrtBCInfo dc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "deleteColMoveEnd",                  // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/deleteColMoveEnd.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void deleteColMoveEnd(const emlrtStack *sp, e_struct_T *obj, int32_T idx)
{
  int32_T i;
  int32_T b_i;
  real_T a;
  real_T b;
  real_T c;
  real_T s;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (obj->usedPivoting) {
    boolean_T exitg1;
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i <= obj->ncols)) {
      b_i = obj->jpvt.size(0);
      if ((i < 1) || (i > b_i)) {
        emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &dc_emlrtBCI, sp);
      }

      if (obj->jpvt[i - 1] != idx) {
        i++;
      } else {
        exitg1 = true;
      }
    }

    idx = i;
  }

  if (idx >= obj->ncols) {
    obj->ncols--;
  } else {
    boolean_T overflow;
    int32_T k;
    int32_T b_k;
    b_i = obj->jpvt.size(0);
    if ((obj->ncols < 1) || (obj->ncols > b_i)) {
      emlrtDynamicBoundsCheckR2012b(obj->ncols, 1, b_i, &dc_emlrtBCI, sp);
    }

    b_i = obj->jpvt.size(0);
    if ((idx < 1) || (idx > b_i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &dc_emlrtBCI, sp);
    }

    obj->jpvt[idx - 1] = obj->jpvt[obj->ncols - 1];
    i = obj->minRowCol;
    st.site = &df_emlrtRSI;
    overflow = ((1 <= obj->minRowCol) && (obj->minRowCol > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = 0; k < i; k++) {
      b_k = k + 1;
      b_i = obj->QR.size(1);
      if ((obj->ncols < 1) || (obj->ncols > b_i)) {
        emlrtDynamicBoundsCheckR2012b(obj->ncols, 1, b_i, &dc_emlrtBCI, sp);
      }

      b_i = obj->QR.size(0);
      if ((b_k < 1) || (b_k > b_i)) {
        emlrtDynamicBoundsCheckR2012b(b_k, 1, b_i, &dc_emlrtBCI, sp);
      }

      b_i = obj->QR.size(1);
      if (idx > b_i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &dc_emlrtBCI, sp);
      }

      b_i = obj->QR.size(0);
      if (b_k > b_i) {
        emlrtDynamicBoundsCheckR2012b(b_k, 1, b_i, &dc_emlrtBCI, sp);
      }

      obj->QR[(idx + obj->QR.size(1) * (b_k - 1)) - 1] = obj->QR[(obj->ncols +
        obj->QR.size(1) * (b_k - 1)) - 1];
    }

    obj->ncols--;
    obj->minRowCol = muIntScalarMin_sint32(obj->mrows, obj->ncols);
    if (idx < obj->mrows) {
      int32_T endIdx;
      i = obj->mrows - 1;
      endIdx = muIntScalarMin_sint32(i, obj->ncols);
      for (b_k = endIdx; b_k >= idx; b_k--) {
        st.site = &df_emlrtRSI;
        b_i = obj->QR.size(1);
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        if (b_k > b_i) {
          emlrtDynamicBoundsCheckR2012b(b_k, 1, b_i, &dc_emlrtBCI, &st);
        }

        a = obj->QR[(idx + obj->QR.size(1) * (b_k - 1)) - 1];
        b_i = obj->QR.size(1);
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        i = b_k + 1;
        if ((i < 1) || (i > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &dc_emlrtBCI, &st);
        }

        b = obj->QR[(idx + obj->QR.size(1) * (i - 1)) - 1];
        c = 0.0;
        s = 0.0;
        drotg(&a, &b, &c, &s);
        b_i = obj->QR.size(1);
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        if (b_k > b_i) {
          emlrtDynamicBoundsCheckR2012b(b_k, 1, b_i, &dc_emlrtBCI, &st);
        }

        obj->QR[(idx + obj->QR.size(1) * (b_k - 1)) - 1] = a;
        b_i = obj->QR.size(1);
        if (idx > b_i) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        i = b_k + 1;
        if ((i < 1) || (i > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &dc_emlrtBCI, &st);
        }

        obj->QR[(idx + obj->QR.size(1) * (i - 1)) - 1] = b;
        b_i = obj->QR.size(1);
        if (b_k > b_i) {
          emlrtDynamicBoundsCheckR2012b(b_k, 1, b_i, &dc_emlrtBCI, sp);
        }

        b_i = obj->QR.size(0);
        i = b_k + 1;
        if ((i < 1) || (i > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &dc_emlrtBCI, sp);
        }

        obj->QR[(b_k + obj->QR.size(1) * (i - 1)) - 1] = 0.0;
        i = b_k + obj->ldq * idx;
        st.site = &df_emlrtRSI;
        b_xrot(&st, obj->ncols - idx, obj->QR, i, obj->ldq, i + 1, obj->ldq, c,
               s);
        i = obj->ldq * (b_k - 1) + 1;
        st.site = &df_emlrtRSI;
        xrot(&st, obj->mrows, obj->Q, i, obj->ldq + i, c, s);
      }

      b_k = idx + 1;
      st.site = &df_emlrtRSI;
      overflow = ((idx + 1 <= endIdx) && (endIdx > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (k = b_k; k <= endIdx; k++) {
        st.site = &df_emlrtRSI;
        b_i = obj->QR.size(1);
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &dc_emlrtBCI, &st);
        }

        a = obj->QR[(k + obj->QR.size(1) * (k - 1)) - 1];
        b_i = obj->QR.size(1);
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        i = k + 1;
        if ((i < 1) || (i > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &dc_emlrtBCI, &st);
        }

        b = obj->QR[(k + obj->QR.size(1) * (i - 1)) - 1];
        c = 0.0;
        s = 0.0;
        drotg(&a, &b, &c, &s);
        b_i = obj->QR.size(1);
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &dc_emlrtBCI, &st);
        }

        obj->QR[(k + obj->QR.size(1) * (k - 1)) - 1] = a;
        b_i = obj->QR.size(1);
        if (k > b_i) {
          emlrtDynamicBoundsCheckR2012b(k, 1, b_i, &dc_emlrtBCI, &st);
        }

        b_i = obj->QR.size(0);
        i = k + 1;
        if ((i < 1) || (i > b_i)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, b_i, &dc_emlrtBCI, &st);
        }

        obj->QR[(k + obj->QR.size(1) * (i - 1)) - 1] = b;
        i = k * (obj->ldq + 1);
        st.site = &df_emlrtRSI;
        b_xrot(&st, obj->ncols - k, obj->QR, i, obj->ldq, i + 1, obj->ldq, c, s);
        i = obj->ldq * (k - 1) + 1;
        st.site = &df_emlrtRSI;
        xrot(&st, obj->mrows, obj->Q, i, obj->ldq + i, c, s);
      }
    }
  }
}

// End of code generation (deleteColMoveEnd.cpp)

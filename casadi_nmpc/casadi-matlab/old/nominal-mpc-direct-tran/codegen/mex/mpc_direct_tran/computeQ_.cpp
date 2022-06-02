//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeQ_.cpp
//
//  Code generation for function 'computeQ_'
//


// Include files
#include "computeQ_.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "lapacke.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo ed_emlrtRSI = { 1,  // lineNo
  "computeQ_",                         // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/computeQ_.p"// pathName 
};

static emlrtRSInfo fd_emlrtRSI = { 59, // lineNo
  "ceval_xorgqr",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m"// pathName 
};

static emlrtRSInfo jd_emlrtRSI = { 14, // lineNo
  "xorgqr",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m"// pathName 
};

static emlrtRTEInfo tb_emlrtRTEI = { 52,// lineNo
  36,                                  // colNo
  "xorgqr",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m"// pName 
};

static emlrtRTEInfo ub_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "computeQ_",                         // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/computeQ_.p"// pName 
};

// Function Definitions
void computeQ_(const emlrtStack *sp, e_struct_T *obj, int32_T nrows)
{
  int32_T b;
  boolean_T overflow;
  int32_T idx;
  int32_T iQR0;
  coder::array<real_T, 2U> r;
  ptrdiff_t info_t;
  int32_T i;
  static const char_T b_cv[14] = { 'L', 'A', 'P', 'A', 'C', 'K', 'E', '_', 'd',
    'o', 'r', 'g', 'q', 'r' };

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
  b = obj->minRowCol;
  st.site = &ed_emlrtRSI;
  overflow = ((1 <= obj->minRowCol) && (obj->minRowCol > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < b; idx++) {
    iQR0 = (obj->ldq * idx + idx) + 2;
    st.site = &ed_emlrtRSI;
    b_xcopy(&st, (obj->mrows - idx) - 1, obj->QR, iQR0, obj->Q, iQR0);
  }

  st.site = &ed_emlrtRSI;
  b_st.site = &jd_emlrtRSI;
  r.set_size((&tb_emlrtRTEI), (&b_st), obj->Q.size(1), obj->Q.size(0));
  b = obj->Q.size(1);
  for (idx = 0; idx < b; idx++) {
    iQR0 = obj->Q.size(0);
    for (i = 0; i < iQR0; i++) {
      r[i + r.size(1) * idx] = obj->Q[idx + obj->Q.size(1) * i];
    }
  }

  info_t = LAPACKE_dorgqr(102, (ptrdiff_t)obj->mrows, (ptrdiff_t)nrows,
    (ptrdiff_t)obj->minRowCol, &(r.data())[0], (ptrdiff_t)obj->ldq,
    &(obj->tau.data())[0]);
  obj->Q.set_size((&ub_emlrtRTEI), (&b_st), r.size(1), r.size(0));
  b = r.size(1);
  for (idx = 0; idx < b; idx++) {
    iQR0 = r.size(0);
    for (i = 0; i < iQR0; i++) {
      obj->Q[i + obj->Q.size(1) * idx] = r[idx + r.size(1) * i];
    }
  }

  b = (int32_T)info_t;
  c_st.site = &fd_emlrtRSI;
  if (b != 0) {
    boolean_T p;
    overflow = true;
    p = false;
    if (b == -7) {
      p = true;
    } else {
      if (b == -5) {
        p = true;
      }
    }

    if (!p) {
      if (b == -1010) {
        emlrtErrorWithMessageIdR2018a(&c_st, &n_emlrtRTEI, "MATLAB:nomem",
          "MATLAB:nomem", 0);
      } else {
        emlrtErrorWithMessageIdR2018a(&c_st, &m_emlrtRTEI,
          "Coder:toolbox:LAPACKCallErrorInfo",
          "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, b_cv, 12, b);
      }
    }
  } else {
    overflow = false;
  }

  if (overflow) {
    b = obj->Q.size(1) * obj->Q.size(0) - 1;
    for (idx = 0; idx <= b; idx++) {
      obj->Q[idx] = rtNaN;
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (computeQ_.cpp)

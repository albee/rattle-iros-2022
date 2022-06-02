//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgeqrf.cpp
//
//  Code generation for function 'xgeqrf'
//


// Include files
#include "xgeqrf.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "lapacke.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo td_emlrtRSI = { 27, // lineNo
  "xgeqrf",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo ud_emlrtRSI = { 102,// lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo vd_emlrtRSI = { 99, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo wd_emlrtRSI = { 94, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo xd_emlrtRSI = { 93, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo yd_emlrtRSI = { 91, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo ae_emlrtRSI = { 84, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo be_emlrtRSI = { 79, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo ce_emlrtRSI = { 53, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRSInfo de_emlrtRSI = { 52, // lineNo
  "ceval_xgeqrf",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pathName 
};

static emlrtRTEInfo gc_emlrtRTEI = { 27,// lineNo
  5,                                   // colNo
  "xgeqrf",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pName 
};

static emlrtRTEInfo hc_emlrtRTEI = { 85,// lineNo
  15,                                  // colNo
  "xgeqrf",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pName 
};

static emlrtRTEInfo ic_emlrtRTEI = { 75,// lineNo
  5,                                   // colNo
  "xgeqrf",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqrf.m"// pName 
};

// Function Definitions
void xgeqrf(const emlrtStack *sp, coder::array<real_T, 2U> &A, int32_T m,
            int32_T n, coder::array<real_T, 1U> &tau)
{
  int32_T ma;
  int32_T na;
  int32_T minmana;
  coder::array<real_T, 2U> r;
  static const char_T b_cv[14] = { 'L', 'A', 'P', 'A', 'C', 'K', 'E', '_', 'd',
    'g', 'e', 'q', 'r', 'f' };

  boolean_T b_overflow;
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
  st.site = &td_emlrtRSI;
  ma = A.size(0);
  na = A.size(1);
  minmana = muIntScalarMin_sint32(ma, na);
  b_st.site = &de_emlrtRSI;
  b_st.site = &ce_emlrtRSI;
  tau.set_size((&gc_emlrtRTEI), (&st), minmana);
  if (n == 0) {
    tau.set_size((&ic_emlrtRTEI), (&st), minmana);
    for (int32_T i = 0; i < minmana; i++) {
      tau[i] = 0.0;
    }
  } else {
    int32_T i;
    ptrdiff_t info_t;
    int32_T loop_ub;
    int32_T i1;
    boolean_T overflow;
    b_st.site = &be_emlrtRSI;
    b_st.site = &ae_emlrtRSI;
    r.set_size((&hc_emlrtRTEI), (&st), A.size(1), A.size(0));
    na = A.size(1);
    for (i = 0; i < na; i++) {
      loop_ub = A.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    info_t = LAPACKE_dgeqrf(102, (ptrdiff_t)m, (ptrdiff_t)n, &(r.data())[0],
      (ptrdiff_t)A.size(0), &(tau.data())[0]);
    A.set_size((&hc_emlrtRTEI), (&st), r.size(1), r.size(0));
    na = r.size(1);
    for (i = 0; i < na; i++) {
      loop_ub = r.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        A[i1 + A.size(1) * i] = r[i + r.size(1) * i1];
      }
    }

    na = (int32_T)info_t;
    b_st.site = &yd_emlrtRSI;
    if (na != 0) {
      overflow = true;
      if (na != -4) {
        if (na == -1010) {
          emlrtErrorWithMessageIdR2018a(&b_st, &n_emlrtRTEI, "MATLAB:nomem",
            "MATLAB:nomem", 0);
        } else {
          emlrtErrorWithMessageIdR2018a(&b_st, &m_emlrtRTEI,
            "Coder:toolbox:LAPACKCallErrorInfo",
            "Coder:toolbox:LAPACKCallErrorInfo", 5, 4, 14, b_cv, 12, na);
        }
      }
    } else {
      overflow = false;
    }

    if (overflow) {
      int32_T b_i;
      b_st.site = &xd_emlrtRSI;
      overflow = ((1 <= n) && (n > 2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      if (0 <= n - 1) {
        b_overflow = ((1 <= m) && (m > 2147483646));
      }

      for (na = 0; na < n; na++) {
        b_st.site = &wd_emlrtRSI;
        if (b_overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (b_i = 0; b_i < m; b_i++) {
          i = na * ma + b_i;
          i1 = A.size(0);
          A[i % i1 * A.size(1) + i / i1] = rtNaN;
        }
      }

      na = muIntScalarMin_sint32(m, n);
      b_st.site = &vd_emlrtRSI;
      overflow = ((1 <= na) && (na > 2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (b_i = 0; b_i < na; b_i++) {
        tau[b_i] = rtNaN;
      }

      loop_ub = na + 1;
      b_st.site = &ud_emlrtRSI;
      overflow = ((na + 1 <= minmana) && (minmana > 2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (b_i = loop_ub; b_i <= minmana; b_i++) {
        tau[b_i - 1] = 0.0;
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (xgeqrf.cpp)

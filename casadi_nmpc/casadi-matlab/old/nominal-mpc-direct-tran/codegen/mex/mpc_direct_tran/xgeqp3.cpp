//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgeqp3.cpp
//
//  Code generation for function 'xgeqp3'
//


// Include files
#include "xgeqp3.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "lapacke.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo gc_emlrtRSI = { 57, // lineNo
  "xgeqp3",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo hc_emlrtRSI = { 152,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo ic_emlrtRSI = { 148,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo jc_emlrtRSI = { 145,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo kc_emlrtRSI = { 142,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo lc_emlrtRSI = { 137,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo mc_emlrtRSI = { 135,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo nc_emlrtRSI = { 132,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo oc_emlrtRSI = { 123,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo pc_emlrtRSI = { 119,// lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo qc_emlrtRSI = { 91, // lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo rc_emlrtRSI = { 86, // lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRSInfo sc_emlrtRSI = { 85, // lineNo
  "ceval_xgeqp3",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pathName 
};

static emlrtRTEInfo pb_emlrtRTEI = { 57,// lineNo
  5,                                   // colNo
  "xgeqp3",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pName 
};

static emlrtRTEInfo qb_emlrtRTEI = { 98,// lineNo
  1,                                   // colNo
  "xgeqp3",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pName 
};

static emlrtRTEInfo rb_emlrtRTEI = { 125,// lineNo
  15,                                  // colNo
  "xgeqp3",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pName 
};

static emlrtRTEInfo sb_emlrtRTEI = { 90,// lineNo
  5,                                   // colNo
  "xgeqp3",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xgeqp3.m"// pName 
};

// Function Definitions
void xgeqp3(const emlrtStack *sp, coder::array<real_T, 2U> &A, int32_T m,
            int32_T n, coder::array<int32_T, 1U> &jpvt, coder::array<real_T, 1U>
            &tau)
{
  int32_T ma;
  int32_T na;
  int32_T minmana;
  coder::array<ptrdiff_t, 1U> jpvt_t;
  coder::array<real_T, 2U> r;
  static const char_T b_cv[14] = { 'L', 'A', 'P', 'A', 'C', 'K', 'E', '_', 'd',
    'g', 'e', 'q', 'p', '3' };

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
  st.site = &gc_emlrtRSI;
  ma = A.size(0);
  na = A.size(1);
  minmana = muIntScalarMin_sint32(ma, na);
  b_st.site = &sc_emlrtRSI;
  b_st.site = &rc_emlrtRSI;
  tau.set_size((&pb_emlrtRTEI), (&st), minmana);
  if (n < 1) {
    tau.set_size((&sb_emlrtRTEI), (&st), minmana);
    for (int32_T i = 0; i < minmana; i++) {
      tau[i] = 0.0;
    }

    b_st.site = &qc_emlrtRSI;
    for (ma = 0; ma < n; ma++) {
      jpvt[ma] = ma + 1;
    }
  } else {
    int32_T i;
    ptrdiff_t info_t;
    int32_T loop_ub;
    int32_T i1;
    boolean_T overflow;
    jpvt_t.set_size((&qb_emlrtRTEI), (&st), jpvt.size(0));
    na = jpvt.size(0);
    for (i = 0; i < na; i++) {
      jpvt_t[i] = (ptrdiff_t)jpvt[i];
    }

    b_st.site = &pc_emlrtRSI;
    b_st.site = &oc_emlrtRSI;
    r.set_size((&rb_emlrtRTEI), (&st), A.size(1), A.size(0));
    na = A.size(1);
    for (i = 0; i < na; i++) {
      loop_ub = A.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    info_t = LAPACKE_dgeqp3(102, (ptrdiff_t)m, (ptrdiff_t)n, &(r.data())[0],
      (ptrdiff_t)A.size(0), &(jpvt_t.data())[0], &(tau.data())[0]);
    A.set_size((&rb_emlrtRTEI), (&st), r.size(1), r.size(0));
    na = r.size(1);
    for (i = 0; i < na; i++) {
      loop_ub = r.size(0);
      for (i1 = 0; i1 < loop_ub; i1++) {
        A[i1 + A.size(1) * i] = r[i + r.size(1) * i1];
      }
    }

    na = (int32_T)info_t;
    b_st.site = &nc_emlrtRSI;
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
      b_st.site = &mc_emlrtRSI;
      if (n > 2147483646) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      overflow = ((1 <= m) && (m > 2147483646));
      for (na = 0; na < n; na++) {
        b_st.site = &lc_emlrtRSI;
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (loop_ub = 0; loop_ub < m; loop_ub++) {
          i = na * ma + loop_ub;
          i1 = A.size(0);
          A[i % i1 * A.size(1) + i / i1] = rtNaN;
        }
      }

      na = muIntScalarMin_sint32(m, n);
      b_st.site = &kc_emlrtRSI;
      for (ma = 0; ma < na; ma++) {
        tau[ma] = rtNaN;
      }

      loop_ub = na + 1;
      b_st.site = &jc_emlrtRSI;
      overflow = ((na + 1 <= minmana) && (minmana > 2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (ma = loop_ub; ma <= minmana; ma++) {
        tau[ma - 1] = 0.0;
      }

      b_st.site = &ic_emlrtRSI;
      for (ma = 0; ma < n; ma++) {
        jpvt[ma] = ma + 1;
      }
    } else {
      b_st.site = &hc_emlrtRSI;
      if (n > 2147483646) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (ma = 0; ma < n; ma++) {
        jpvt[ma] = (int32_T)jpvt_t[ma];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (xgeqp3.cpp)

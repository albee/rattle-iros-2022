//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xcopy.cpp
//
//  Code generation for function 'xcopy'
//


// Include files
#include "xcopy.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo pb_emlrtRSI = { 69, // lineNo
  "xcopy",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+refblas/xcopy.m"// pathName 
};

static emlrtRSInfo we_emlrtRSI = { 38, // lineNo
  "xcopy",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xcopy.m"// pathName 
};

static emlrtRTEInfo mb_emlrtRTEI = { 51,// lineNo
  9,                                   // colNo
  "xcopy",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xcopy.m"// pName 
};

// Function Definitions
void b_xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 2U> &x,
             int32_T ix0, coder::array<real_T, 2U> &y, int32_T iy0)
{
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if (n >= 1) {
    int32_T loop_ub;
    int32_T i;
    int32_T b_loop_ub;
    int32_T i1;
    st.site = &nb_emlrtRSI;
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&vb_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    r1.set_size((&lb_emlrtRTEI), (&st), y.size(1), y.size(0));
    loop_ub = y.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = y.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = y[i + y.size(1) * i1];
      }
    }

    dcopy(&n_t, &r[ix0 - 1], &incx_t, &r1[iy0 - 1], &incy_t);
    y.set_size((&mb_emlrtRTEI), (&st), r1.size(1), r1.size(0));
    loop_ub = r1.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r1.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        y[i1 + y.size(1) * i] = r1[i + r1.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void c_xcopy(const emlrtStack *sp, int32_T n, coder::array<real_T, 1U> &y)
{
  boolean_T overflow;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &we_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  b_st.site = &pb_emlrtRSI;
  overflow = ((1 <= n) && (n > 2147483646));
  if (overflow) {
    c_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&c_st);
  }

  for (int32_T k = 0; k < n; k++) {
    y[k] = 0.0;
  }
}

void d_xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 2U> &x,
             int32_T ix0, coder::array<real_T, 2U> &y, int32_T iy0)
{
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if (n >= 1) {
    int32_T loop_ub;
    int32_T i;
    int32_T i1;
    st.site = &nb_emlrtRSI;
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&vb_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      int32_T b_loop_ub;
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    r1.set_size((&lb_emlrtRTEI), (&st), 48, y.size(0));
    loop_ub = y.size(0);
    for (i = 0; i < 48; i++) {
      for (i1 = 0; i1 < loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = y[i + 48 * i1];
      }
    }

    dcopy(&n_t, &r[ix0 - 1], &incx_t, &r1[iy0 - 1], &incy_t);
    y.set_size((&mb_emlrtRTEI), (&st), r1.size(1), 48);
    loop_ub = r1.size(1);
    for (i = 0; i < loop_ub; i++) {
      for (i1 = 0; i1 < 48; i1++) {
        y[i1 + 48 * i] = r1[i + r1.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void e_xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 2U> &x,
             coder::array<real_T, 2U> &y)
{
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if (n >= 1) {
    int32_T loop_ub;
    int32_T i;
    int32_T b_loop_ub;
    int32_T i1;
    st.site = &nb_emlrtRSI;
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&vb_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(0) * x.size(1);
    for (i = 0; i < loop_ub; i++) {
      r[i] = x[i];
    }

    r1.set_size((&lb_emlrtRTEI), (&st), y.size(1), y.size(0));
    loop_ub = y.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = y.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = y[i + y.size(1) * i1];
      }
    }

    dcopy(&n_t, &(r.data())[0], &incx_t, &(r1.data())[0], &incy_t);
    y.set_size((&mb_emlrtRTEI), (&st), r1.size(1), r1.size(0));
    loop_ub = r1.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r1.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        y[i1 + y.size(1) * i] = r1[i + r1.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void xcopy(const emlrtStack *sp, int32_T n, const coder::array<real_T, 1U> &x,
           coder::array<real_T, 2U> &y)
{
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if (n >= 1) {
    int32_T loop_ub;
    int32_T i;
    int32_T b_loop_ub;
    int32_T i1;
    st.site = &nb_emlrtRSI;
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&lb_emlrtRTEI), (&st), y.size(1), y.size(0));
    loop_ub = y.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = y.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = y[i + y.size(1) * i1];
      }
    }

    dcopy(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0], &incx_t,
          &(r.data())[0], &incy_t);
    y.set_size((&mb_emlrtRTEI), (&st), r.size(1), r.size(0));
    loop_ub = r.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        y[i1 + y.size(1) * i] = r[i + r.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (xcopy.cpp)

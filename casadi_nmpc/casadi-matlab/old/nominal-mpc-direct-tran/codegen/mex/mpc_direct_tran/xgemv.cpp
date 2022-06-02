//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgemv.cpp
//
//  Code generation for function 'xgemv'
//


// Include files
#include "xgemv.h"
#include "blas.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRTEInfo bd_emlrtRTEI = { 87,// lineNo
  9,                                   // colNo
  "xgemv",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m"// pName 
};

// Function Definitions
void b_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 2U> &x,
             int32_T ix0, coder::array<real_T, 1U> &y)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
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
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&jc_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &r1[ix0 - 1],
          &incx_t, &beta1, &(y.data())[0], &incy_t);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void c_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 1U> &y)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if (n >= 1) {
    int32_T loop_ub;
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (int32_T i = 0; i < loop_ub; i++) {
      int32_T b_loop_ub;
      b_loop_ub = A.size(0);
      for (int32_T i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &(((coder::array<
             real_T, 1U> *)&x)->data())[0], &incx_t, &beta1, &(y.data())[0],
          &incy_t);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void d_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 1U> &y)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if ((m >= 1) && (n >= 1)) {
    int32_T loop_ub;
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (int32_T i = 0; i < loop_ub; i++) {
      int32_T b_loop_ub;
      b_loop_ub = A.size(0);
      for (int32_T i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &(((coder::array<
             real_T, 1U> *)&x)->data())[0], &incx_t, &beta1, &(y.data())[0],
          &incy_t);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void e_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T ia0, int32_T lda, const coder::array<real_T,
             2U> &x, coder::array<real_T, 1U> &y)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if (m >= 1) {
    int32_T loop_ub;
    int32_T i;
    int32_T b_loop_ub;
    int32_T i1;
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&jc_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &r[ia0 - 1], &lda_t, &(r1.data())[0],
          &incx_t, &beta1, &(y.data())[0], &incy_t);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void f_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 2U> &y)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
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
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&xb_emlrtRTEI), (&st), y.size(1), y.size(0));
    loop_ub = y.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = y.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = y[i + y.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &(((coder::array<
             real_T, 1U> *)&x)->data())[0], &incx_t, &beta1, &(r1.data())[0],
          &incy_t);
    y.set_size((&bd_emlrtRTEI), (&st), r1.size(1), r1.size(0));
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

void g_xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<
             real_T, 2U> &A, int32_T lda, const coder::array<real_T, 1U> &x,
             coder::array<real_T, 2U> &y, int32_T iy0)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
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
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&xb_emlrtRTEI), (&st), y.size(1), y.size(0));
    loop_ub = y.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = y.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = y[i + y.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &(((coder::array<
             real_T, 1U> *)&x)->data())[0], &incx_t, &beta1, &r1[iy0 - 1],
          &incy_t);
    y.set_size((&bd_emlrtRTEI), (&st), r1.size(1), r1.size(0));
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

void xgemv(const emlrtStack *sp, int32_T m, int32_T n, const coder::array<real_T,
           2U> &A, int32_T lda, const coder::array<real_T, 2U> &x, coder::array<
           real_T, 1U> &y)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSA;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
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
    st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = -1.0;
    TRANSA = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    lda_t = (ptrdiff_t)lda;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&jc_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &(r1.data())[0],
          &incx_t, &beta1, &(y.data())[0], &incy_t);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (xgemv.cpp)

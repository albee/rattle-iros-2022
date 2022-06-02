//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xgemm.cpp
//
//  Code generation for function 'xgemm'
//


// Include files
#include "xgemm.h"
#include "blas.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRTEInfo yc_emlrtRTEI = { 88,// lineNo
  9,                                   // colNo
  "xgemm",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemm.m"// pName 
};

// Function Definitions
void b_xgemm(const emlrtStack *sp, int32_T m, int32_T n, int32_T k, const coder::
             array<real_T, 2U> &A, int32_T ia0, int32_T lda, const coder::array<
             real_T, 2U> &B, int32_T ldb, coder::array<real_T, 2U> &C, int32_T
             ldc)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSB1;
  char_T TRANSA1;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  coder::array<real_T, 2U> r2;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if ((m >= 1) && (n >= 1) && (k >= 1)) {
    int32_T loop_ub;
    int32_T i;
    int32_T b_loop_ub;
    int32_T i1;
    st.site = &ee_emlrtRSI;
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSB1 = 'N';
    TRANSA1 = 'T';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    k_t = (ptrdiff_t)k;
    lda_t = (ptrdiff_t)lda;
    ldb_t = (ptrdiff_t)ldb;
    ldc_t = (ptrdiff_t)ldc;
    r.set_size((&cc_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&dc_emlrtRTEI), (&st), B.size(1), B.size(0));
    loop_ub = B.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = B.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = B[i + B.size(1) * i1];
      }
    }

    r2.set_size((&ec_emlrtRTEI), (&st), C.size(1), C.size(0));
    loop_ub = C.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = C.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r2[i1 + r2.size(1) * i] = C[i + C.size(1) * i1];
      }
    }

    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &r[ia0 - 1], &lda_t,
          &(r1.data())[0], &ldb_t, &beta1, &(r2.data())[0], &ldc_t);
    C.set_size((&yc_emlrtRTEI), (&st), r2.size(1), r2.size(0));
    loop_ub = r2.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r2.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        C[i1 + C.size(1) * i] = r2[i + r2.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void xgemm(const emlrtStack *sp, int32_T m, int32_T n, int32_T k, const coder::
           array<real_T, 2U> &A, int32_T lda, const coder::array<real_T, 2U> &B,
           int32_T ib0, int32_T ldb, coder::array<real_T, 2U> &C, int32_T ldc)
{
  real_T alpha1;
  real_T beta1;
  char_T TRANSB1;
  char_T TRANSA1;
  ptrdiff_t m_t;
  ptrdiff_t n_t;
  ptrdiff_t k_t;
  ptrdiff_t lda_t;
  ptrdiff_t ldb_t;
  ptrdiff_t ldc_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  coder::array<real_T, 2U> r2;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if ((m >= 1) && (n >= 1) && (k >= 1)) {
    int32_T loop_ub;
    int32_T i;
    int32_T b_loop_ub;
    int32_T i1;
    st.site = &ee_emlrtRSI;
    alpha1 = 1.0;
    beta1 = 0.0;
    TRANSB1 = 'N';
    TRANSA1 = 'N';
    m_t = (ptrdiff_t)m;
    n_t = (ptrdiff_t)n;
    k_t = (ptrdiff_t)k;
    lda_t = (ptrdiff_t)lda;
    ldb_t = (ptrdiff_t)ldb;
    ldc_t = (ptrdiff_t)ldc;
    r.set_size((&cc_emlrtRTEI), (&st), A.size(1), A.size(0));
    loop_ub = A.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = A.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = A[i + A.size(1) * i1];
      }
    }

    r1.set_size((&dc_emlrtRTEI), (&st), B.size(1), B.size(0));
    loop_ub = B.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = B.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = B[i + B.size(1) * i1];
      }
    }

    r2.set_size((&ec_emlrtRTEI), (&st), C.size(1), C.size(0));
    loop_ub = C.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = C.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r2[i1 + r2.size(1) * i] = C[i + C.size(1) * i1];
      }
    }

    dgemm(&TRANSA1, &TRANSB1, &m_t, &n_t, &k_t, &alpha1, &(r.data())[0], &lda_t,
          &r1[ib0 - 1], &ldb_t, &beta1, &(r2.data())[0], &ldc_t);
    C.set_size((&yc_emlrtRTEI), (&st), r2.size(1), r2.size(0));
    loop_ub = r2.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r2.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        C[i1 + C.size(1) * i] = r2[i + r2.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (xgemm.cpp)

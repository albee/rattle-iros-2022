//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  xrot.cpp
//
//  Code generation for function 'xrot'
//


// Include files
#include "xrot.h"
#include "blas.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo bf_emlrtRSI = { 48, // lineNo
  "xrot",                              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xrot.m"// pathName 
};

static emlrtRTEInfo nc_emlrtRTEI = { 114,// lineNo
  31,                                  // colNo
  "xrot",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xrot.m"// pName 
};

// Function Definitions
void b_xrot(const emlrtStack *sp, int32_T n, coder::array<real_T, 2U> &x,
            int32_T ix0, int32_T incx, int32_T iy0, int32_T incy, real_T c,
            real_T s)
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
    st.site = &bf_emlrtRSI;
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)incx;
    incy_t = (ptrdiff_t)incy;
    r.set_size((&nc_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    drot(&n_t, &r[ix0 - 1], &incx_t, &r[iy0 - 1], &incy_t, &c, &s);
    x.set_size((&nc_emlrtRTEI), (&st), r.size(1), r.size(0));
    loop_ub = r.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        x[i1 + x.size(1) * i] = r[i + r.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

void xrot(const emlrtStack *sp, int32_T n, coder::array<real_T, 2U> &x, int32_T
          ix0, int32_T iy0, real_T c, real_T s)
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
    st.site = &bf_emlrtRSI;
    n_t = (ptrdiff_t)n;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&nc_emlrtRTEI), (&st), x.size(1), x.size(0));
    loop_ub = x.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = x.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = x[i + x.size(1) * i1];
      }
    }

    drot(&n_t, &r[ix0 - 1], &incx_t, &r[iy0 - 1], &incy_t, &c, &s);
    x.set_size((&nc_emlrtRTEI), (&st), r.size(1), r.size(0));
    loop_ub = r.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        x[i1 + x.size(1) * i] = r[i + r.size(1) * i1];
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (xrot.cpp)

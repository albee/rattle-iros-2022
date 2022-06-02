//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  linearForm_.cpp
//
//  Code generation for function 'linearForm_'
//


// Include files
#include "linearForm_.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo re_emlrtRSI = { 1,  // lineNo
  "linearForm_",                       // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/linearForm_.p"// pathName 
};

static emlrtBCInfo ub_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "linearForm_",                       // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/linearForm_.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo lc_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "linearForm_",                       // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/linearForm_.p"// pName 
};

// Function Definitions
void linearForm_(const emlrtStack *sp, boolean_T obj_hasLinear, int32_T obj_nvar,
                 coder::array<real_T, 2U> &workspace, const coder::array<real_T,
                 2U> &H, const coder::array<real_T, 1U> &f, const coder::array<
                 real_T, 1U> &x)
{
  real_T beta1;
  int32_T i;
  real_T alpha1;
  char_T TRANSA;
  int32_T b_i;
  ptrdiff_t m_t;
  int32_T i1;
  ptrdiff_t n_t;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  coder::array<real_T, 2U> r1;
  int32_T loop_ub;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  beta1 = 0.0;
  if (obj_hasLinear) {
    boolean_T overflow;
    st.site = &re_emlrtRSI;
    overflow = ((1 <= obj_nvar) && (obj_nvar > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (i = 0; i < obj_nvar; i++) {
      b_i = workspace.size(0) * workspace.size(1);
      i1 = i + 1;
      if ((i1 < 1) || (i1 > b_i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, b_i, &ub_emlrtBCI, sp);
      }

      b_i = workspace.size(0);
      i1 = i + 1;
      if ((i1 < 1) || (i1 > f.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, f.size(0), &ub_emlrtBCI, sp);
      }

      workspace[i % b_i * workspace.size(1) + i / b_i] = f[i1 - 1];
    }

    beta1 = 1.0;
  }

  st.site = &re_emlrtRSI;
  b_st.site = &ad_emlrtRSI;
  alpha1 = 0.5;
  TRANSA = 'N';
  m_t = (ptrdiff_t)obj_nvar;
  n_t = (ptrdiff_t)obj_nvar;
  lda_t = (ptrdiff_t)obj_nvar;
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  r.set_size((&wb_emlrtRTEI), (&b_st), H.size(1), H.size(0));
  i = H.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    loop_ub = H.size(0);
    for (i1 = 0; i1 < loop_ub; i1++) {
      r[i1 + r.size(1) * b_i] = H[b_i + H.size(1) * i1];
    }
  }

  r1.set_size((&xb_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
  i = workspace.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    loop_ub = workspace.size(0);
    for (i1 = 0; i1 < loop_ub; i1++) {
      r1[i1 + r1.size(1) * b_i] = workspace[b_i + workspace.size(1) * i1];
    }
  }

  dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &(((coder::array<
           real_T, 1U> *)&x)->data())[0], &incx_t, &beta1, &(r1.data())[0],
        &incy_t);
  workspace.set_size((&lc_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
  i = r1.size(1);
  for (b_i = 0; b_i < i; b_i++) {
    loop_ub = r1.size(0);
    for (i1 = 0; i1 < loop_ub; i1++) {
      workspace[i1 + workspace.size(1) * b_i] = r1[b_i + r1.size(1) * i1];
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (linearForm_.cpp)

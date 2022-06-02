//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeGrad_StoreHx.cpp
//
//  Code generation for function 'computeGrad_StoreHx'
//


// Include files
#include "computeGrad_StoreHx.h"
#include "blas.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemv.h"

// Variable Definitions
static emlrtRSInfo ue_emlrtRSI = { 1,  // lineNo
  "computeGrad_StoreHx",               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/computeGrad_StoreHx.p"// pathName 
};

static emlrtBCInfo bc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "computeGrad_StoreHx",               // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/computeGrad_StoreHx.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void computeGrad_StoreHx(const emlrtStack *sp, d_struct_T *obj, const coder::
  array<real_T, 2U> &H, const coder::array<real_T, 1U> &f, const coder::array<
  real_T, 1U> &x)
{
  real_T a;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  switch (obj->objtype) {
   case 5:
    {
      int32_T b;
      int32_T i;
      boolean_T overflow;
      b = obj->nvar;
      st.site = &ue_emlrtRSI;
      i = obj->nvar - 1;
      overflow = ((1 <= i) && (i > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (int32_T c_i = 0; c_i <= b - 2; c_i++) {
        int32_T b_i;
        i = obj->grad.size(0);
        b_i = c_i + 1;
        if ((b_i < 1) || (b_i > i)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, i, &bc_emlrtBCI, sp);
        }

        obj->grad[b_i - 1] = 0.0;
      }

      i = obj->grad.size(0);
      if ((obj->nvar < 1) || (obj->nvar > i)) {
        emlrtDynamicBoundsCheckR2012b(obj->nvar, 1, i, &bc_emlrtBCI, sp);
      }

      obj->grad[obj->nvar - 1] = obj->gammaScalar;
    }
    break;

   case 3:
    {
      int32_T b;
      boolean_T overflow;
      st.site = &ue_emlrtRSI;
      d_xgemv(&st, obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      b = obj->nvar;
      st.site = &ue_emlrtRSI;
      overflow = ((1 <= obj->nvar) && (obj->nvar > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (int32_T c_i = 0; c_i < b; c_i++) {
        int32_T i;
        int32_T b_i;
        b_i = c_i + 1;
        i = obj->Hx.size(0);
        if ((b_i < 1) || (b_i > i)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, i, &bc_emlrtBCI, sp);
        }

        i = obj->grad.size(0);
        if (b_i > i) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, i, &bc_emlrtBCI, sp);
        }

        obj->grad[b_i - 1] = obj->Hx[b_i - 1];
      }

      if (obj->hasLinear) {
        st.site = &ue_emlrtRSI;
        if (obj->nvar >= 1) {
          a = 1.0;
          n_t = (ptrdiff_t)obj->nvar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          daxpy(&n_t, &a, &(((coder::array<real_T, 1U> *)&f)->data())[0],
                &incx_t, &(obj->grad.data())[0], &incy_t);
        }
      }
    }
    break;

   case 4:
    {
      int32_T b;
      int32_T i;
      int32_T b_i;
      boolean_T overflow;
      int32_T c_i;
      b = obj->maxVar - 1;
      st.site = &ue_emlrtRSI;
      d_xgemv(&st, obj->nvar, obj->nvar, H, obj->nvar, x, obj->Hx);
      b_i = obj->nvar + 1;
      st.site = &ue_emlrtRSI;
      overflow = ((b_i <= obj->maxVar - 1) && (obj->maxVar - 1 > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (c_i = b_i; c_i <= b; c_i++) {
        if ((c_i < 1) || (c_i > x.size(0))) {
          emlrtDynamicBoundsCheckR2012b(c_i, 1, x.size(0), &bc_emlrtBCI, sp);
        }

        i = obj->Hx.size(0);
        if (c_i > i) {
          emlrtDynamicBoundsCheckR2012b(c_i, 1, i, &bc_emlrtBCI, sp);
        }

        obj->Hx[c_i - 1] = 0.0 * x[c_i - 1];
      }

      st.site = &ue_emlrtRSI;
      overflow = ((1 <= obj->maxVar - 1) && (obj->maxVar - 1 > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (c_i = 0; c_i < b; c_i++) {
        b_i = c_i + 1;
        i = obj->Hx.size(0);
        if ((b_i < 1) || (b_i > i)) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, i, &bc_emlrtBCI, sp);
        }

        i = obj->grad.size(0);
        if (b_i > i) {
          emlrtDynamicBoundsCheckR2012b(b_i, 1, i, &bc_emlrtBCI, sp);
        }

        obj->grad[b_i - 1] = obj->Hx[b_i - 1];
      }

      if (obj->hasLinear) {
        st.site = &ue_emlrtRSI;
        if (obj->nvar >= 1) {
          a = 1.0;
          n_t = (ptrdiff_t)obj->nvar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          daxpy(&n_t, &a, &(((coder::array<real_T, 1U> *)&f)->data())[0],
                &incx_t, &(obj->grad.data())[0], &incy_t);
        }
      }
    }
    break;
  }
}

// End of code generation (computeGrad_StoreHx.cpp)

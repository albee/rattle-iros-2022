//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFval_ReuseHx.cpp
//
//  Code generation for function 'computeFval_ReuseHx'
//


// Include files
#include "computeFval_ReuseHx.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo ve_emlrtRSI = { 1,  // lineNo
  "computeFval_ReuseHx",               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/computeFval_ReuseHx.p"// pathName 
};

static emlrtBCInfo cc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "computeFval_ReuseHx",               // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/computeFval_ReuseHx.p",// pName 
  0                                    // checkKind
};

// Function Definitions
real_T computeFval_ReuseHx(const emlrtStack *sp, const d_struct_T *obj, coder::
  array<real_T, 2U> &workspace, const coder::array<real_T, 1U> &f, const coder::
  array<real_T, 1U> &x)
{
  real_T val;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  val = 0.0;
  switch (obj->objtype) {
   case 5:
    if ((obj->nvar < 1) || (obj->nvar > x.size(0))) {
      emlrtDynamicBoundsCheckR2012b(obj->nvar, 1, x.size(0), &cc_emlrtBCI, sp);
    }

    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int32_T b;
        boolean_T overflow;
        int32_T b_i;
        int32_T c_i;
        int32_T i1;
        b = obj->nvar;
        st.site = &ve_emlrtRSI;
        overflow = ((1 <= obj->nvar) && (obj->nvar > 2147483646));
        if (overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (int32_T i = 0; i < b; i++) {
          b_i = i + 1;
          c_i = workspace.size(0) * workspace.size(1);
          if ((b_i < 1) || (b_i > c_i)) {
            emlrtDynamicBoundsCheckR2012b(b_i, 1, c_i, &cc_emlrtBCI, sp);
          }

          c_i = workspace.size(0);
          i1 = obj->Hx.size(0);
          if (b_i > i1) {
            emlrtDynamicBoundsCheckR2012b(b_i, 1, i1, &cc_emlrtBCI, sp);
          }

          if (b_i > f.size(0)) {
            emlrtDynamicBoundsCheckR2012b(b_i, 1, f.size(0), &cc_emlrtBCI, sp);
          }

          workspace[(b_i - 1) % c_i * workspace.size(1) + (b_i - 1) / c_i] = 0.5
            * obj->Hx[b_i - 1] + f[b_i - 1];
        }

        st.site = &ve_emlrtRSI;
        if (obj->nvar >= 1) {
          n_t = (ptrdiff_t)obj->nvar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&kc_emlrtRTEI), (&st), workspace.size(1), workspace.size(0));
          b_i = workspace.size(1);
          for (c_i = 0; c_i < b_i; c_i++) {
            b = workspace.size(0);
            for (i1 = 0; i1 < b; i1++) {
              r[i1 + r.size(1) * c_i] = workspace[c_i + workspace.size(1) * i1];
            }
          }

          val = ddot(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0],
                     &incx_t, &(r.data())[0], &incy_t);
        }
      } else {
        st.site = &ve_emlrtRSI;
        if (obj->nvar >= 1) {
          n_t = (ptrdiff_t)obj->nvar;
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          val = ddot(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0],
                     &incx_t, &(((coder::array<real_T, 1U> *)&obj->Hx)->data())
                     [0], &incy_t);
        }

        val *= 0.5;
      }
    }
    break;

   case 4:
    {
      int32_T maxRegVar_tmp;
      maxRegVar_tmp = obj->maxVar - 1;
      if (obj->hasLinear) {
        int32_T b;
        boolean_T overflow;
        int32_T i;
        int32_T b_i;
        int32_T c_i;
        int32_T i1;
        b = obj->nvar;
        st.site = &ve_emlrtRSI;
        overflow = ((1 <= obj->nvar) && (obj->nvar > 2147483646));
        if (overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (i = 0; i < b; i++) {
          c_i = workspace.size(0) * workspace.size(1);
          i1 = i + 1;
          if ((i1 < 1) || (i1 > c_i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, c_i, &cc_emlrtBCI, sp);
          }

          c_i = workspace.size(0);
          i1 = i + 1;
          if ((i1 < 1) || (i1 > f.size(0))) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, f.size(0), &cc_emlrtBCI, sp);
          }

          workspace[i % c_i * workspace.size(1) + i / c_i] = f[i1 - 1];
        }

        b = (obj->maxVar - obj->nvar) - 1;
        st.site = &ve_emlrtRSI;
        overflow = ((1 <= b) && (b > 2147483646));
        if (overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (i = 0; i < b; i++) {
          c_i = workspace.size(0) * workspace.size(1);
          i1 = obj->nvar + i;
          b_i = i1 + 1;
          if ((b_i < 1) || (b_i > c_i)) {
            emlrtDynamicBoundsCheckR2012b(b_i, 1, c_i, &cc_emlrtBCI, sp);
          }

          c_i = workspace.size(0);
          workspace[i1 % c_i * workspace.size(1) + i1 / c_i] = 0.0;
        }

        st.site = &ve_emlrtRSI;
        overflow = ((1 <= maxRegVar_tmp) && (maxRegVar_tmp > 2147483646));
        if (overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (i = 0; i < maxRegVar_tmp; i++) {
          c_i = workspace.size(0) * workspace.size(1);
          i1 = i + 1;
          if ((i1 < 1) || (i1 > c_i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, c_i, &cc_emlrtBCI, sp);
          }

          c_i = workspace.size(0) * workspace.size(1);
          i1 = i + 1;
          if ((i1 < 1) || (i1 > c_i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, c_i, &cc_emlrtBCI, sp);
          }

          c_i = workspace.size(0);
          i1 = workspace.size(0);
          b_i = obj->Hx.size(0);
          b = i + 1;
          if ((b < 1) || (b > b_i)) {
            emlrtDynamicBoundsCheckR2012b(b, 1, b_i, &cc_emlrtBCI, sp);
          }

          workspace[i % c_i * workspace.size(1) + i / c_i] = workspace[i % i1 *
            workspace.size(1) + i / i1] + 0.5 * obj->Hx[b - 1];
        }

        st.site = &ve_emlrtRSI;
        if (maxRegVar_tmp >= 1) {
          n_t = (ptrdiff_t)(obj->maxVar - 1);
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&kc_emlrtRTEI), (&st), workspace.size(1), workspace.size(0));
          b_i = workspace.size(1);
          for (c_i = 0; c_i < b_i; c_i++) {
            b = workspace.size(0);
            for (i1 = 0; i1 < b; i1++) {
              r[i1 + r.size(1) * c_i] = workspace[c_i + workspace.size(1) * i1];
            }
          }

          val = ddot(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0],
                     &incx_t, &(r.data())[0], &incy_t);
        }
      } else {
        boolean_T overflow;
        int32_T b_i;
        st.site = &ve_emlrtRSI;
        if (maxRegVar_tmp >= 1) {
          n_t = (ptrdiff_t)(obj->maxVar - 1);
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          val = ddot(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0],
                     &incx_t, &(((coder::array<real_T, 1U> *)&obj->Hx)->data())
                     [0], &incy_t);
        }

        val *= 0.5;
        b_i = obj->nvar + 1;
        st.site = &ve_emlrtRSI;
        overflow = ((b_i <= maxRegVar_tmp) && (maxRegVar_tmp > 2147483646));
        if (overflow) {
          b_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&b_st);
        }

        for (int32_T b = b_i; b <= maxRegVar_tmp; b++) {
          if ((b < 1) || (b > x.size(0))) {
            emlrtDynamicBoundsCheckR2012b(b, 1, x.size(0), &cc_emlrtBCI, sp);
          }

          val += x[b - 1] * 0.0;
        }
      }
    }
    break;
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return val;
}

// End of code generation (computeFval_ReuseHx.cpp)

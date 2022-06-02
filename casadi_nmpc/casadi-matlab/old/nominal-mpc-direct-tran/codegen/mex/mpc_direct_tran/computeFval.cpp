//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFval.cpp
//
//  Code generation for function 'computeFval'
//


// Include files
#include "computeFval.h"
#include "blas.h"
#include "eml_int_forloop_overflow_check.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo kd_emlrtRSI = { 49, // lineNo
  "xdot",                              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xdot.m"// pathName 
};

static emlrtRSInfo qe_emlrtRSI = { 1,  // lineNo
  "computeFval",                       // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/computeFval.p"// pathName 
};

static emlrtRSInfo se_emlrtRSI = { 1,  // lineNo
  "linearFormReg_",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/linearFormReg_.p"// pathName 
};

static emlrtBCInfo sb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "linearFormReg_",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/linearFormReg_.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo tb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "computeFval",                       // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+Objective/computeFval.p",// pName 
  0                                    // checkKind
};

// Function Definitions
real_T computeFval(const emlrtStack *sp, const d_struct_T *obj, coder::array<
                   real_T, 2U> &workspace, const coder::array<real_T, 2U> &H,
                   const coder::array<real_T, 1U> &f, const coder::array<real_T,
                   1U> &x)
{
  real_T val;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
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
  switch (obj->objtype) {
   case 5:
    if ((obj->nvar < 1) || (obj->nvar > x.size(0))) {
      emlrtDynamicBoundsCheckR2012b(obj->nvar, 1, x.size(0), &tb_emlrtBCI, sp);
    }

    val = x[obj->nvar - 1];
    break;

   case 3:
    {
      st.site = &qe_emlrtRSI;
      linearForm_(&st, obj->hasLinear, obj->nvar, workspace, H, f, x);
      st.site = &qe_emlrtRSI;
      if (obj->nvar < 1) {
        val = 0.0;
      } else {
        int32_T a_tmp;
        b_st.site = &kd_emlrtRSI;
        n_t = (ptrdiff_t)obj->nvar;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        r.set_size((&kc_emlrtRTEI), (&st), workspace.size(1), workspace.size(0));
        a_tmp = workspace.size(1);
        for (int32_T i = 0; i < a_tmp; i++) {
          int32_T b_tmp;
          b_tmp = workspace.size(0);
          for (int32_T idx = 0; idx < b_tmp; idx++) {
            r[idx + r.size(1) * i] = workspace[i + workspace.size(1) * idx];
          }
        }

        val = ddot(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0], &incx_t,
                   &(r.data())[0], &incy_t);
      }
    }
    break;

   default:
    {
      int32_T a_tmp;
      int32_T b_tmp;
      boolean_T overflow;
      int32_T idx;
      int32_T i;
      st.site = &qe_emlrtRSI;
      linearForm_(&st, obj->hasLinear, obj->nvar, workspace, H, f, x);
      st.site = &qe_emlrtRSI;
      a_tmp = obj->nvar + 1;
      b_tmp = obj->maxVar - 1;
      b_st.site = &se_emlrtRSI;
      overflow = ((a_tmp <= b_tmp) && (b_tmp > 2147483646));
      if (overflow) {
        c_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&c_st);
      }

      for (idx = a_tmp; idx <= b_tmp; idx++) {
        i = workspace.size(0) * workspace.size(1);
        if ((idx < 1) || (idx > i)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, i, &sb_emlrtBCI, &st);
        }

        i = workspace.size(0);
        if (idx > x.size(0)) {
          emlrtDynamicBoundsCheckR2012b(idx, 1, x.size(0), &sb_emlrtBCI, &st);
        }

        workspace[(idx - 1) % i * workspace.size(1) + (idx - 1) / i] = 0.0 *
          x[idx - 1];
      }

      st.site = &qe_emlrtRSI;
      if (b_tmp < 1) {
        val = 0.0;
      } else {
        b_st.site = &kd_emlrtRSI;
        n_t = (ptrdiff_t)(obj->maxVar - 1);
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        r.set_size((&kc_emlrtRTEI), (&st), workspace.size(1), workspace.size(0));
        a_tmp = workspace.size(1);
        for (i = 0; i < a_tmp; i++) {
          b_tmp = workspace.size(0);
          for (idx = 0; idx < b_tmp; idx++) {
            r[idx + r.size(1) * i] = workspace[i + workspace.size(1) * idx];
          }
        }

        val = ddot(&n_t, &(((coder::array<real_T, 1U> *)&x)->data())[0], &incx_t,
                   &(r.data())[0], &incy_t);
      }
    }
    break;
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return val;
}

// End of code generation (computeFval.cpp)

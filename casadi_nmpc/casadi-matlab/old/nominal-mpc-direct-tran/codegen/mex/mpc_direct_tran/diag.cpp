//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  diag.cpp
//
//  Code generation for function 'diag'
//


// Include files
#include "diag.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo t_emlrtRSI = { 90,  // lineNo
  "diag",                              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/diag.m"// pathName
};

static emlrtRTEInfo db_emlrtRTEI = { 82,// lineNo
  5,                                   // colNo
  "diag",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/diag.m"// pName
};

// Function Definitions
void diag(const emlrtStack *sp, const coder::array<real_T, 2U> &v, coder::array<
          real_T, 2U> &d)
{
  int32_T nv;
  int32_T loop_ub;
  boolean_T overflow;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  nv = v.size(1);
  d.set_size((&db_emlrtRTEI), sp, v.size(1), v.size(1));
  loop_ub = v.size(1) * v.size(1);
  for (int32_T i = 0; i < loop_ub; i++) {
    d[i] = 0.0;
  }

  st.site = &t_emlrtRSI;
  overflow = ((1 <= v.size(1)) && (v.size(1) > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (loop_ub = 0; loop_ub < nv; loop_ub++) {
    d[loop_ub + d.size(1) * loop_ub] = v[loop_ub];
  }
}

// End of code generation (diag.cpp)

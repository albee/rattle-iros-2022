//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  all.cpp
//
//  Code generation for function 'all'
//


// Include files
#include "all.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo db_emlrtRSI = { 13, // lineNo
  "all",                               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/ops/all.m"// pathName
};

static emlrtRSInfo eb_emlrtRSI = { 121,// lineNo
  "allOrAny",                          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/allOrAny.m"// pathName 
};

static emlrtRSInfo fb_emlrtRSI = { 65, // lineNo
  "applyVectorFunction",               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/applyVectorFunction.m"// pathName 
};

static emlrtRSInfo gb_emlrtRSI = { 89, // lineNo
  "looper",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/applyVectorFunction.m"// pathName 
};

static emlrtRSInfo hb_emlrtRSI = { 101,// lineNo
  "looper",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/applyVectorFunction.m"// pathName 
};

static emlrtRSInfo ib_emlrtRSI = { 39, // lineNo
  "function_handle/parenReference",    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/function_handle.m"// pathName 
};

static emlrtRSInfo jb_emlrtRSI = { 115,// lineNo
  "@(X,indToSubX,Y,ind2SubY)allFun(X,indToSubX,Y,ind2SubY,n)",// fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/allOrAny.m"// pathName 
};

static emlrtRSInfo kb_emlrtRSI = { 177,// lineNo
  "allFun",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/allOrAny.m"// pathName 
};

// Function Definitions
boolean_T all(const emlrtStack *sp, const coder::array<boolean_T, 1U> &x)
{
  boolean_T y;
  boolean_T overflow;
  int32_T k;
  boolean_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  emlrtStack e_st;
  emlrtStack f_st;
  emlrtStack g_st;
  emlrtStack h_st;
  emlrtStack i_st;
  emlrtStack j_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &db_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  e_st.prev = &d_st;
  e_st.tls = d_st.tls;
  f_st.prev = &e_st;
  f_st.tls = e_st.tls;
  g_st.prev = &f_st;
  g_st.tls = f_st.tls;
  h_st.prev = &g_st;
  h_st.tls = g_st.tls;
  i_st.prev = &h_st;
  i_st.tls = h_st.tls;
  j_st.prev = &i_st;
  j_st.tls = i_st.tls;
  b_st.site = &eb_emlrtRSI;
  c_st.site = &fb_emlrtRSI;
  d_st.site = &gb_emlrtRSI;
  e_st.site = &gb_emlrtRSI;
  f_st.site = &hb_emlrtRSI;
  g_st.site = &ib_emlrtRSI;
  h_st.site = &jb_emlrtRSI;
  y = true;
  i_st.site = &kb_emlrtRSI;
  overflow = ((1 <= x.size(0)) && (x.size(0) > 2147483646));
  if (overflow) {
    j_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&j_st);
  }

  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= x.size(0) - 1)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

// End of code generation (all.cpp)

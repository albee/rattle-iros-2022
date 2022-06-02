//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  eye.cpp
//
//  Code generation for function 'eye'
//


// Include files
#include "eye.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include <string.h>

// Variable Definitions
static emlrtRSInfo x_emlrtRSI = { 50,  // lineNo
  "eye",                               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/eye.m"// pathName
};

static emlrtRSInfo y_emlrtRSI = { 96,  // lineNo
  "eye",                               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/eye.m"// pathName
};

static emlrtRSInfo ab_emlrtRSI = { 21, // lineNo
  "checkAndSaturateExpandSize",        // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/checkAndSaturateExpandSize.m"// pathName 
};

static emlrtRTEInfo eb_emlrtRTEI = { 94,// lineNo
  5,                                   // colNo
  "eye",                               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/eye.m"// pName
};

// Function Definitions
void b_eye(const emlrtStack *sp, real_T varargin_1, coder::array<real_T, 2U>
           &b_I)
{
  real_T n;
  int32_T m_tmp;
  int32_T loop_ub;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &x_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b_st.site = &ab_emlrtRSI;
  if ((varargin_1 != muDoubleScalarFloor(varargin_1)) || muDoubleScalarIsInf
      (varargin_1) || (varargin_1 < -2.147483648E+9) || (varargin_1 >
       2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &e_emlrtRTEI,
      "Coder:MATLAB:NonIntegerInput", "Coder:MATLAB:NonIntegerInput", 4, 12,
      MIN_int32_T, 12, MAX_int32_T);
  }

  if (varargin_1 <= 0.0) {
    n = 0.0;
  } else {
    n = varargin_1;
  }

  if (!(n <= 2.147483647E+9)) {
    emlrtErrorWithMessageIdR2018a(&b_st, &f_emlrtRTEI, "Coder:MATLAB:pmaxsize",
      "Coder:MATLAB:pmaxsize", 0);
  }

  m_tmp = static_cast<int32_T>(varargin_1);
  b_I.set_size((&eb_emlrtRTEI), sp, m_tmp, m_tmp);
  loop_ub = m_tmp * m_tmp;
  for (int32_T i = 0; i < loop_ub; i++) {
    b_I[i] = 0.0;
  }

  if (m_tmp > 0) {
    st.site = &y_emlrtRSI;
    if (m_tmp > 2147483646) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (loop_ub = 0; loop_ub < m_tmp; loop_ub++) {
      b_I[loop_ub + b_I.size(1) * loop_ub] = 1.0;
    }
  }
}

void eye(real_T b_I[36])
{
  memset(&b_I[0], 0, 36U * sizeof(real_T));
  for (int32_T k = 0; k < 6; k++) {
    b_I[k + 6 * k] = 1.0;
  }
}

// End of code generation (eye.cpp)

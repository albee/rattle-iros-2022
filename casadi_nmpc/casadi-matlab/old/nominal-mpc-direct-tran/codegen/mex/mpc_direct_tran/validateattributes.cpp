//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  validateattributes.cpp
//
//  Code generation for function 'validateattributes'
//


// Include files
#include "validateattributes.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo xb_emlrtRSI = { 76, // lineNo
  "validateattributes",                // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/lang/validateattributes.m"// pathName 
};

static emlrtRTEInfo j_emlrtRTEI = { 13,// lineNo
  37,                                  // colNo
  "validatesquare",                    // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+valattr/validatesquare.m"// pName 
};

static emlrtRTEInfo k_emlrtRTEI = { 13,// lineNo
  37,                                  // colNo
  "validatenonempty",                  // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+valattr/validatenonempty.m"// pName 
};

// Function Definitions
void validateattributes(const emlrtStack *sp, const coder::array<real_T, 2U> &a)
{
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &xb_emlrtRSI;
  if (a.size(0) != a.size(1)) {
    emlrtErrorWithMessageIdR2018a(&st, &j_emlrtRTEI,
      "Coder:toolbox:ValidateattributesexpectedSquare", "MATLAB:expectedSquare",
      3, 4, 5, "input");
  }

  st.site = &xb_emlrtRSI;
  if (a.size(1) == 0) {
    emlrtErrorWithMessageIdR2018a(&st, &k_emlrtRTEI,
      "Coder:toolbox:ValidateattributesexpectedNonempty",
      "MATLAB:expectedNonempty", 3, 4, 5, "input");
  }
}

// End of code generation (validateattributes.cpp)

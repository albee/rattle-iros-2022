//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  indexShapeCheck.cpp
//
//  Code generation for function 'indexShapeCheck'
//


// Include files
#include "indexShapeCheck.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo yf_emlrtRSI = { 43, // lineNo
  "indexShapeCheck",                   // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/indexShapeCheck.m"// pathName 
};

static emlrtRTEInfo l_emlrtRTEI = { 121,// lineNo
  5,                                   // colNo
  "errOrWarnIf",                       // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/indexShapeCheck.m"// pName 
};

// Function Definitions
void b_indexShapeCheck(const emlrtStack *sp, int32_T matrixSize)
{
  boolean_T c;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  c = (matrixSize == 1);
  st.site = &yf_emlrtRSI;
  if (c) {
    emlrtErrorWithMessageIdR2018a(&st, &l_emlrtRTEI,
      "Coder:FE:PotentialVectorVector", "Coder:FE:PotentialVectorVector", 0);
  }
}

void indexShapeCheck(const emlrtStack *sp, int32_T matrixSize, const int32_T
                     indexSize[2])
{
  boolean_T c;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  if ((matrixSize != 1) || (indexSize[1] == 1)) {
    c = false;
  } else {
    c = true;
  }

  st.site = &yf_emlrtRSI;
  if (c) {
    emlrtErrorWithMessageIdR2018a(&st, &l_emlrtRTEI,
      "Coder:FE:PotentialVectorVector", "Coder:FE:PotentialVectorVector", 0);
  }
}

// End of code generation (indexShapeCheck.cpp)

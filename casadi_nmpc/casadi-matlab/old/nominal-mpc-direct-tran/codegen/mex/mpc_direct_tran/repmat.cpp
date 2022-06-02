//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  repmat.cpp
//
//  Code generation for function 'repmat'
//


// Include files
#include "repmat.h"
#include "assertValidSizeArg.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo bb_emlrtRSI = { 28, // lineNo
  "repmat",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/repmat.m"// pathName
};

static emlrtMCInfo emlrtMCI = { 47,    // lineNo
  5,                                   // colNo
  "repmat",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/repmat.m"// pName
};

static emlrtDCInfo u_emlrtDCI = { 31,  // lineNo
  14,                                  // colNo
  "repmat",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/repmat.m",// pName
  4                                    // checkKind
};

static emlrtRTEInfo fb_emlrtRTEI = { 1,// lineNo
  14,                                  // colNo
  "repmat",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/repmat.m"// pName
};

static const char_T cv[15] = { 'M', 'A', 'T', 'L', 'A', 'B', ':', 'p', 'm', 'a',
  'x', 's', 'i', 'z', 'e' };

static emlrtRSInfo ag_emlrtRSI = { 47, // lineNo
  "repmat",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/repmat.m"// pathName
};

// Function Declarations
static void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location);

// Function Definitions
static void error(const emlrtStack *sp, const mxArray *b, emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = b;
  emlrtCallMATLABR2012b(sp, 0, NULL, 1, &pArray, "error", true, location);
}

void b_repmat(const emlrtStack *sp, real_T varargin_1, coder::array<real_T, 1U>
              &b)
{
  int32_T outsize_idx_0_tmp;
  int32_T outsize_idx_0;
  const mxArray *y;
  static const int32_T iv[2] = { 1, 15 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bb_emlrtRSI;
  assertValidSizeArg(&st, varargin_1);
  if (!(varargin_1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(varargin_1, &u_emlrtDCI, sp);
  }

  outsize_idx_0_tmp = static_cast<int32_T>(varargin_1);
  outsize_idx_0 = 6 * outsize_idx_0_tmp;
  if (!(outsize_idx_0 == 6.0 * static_cast<real_T>(outsize_idx_0_tmp))) {
    const mxArray *m;
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(sp, 15, m, &cv[0]);
    emlrtAssign(&y, m);
    st.site = &ag_emlrtRSI;
    error(&st, y, &emlrtMCI);
  }

  b.set_size((&fb_emlrtRTEI), sp, outsize_idx_0);
  if (outsize_idx_0 != 0) {
    outsize_idx_0_tmp--;
    for (int32_T t = 0; t <= outsize_idx_0_tmp; t++) {
      outsize_idx_0 = t * 6;
      for (int32_T k = 0; k < 6; k++) {
        b[outsize_idx_0 + k] = 10.0;
      }
    }
  }
}

void repmat(const emlrtStack *sp, real_T varargin_1, coder::array<real_T, 1U> &b)
{
  int32_T outsize_idx_0_tmp;
  int32_T outsize_idx_0;
  const mxArray *y;
  static const int32_T iv[2] = { 1, 15 };

  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &bb_emlrtRSI;
  assertValidSizeArg(&st, varargin_1);
  if (!(varargin_1 >= 0.0)) {
    emlrtNonNegativeCheckR2012b(varargin_1, &u_emlrtDCI, sp);
  }

  outsize_idx_0_tmp = static_cast<int32_T>(varargin_1);
  outsize_idx_0 = 3 * outsize_idx_0_tmp;
  if (!(outsize_idx_0 == 3.0 * static_cast<real_T>(outsize_idx_0_tmp))) {
    const mxArray *m;
    y = NULL;
    m = emlrtCreateCharArray(2, &iv[0]);
    emlrtInitCharArrayR2013a(sp, 15, m, &cv[0]);
    emlrtAssign(&y, m);
    st.site = &ag_emlrtRSI;
    error(&st, y, &emlrtMCI);
  }

  b.set_size((&fb_emlrtRTEI), sp, outsize_idx_0);
  if (outsize_idx_0 != 0) {
    outsize_idx_0_tmp--;
    for (int32_T t = 0; t <= outsize_idx_0_tmp; t++) {
      outsize_idx_0 = t * 3;
      b[outsize_idx_0] = 0.4;
      b[outsize_idx_0 + 1] = 0.4;
      b[outsize_idx_0 + 2] = 0.4;
    }
  }
}

// End of code generation (repmat.cpp)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  countsort.cpp
//
//  Code generation for function 'countsort'
//


// Include files
#include "countsort.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo md_emlrtRSI = { 1,  // lineNo
  "countsort",                         // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+utils/countsort.p"// pathName 
};

static emlrtBCInfo gb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "countsort",                         // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+utils/countsort.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void countsort(const emlrtStack *sp, coder::array<int32_T, 1U> &x, int32_T xLen,
               coder::array<int32_T, 1U> &workspace, int32_T xMin, int32_T xMax)
{
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if ((xLen > 1) && (xMax > xMin)) {
    int32_T b_tmp;
    boolean_T overflow;
    int32_T idx;
    int32_T i;
    int32_T idxFill;
    int32_T idxEnd;
    int32_T idxStart;
    b_tmp = (xMax - xMin) + 1;
    st.site = &md_emlrtRSI;
    overflow = ((1 <= b_tmp) && (b_tmp > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < b_tmp; idx++) {
      i = workspace.size(0);
      idxFill = idx + 1;
      if ((idxFill < 1) || (idxFill > i)) {
        emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &gb_emlrtBCI, sp);
      }

      workspace[idxFill - 1] = 0;
    }

    st.site = &md_emlrtRSI;
    if (xLen > 2147483646) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < xLen; idx++) {
      i = workspace.size(0);
      idxFill = x.size(0);
      idxEnd = idx + 1;
      if (idxEnd > idxFill) {
        emlrtDynamicBoundsCheckR2012b(idxEnd, 1, idxFill, &gb_emlrtBCI, sp);
      }

      idxFill = (x[idxEnd - 1] - xMin) + 1;
      if ((idxFill < 1) || (idxFill > i)) {
        emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &gb_emlrtBCI, sp);
      }

      i = workspace.size(0);
      idxEnd = x.size(0);
      idxStart = idx + 1;
      if (idxStart > idxEnd) {
        emlrtDynamicBoundsCheckR2012b(idxStart, 1, idxEnd, &gb_emlrtBCI, sp);
      }

      idxEnd = (x[idxStart - 1] - xMin) + 1;
      if ((idxEnd < 1) || (idxEnd > i)) {
        emlrtDynamicBoundsCheckR2012b(idxEnd, 1, i, &gb_emlrtBCI, sp);
      }

      workspace[idxEnd - 1] = workspace[idxFill - 1] + 1;
    }

    st.site = &md_emlrtRSI;
    overflow = ((2 <= b_tmp) && (b_tmp > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 2; idx <= b_tmp; idx++) {
      i = workspace.size(0);
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &gb_emlrtBCI, sp);
      }

      i = workspace.size(0);
      idxFill = idx - 1;
      if (idxFill > i) {
        emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &gb_emlrtBCI, sp);
      }

      i = workspace.size(0);
      if (idx > i) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &gb_emlrtBCI, sp);
      }

      workspace[idx - 1] = workspace[idx - 1] + workspace[idxFill - 1];
    }

    idxStart = 1;
    idxEnd = workspace[0];
    st.site = &md_emlrtRSI;
    overflow = ((1 <= b_tmp - 1) && (b_tmp - 1 > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx <= b_tmp - 2; idx++) {
      st.site = &md_emlrtRSI;
      overflow = ((idxStart <= idxEnd) && (idxEnd > 2147483646));
      if (overflow) {
        b_st.site = &u_emlrtRSI;
        check_forloop_overflow_error(&b_st);
      }

      for (idxFill = idxStart; idxFill <= idxEnd; idxFill++) {
        i = x.size(0);
        if ((idxFill < 1) || (idxFill > i)) {
          emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &gb_emlrtBCI, sp);
        }

        x[idxFill - 1] = idx + xMin;
      }

      i = workspace.size(0);
      idxFill = idx + 1;
      if ((idxFill < 1) || (idxFill > i)) {
        emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &gb_emlrtBCI, sp);
      }

      idxStart = workspace[idxFill - 1] + 1;
      i = workspace.size(0);
      idxFill = idx + 2;
      if ((idxFill < 1) || (idxFill > i)) {
        emlrtDynamicBoundsCheckR2012b(idxFill, 1, i, &gb_emlrtBCI, sp);
      }

      idxEnd = workspace[idxFill - 1];
    }

    st.site = &md_emlrtRSI;
    overflow = ((idxStart <= idxEnd) && (idxEnd > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = idxStart; idx <= idxEnd; idx++) {
      i = x.size(0);
      if ((idx < 1) || (idx > i)) {
        emlrtDynamicBoundsCheckR2012b(idx, 1, i, &gb_emlrtBCI, sp);
      }

      x[idx - 1] = xMax;
    }
  }
}

// End of code generation (countsort.cpp)

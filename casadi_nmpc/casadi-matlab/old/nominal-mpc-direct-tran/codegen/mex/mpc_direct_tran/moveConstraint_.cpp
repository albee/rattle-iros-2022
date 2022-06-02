//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  moveConstraint_.cpp
//
//  Code generation for function 'moveConstraint_'
//


// Include files
#include "moveConstraint_.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo pd_emlrtRSI = { 1,  // lineNo
  "moveConstraint_",                   // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/moveConstraint_.p"// pathName 
};

static emlrtBCInfo hb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "moveConstraint_",                   // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/moveConstraint_.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void moveConstraint_(const emlrtStack *sp, g_struct_T *obj, int32_T
                     idx_global_start, int32_T idx_global_dest)
{
  int32_T i;
  int32_T b;
  boolean_T overflow;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  i = obj->Wid.size(0);
  if ((idx_global_start < 1) || (idx_global_start > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &hb_emlrtBCI, sp);
  }

  i = obj->Wid.size(0);
  if ((idx_global_dest < 1) || (idx_global_dest > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &hb_emlrtBCI, sp);
  }

  obj->Wid[idx_global_dest - 1] = obj->Wid[idx_global_start - 1];
  i = obj->Wlocalidx.size(0);
  if (idx_global_start > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &hb_emlrtBCI, sp);
  }

  i = obj->Wlocalidx.size(0);
  if (idx_global_dest > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &hb_emlrtBCI, sp);
  }

  obj->Wlocalidx[idx_global_dest - 1] = obj->Wlocalidx[idx_global_start - 1];
  b = obj->nVar;
  st.site = &pd_emlrtRSI;
  overflow = ((1 <= obj->nVar) && (obj->nVar > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (int32_T idx = 0; idx < b; idx++) {
    int32_T b_idx;
    b_idx = idx + 1;
    i = obj->ATwset.size(1);
    if (idx_global_start > i) {
      emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &hb_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if ((b_idx < 1) || (b_idx > i)) {
      emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &hb_emlrtBCI, sp);
    }

    i = obj->ATwset.size(1);
    if (idx_global_dest > i) {
      emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &hb_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if (b_idx > i) {
      emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &hb_emlrtBCI, sp);
    }

    obj->ATwset[(idx_global_dest + obj->ATwset.size(1) * (b_idx - 1)) - 1] =
      obj->ATwset[(idx_global_start + obj->ATwset.size(1) * (b_idx - 1)) - 1];
  }

  i = obj->bwset.size(0);
  if (idx_global_start > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_start, 1, i, &hb_emlrtBCI, sp);
  }

  i = obj->bwset.size(0);
  if (idx_global_dest > i) {
    emlrtDynamicBoundsCheckR2012b(idx_global_dest, 1, i, &hb_emlrtBCI, sp);
  }

  obj->bwset[idx_global_dest - 1] = obj->bwset[idx_global_start - 1];
}

// End of code generation (moveConstraint_.cpp)

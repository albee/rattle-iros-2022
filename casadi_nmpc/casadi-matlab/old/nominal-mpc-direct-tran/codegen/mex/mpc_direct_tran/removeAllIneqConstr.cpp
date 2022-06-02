//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  removeAllIneqConstr.cpp
//
//  Code generation for function 'removeAllIneqConstr'
//


// Include files
#include "removeAllIneqConstr.h"
#include "eml_int_forloop_overflow_check.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo oe_emlrtRSI = { 1,  // lineNo
  "removeAllIneqConstr",               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeAllIneqConstr.p"// pathName 
};

static emlrtBCInfo ob_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeAllIneqConstr",               // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeAllIneqConstr.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo pb_emlrtBCI = { 1,  // iFirst
  6,                                   // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeAllIneqConstr",               // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeAllIneqConstr.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void removeAllIneqConstr(const emlrtStack *sp, g_struct_T *obj)
{
  int32_T idxStartIneq;
  int32_T idxEndIneq;
  boolean_T overflow;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  idxStartIneq = (obj->nWConstr[0] + obj->nWConstr[1]) + 1;
  idxEndIneq = obj->nActiveConstr;
  st.site = &oe_emlrtRSI;
  overflow = ((idxStartIneq <= obj->nActiveConstr) && (obj->nActiveConstr >
    2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (int32_T idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++)
  {
    int32_T i;
    int32_T i1;
    i = obj->Wid.size(0);
    if ((idx_global < 1) || (idx_global > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_global, 1, i, &ob_emlrtBCI, sp);
    }

    i = obj->Wlocalidx.size(0);
    if (idx_global > i) {
      emlrtDynamicBoundsCheckR2012b(idx_global, 1, i, &ob_emlrtBCI, sp);
    }

    i = obj->Wid[idx_global - 1];
    if ((i < 1) || (i > 6)) {
      emlrtDynamicBoundsCheckR2012b(obj->Wid[idx_global - 1], 1, 6, &pb_emlrtBCI,
        sp);
    }

    i1 = obj->isActiveConstr.size(0);
    i = (obj->isActiveIdx[i - 1] + obj->Wlocalidx[idx_global - 1]) - 1;
    if ((i < 1) || (i > i1)) {
      emlrtDynamicBoundsCheckR2012b(i, 1, i1, &ob_emlrtBCI, sp);
    }

    obj->isActiveConstr[i - 1] = false;
  }

  obj->nWConstr[2] = 0;
  obj->nWConstr[3] = 0;
  obj->nWConstr[4] = 0;
  obj->nActiveConstr = obj->nWConstr[0] + obj->nWConstr[1];
}

// End of code generation (removeAllIneqConstr.cpp)

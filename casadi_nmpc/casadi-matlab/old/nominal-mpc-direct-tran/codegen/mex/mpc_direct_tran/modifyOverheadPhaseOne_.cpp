//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  modifyOverheadPhaseOne_.cpp
//
//  Code generation for function 'modifyOverheadPhaseOne_'
//


// Include files
#include "modifyOverheadPhaseOne_.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo tb_emlrtRSI = { 1,  // lineNo
  "modifyOverheadPhaseOne_",           // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/modifyOverheadPhaseOne_.p"// pathName 
};

static emlrtBCInfo s_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "modifyOverheadPhaseOne_",           // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/modifyOverheadPhaseOne_.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void modifyOverheadPhaseOne_(const emlrtStack *sp, g_struct_T *obj)
{
  int32_T b;
  boolean_T overflow;
  int32_T idx;
  int32_T i;
  int32_T idxStartIneq;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &tb_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  b = obj->sizes[1];
  st.site = &tb_emlrtRSI;
  overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < b; idx++) {
    i = obj->Aeq.size(1);
    idxStartIneq = idx + 1;
    if ((idxStartIneq < 1) || (idxStartIneq > i)) {
      emlrtDynamicBoundsCheckR2012b(idxStartIneq, 1, i, &s_emlrtBCI, sp);
    }

    i = obj->Aeq.size(0);
    if ((obj->nVar < 1) || (obj->nVar > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &s_emlrtBCI, sp);
    }

    obj->Aeq[(idxStartIneq + obj->Aeq.size(1) * (obj->nVar - 1)) - 1] = 0.0;
    i = obj->ATwset.size(1);
    idxStartIneq = obj->isActiveIdx[1] + idx;
    if ((idxStartIneq < 1) || (idxStartIneq > i)) {
      emlrtDynamicBoundsCheckR2012b(idxStartIneq, 1, i, &s_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if ((obj->nVar < 1) || (obj->nVar > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &s_emlrtBCI, sp);
    }

    obj->ATwset[(idxStartIneq + obj->ATwset.size(1) * (obj->nVar - 1)) - 1] =
      0.0;
  }

  b = obj->sizes[2];
  st.site = &tb_emlrtRSI;
  overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < b; idx++) {
    i = obj->Aineq.size(1);
    idxStartIneq = idx + 1;
    if ((idxStartIneq < 1) || (idxStartIneq > i)) {
      emlrtDynamicBoundsCheckR2012b(idxStartIneq, 1, i, &s_emlrtBCI, sp);
    }

    i = obj->Aineq.size(0);
    if ((obj->nVar < 1) || (obj->nVar > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &s_emlrtBCI, sp);
    }

    obj->Aineq[(idxStartIneq + obj->Aineq.size(1) * (obj->nVar - 1)) - 1] = -1.0;
  }

  i = obj->indexLB.size(0);
  if ((obj->sizes[3] < 1) || (obj->sizes[3] > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->sizes[3], 1, i, &s_emlrtBCI, sp);
  }

  obj->indexLB[obj->sizes[3] - 1] = obj->nVar;
  i = obj->lb.size(0);
  if ((obj->nVar < 1) || (obj->nVar > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &s_emlrtBCI, sp);
  }

  obj->lb[obj->nVar - 1] = obj->SLACK0;
  idxStartIneq = obj->isActiveIdx[2];
  b = obj->nActiveConstr;
  st.site = &tb_emlrtRSI;
  overflow = ((obj->isActiveIdx[2] <= obj->nActiveConstr) && (obj->nActiveConstr
    > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = idxStartIneq; idx <= b; idx++) {
    i = obj->ATwset.size(1);
    if ((idx < 1) || (idx > i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &s_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if ((obj->nVar < 1) || (obj->nVar > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &s_emlrtBCI, sp);
    }

    obj->ATwset[(idx + obj->ATwset.size(1) * (obj->nVar - 1)) - 1] = -1.0;
  }
}

// End of code generation (modifyOverheadPhaseOne_.cpp)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  addBoundToActiveSetMatrix_.cpp
//
//  Code generation for function 'addBoundToActiveSetMatrix_'
//


// Include files
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleX0ForWorkingSet.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo tf_emlrtRSI = { 1,  // lineNo
  "addBoundToActiveSetMatrix_",        // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addBoundToActiveSetMatrix_.p"// pathName 
};

static emlrtBCInfo oc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "addBoundToActiveSetMatrix_",        // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addBoundToActiveSetMatrix_.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void addBoundToActiveSetMatrix_(const emlrtStack *sp, g_struct_T *obj, int32_T
  TYPE, int32_T idx_local)
{
  int32_T i;
  int32_T idx_bnd_local_tmp;
  int32_T idx_global;
  int32_T idx_bnd_local;
  boolean_T overflow;
  int32_T idx;
  int32_T b;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  st.site = &tf_emlrtRSI;
  b_st.prev = &st;
  b_st.tls = st.tls;
  obj->nWConstr[TYPE - 1]++;
  i = obj->isActiveConstr.size(0);
  idx_bnd_local_tmp = (obj->isActiveIdx[TYPE - 1] + idx_local) - 1;
  if ((idx_bnd_local_tmp < 1) || (idx_bnd_local_tmp > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_bnd_local_tmp, 1, i, &xb_emlrtBCI, &st);
  }

  obj->isActiveConstr[idx_bnd_local_tmp - 1] = true;
  obj->nActiveConstr++;
  i = obj->Wid.size(0);
  if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &xb_emlrtBCI, &st);
  }

  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  i = obj->Wlocalidx.size(0);
  if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &xb_emlrtBCI, &st);
  }

  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  idx_global = obj->nActiveConstr;
  switch (TYPE) {
   case 5:
    i = obj->indexUB.size(0);
    if ((idx_local < 1) || (idx_local > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local, 1, i, &oc_emlrtBCI, sp);
    }

    idx_bnd_local = 2147483646;
    i = obj->ub.size(0);
    if (MAX_int32_T > i) {
      emlrtDynamicBoundsCheckR2012b(MAX_int32_T, 1, i, &oc_emlrtBCI, sp);
    }

    i = obj->bwset.size(0);
    if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &oc_emlrtBCI, sp);
    }

    obj->bwset[obj->nActiveConstr - 1] = 1.7976931348623157E+308;
    break;

   default:
    i = obj->indexLB.size(0);
    if ((idx_local < 1) || (idx_local > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_local, 1, i, &oc_emlrtBCI, sp);
    }

    idx_bnd_local_tmp = obj->indexLB[idx_local - 1];
    idx_bnd_local = idx_bnd_local_tmp - 1;
    i = obj->lb.size(0);
    if ((idx_bnd_local_tmp < 1) || (idx_bnd_local_tmp > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->indexLB[idx_local - 1], 1, i,
        &oc_emlrtBCI, sp);
    }

    i = obj->bwset.size(0);
    if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &oc_emlrtBCI, sp);
    }

    obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local];
    break;
  }

  st.site = &tf_emlrtRSI;
  overflow = ((1 <= idx_bnd_local) && (idx_bnd_local > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < idx_bnd_local; idx++) {
    i = obj->ATwset.size(1);
    if ((idx_global < 1) || (idx_global > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_global, 1, i, &oc_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    idx_bnd_local_tmp = idx + 1;
    if ((idx_bnd_local_tmp < 1) || (idx_bnd_local_tmp > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_bnd_local_tmp, 1, i, &oc_emlrtBCI, sp);
    }

    obj->ATwset[(idx_global + obj->ATwset.size(1) * (idx_bnd_local_tmp - 1)) - 1]
      = 0.0;
  }

  i = obj->ATwset.size(1);
  if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &oc_emlrtBCI, sp);
  }

  i = obj->ATwset.size(0);
  idx_bnd_local_tmp = idx_bnd_local + 1;
  if ((idx_bnd_local_tmp < 1) || (idx_bnd_local_tmp > i)) {
    emlrtDynamicBoundsCheckR2012b(idx_bnd_local_tmp, 1, i, &oc_emlrtBCI, sp);
  }

  obj->ATwset[(obj->nActiveConstr + obj->ATwset.size(1) * (idx_bnd_local_tmp - 1))
    - 1] = 2.0 * static_cast<real_T>(TYPE == 5) - 1.0;
  idx_bnd_local_tmp = idx_bnd_local + 2;
  b = obj->nVar;
  st.site = &tf_emlrtRSI;
  overflow = ((idx_bnd_local + 2 <= obj->nVar) && (obj->nVar > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = idx_bnd_local_tmp; idx <= b; idx++) {
    i = obj->ATwset.size(1);
    if ((idx_global < 1) || (idx_global > i)) {
      emlrtDynamicBoundsCheckR2012b(idx_global, 1, i, &oc_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if ((idx < 1) || (idx > i)) {
      emlrtDynamicBoundsCheckR2012b(idx, 1, i, &oc_emlrtBCI, sp);
    }

    obj->ATwset[(idx_global + obj->ATwset.size(1) * (idx - 1)) - 1] = 0.0;
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    i = obj->ATwset.size(1);
    if ((obj->nActiveConstr < 1) || (obj->nActiveConstr > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nActiveConstr, 1, i, &oc_emlrtBCI, sp);
    }

    i = obj->ATwset.size(0);
    if ((obj->nVar < 1) || (obj->nVar > i)) {
      emlrtDynamicBoundsCheckR2012b(obj->nVar, 1, i, &oc_emlrtBCI, sp);
    }

    obj->ATwset[(obj->nActiveConstr + obj->ATwset.size(1) * (obj->nVar - 1)) - 1]
      = -1.0;
    break;
  }
}

// End of code generation (addBoundToActiveSetMatrix_.cpp)

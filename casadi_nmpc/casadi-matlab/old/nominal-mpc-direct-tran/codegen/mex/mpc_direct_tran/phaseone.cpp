//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  phaseone.cpp
//
//  Code generation for function 'phaseone'
//


// Include files
#include "phaseone.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "modifyOverheadPhaseOne_.h"
#include "modifyOverheadRegularized_.h"
#include "moveConstraint_.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "removeAllIneqConstr.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo sb_emlrtRSI = { 1,  // lineNo
  "setProblemType",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/setProblemType.p"// pathName 
};

static emlrtRSInfo pe_emlrtRSI = { 1,  // lineNo
  "phaseone",                          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/phaseone.p"// pathName 
};

static emlrtBCInfo rb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "phaseone",                          // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/phaseone.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void phaseone(const emlrtStack *sp, const coder::array<real_T, 2U> &H, const
              coder::array<real_T, 1U> &f, b_struct_T *solution, c_struct_T
              *memspace, g_struct_T *workingset, e_struct_T *qrmanager,
              f_struct_T *cholmanager, d_struct_T *objective, h_struct_T
              *options, const struct_T *runTimeOptions)
{
  int32_T PROBTYPE_ORIG;
  int32_T nVar;
  int32_T nVarP1_tmp;
  int32_T i;
  int32_T mConstr;
  int32_T i1;
  int32_T mEqFixed;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  PROBTYPE_ORIG = workingset->probType;
  nVar = workingset->nVar;
  nVarP1_tmp = workingset->nVar + 1;
  i = solution->xstar.size(0);
  if ((nVarP1_tmp < 1) || (nVarP1_tmp > i)) {
    emlrtDynamicBoundsCheckR2012b(nVarP1_tmp, 1, i, &rb_emlrtBCI, sp);
  }

  solution->xstar[nVarP1_tmp - 1] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    mConstr = 1;
  } else {
    mConstr = 4;
  }

  st.site = &pe_emlrtRSI;
  removeAllIneqConstr(&st, workingset);
  st.site = &pe_emlrtRSI;
  switch (mConstr) {
   case 3:
    workingset->nVar = workingset->nVarOrig;
    workingset->mConstr = workingset->mConstrOrig;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesNormal[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxNormal[i];
    }
    break;

   case 1:
    workingset->nVar = workingset->nVarOrig + 1;
    workingset->mConstr = workingset->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesPhaseOne[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxPhaseOne[i];
    }

    b_st.site = &sb_emlrtRSI;
    modifyOverheadPhaseOne_(&b_st, workingset);
    break;

   case 2:
    workingset->nVar = workingset->nVarMax - 1;
    workingset->mConstr = workingset->mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesRegularized[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxRegularized[i];
    }

    if (workingset->probType != 4) {
      b_st.site = &sb_emlrtRSI;
      modifyOverheadRegularized_(&b_st, workingset);
    }
    break;

   default:
    workingset->nVar = workingset->nVarMax;
    workingset->mConstr = workingset->mConstrMax;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesRegPhaseOne[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxRegPhaseOne[i];
    }

    b_st.site = &sb_emlrtRSI;
    modifyOverheadPhaseOne_(&b_st, workingset);
    break;
  }

  workingset->probType = mConstr;
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVarP1_tmp;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  options->StepTolerance = 1.4901161193847657E-10;
  st.site = &pe_emlrtRSI;
  solution->fstar = computeFval(&st, objective, memspace->workspace_double, H, f,
    solution->xstar);
  solution->state = 5;
  st.site = &pe_emlrtRSI;
  iterate(&st, H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->ObjectiveLimit, options->StepTolerance,
          runTimeOptions->MaxIterations, runTimeOptions->ConstrRelTolFactor,
          runTimeOptions->ProbRelTolFactor, runTimeOptions->RemainFeasible);
  st.site = &pe_emlrtRSI;
  i = workingset->isActiveConstr.size(0);
  i1 = workingset->isActiveIdx[3] + workingset->sizes[3];
  mConstr = i1 - 1;
  if ((mConstr < 1) || (mConstr > i)) {
    emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &qb_emlrtBCI, &st);
  }

  if (workingset->isActiveConstr[i1 - 2]) {
    boolean_T overflow;
    boolean_T exitg1;
    st.site = &pe_emlrtRSI;
    overflow = ((workingset->sizes[1] + 1 <= workingset->nActiveConstr) &&
                (workingset->nActiveConstr > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    mConstr = workingset->sizes[1] + 1;
    exitg1 = false;
    while ((!exitg1) && (mConstr <= workingset->nActiveConstr)) {
      i = workingset->Wid.size(0);
      if ((mConstr < 1) || (mConstr > i)) {
        emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &rb_emlrtBCI, sp);
      }

      i = workingset->Wid[mConstr - 1];
      if (i == 4) {
        i = workingset->Wlocalidx.size(0);
        if (mConstr > i) {
          emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &rb_emlrtBCI, sp);
        }

        i = workingset->Wlocalidx[mConstr - 1];
        if (i == workingset->sizes[3]) {
          st.site = &pe_emlrtRSI;
          i1 = workingset->Wid.size(0);
          if (mConstr > i1) {
            emlrtDynamicBoundsCheckR2012b(mConstr, 1, i1, &x_emlrtBCI, &st);
          }

          i1 = workingset->Wlocalidx.size(0);
          if (mConstr > i1) {
            emlrtDynamicBoundsCheckR2012b(mConstr, 1, i1, &x_emlrtBCI, &st);
          }

          i1 = workingset->isActiveConstr.size(0);
          i = (workingset->isActiveIdx[3] + i) - 1;
          if ((i < 1) || (i > i1)) {
            emlrtDynamicBoundsCheckR2012b(i, 1, i1, &x_emlrtBCI, &st);
          }

          workingset->isActiveConstr[i - 1] = false;
          b_st.site = &od_emlrtRSI;
          moveConstraint_(&b_st, workingset, workingset->nActiveConstr, mConstr);
          workingset->nActiveConstr--;
          workingset->nWConstr[3]--;
          exitg1 = true;
        } else {
          mConstr++;
        }
      } else {
        mConstr++;
      }
    }
  }

  mConstr = workingset->nActiveConstr;
  mEqFixed = workingset->sizes[1];
  while ((mConstr > mEqFixed) && (mConstr > nVar)) {
    int32_T TYPE_tmp;
    st.site = &pe_emlrtRSI;
    i = workingset->Wid.size(0);
    if ((mConstr < 1) || (mConstr > i)) {
      emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &x_emlrtBCI, &st);
    }

    TYPE_tmp = workingset->Wid[mConstr - 1];
    i = workingset->Wlocalidx.size(0);
    if (mConstr > i) {
      emlrtDynamicBoundsCheckR2012b(mConstr, 1, i, &x_emlrtBCI, &st);
    }

    if ((TYPE_tmp < 1) || (TYPE_tmp > 6)) {
      emlrtDynamicBoundsCheckR2012b(workingset->Wid[mConstr - 1], 1, 6,
        &w_emlrtBCI, &st);
    }

    i = workingset->isActiveConstr.size(0);
    i1 = (workingset->isActiveIdx[TYPE_tmp - 1] + workingset->Wlocalidx[mConstr
          - 1]) - 1;
    if ((i1 < 1) || (i1 > i)) {
      emlrtDynamicBoundsCheckR2012b(i1, 1, i, &x_emlrtBCI, &st);
    }

    workingset->isActiveConstr[i1 - 1] = false;
    b_st.site = &od_emlrtRSI;
    moveConstraint_(&b_st, workingset, workingset->nActiveConstr, mConstr);
    workingset->nActiveConstr--;
    if (TYPE_tmp > 5) {
      emlrtDynamicBoundsCheckR2012b(6, 1, 5, &fb_emlrtBCI, &st);
    }

    workingset->nWConstr[TYPE_tmp - 1]--;
    mConstr--;
  }

  i = solution->xstar.size(0);
  if (nVarP1_tmp > i) {
    emlrtDynamicBoundsCheckR2012b(nVarP1_tmp, 1, i, &rb_emlrtBCI, sp);
  }

  solution->maxConstr = solution->xstar[nVarP1_tmp - 1];
  st.site = &pe_emlrtRSI;
  switch (PROBTYPE_ORIG) {
   case 3:
    workingset->nVar = workingset->nVarOrig;
    workingset->mConstr = workingset->mConstrOrig;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesNormal[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxNormal[i];
    }
    break;

   case 1:
    workingset->nVar = workingset->nVarOrig + 1;
    workingset->mConstr = workingset->mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesPhaseOne[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxPhaseOne[i];
    }

    b_st.site = &sb_emlrtRSI;
    modifyOverheadPhaseOne_(&b_st, workingset);
    break;

   case 2:
    workingset->nVar = workingset->nVarMax - 1;
    workingset->mConstr = workingset->mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesRegularized[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxRegularized[i];
    }

    if (workingset->probType != 4) {
      b_st.site = &sb_emlrtRSI;
      modifyOverheadRegularized_(&b_st, workingset);
    }
    break;

   default:
    workingset->nVar = workingset->nVarMax;
    workingset->mConstr = workingset->mConstrMax;
    for (i = 0; i < 5; i++) {
      workingset->sizes[i] = workingset->sizesRegPhaseOne[i];
    }

    for (i = 0; i < 6; i++) {
      workingset->isActiveIdx[i] = workingset->isActiveIdxRegPhaseOne[i];
    }

    b_st.site = &sb_emlrtRSI;
    modifyOverheadPhaseOne_(&b_st, workingset);
    break;
  }

  workingset->probType = PROBTYPE_ORIG;
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 1.0E-8;
}

// End of code generation (phaseone.cpp)

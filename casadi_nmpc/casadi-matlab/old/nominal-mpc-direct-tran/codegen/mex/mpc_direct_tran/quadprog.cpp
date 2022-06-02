//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  quadprog.cpp
//
//  Code generation for function 'quadprog'
//


// Include files
#include "quadprog.h"
#include "RemoveDependentEq_.h"
#include "all.h"
#include "blas.h"
#include "checkLinearInputs.h"
#include "computeFval.h"
#include "computePhaseOneRelativeTolerances.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "initActiveSet.h"
#include "iterate.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "updateRelativeTolerancesForPhaseTwo.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo cb_emlrtRSI = { 1,  // lineNo
  "quadprog",                          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/eml/quadprog.p"// pathName
};

static emlrtBCInfo o_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "quadprog",                          // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/eml/quadprog.p",// pName
  0                                    // checkKind
};

static emlrtRTEInfo jb_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "factoryConstruct",                  // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/factoryConstruct.p"// pName 
};

// Function Definitions
void quadprog(const emlrtStack *sp, const coder::array<real_T, 2U> &H, const
              coder::array<real_T, 1U> &f, const coder::array<real_T, 2U> &Aineq,
              const coder::array<real_T, 1U> &bineq, const coder::array<real_T,
              2U> &Aeq, const coder::array<real_T, 1U> &beq, const coder::array<
              real_T, 1U> &x0, coder::array<real_T, 1U> &x)
{
  int32_T nVarMax;
  int32_T i;
  coder::array<boolean_T, 1U> r;
  int32_T i1;
  int32_T loop_ub;
  int32_T mConstrMax;
  coder::array<real_T, 1U> r1;
  int32_T maxDims;
  coder::array<boolean_T, 1U> r2;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  coder::array<real_T, 2U> r3;
  b_struct_T solution;
  ptrdiff_t incy_t;
  d_struct_T QPObjective;
  e_struct_T QRManager;
  f_struct_T CholRegManager;
  g_struct_T WorkingSet;
  c_struct_T memspace;
  real_T c_runTimeOptions_ConstrRelTolFa;
  real_T runTimeOptions_ProbRelTolFactor;
  struct_T expl_temp;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if ((H.size(0) != 0) && (H.size(1) != 0)) {
    nVarMax = H.size(0) * H.size(1);
    i = 0;
    i1 = 0;
    mConstrMax = 0;
    r1.set_size((&g_emlrtRTEI), sp, nVarMax);
    for (maxDims = 0; maxDims < nVarMax; maxDims++) {
      r1[i] = H[mConstrMax + H.size(1) * i1];
      i++;
      i1++;
      if (i1 > H.size(0) - 1) {
        i1 = 0;
        mConstrMax++;
      }
    }

    r.set_size((&gb_emlrtRTEI), sp, r1.size(0));
    loop_ub = r1.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = muDoubleScalarIsInf(r1[i]);
    }

    r2.set_size((&hb_emlrtRTEI), sp, r1.size(0));
    loop_ub = r1.size(0);
    for (i = 0; i < loop_ub; i++) {
      r2[i] = muDoubleScalarIsNaN(r1[i]);
    }

    loop_ub = r.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = ((!r[i]) && (!r2[i]));
    }

    st.site = &cb_emlrtRSI;
    if (!all(&st, r)) {
      emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
        "optim_codegen:common:InfNaNComplexDetected",
        "optim_codegen:common:InfNaNComplexDetected", 3, 4, 1, "H");
    }
  }

  if (f.size(0) != 0) {
    r.set_size((&gb_emlrtRTEI), sp, f.size(0));
    loop_ub = f.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = muDoubleScalarIsInf(f[i]);
    }

    r2.set_size((&hb_emlrtRTEI), sp, f.size(0));
    loop_ub = f.size(0);
    for (i = 0; i < loop_ub; i++) {
      r2[i] = muDoubleScalarIsNaN(f[i]);
    }

    loop_ub = r.size(0);
    for (i = 0; i < loop_ub; i++) {
      r[i] = ((!r[i]) && (!r2[i]));
    }

    st.site = &cb_emlrtRSI;
    if (!all(&st, r)) {
      emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
        "optim_codegen:common:InfNaNComplexDetected",
        "optim_codegen:common:InfNaNComplexDetected", 3, 4, 1, "f");
    }
  }

  if ((H.size(0) == 0) || (H.size(1) == 0)) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
      "optim_codegen:quadprog:NullHessian", "optim_codegen:quadprog:NullHessian",
      0);
  }

  if (H.size(0) != H.size(1)) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
      "optim:quadprog:NonSquareHessian", "optim:quadprog:NonSquareHessian", 0);
  }

  st.site = &cb_emlrtRSI;
  nVarMax = H.size(0) * H.size(1);
  if (nVarMax < 1) {
    nVarMax = 0;
  } else {
    n_t = (ptrdiff_t)nVarMax;
    incx_t = (ptrdiff_t)1;
    r3.set_size((&ib_emlrtRTEI), (&st), H.size(1), H.size(0));
    loop_ub = H.size(1);
    for (i = 0; i < loop_ub; i++) {
      nVarMax = H.size(0);
      for (i1 = 0; i1 < nVarMax; i1++) {
        r3[i1 + r3.size(1) * i] = H[i + H.size(1) * i1];
      }
    }

    n_t = idamax(&n_t, &(r3.data())[0], &incx_t);
    nVarMax = (int32_T)n_t;
  }

  i = H.size(0) * H.size(1);
  if ((nVarMax < 1) || (nVarMax > i)) {
    emlrtDynamicBoundsCheckR2012b(nVarMax, 1, i, &o_emlrtBCI, sp);
  }

  if (muDoubleScalarAbs(H[(nVarMax - 1) % H.size(0) * H.size(1) + (nVarMax - 1) /
                        H.size(0)]) < 2.2204460492503131E-16) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
      "optim_codegen:quadprog:NullHessian", "optim_codegen:quadprog:NullHessian",
      0);
  }

  if ((f.size(0) != 0) && (f.size(0) != H.size(0))) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
      "optim:quadprog:MismatchObjCoefSize", "optim:quadprog:MismatchObjCoefSize",
      0);
  }

  if (x0.size(0) == 0) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
      "optim_codegen:common:EmptyX", "optim_codegen:common:EmptyX", 0);
  }

  if (H.size(0) != x0.size(0)) {
    emlrtErrorWithMessageIdR2018a(sp, &g_emlrtRTEI,
      "optim:quadprog:InvalidSizesOfHAndX0",
      "optim:quadprog:InvalidSizesOfHAndX0", 0);
  }

  st.site = &cb_emlrtRSI;
  checkLinearInputs(&st, x0, Aineq, bineq, Aeq, beq);
  nVarMax = x0.size(0) + 1;
  mConstrMax = (bineq.size(0) + beq.size(0)) + 1;
  solution.xstar.set_size((&g_emlrtRTEI), sp, (x0.size(0) + 1));
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  solution.lambda.set_size((&g_emlrtRTEI), sp, mConstrMax);
  for (i = 0; i < mConstrMax; i++) {
    solution.lambda[i] = 0.0;
  }

  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  solution.searchDir.set_size((&g_emlrtRTEI), sp, (x0.size(0) + 1));
  loop_ub = x0.size(0);
  for (i = 0; i <= loop_ub; i++) {
    solution.searchDir[i] = 0.0;
  }

  n_t = (ptrdiff_t)x0.size(0);
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dcopy(&n_t, &(((coder::array<real_T, 1U> *)&x0)->data())[0], &incx_t,
        &(solution.xstar.data())[0], &incy_t);
  QPObjective.grad.set_size((&g_emlrtRTEI), sp, (x0.size(0) + 1));
  QPObjective.Hx.set_size((&g_emlrtRTEI), sp, x0.size(0));
  QPObjective.maxVar = x0.size(0) + 1;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  QPObjective.hasLinear = (f.size(0) != 0);
  QPObjective.nvar = x0.size(0);
  QPObjective.objtype = 3;
  maxDims = muIntScalarMax_sint32(nVarMax, mConstrMax);
  QRManager.ldq = maxDims;
  QRManager.QR.set_size((&g_emlrtRTEI), sp, maxDims, maxDims);
  QRManager.Q.set_size((&jb_emlrtRTEI), sp, maxDims, maxDims);
  loop_ub = maxDims * maxDims;
  for (i = 0; i < loop_ub; i++) {
    QRManager.Q[i] = 0.0;
  }

  QRManager.jpvt.set_size((&g_emlrtRTEI), sp, maxDims);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt[i] = 0;
  }

  QRManager.mrows = 0;
  QRManager.ncols = 0;
  QRManager.tau.set_size((&g_emlrtRTEI), sp, muIntScalarMin_sint32(maxDims,
    maxDims));
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  CholRegManager.FMat.set_size((&g_emlrtRTEI), sp, maxDims, maxDims);
  CholRegManager.ldm = maxDims;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.workspace_.set_size((&g_emlrtRTEI), sp, maxDims, 48);
  CholRegManager.workspace2_.set_size((&g_emlrtRTEI), sp, maxDims, 48);
  CholRegManager.scaleFactor = 100.0;
  st.site = &cb_emlrtRSI;
  factoryConstruct(&st, bineq.size(0), bineq.size(0), Aineq, bineq, beq.size(0),
                   beq.size(0), Aeq, beq, x0.size(0), x0.size(0) + 1, mConstrMax,
                   &WorkingSet);
  st.site = &cb_emlrtRSI;
  initActiveSet(&st, &WorkingSet);
  WorkingSet.SLACK0 = 0.0;
  if (2 < x0.size(0) + 1) {
    nVarMax = x0.size(0) + 1;
  } else {
    nVarMax = 2;
  }

  memspace.workspace_double.set_size((&g_emlrtRTEI), sp, maxDims, nVarMax);
  memspace.workspace_int.set_size((&g_emlrtRTEI), sp, maxDims);
  memspace.workspace_sort.set_size((&g_emlrtRTEI), sp, maxDims);
  st.site = &cb_emlrtRSI;
  c_runTimeOptions_ConstrRelTolFa = c_computePhaseOneRelativeTolera(&st,
    WorkingSet.nVarOrig, WorkingSet.Aineq, WorkingSet.Aeq, WorkingSet.sizes);
  runTimeOptions_ProbRelTolFactor = c_runTimeOptions_ConstrRelTolFa;
  st.site = &cb_emlrtRSI;
  c_updateRelativeTolerancesForPh(&st, &runTimeOptions_ProbRelTolFactor, H, f);
  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor = runTimeOptions_ProbRelTolFactor;
  expl_temp.ConstrRelTolFactor = c_runTimeOptions_ConstrRelTolFa;
  expl_temp.MaxIterations = 10 * ((x0.size(0) + beq.size(0)) + bineq.size(0));
  st.site = &cb_emlrtRSI;
  driver(&st, H, f, &solution, &memspace, &WorkingSet, &QRManager,
         &CholRegManager, &QPObjective, expl_temp);
  x.set_size((&g_emlrtRTEI), sp, x0.size(0));
  n_t = (ptrdiff_t)x0.size(0);
  incx_t = (ptrdiff_t)1;
  incy_t = (ptrdiff_t)1;
  dcopy(&n_t, &(solution.xstar.data())[0], &incx_t, &(x.data())[0], &incy_t);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (quadprog.cpp)

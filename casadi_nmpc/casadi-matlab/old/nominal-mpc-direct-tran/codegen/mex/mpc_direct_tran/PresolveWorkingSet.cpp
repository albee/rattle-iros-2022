//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  PresolveWorkingSet.cpp
//
//  Code generation for function 'PresolveWorkingSet'
//


// Include files
#include "PresolveWorkingSet.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "removeAllIneqConstr.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo cc_emlrtRSI = { 1,  // lineNo
  "PresolveWorkingSet",                // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+initialize/PresolveWorkingSet.p"// pathName 
};

// Function Definitions
void PresolveWorkingSet(const emlrtStack *sp, b_struct_T *solution, c_struct_T
  *memspace, g_struct_T *workingset, e_struct_T *qrmanager, h_struct_T *options)
{
  static h_struct_T r = { { 'f', 'o', 'r', 'w', 'a', 'r', 'd' },// FiniteDifferenceType 
    false,                             // ScaleProblem
    false,                             // SpecifyConstraintGradient
    false,                             // SpecifyObjectiveGradient
    true,                              // NonFiniteSupport
    false,                             // IterDisplaySQP
    -1.0,                              // FiniteDifferenceStepSize
    -1.0,                              // MaxFunctionEvaluations
    false,                             // IterDisplayQP
    0.0,                               // PricingTolerance
    { 'a', 'c', 't', 'i', 'v', 'e', '-', 's', 'e', 't' },// Algorithm
    -1.0E+20,                          // ObjectiveLimit
    1.0E-8,                            // ConstraintTolerance
    1.0E-8,                            // OptimalityTolerance
    1.0E-8,                            // StepTolerance
    -1.0,                              // MaxIterations
    { 'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g' },// SolverName
    false,                             // CheckGradients
    { 'o', 'f', 'f' },                 // Diagnostics
    0.0,                               // DiffMaxChange
    0.0,                               // DiffMinChange
    { 'o', 'f', 'f' },                 // Display
    { 'o', 'f', 'f' },                 // FunValCheck
    false,                             // UseParallel
    1.4901161193847656E-8,             // FunctionTolerance
    { 'a', 'u', 't', 'o' },            // LinearSolver
    { 'c', 'g' }                       // SubproblemAlgorithm
  };

  int32_T i;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  r.DiffMaxChange = rtInf;
  *options = r;
  solution->state = 82;
  st.site = &cc_emlrtRSI;
  i = RemoveDependentEq_(&st, memspace, workingset, qrmanager);
  if (i != -1) {
    boolean_T okWorkingSet;
    boolean_T guard1 = false;
    st.site = &cc_emlrtRSI;
    RemoveDependentIneq_(&st, workingset, qrmanager, memspace);
    st.site = &cc_emlrtRSI;
    okWorkingSet = feasibleX0ForWorkingSet(&st, memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      st.site = &cc_emlrtRSI;
      b_RemoveDependentIneq_(&st, workingset, qrmanager, memspace);
      st.site = &cc_emlrtRSI;
      okWorkingSet = feasibleX0ForWorkingSet(&st, memspace->workspace_double,
        solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      real_T constrViolation;
      st.site = &cc_emlrtRSI;
      constrViolation = c_maxConstraintViolation(&st, workingset,
        solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    st.site = &cc_emlrtRSI;
    removeAllIneqConstr(&st, workingset);
  }
}

void b_PresolveWorkingSet(const emlrtStack *sp, b_struct_T *solution, c_struct_T
  *memspace, g_struct_T *workingset, e_struct_T *qrmanager)
{
  int32_T i;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  solution->state = 82;
  st.site = &cc_emlrtRSI;
  i = RemoveDependentEq_(&st, memspace, workingset, qrmanager);
  if (i != -1) {
    boolean_T okWorkingSet;
    boolean_T guard1 = false;
    st.site = &cc_emlrtRSI;
    RemoveDependentIneq_(&st, workingset, qrmanager, memspace);
    st.site = &cc_emlrtRSI;
    okWorkingSet = feasibleX0ForWorkingSet(&st, memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      st.site = &cc_emlrtRSI;
      b_RemoveDependentIneq_(&st, workingset, qrmanager, memspace);
      st.site = &cc_emlrtRSI;
      okWorkingSet = feasibleX0ForWorkingSet(&st, memspace->workspace_double,
        solution->xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution->state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }

    if (guard1 && (workingset->nWConstr[0] + workingset->nWConstr[1] ==
                   workingset->nVar)) {
      real_T constrViolation;
      st.site = &cc_emlrtRSI;
      constrViolation = c_maxConstraintViolation(&st, workingset,
        solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    solution->state = -3;
    st.site = &cc_emlrtRSI;
    removeAllIneqConstr(&st, workingset);
  }
}

// End of code generation (PresolveWorkingSet.cpp)

//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PresolveWorkingSet.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "PresolveWorkingSet.h"
#include "RemoveDependentEq_.h"
#include "RemoveDependentIneq_.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "removeConstr.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : b_struct_T *solution
//                c_struct_T *memspace
//                g_struct_T *workingset
//                e_struct_T *qrmanager
//                h_struct_T *options
// Return Type  : void
//
void PresolveWorkingSet(b_struct_T *solution, c_struct_T *memspace, g_struct_T
  *workingset, e_struct_T *qrmanager, h_struct_T *options)
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

  int idxStartIneq;
  r.DiffMaxChange = rtGetInf();
  *options = r;
  solution->state = 82;
  idxStartIneq = RemoveDependentEq_(memspace, workingset, qrmanager);
  if (idxStartIneq != -1) {
    boolean_T okWorkingSet;
    boolean_T guard1 = false;
    RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      b_RemoveDependentIneq_(workingset, qrmanager, memspace);
      okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
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
      double constrViolation;
      constrViolation = maxConstraintViolation(workingset, solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    int idxEndIneq;
    solution->state = -3;
    idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (int idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++)
    {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
        Wid[idx_global - 1] - 1] + workingset->Wlocalidx[idx_global - 1]) - 2] =
        false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

//
// Arguments    : b_struct_T *solution
//                c_struct_T *memspace
//                g_struct_T *workingset
//                e_struct_T *qrmanager
// Return Type  : void
//
void b_PresolveWorkingSet(b_struct_T *solution, c_struct_T *memspace, g_struct_T
  *workingset, e_struct_T *qrmanager)
{
  int idxStartIneq;
  solution->state = 82;
  idxStartIneq = RemoveDependentEq_(memspace, workingset, qrmanager);
  if (idxStartIneq != -1) {
    boolean_T okWorkingSet;
    boolean_T guard1 = false;
    RemoveDependentIneq_(workingset, qrmanager, memspace);
    okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
      solution->xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      b_RemoveDependentIneq_(workingset, qrmanager, memspace);
      okWorkingSet = feasibleX0ForWorkingSet(memspace->workspace_double,
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
      double constrViolation;
      constrViolation = maxConstraintViolation(workingset, solution->xstar);
      if (constrViolation > 1.0E-8) {
        solution->state = -2;
      }
    }
  } else {
    int idxEndIneq;
    solution->state = -3;
    idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
    idxEndIneq = workingset->nActiveConstr;
    for (int idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++)
    {
      workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
        Wid[idx_global - 1] - 1] + workingset->Wlocalidx[idx_global - 1]) - 2] =
        false;
    }

    workingset->nWConstr[2] = 0;
    workingset->nWConstr[3] = 0;
    workingset->nWConstr[4] = 0;
    workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  }
}

//
// File trailer for PresolveWorkingSet.cpp
//
// [EOF]
//

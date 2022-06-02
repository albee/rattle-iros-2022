//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: phaseone.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "phaseone.h"
#include "computeFval.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"

// Function Definitions

//
// Arguments    : const coder::array<double, 2U> *H
//                const coder::array<double, 1U> *f
//                b_struct_T *solution
//                c_struct_T *memspace
//                g_struct_T *workingset
//                e_struct_T *qrmanager
//                f_struct_T *cholmanager
//                d_struct_T *objective
//                h_struct_T *options
//                const struct_T *runTimeOptions
// Return Type  : void
//
void phaseone(const coder::array<double, 2U> &H, const coder::array<double, 1U>
              &f, b_struct_T *solution, c_struct_T *memspace, g_struct_T
              *workingset, e_struct_T *qrmanager, f_struct_T *cholmanager,
              d_struct_T *objective, h_struct_T *options, const struct_T
              *runTimeOptions)
{
  int PROBTYPE_ORIG;
  int nVar;
  int nVarP1;
  int PHASEONE;
  int idxStartIneq;
  int idxEndIneq;
  PROBTYPE_ORIG = workingset->probType;
  nVar = workingset->nVar;
  nVarP1 = workingset->nVar;
  solution->xstar[workingset->nVar] = solution->maxConstr + 1.0;
  if (workingset->probType == 3) {
    PHASEONE = 1;
  } else {
    PHASEONE = 4;
  }

  idxStartIneq = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
  idxEndIneq = workingset->nActiveConstr;
  for (int idx_global = idxStartIneq; idx_global <= idxEndIneq; idx_global++) {
    workingset->isActiveConstr[(workingset->isActiveIdx[workingset->
      Wid[idx_global - 1] - 1] + workingset->Wlocalidx[idx_global - 1]) - 2] =
      false;
  }

  workingset->nWConstr[2] = 0;
  workingset->nWConstr[3] = 0;
  workingset->nWConstr[4] = 0;
  workingset->nActiveConstr = workingset->nWConstr[0] + workingset->nWConstr[1];
  setProblemType(workingset, PHASEONE);
  objective->prev_objtype = objective->objtype;
  objective->prev_nvar = objective->nvar;
  objective->prev_hasLinear = objective->hasLinear;
  objective->objtype = 5;
  objective->nvar = nVarP1 + 1;
  objective->gammaScalar = 1.0;
  objective->hasLinear = true;
  options->ObjectiveLimit = 1.0E-8 * runTimeOptions->ConstrRelTolFactor;
  options->StepTolerance = 1.4901161193847657E-10;
  solution->fstar = computeFval(objective, memspace->workspace_double, H, f,
    solution->xstar);
  solution->state = 5;
  iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
          objective, options->ObjectiveLimit, options->StepTolerance,
          runTimeOptions->MaxIterations, runTimeOptions->ConstrRelTolFactor,
          runTimeOptions->ProbRelTolFactor, runTimeOptions->RemainFeasible);
  if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + workingset->
       sizes[3]) - 2]) {
    boolean_T exitg1;
    PHASEONE = workingset->sizes[1];
    exitg1 = false;
    while ((!exitg1) && (PHASEONE + 1 <= workingset->nActiveConstr)) {
      if ((workingset->Wid[PHASEONE] == 4) && (workingset->Wlocalidx[PHASEONE] ==
           workingset->sizes[3])) {
        removeConstr(workingset, PHASEONE + 1);
        exitg1 = true;
      } else {
        PHASEONE++;
      }
    }
  }

  PHASEONE = workingset->nActiveConstr;
  idxStartIneq = workingset->sizes[1];
  while ((PHASEONE > idxStartIneq) && (PHASEONE > nVar)) {
    removeConstr(workingset, PHASEONE);
    PHASEONE--;
  }

  solution->maxConstr = solution->xstar[nVarP1];
  setProblemType(workingset, PROBTYPE_ORIG);
  objective->objtype = objective->prev_objtype;
  objective->nvar = objective->prev_nvar;
  objective->hasLinear = objective->prev_hasLinear;
  options->ObjectiveLimit = -1.0E+20;
  options->StepTolerance = 1.0E-8;
}

//
// File trailer for phaseone.cpp
//
// [EOF]
//

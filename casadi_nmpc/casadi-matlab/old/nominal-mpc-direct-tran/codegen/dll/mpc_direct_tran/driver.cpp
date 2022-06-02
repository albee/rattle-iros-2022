//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: driver.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "driver.h"
#include "PresolveWorkingSet.h"
#include "computeFirstOrderOpt.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "phaseone.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

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
//                struct_T runTimeOptions
// Return Type  : void
//
void driver(const coder::array<double, 2U> &H, const coder::array<double, 1U> &f,
            b_struct_T *solution, c_struct_T *memspace, g_struct_T *workingset,
            e_struct_T *qrmanager, f_struct_T *cholmanager, d_struct_T
            *objective, struct_T runTimeOptions)
{
  int nVar;
  int kstr;
  int idx;
  h_struct_T options;
  static const char cv[128] = { '\x00', '\x01', '\x02', '\x03', '\x04', '\x05',
    '\x06', '\x07', '\x08', '\x09', '\x0a', '\x0b', '\x0c', '\x0d', '\x0e',
    '\x0f', '\x10', '\x11', '\x12', '\x13', '\x14', '\x15', '\x16', '\x17',
    '\x18', '\x19', '\x1a', '\x1b', '\x1c', '\x1d', '\x1e', '\x1f', ' ', '!',
    '\"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0',
    '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
    '@', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n',
    'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '[', '\\', ']',
    '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l',
    'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{',
    '|', '}', '~', '\x7f' };

  static const char cv1[8] = { 'q', 'u', 'a', 'd', 'p', 'r', 'o', 'g' };

  solution->iterations = 0;
  runTimeOptions.RemainFeasible = true;
  nVar = workingset->nVar - 1;
  kstr = workingset->sizes[3];
  for (idx = 0; idx < kstr; idx++) {
    if (workingset->isActiveConstr[(workingset->isActiveIdx[3] + idx) - 1]) {
      solution->xstar[workingset->indexLB[idx] - 1] = -workingset->lb
        [workingset->indexLB[idx] - 1];
    }
  }

  PresolveWorkingSet(solution, memspace, workingset, qrmanager, &options);
  if (solution->state >= 0) {
    double maxConstr_new;
    boolean_T guard1 = false;
    solution->iterations = 0;
    solution->maxConstr = maxConstraintViolation(workingset, solution->xstar);
    maxConstr_new = 1.0E-8 * runTimeOptions.ConstrRelTolFactor;
    guard1 = false;
    if (solution->maxConstr > maxConstr_new) {
      phaseone(H, f, solution, memspace, workingset, qrmanager, cholmanager,
               objective, &options, &runTimeOptions);
      if (solution->state != 0) {
        solution->maxConstr = maxConstraintViolation(workingset, solution->xstar);
        if (solution->maxConstr > maxConstr_new) {
          kstr = workingset->mConstrMax;
          for (idx = 0; idx < kstr; idx++) {
            solution->lambda[idx] = 0.0;
          }

          solution->fstar = computeFval(objective, memspace->workspace_double, H,
            f, solution->xstar);
          solution->state = -2;
        } else {
          if (solution->maxConstr > 0.0) {
            for (idx = 0; idx <= nVar; idx++) {
              solution->searchDir[idx] = solution->xstar[idx];
            }

            b_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
            maxConstr_new = maxConstraintViolation(workingset, solution->xstar);
            if (maxConstr_new >= solution->maxConstr) {
              solution->maxConstr = maxConstr_new;
              for (idx = 0; idx <= nVar; idx++) {
                solution->xstar[idx] = solution->searchDir[idx];
              }
            }
          }

          guard1 = true;
        }
      }
    } else {
      guard1 = true;
    }

    if (guard1) {
      boolean_T b_bool;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.ObjectiveLimit, options.StepTolerance,
              runTimeOptions.MaxIterations, runTimeOptions.ConstrRelTolFactor,
              runTimeOptions.ProbRelTolFactor, true);
      b_bool = false;
      kstr = 0;
      int exitg1;
      do {
        exitg1 = 0;
        if (kstr < 8) {
          if (cv[static_cast<unsigned char>(options.SolverName[kstr])] != cv[
              static_cast<int>(cv1[kstr])]) {
            exitg1 = 1;
          } else {
            kstr++;
          }
        } else {
          b_bool = true;
          exitg1 = 1;
        }
      } while (exitg1 == 0);

      if (b_bool && (solution->state != -6)) {
        solution->maxConstr = maxConstraintViolation(workingset, solution->xstar);
        computeFirstOrderOpt(solution, objective, workingset->nVar,
                             workingset->ldA, workingset->ATwset,
                             workingset->nActiveConstr,
                             memspace->workspace_double);
        runTimeOptions.RemainFeasible = false;
        while ((solution->iterations < runTimeOptions.MaxIterations) &&
               ((solution->state == -7) || ((solution->state == 1) &&
                 ((solution->maxConstr > 1.0E-8 *
                   runTimeOptions.ConstrRelTolFactor) ||
                  (solution->firstorderopt > 1.0E-8 *
                   runTimeOptions.ProbRelTolFactor))))) {
          feasibleX0ForWorkingSet(memspace->workspace_double, solution->xstar,
            workingset, qrmanager);
          b_PresolveWorkingSet(solution, memspace, workingset, qrmanager);
          phaseone(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                   objective, &options, &runTimeOptions);
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options.ObjectiveLimit, options.StepTolerance,
                  runTimeOptions.MaxIterations,
                  runTimeOptions.ConstrRelTolFactor,
                  runTimeOptions.ProbRelTolFactor, false);
          solution->maxConstr = maxConstraintViolation(workingset,
            solution->xstar);
          computeFirstOrderOpt(solution, objective, workingset->nVar,
                               workingset->ldA, workingset->ATwset,
                               workingset->nActiveConstr,
                               memspace->workspace_double);
        }
      }
    }
  }
}

//
// File trailer for driver.cpp
//
// [EOF]
//

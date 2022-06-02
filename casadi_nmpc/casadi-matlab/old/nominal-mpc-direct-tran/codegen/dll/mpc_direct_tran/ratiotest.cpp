//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: ratiotest.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "ratiotest.h"
#include "computeFval.h"
#include "feasibleratiotest.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xnrm2.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const coder::array<double, 1U> *solution_xstar
//                const coder::array<double, 1U> *solution_searchDir
//                coder::array<double, 2U> *workspace
//                int workingset_nVar
//                int workingset_ldA
//                const coder::array<double, 2U> *workingset_Aineq
//                const coder::array<double, 2U> *workingset_bineq
//                const coder::array<double, 1U> *workingset_lb
//                const coder::array<int, 1U> *workingset_indexLB
//                const int workingset_sizes[5]
//                const int workingset_isActiveIdx[6]
//                const coder::array<boolean_T, 1U> *workingset_isActiveConstr
//                const int workingset_nWConstr[5]
//                double *toldelta
//                double *alpha
//                boolean_T *newBlocking
//                int *constrType
//                int *constrIdx
// Return Type  : void
//
void ratiotest(const coder::array<double, 1U> &solution_xstar, const coder::
               array<double, 1U> &solution_searchDir, coder::array<double, 2U>
               &workspace, int workingset_nVar, int workingset_ldA, const coder::
               array<double, 2U> &workingset_Aineq, const coder::array<double,
               2U> &workingset_bineq, const coder::array<double, 1U>
               &workingset_lb, const coder::array<int, 1U> &workingset_indexLB,
               const int workingset_sizes[5], const int workingset_isActiveIdx[6],
               const coder::array<boolean_T, 1U> &workingset_isActiveConstr,
               const int workingset_nWConstr[5], double *toldelta, double *alpha,
               boolean_T *newBlocking, int *constrType, int *constrIdx)
{
  int totalIneq;
  double p_max;
  double denomTol;
  int ldw;
  double alphaTemp;
  int idx;
  double u0;
  totalIneq = workingset_sizes[2] - 1;
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  denomTol = 2.2204460492503131E-13 * b_xnrm2(workingset_nVar,
    solution_searchDir);
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    for (ldw = 0; ldw <= totalIneq; ldw++) {
      workspace[ldw] = workingset_bineq[ldw];
    }

    d_xgemv(workingset_nVar, workingset_sizes[2], workingset_Aineq,
            workingset_ldA, solution_xstar, workspace);
    ldw = workspace.size(0);
    e_xgemv(workingset_nVar, workingset_sizes[2], workingset_Aineq,
            workingset_ldA, solution_searchDir, workspace, workspace.size(0) + 1);
    for (idx = 0; idx <= totalIneq; idx++) {
      if ((workspace[ldw + idx] > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[2] + idx) - 1])) {
        u0 = std::abs(workspace[idx] - *toldelta);
        alphaTemp = (1.0E-8 - workspace[idx]) + *toldelta;
        if ((u0 < alphaTemp) || rtIsNaN(alphaTemp)) {
          alphaTemp = u0;
        }

        alphaTemp /= workspace[ldw + idx];
        if ((alphaTemp <= *alpha) && (std::abs(workspace[ldw + idx]) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }

        u0 = std::abs(workspace[idx]);
        if ((!(u0 < 1.0E-8 - workspace[idx])) && (!rtIsNaN(1.0E-8 -
              workspace[idx]))) {
          u0 = 1.0E-8 - workspace[idx];
        }

        alphaTemp = u0 / workspace[ldw + idx];
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 3;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(workspace[ldw + idx]);
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    double phaseOneCorrectionX;
    double phaseOneCorrectionP;
    double ratio_tmp;
    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    ldw = workingset_sizes[3];
    for (idx = 0; idx <= ldw - 2; idx++) {
      double pk_corrected;
      pk_corrected = -solution_searchDir[workingset_indexLB[idx] - 1] -
        phaseOneCorrectionP;
      if ((pk_corrected > denomTol) && (!workingset_isActiveConstr
           [(workingset_isActiveIdx[3] + idx) - 1])) {
        ratio_tmp = -solution_xstar[workingset_indexLB[idx] - 1] -
          workingset_lb[workingset_indexLB[idx] - 1];
        alphaTemp = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
        u0 = std::abs(alphaTemp);
        if ((!(u0 < 1.0E-8 - alphaTemp)) && (!rtIsNaN(1.0E-8 - alphaTemp))) {
          u0 = 1.0E-8 - alphaTemp;
        }

        alphaTemp = u0 / pk_corrected;
        if ((alphaTemp <= *alpha) && (std::abs(pk_corrected) > p_max)) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
        }

        alphaTemp = ratio_tmp - phaseOneCorrectionX;
        u0 = std::abs(alphaTemp);
        if ((!(u0 < 1.0E-8 - alphaTemp)) && (!rtIsNaN(1.0E-8 - alphaTemp))) {
          u0 = 1.0E-8 - alphaTemp;
        }

        alphaTemp = u0 / pk_corrected;
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = idx + 1;
          *newBlocking = true;
          p_max = std::abs(pk_corrected);
        }
      }
    }

    ldw = workingset_indexLB[workingset_sizes[3] - 1] - 1;
    if ((-solution_searchDir[ldw] > denomTol) && (!workingset_isActiveConstr
         [(workingset_isActiveIdx[3] + workingset_sizes[3]) - 2])) {
      ratio_tmp = -solution_xstar[ldw] - workingset_lb[ldw];
      alphaTemp = ratio_tmp - *toldelta;
      u0 = std::abs(alphaTemp);
      if ((!(u0 < 1.0E-8 - alphaTemp)) && (!rtIsNaN(1.0E-8 - alphaTemp))) {
        u0 = 1.0E-8 - alphaTemp;
      }

      alphaTemp = u0 / -solution_searchDir[ldw];
      if ((alphaTemp <= *alpha) && (std::abs(solution_searchDir[ldw]) > p_max))
      {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
      }

      u0 = std::abs(ratio_tmp);
      if ((!(u0 < 1.0E-8 - ratio_tmp)) && (!rtIsNaN(1.0E-8 - ratio_tmp))) {
        u0 = 1.0E-8 - ratio_tmp;
      }

      alphaTemp = u0 / -solution_searchDir[ldw];
      if (alphaTemp < *alpha) {
        *alpha = alphaTemp;
        *constrType = 4;
        *constrIdx = workingset_sizes[3];
        *newBlocking = true;
        p_max = std::abs(solution_searchDir[ldw]);
      }
    }
  }

  *toldelta += 6.608625846508183E-7;
  if (p_max > 0.0) {
    alphaTemp = 6.608625846508183E-7 / p_max;
    if (!(*alpha > alphaTemp)) {
      *alpha = alphaTemp;
    }
  }

  if ((*newBlocking) && (*alpha > 1.0)) {
    *newBlocking = false;
  }

  if (!(*alpha < 1.0)) {
    *alpha = 1.0;
  }
}

//
// File trailer for ratiotest.cpp
//
// [EOF]
//

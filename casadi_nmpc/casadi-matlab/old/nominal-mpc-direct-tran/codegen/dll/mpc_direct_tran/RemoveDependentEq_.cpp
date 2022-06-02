//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RemoveDependentEq_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "RemoveDependentEq_.h"
#include "computeFval.h"
#include "computeQ_.h"
#include "countsort.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include <cmath>

// Function Definitions

//
// Arguments    : c_struct_T *memspace
//                g_struct_T *workingset
//                e_struct_T *qrmanager
// Return Type  : int
//
int RemoveDependentEq_(c_struct_T *memspace, g_struct_T *workingset, e_struct_T *
  qrmanager)
{
  int nDepInd;
  int nVar;
  int mWorkingFixed;
  int mTotalWorkingEq_tmp_tmp;
  coder::array<int, 1U> r;
  double qtb;
  nVar = workingset->nVar - 1;
  mWorkingFixed = workingset->nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset->nWConstr[1] + workingset->nWConstr[0];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int iy;
    int idx_col;
    int idx;
    double tol;
    for (iy = 0; iy < mTotalWorkingEq_tmp_tmp; iy++) {
      for (idx_col = 0; idx_col <= nVar; idx_col++) {
        qrmanager->QR[iy + qrmanager->QR.size(0) * idx_col] = workingset->
          ATwset[idx_col + workingset->ATwset.size(0) * iy];
      }
    }

    nDepInd = mTotalWorkingEq_tmp_tmp - workingset->nVar;
    if (0 > nDepInd) {
      nDepInd = 0;
    }

    for (idx = 0; idx <= nVar; idx++) {
      qrmanager->jpvt[idx] = 0;
    }

    qrmanager->usedPivoting = true;
    qrmanager->mrows = mTotalWorkingEq_tmp_tmp;
    qrmanager->ncols = workingset->nVar;
    nVar = workingset->nVar;
    if (mTotalWorkingEq_tmp_tmp < nVar) {
      nVar = mTotalWorkingEq_tmp_tmp;
    }

    qrmanager->minRowCol = nVar;
    xgeqp3(qrmanager->QR, mTotalWorkingEq_tmp_tmp, workingset->nVar,
           qrmanager->jpvt, qrmanager->tau);
    tol = 100.0 * static_cast<double>(workingset->nVar) * 2.2204460492503131E-16;
    nVar = workingset->nVar;
    if (nVar >= mTotalWorkingEq_tmp_tmp) {
      nVar = mTotalWorkingEq_tmp_tmp;
    }

    while ((nVar > 0) && (std::abs(qrmanager->QR[(nVar + qrmanager->QR.size(0) *
              (nVar - 1)) - 1]) < tol)) {
      nVar--;
      nDepInd++;
    }

    if (nDepInd > 0) {
      boolean_T exitg1;
      computeQ_(qrmanager, mTotalWorkingEq_tmp_tmp);
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= nDepInd - 1)) {
        nVar = qrmanager->ldq * ((mTotalWorkingEq_tmp_tmp - idx) - 1);
        qtb = 0.0;
        iy = 0;
        for (idx_col = 0; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
          qtb += qrmanager->Q[nVar] * workingset->bwset[iy];
          nVar++;
          iy++;
        }

        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx++;
        }
      }
    }

    if (nDepInd > 0) {
      int i;
      r.set_size(memspace->workspace_int.size(0));
      nVar = memspace->workspace_int.size(0);
      for (i = 0; i < nVar; i++) {
        r[i] = memspace->workspace_int[i];
      }

      for (idx = 0; idx < mWorkingFixed; idx++) {
        qrmanager->jpvt[idx] = 1;
      }

      i = workingset->nWConstr[0] + 1;
      for (idx = i; idx <= mTotalWorkingEq_tmp_tmp; idx++) {
        qrmanager->jpvt[idx - 1] = 0;
      }

      factorQRE(qrmanager, workingset->ATwset, workingset->nVar,
                mTotalWorkingEq_tmp_tmp);
      for (idx = 0; idx < nDepInd; idx++) {
        r[idx] = qrmanager->jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx];
      }

      memspace->workspace_int.set_size(r.size(0));
      nVar = r.size(0);
      for (i = 0; i < nVar; i++) {
        memspace->workspace_int[i] = r[i];
      }

      countsort(memspace->workspace_int, nDepInd, memspace->workspace_sort, 1,
                mTotalWorkingEq_tmp_tmp);
      for (idx = nDepInd; idx >= 1; idx--) {
        iy = (workingset->nWConstr[0] + workingset->nWConstr[1]) - 1;
        if (iy + 1 != 0) {
          i = memspace->workspace_int[idx - 1];
          if (i <= iy + 1) {
            if ((workingset->nActiveConstr == iy + 1) || (i == iy + 1)) {
              workingset->mEqRemoved++;
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx[memspace->workspace_int[idx - 1] - 1];
              removeConstr(workingset, memspace->workspace_int[idx - 1]);
            } else {
              workingset->mEqRemoved++;
              idx_col = i - 1;
              mWorkingFixed = workingset->Wid[idx_col] - 1;
              nVar = workingset->Wlocalidx[idx_col];
              workingset->indexEqRemoved[workingset->mEqRemoved - 1] =
                workingset->Wlocalidx[idx_col];
              workingset->isActiveConstr[(workingset->isActiveIdx[mWorkingFixed]
                + nVar) - 2] = false;
              workingset->Wid[idx_col] = workingset->Wid[iy];
              workingset->Wlocalidx[idx_col] = workingset->Wlocalidx[iy];
              i = workingset->nVar;
              for (nVar = 0; nVar < i; nVar++) {
                workingset->ATwset[nVar + workingset->ATwset.size(0) * idx_col] =
                  workingset->ATwset[nVar + workingset->ATwset.size(0) * iy];
              }

              workingset->bwset[idx_col] = workingset->bwset[iy];
              workingset->Wid[iy] = workingset->Wid[workingset->nActiveConstr -
                1];
              workingset->Wlocalidx[iy] = workingset->Wlocalidx
                [workingset->nActiveConstr - 1];
              i = workingset->nVar;
              for (nVar = 0; nVar < i; nVar++) {
                workingset->ATwset[nVar + workingset->ATwset.size(0) * iy] =
                  workingset->ATwset[nVar + workingset->ATwset.size(0) *
                  (workingset->nActiveConstr - 1)];
              }

              workingset->bwset[iy] = workingset->bwset
                [workingset->nActiveConstr - 1];
              workingset->nActiveConstr--;
              workingset->nWConstr[mWorkingFixed]--;
            }
          }
        }
      }
    }
  }

  return nDepInd;
}

//
// File trailer for RemoveDependentEq_.cpp
//
// [EOF]
//

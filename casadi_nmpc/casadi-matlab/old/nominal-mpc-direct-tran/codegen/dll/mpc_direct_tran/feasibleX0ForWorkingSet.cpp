//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: feasibleX0ForWorkingSet.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "feasibleX0ForWorkingSet.h"
#include "computeFval.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include "xzgeqp3.h"
#include <cmath>

// Function Definitions

//
// Arguments    : coder::array<double, 2U> *workspace
//                coder::array<double, 1U> *xCurrent
//                g_struct_T *workingset
//                e_struct_T *qrmanager
// Return Type  : boolean_T
//
boolean_T feasibleX0ForWorkingSet(coder::array<double, 2U> &workspace, coder::
  array<double, 1U> &xCurrent, g_struct_T *workingset, e_struct_T *qrmanager)
{
  boolean_T nonDegenerateWset;
  int mWConstr_tmp;
  int nVar;
  int ldw;
  int ar;
  int ix;
  double c;
  coder::array<double, 2U> B;
  int cr;
  coder::array<double, 1U> tau;
  mWConstr_tmp = workingset->nActiveConstr - 1;
  nVar = workingset->nVar;
  ldw = workingset->ATwset.size(0);
  nonDegenerateWset = true;
  if (workingset->nActiveConstr != 0) {
    int idx;
    int iy;
    int i;
    int mEq;
    int i1;
    int ia;
    for (idx = 0; idx <= mWConstr_tmp; idx++) {
      workspace[idx] = workingset->bwset[idx];
      workspace[idx + workspace.size(0)] = workingset->bwset[idx];
    }

    if (workingset->nActiveConstr != 0) {
      iy = 0;
      i = workingset->ATwset.size(0) * mWConstr_tmp + 1;
      for (ar = 1; ldw < 0 ? ar >= i : ar <= i; ar += ldw) {
        ix = 0;
        c = 0.0;
        i1 = (ar + nVar) - 1;
        for (ia = ar; ia <= i1; ia++) {
          c += workingset->ATwset[ia - 1] * xCurrent[ix];
          ix++;
        }

        workspace[iy] = workspace[iy] + -c;
        iy++;
      }
    }

    if (workingset->nActiveConstr >= workingset->nVar) {
      int ldq;
      for (iy = 0; iy < nVar; iy++) {
        for (ar = 0; ar <= mWConstr_tmp; ar++) {
          qrmanager->QR[ar + qrmanager->QR.size(0) * iy] = workingset->ATwset[iy
            + workingset->ATwset.size(0) * ar];
        }
      }

      qrmanager->usedPivoting = false;
      qrmanager->mrows = workingset->nActiveConstr;
      qrmanager->ncols = workingset->nVar;
      i = workingset->nVar;
      for (idx = 0; idx < i; idx++) {
        qrmanager->jpvt[idx] = idx + 1;
      }

      ix = workingset->nActiveConstr;
      ar = workingset->nVar;
      if (ix < ar) {
        ar = ix;
      }

      qrmanager->minRowCol = ar;
      B.set_size(qrmanager->QR.size(0), qrmanager->QR.size(1));
      ix = qrmanager->QR.size(0) * qrmanager->QR.size(1);
      for (i = 0; i < ix; i++) {
        B[i] = qrmanager->QR[i];
      }

      ix = qrmanager->QR.size(0);
      iy = qrmanager->QR.size(1);
      if (ix < iy) {
        iy = ix;
      }

      tau.set_size(iy);
      for (i = 0; i < iy; i++) {
        tau[i] = 0.0;
      }

      if (ar >= 1) {
        qrf(B, workingset->nActiveConstr, workingset->nVar, ar, tau);
      }

      qrmanager->QR.set_size(B.size(0), B.size(1));
      ix = B.size(0) * B.size(1);
      for (i = 0; i < ix; i++) {
        qrmanager->QR[i] = B[i];
      }

      qrmanager->tau.set_size(tau.size(0));
      ix = tau.size(0);
      for (i = 0; i < ix; i++) {
        qrmanager->tau[i] = tau[i];
      }

      computeQ_(qrmanager, workingset->nActiveConstr);
      ldq = qrmanager->ldq;
      ldw = workspace.size(0);
      B.set_size(workspace.size(0), workspace.size(1));
      ix = workspace.size(0) * workspace.size(1);
      for (i = 0; i < ix; i++) {
        B[i] = workspace[i];
      }

      for (cr = 0; ldw < 0 ? cr >= ldw : cr <= ldw; cr += ldw) {
        i = cr + 1;
        i1 = cr + nVar;
        for (idx = i; idx <= i1; idx++) {
          workspace[idx - 1] = 0.0;
        }
      }

      ix = -1;
      for (cr = 0; ldw < 0 ? cr >= ldw : cr <= ldw; cr += ldw) {
        ar = -1;
        i = cr + 1;
        i1 = cr + nVar;
        for (idx = i; idx <= i1; idx++) {
          c = 0.0;
          for (mEq = 0; mEq <= mWConstr_tmp; mEq++) {
            iy = mEq + 1;
            c += qrmanager->Q[iy + ar] * B[iy + ix];
          }

          workspace[idx - 1] = workspace[idx - 1] + c;
          ar += ldq;
        }

        ix += ldw;
      }

      for (idx = nVar; idx >= 1; idx--) {
        ar = ldq * (idx - 1) - 1;
        i = idx + -1;
        if (workspace[i] != 0.0) {
          workspace[i] = workspace[i] / qrmanager->QR[idx + ar];
          for (mEq = 0; mEq <= idx - 2; mEq++) {
            workspace[mEq] = workspace[mEq] - workspace[i] * qrmanager->QR[(mEq
              + ar) + 1];
          }
        }
      }

      iy = ldw - 1;
      for (idx = nVar; idx >= 1; idx--) {
        ar = ldq * (idx - 1) - 1;
        i = idx + iy;
        if (workspace[i] != 0.0) {
          workspace[i] = workspace[i] / qrmanager->QR[idx + ar];
          for (mEq = 0; mEq <= idx - 2; mEq++) {
            i1 = (mEq + iy) + 1;
            workspace[i1] = workspace[i1] - workspace[i] * qrmanager->QR[(mEq +
              ar) + 1];
          }
        }
      }
    } else {
      int ldq;
      factorQR(qrmanager, workingset->ATwset, workingset->nVar,
               workingset->nActiveConstr);
      computeQ_(qrmanager, qrmanager->minRowCol);
      ldq = qrmanager->ldq;
      ldw = workspace.size(0);
      for (mEq = 0; mEq <= mWConstr_tmp; mEq++) {
        ix = ldq * mEq;
        c = workspace[mEq];
        for (idx = 0; idx < mEq; idx++) {
          c -= qrmanager->QR[idx + ix] * workspace[idx];
        }

        workspace[mEq] = c / qrmanager->QR[mEq + ix];
      }

      for (mEq = 0; mEq <= mWConstr_tmp; mEq++) {
        ix = ldq * mEq;
        iy = mEq + ldw;
        c = workspace[iy];
        for (idx = 0; idx < mEq; idx++) {
          c -= qrmanager->QR[idx + ix] * workspace[idx + ldw];
        }

        workspace[iy] = c / qrmanager->QR[mEq + ix];
      }

      B.set_size(workspace.size(0), workspace.size(1));
      ix = workspace.size(0) * workspace.size(1);
      for (i = 0; i < ix; i++) {
        B[i] = workspace[i];
      }

      for (cr = 0; ldw < 0 ? cr >= ldw : cr <= ldw; cr += ldw) {
        i = cr + 1;
        i1 = cr + nVar;
        for (idx = i; idx <= i1; idx++) {
          workspace[idx - 1] = 0.0;
        }
      }

      ix = 1;
      for (cr = 0; ldw < 0 ? cr >= ldw : cr <= ldw; cr += ldw) {
        ar = -1;
        i = ix + mWConstr_tmp;
        for (mEq = ix; mEq <= i; mEq++) {
          ia = ar;
          i1 = cr + 1;
          iy = cr + nVar;
          for (idx = i1; idx <= iy; idx++) {
            ia++;
            workspace[idx - 1] = workspace[idx - 1] + B[mEq - 1] * qrmanager->
              Q[ia];
          }

          ar += ldq;
        }

        ix += ldw;
      }
    }

    idx = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        c = workspace[idx];
        if (rtIsInf(c) || rtIsNaN(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + workspace.size(0)];
          if (rtIsInf(c) || rtIsNaN(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        double v;
        double b_v;
        ix = nVar - 1;
        for (idx = 0; idx <= ix; idx++) {
          workspace[idx] = workspace[idx] + xCurrent[idx];
        }

        ar = workingset->sizes[3];
        switch (workingset->probType) {
         case 2:
          v = 0.0;
          ix = workingset->sizes[2] - 1;
          mEq = workingset->sizes[1] - 1;
          if ((workingset->Aineq.size(0) != 0) && (workingset->Aineq.size(1) !=
               0)) {
            for (idx = 0; idx <= ix; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->bineq[idx];
            }

            xgemv(workingset->nVarOrig, workingset->sizes[2], workingset->Aineq,
                  workingset->ldA, workspace, workingset->maxConstrWorkspace);
            for (idx = 0; idx <= ix; idx++) {
              workingset->maxConstrWorkspace[idx] =
                workingset->maxConstrWorkspace[idx] - workspace
                [workingset->nVarOrig + idx];
              if ((!(v > workingset->maxConstrWorkspace[idx])) && (!rtIsNaN
                   (workingset->maxConstrWorkspace[idx]))) {
                v = workingset->maxConstrWorkspace[idx];
              }
            }
          }

          if ((workingset->Aeq.size(0) != 0) && (workingset->Aeq.size(1) != 0))
          {
            for (idx = 0; idx <= mEq; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->beq[idx];
            }

            xgemv(workingset->nVarOrig, workingset->sizes[1], workingset->Aeq,
                  workingset->ldA, workspace, workingset->maxConstrWorkspace);
            ix = workingset->nVarOrig + workingset->sizes[2];
            iy = ix + workingset->sizes[1];
            for (idx = 0; idx <= mEq; idx++) {
              workingset->maxConstrWorkspace[idx] =
                (workingset->maxConstrWorkspace[idx] - workspace[ix + idx]) +
                workspace[iy + idx];
              c = std::abs(workingset->maxConstrWorkspace[idx]);
              if ((!(v > c)) && (!rtIsNaN(c))) {
                v = c;
              }
            }
          }
          break;

         default:
          v = 0.0;
          ix = workingset->sizes[2] - 1;
          mEq = workingset->sizes[1] - 1;
          if ((workingset->Aineq.size(0) != 0) && (workingset->Aineq.size(1) !=
               0)) {
            for (idx = 0; idx <= ix; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->bineq[idx];
            }

            xgemv(workingset->nVar, workingset->sizes[2], workingset->Aineq,
                  workingset->ldA, workspace, workingset->maxConstrWorkspace);
            for (idx = 0; idx <= ix; idx++) {
              if ((!(v > workingset->maxConstrWorkspace[idx])) && (!rtIsNaN
                   (workingset->maxConstrWorkspace[idx]))) {
                v = workingset->maxConstrWorkspace[idx];
              }
            }
          }

          if ((workingset->Aeq.size(0) != 0) && (workingset->Aeq.size(1) != 0))
          {
            for (idx = 0; idx <= mEq; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->beq[idx];
            }

            xgemv(workingset->nVar, workingset->sizes[1], workingset->Aeq,
                  workingset->ldA, workspace, workingset->maxConstrWorkspace);
            for (idx = 0; idx <= mEq; idx++) {
              c = std::abs(workingset->maxConstrWorkspace[idx]);
              if ((!(v > c)) && (!rtIsNaN(c))) {
                v = c;
              }
            }
          }
          break;
        }

        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < ar; idx++) {
            c = -workspace[workingset->indexLB[idx] - 1] - workingset->
              lb[workingset->indexLB[idx] - 1];
            if ((!(v > c)) && (!rtIsNaN(c))) {
              v = c;
            }
          }
        }

        cr = workspace.size(0) + 1;
        ar = workingset->sizes[3];
        switch (workingset->probType) {
         case 2:
          b_v = 0.0;
          ix = workingset->sizes[2] - 1;
          mEq = workingset->sizes[1] - 1;
          if ((workingset->Aineq.size(0) != 0) && (workingset->Aineq.size(1) !=
               0)) {
            for (idx = 0; idx <= ix; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->bineq[idx];
            }

            b_xgemv(workingset->nVarOrig, workingset->sizes[2],
                    workingset->Aineq, workingset->ldA, workspace,
                    workspace.size(0) + 1, workingset->maxConstrWorkspace);
            for (idx = 0; idx <= ix; idx++) {
              workingset->maxConstrWorkspace[idx] =
                workingset->maxConstrWorkspace[idx] - workspace[((cr +
                workingset->nVarOrig) + idx) - 1];
              if ((!(b_v > workingset->maxConstrWorkspace[idx])) && (!rtIsNaN
                   (workingset->maxConstrWorkspace[idx]))) {
                b_v = workingset->maxConstrWorkspace[idx];
              }
            }
          }

          if ((workingset->Aeq.size(0) != 0) && (workingset->Aeq.size(1) != 0))
          {
            for (idx = 0; idx <= mEq; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->beq[idx];
            }

            b_xgemv(workingset->nVarOrig, workingset->sizes[1], workingset->Aeq,
                    workingset->ldA, workspace, workspace.size(0) + 1,
                    workingset->maxConstrWorkspace);
            ix = (workingset->nVarOrig + workingset->sizes[2]) - 1;
            iy = ix + workingset->sizes[1];
            for (idx = 0; idx <= mEq; idx++) {
              workingset->maxConstrWorkspace[idx] =
                (workingset->maxConstrWorkspace[idx] - workspace[(cr + ix) + idx])
                + workspace[(cr + iy) + idx];
              c = std::abs(workingset->maxConstrWorkspace[idx]);
              if ((!(b_v > c)) && (!rtIsNaN(c))) {
                b_v = c;
              }
            }
          }
          break;

         default:
          b_v = 0.0;
          ix = workingset->sizes[2] - 1;
          mEq = workingset->sizes[1] - 1;
          if ((workingset->Aineq.size(0) != 0) && (workingset->Aineq.size(1) !=
               0)) {
            for (idx = 0; idx <= ix; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->bineq[idx];
            }

            b_xgemv(workingset->nVar, workingset->sizes[2], workingset->Aineq,
                    workingset->ldA, workspace, workspace.size(0) + 1,
                    workingset->maxConstrWorkspace);
            for (idx = 0; idx <= ix; idx++) {
              if ((!(b_v > workingset->maxConstrWorkspace[idx])) && (!rtIsNaN
                   (workingset->maxConstrWorkspace[idx]))) {
                b_v = workingset->maxConstrWorkspace[idx];
              }
            }
          }

          if ((workingset->Aeq.size(0) != 0) && (workingset->Aeq.size(1) != 0))
          {
            for (idx = 0; idx <= mEq; idx++) {
              workingset->maxConstrWorkspace[idx] = workingset->beq[idx];
            }

            b_xgemv(workingset->nVar, workingset->sizes[1], workingset->Aeq,
                    workingset->ldA, workspace, workspace.size(0) + 1,
                    workingset->maxConstrWorkspace);
            for (idx = 0; idx <= mEq; idx++) {
              c = std::abs(workingset->maxConstrWorkspace[idx]);
              if ((!(b_v > c)) && (!rtIsNaN(c))) {
                b_v = c;
              }
            }
          }
          break;
        }

        if (workingset->sizes[3] > 0) {
          for (idx = 0; idx < ar; idx++) {
            c = -workspace[(cr + workingset->indexLB[idx]) - 2] - workingset->
              lb[workingset->indexLB[idx] - 1];
            if ((!(b_v > c)) && (!rtIsNaN(c))) {
              b_v = c;
            }
          }
        }

        if ((v <= 2.2204460492503131E-16) || (v < b_v)) {
          for (idx = 0; idx < nVar; idx++) {
            xCurrent[idx] = workspace[idx];
          }
        } else {
          for (idx = 0; idx < nVar; idx++) {
            xCurrent[idx] = workspace[workspace.size(0) + idx];
          }
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return nonDegenerateWset;
}

//
// File trailer for feasibleX0ForWorkingSet.cpp
//
// [EOF]
//

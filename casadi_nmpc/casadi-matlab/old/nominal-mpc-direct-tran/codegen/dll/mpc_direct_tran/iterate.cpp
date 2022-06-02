//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: iterate.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "iterate.h"
#include "addBoundToActiveSetMatrix_.h"
#include "computeFval.h"
#include "computeFval_ReuseHx.h"
#include "computeGrad_StoreHx.h"
#include "computeQ_.h"
#include "compute_deltax.h"
#include "deleteColMoveEnd.h"
#include "factorQR.h"
#include "feasibleX0ForWorkingSet.h"
#include "feasibleratiotest.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "ratiotest.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include "xrotg.h"
#include <cmath>

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
//                double options_ObjectiveLimit
//                double options_StepTolerance
//                int runTimeOptions_MaxIterations
//                double c_runTimeOptions_ConstrRelTolFa
//                double runTimeOptions_ProbRelTolFactor
//                boolean_T runTimeOptions_RemainFeasible
// Return Type  : void
//
void iterate(const coder::array<double, 2U> &H, const coder::array<double, 1U>
             &f, b_struct_T *solution, c_struct_T *memspace, g_struct_T
             *workingset, e_struct_T *qrmanager, f_struct_T *cholmanager,
             d_struct_T *objective, double options_ObjectiveLimit, double
             options_StepTolerance, int runTimeOptions_MaxIterations, double
             c_runTimeOptions_ConstrRelTolFa, double
             runTimeOptions_ProbRelTolFactor, boolean_T
             runTimeOptions_RemainFeasible)
{
  boolean_T subProblemChanged;
  boolean_T updateFval;
  int activeSetChangeID;
  int TYPE;
  double tolDelta;
  int nVar;
  int globalActiveConstrIdx;
  int n;
  int ix;
  int iyend;
  coder::array<double, 2U> x;
  double temp;
  int iy;
  boolean_T newBlocking;
  double d;
  double c;
  double s;
  subProblemChanged = true;
  updateFval = true;
  activeSetChangeID = 0;
  TYPE = objective->objtype;
  tolDelta = 6.7434957617430445E-7;
  nVar = workingset->nVar;
  globalActiveConstrIdx = 0;
  computeGrad_StoreHx(objective, H, f, solution->xstar);
  solution->fstar = computeFval_ReuseHx(objective, memspace->workspace_double, f,
    solution->xstar);
  if (solution->iterations < runTimeOptions_MaxIterations) {
    solution->state = -5;
  } else {
    solution->state = 0;
  }

  n = workingset->mConstrMax;
  for (ix = 0; ix < n; ix++) {
    solution->lambda[ix] = 0.0;
  }

  int exitg1;
  do {
    exitg1 = 0;
    if (solution->state == -5) {
      boolean_T guard1 = false;
      boolean_T guard2 = false;
      int idx;
      int Qk0;
      int i;
      int b_iy;
      int ia;
      guard1 = false;
      guard2 = false;
      if (subProblemChanged) {
        switch (activeSetChangeID) {
         case 1:
          idx = workingset->ldA * (workingset->nActiveConstr - 1);
          iyend = qrmanager->mrows;
          Qk0 = qrmanager->ncols + 1;
          if (iyend < Qk0) {
            Qk0 = iyend;
          }

          qrmanager->minRowCol = Qk0;
          iy = qrmanager->ldq * qrmanager->ncols;
          Qk0 = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            iyend = iy + qrmanager->mrows;
            for (b_iy = iy + 1; b_iy <= iyend; b_iy++) {
              qrmanager->QR[b_iy - 1] = 0.0;
            }

            i = qrmanager->ldq * (qrmanager->mrows - 1) + 1;
            for (iyend = 1; Qk0 < 0 ? iyend >= i : iyend <= i; iyend += Qk0) {
              ix = idx;
              c = 0.0;
              n = (iyend + qrmanager->mrows) - 1;
              for (ia = iyend; ia <= n; ia++) {
                c += qrmanager->Q[ia - 1] * workingset->ATwset[ix];
                ix++;
              }

              qrmanager->QR[iy] = qrmanager->QR[iy] + c;
              iy++;
            }
          }

          qrmanager->ncols++;
          qrmanager->jpvt[qrmanager->ncols - 1] = qrmanager->ncols;
          for (idx = qrmanager->mrows - 1; idx + 1 > qrmanager->ncols; idx--) {
            d = qrmanager->QR[idx + qrmanager->QR.size(0) * (qrmanager->ncols -
              1)];
            xrotg(&qrmanager->QR[(idx + qrmanager->QR.size(0) *
                                  (qrmanager->ncols - 1)) - 1], &d, &c, &s);
            qrmanager->QR[idx + qrmanager->QR.size(0) * (qrmanager->ncols - 1)] =
              d;
            Qk0 = qrmanager->ldq * (idx - 1);
            n = qrmanager->mrows;
            x.set_size(qrmanager->Q.size(0), qrmanager->Q.size(1));
            iyend = qrmanager->Q.size(0) * qrmanager->Q.size(1);
            for (i = 0; i < iyend; i++) {
              x[i] = qrmanager->Q[i];
            }

            if (qrmanager->mrows >= 1) {
              b_iy = qrmanager->ldq + Qk0;
              for (ix = 0; ix < n; ix++) {
                temp = c * x[Qk0] + s * x[b_iy];
                x[b_iy] = c * x[b_iy] - s * x[Qk0];
                x[Qk0] = temp;
                b_iy++;
                Qk0++;
              }
            }

            qrmanager->Q.set_size(x.size(0), x.size(1));
            iyend = x.size(0) * x.size(1);
            for (i = 0; i < iyend; i++) {
              qrmanager->Q[i] = x[i];
            }
          }
          break;

         case -1:
          deleteColMoveEnd(qrmanager, globalActiveConstrIdx);
          break;

         default:
          factorQR(qrmanager, workingset->ATwset, nVar,
                   workingset->nActiveConstr);
          computeQ_(qrmanager, qrmanager->mrows);
          break;
        }

        compute_deltax(H, solution, memspace, qrmanager, cholmanager, objective);
        if (solution->state != -5) {
          exitg1 = 1;
        } else {
          temp = b_xnrm2(nVar, solution->searchDir);
          if ((temp < options_StepTolerance) || (workingset->nActiveConstr >=
               nVar)) {
            guard2 = true;
          } else {
            updateFval = (TYPE == 5);
            if (updateFval || runTimeOptions_RemainFeasible) {
              feasibleratiotest(solution->xstar, solution->searchDir,
                                memspace->workspace_double, workingset->nVar,
                                workingset->ldA, workingset->Aineq,
                                workingset->bineq, workingset->lb,
                                workingset->indexLB, workingset->sizes,
                                workingset->isActiveIdx,
                                workingset->isActiveConstr, workingset->nWConstr,
                                updateFval, &temp, &newBlocking, &iyend, &iy);
            } else {
              ratiotest(solution->xstar, solution->searchDir,
                        memspace->workspace_double, workingset->nVar,
                        workingset->ldA, workingset->Aineq, workingset->bineq,
                        workingset->lb, workingset->indexLB, workingset->sizes,
                        workingset->isActiveIdx, workingset->isActiveConstr,
                        workingset->nWConstr, &tolDelta, &temp, &newBlocking,
                        &iyend, &iy);
            }

            if (newBlocking) {
              switch (iyend) {
               case 3:
                workingset->nWConstr[2]++;
                workingset->isActiveConstr[(workingset->isActiveIdx[2] + iy) - 2]
                  = true;
                workingset->nActiveConstr++;
                workingset->Wid[workingset->nActiveConstr - 1] = 3;
                workingset->Wlocalidx[workingset->nActiveConstr - 1] = iy;
                idx = workingset->ldA * (iy - 1);
                iyend = workingset->ldA * (workingset->nActiveConstr - 1);
                n = workingset->nVar;
                for (ix = 0; ix < n; ix++) {
                  Qk0 = ix + 1;
                  workingset->ATwset[(iyend + Qk0) - 1] = workingset->Aineq[(idx
                    + Qk0) - 1];
                }

                workingset->bwset[workingset->nActiveConstr - 1] =
                  workingset->bineq[iy - 1];
                break;

               case 4:
                addBoundToActiveSetMatrix_(workingset, 4, iy);
                break;

               default:
                addBoundToActiveSetMatrix_(workingset, 5, iy);
                break;
              }

              activeSetChangeID = 1;
            } else {
              if (objective->objtype == 5) {
                if (b_xnrm2(objective->nvar, solution->searchDir) > 100.0 *
                    static_cast<double>(objective->nvar) * 1.4901161193847656E-8)
                {
                  solution->state = 3;
                } else {
                  solution->state = 4;
                }
              }

              subProblemChanged = false;
              if (workingset->nActiveConstr == 0) {
                solution->state = 1;
              }
            }

            if ((nVar >= 1) && (!(temp == 0.0))) {
              iyend = nVar - 1;
              for (ix = 0; ix <= iyend; ix++) {
                solution->xstar[ix] = solution->xstar[ix] + temp *
                  solution->searchDir[ix];
              }
            }

            computeGrad_StoreHx(objective, H, f, solution->xstar);
            updateFval = true;
            guard1 = true;
          }
        }
      } else {
        for (ix = 0; ix < nVar; ix++) {
          solution->searchDir[ix] = 0.0;
        }

        guard2 = true;
      }

      if (guard2) {
        x.set_size(memspace->workspace_double.size(0),
                   memspace->workspace_double.size(1));
        iyend = memspace->workspace_double.size(0) *
          memspace->workspace_double.size(1);
        for (i = 0; i < iyend; i++) {
          x[i] = memspace->workspace_double[i];
        }

        iy = qrmanager->ncols;
        if (qrmanager->ncols > 0) {
          temp = 100.0 * static_cast<double>(qrmanager->mrows) *
            2.2204460492503131E-16;
          if ((qrmanager->mrows > 0) && (qrmanager->ncols > 0)) {
            updateFval = true;
          } else {
            updateFval = false;
          }

          if (updateFval) {
            boolean_T b_guard1 = false;
            idx = qrmanager->ncols;
            b_guard1 = false;
            if (qrmanager->mrows < qrmanager->ncols) {
              while ((idx > qrmanager->mrows) && (std::abs(qrmanager->QR
                       [(qrmanager->mrows + qrmanager->QR.size(0) * (idx - 1)) -
                       1]) >= temp)) {
                idx--;
              }

              updateFval = (idx == qrmanager->mrows);
              if (updateFval) {
                b_guard1 = true;
              }
            } else {
              b_guard1 = true;
            }

            if (b_guard1) {
              while ((idx >= 1) && (std::abs(qrmanager->QR[(idx +
                        qrmanager->QR.size(0) * (idx - 1)) - 1]) >= temp)) {
                idx--;
              }

              updateFval = (idx == 0);
            }
          }

          if (!updateFval) {
            solution->state = -7;
          } else {
            Qk0 = qrmanager->ldq;
            if (qrmanager->mrows != 0) {
              iyend = qrmanager->ncols;
              for (b_iy = 0; b_iy < iyend; b_iy++) {
                x[b_iy] = 0.0;
              }

              b_iy = 0;
              i = qrmanager->ldq * (qrmanager->ncols - 1) + 1;
              for (iyend = 1; Qk0 < 0 ? iyend >= i : iyend <= i; iyend += Qk0) {
                ix = 0;
                c = 0.0;
                n = (iyend + qrmanager->mrows) - 1;
                for (ia = iyend; ia <= n; ia++) {
                  c += qrmanager->Q[ia - 1] * objective->grad[ix];
                  ix++;
                }

                x[b_iy] = x[b_iy] + c;
                b_iy++;
              }
            }

            for (n = iy; n >= 1; n--) {
              iyend = (n + (n - 1) * Qk0) - 1;
              x[n - 1] = x[n - 1] / qrmanager->QR[iyend];
              for (idx = 0; idx <= n - 2; idx++) {
                ix = (n - idx) - 2;
                x[ix] = x[ix] - x[n - 1] * qrmanager->QR[(iyend - idx) - 1];
              }
            }

            for (idx = 0; idx < iy; idx++) {
              solution->lambda[idx] = -x[idx];
            }
          }
        }

        memspace->workspace_double.set_size(x.size(0), x.size(1));
        iyend = x.size(0) * x.size(1);
        for (i = 0; i < iyend; i++) {
          memspace->workspace_double[i] = x[i];
        }

        if (solution->state != -7) {
          iyend = 0;
          temp = 0.0 * runTimeOptions_ProbRelTolFactor * static_cast<double>
            (TYPE != 5);
          i = (workingset->nWConstr[0] + workingset->nWConstr[1]) + 1;
          n = workingset->nActiveConstr;
          for (idx = i; idx <= n; idx++) {
            d = solution->lambda[idx - 1];
            if (d < temp) {
              temp = d;
              iyend = idx;
            }
          }

          if (iyend == 0) {
            solution->state = 1;
          } else {
            activeSetChangeID = -1;
            globalActiveConstrIdx = iyend;
            subProblemChanged = true;
            removeConstr(workingset, iyend);
            solution->lambda[iyend - 1] = 0.0;
          }
        } else {
          iyend = workingset->nActiveConstr;
          activeSetChangeID = 0;
          globalActiveConstrIdx = workingset->nActiveConstr;
          subProblemChanged = true;
          removeConstr(workingset, workingset->nActiveConstr);
          solution->lambda[iyend - 1] = 0.0;
        }

        updateFval = false;
        guard1 = true;
      }

      if (guard1) {
        solution->iterations++;
        iyend = objective->nvar - 1;
        if ((solution->iterations >= runTimeOptions_MaxIterations) &&
            ((solution->state != 1) || (objective->objtype == 5))) {
          solution->state = 0;
        }

        if (solution->iterations - solution->iterations / 50 * 50 == 0) {
          solution->maxConstr = maxConstraintViolation(workingset,
            solution->xstar);
          if (solution->maxConstr > 1.0E-8 * c_runTimeOptions_ConstrRelTolFa) {
            for (ix = 0; ix <= iyend; ix++) {
              solution->searchDir[ix] = solution->xstar[ix];
            }

            newBlocking = feasibleX0ForWorkingSet(memspace->workspace_double,
              solution->searchDir, workingset, qrmanager);
            if ((!newBlocking) && (solution->state != 0)) {
              solution->state = -2;
            }

            activeSetChangeID = 0;
            temp = maxConstraintViolation(workingset, solution->searchDir);
            if (temp < solution->maxConstr) {
              for (idx = 0; idx <= iyend; idx++) {
                solution->xstar[idx] = solution->searchDir[idx];
              }

              solution->maxConstr = temp;
            }
          }
        }

        if (updateFval) {
          solution->fstar = computeFval_ReuseHx(objective,
            memspace->workspace_double, f, solution->xstar);
          if ((solution->fstar < options_ObjectiveLimit) && ((solution->state !=
                0) || (objective->objtype != 5))) {
            solution->state = 2;
          }
        }
      }
    } else {
      if (!updateFval) {
        solution->fstar = computeFval_ReuseHx(objective,
          memspace->workspace_double, f, solution->xstar);
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);
}

//
// File trailer for iterate.cpp
//
// [EOF]
//

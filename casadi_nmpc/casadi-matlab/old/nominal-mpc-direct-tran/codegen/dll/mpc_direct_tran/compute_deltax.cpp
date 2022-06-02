//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: compute_deltax.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "compute_deltax.h"
#include "computeFval.h"
#include "factor.h"
#include "feasibleX0ForWorkingSet.h"
#include "fullColLDL2_.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "partialColLDL3_.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "solve.h"
#include "xgemm.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const coder::array<double, 2U> *H
//                b_struct_T *solution
//                c_struct_T *memspace
//                const e_struct_T *qrmanager
//                f_struct_T *cholmanager
//                const d_struct_T *objective
// Return Type  : void
//
void compute_deltax(const coder::array<double, 2U> &H, b_struct_T *solution,
                    c_struct_T *memspace, const e_struct_T *qrmanager,
                    f_struct_T *cholmanager, const d_struct_T *objective)
{
  int nVar_tmp;
  int mNull_tmp;
  int idx_row;
  int ix;
  double smax;
  coder::array<double, 2U> r;
  nVar_tmp = qrmanager->mrows - 1;
  mNull_tmp = qrmanager->mrows - qrmanager->ncols;
  if (mNull_tmp <= 0) {
    for (int nVars = 0; nVars <= nVar_tmp; nVars++) {
      solution->searchDir[nVars] = 0.0;
    }
  } else {
    int nVars;
    for (nVars = 0; nVars <= nVar_tmp; nVars++) {
      solution->searchDir[nVars] = -objective->grad[nVars];
    }

    if (qrmanager->ncols <= 0) {
      switch (objective->objtype) {
       case 5:
        break;

       case 3:
        factor(cholmanager, H, qrmanager->mrows, qrmanager->mrows);
        if (cholmanager->info != 0) {
          solution->state = -6;
        } else {
          solve(cholmanager, solution->searchDir);
        }
        break;

       case 4:
        {
          factor(cholmanager, H, objective->nvar, objective->nvar);
          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            int i;
            solve(cholmanager, solution->searchDir);
            nVars = objective->nvar + 1;
            i = qrmanager->mrows;
            for (idx_row = nVars; idx_row <= i; idx_row++) {
              solution->searchDir[idx_row - 1] = rtInf * solution->
                searchDir[idx_row - 1];
            }
          }
        }
        break;
      }
    } else {
      int nullStartIdx_tmp;
      nullStartIdx_tmp = qrmanager->ldq * qrmanager->ncols + 1;
      switch (objective->objtype) {
       case 5:
        {
          for (nVars = 0; nVars < mNull_tmp; nVars++) {
            memspace->workspace_double[nVars] = -qrmanager->Q[nVar_tmp +
              qrmanager->Q.size(0) * (qrmanager->ncols + nVars)];
          }

          nVars = qrmanager->ldq;
          if (qrmanager->mrows != 0) {
            int i;
            int idx_col;
            for (idx_col = 0; idx_col <= nVar_tmp; idx_col++) {
              solution->searchDir[idx_col] = 0.0;
            }

            ix = 0;
            i = nullStartIdx_tmp + qrmanager->ldq * (mNull_tmp - 1);
            for (idx_row = nullStartIdx_tmp; nVars < 0 ? idx_row >= i : idx_row <=
                 i; idx_row += nVars) {
              int ldw;
              idx_col = 0;
              ldw = idx_row + nVar_tmp;
              for (int ia = idx_row; ia <= ldw; ia++) {
                solution->searchDir[idx_col] = solution->searchDir[idx_col] +
                  qrmanager->Q[ia - 1] * memspace->workspace_double[ix];
                idx_col++;
              }

              ix++;
            }
          }
        }
        break;

       default:
        {
          int ldw;
          int i;
          int idx_col;
          double SCALED_REG_PRIMAL;
          switch (objective->objtype) {
           case 3:
            ldw = memspace->workspace_double.size(0);
            xgemm(qrmanager->mrows, mNull_tmp, qrmanager->mrows, H,
                  qrmanager->mrows, qrmanager->Q, nullStartIdx_tmp,
                  qrmanager->ldq, memspace->workspace_double,
                  memspace->workspace_double.size(0));
            b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                    nullStartIdx_tmp, qrmanager->ldq, memspace->workspace_double,
                    ldw, cholmanager->FMat, cholmanager->ldm);
            break;

           default:
            nVars = qrmanager->mrows;
            ldw = memspace->workspace_double.size(0);
            xgemm(objective->nvar, mNull_tmp, objective->nvar, H,
                  objective->nvar, qrmanager->Q, nullStartIdx_tmp,
                  qrmanager->ldq, memspace->workspace_double,
                  memspace->workspace_double.size(0));
            for (idx_col = 0; idx_col < mNull_tmp; idx_col++) {
              i = objective->nvar + 1;
              for (idx_row = i; idx_row <= nVars; idx_row++) {
                memspace->workspace_double[(idx_row +
                  memspace->workspace_double.size(0) * idx_col) - 1] = 0.0 *
                  qrmanager->Q[(idx_row + qrmanager->Q.size(0) * (idx_col +
                  qrmanager->ncols)) - 1];
              }
            }

            b_xgemm(mNull_tmp, mNull_tmp, qrmanager->mrows, qrmanager->Q,
                    nullStartIdx_tmp, qrmanager->ldq, memspace->workspace_double,
                    ldw, cholmanager->FMat, cholmanager->ldm);
            break;
          }

          SCALED_REG_PRIMAL = 1.4901161193847656E-6 * static_cast<double>
            (mNull_tmp);
          idx_col = cholmanager->ldm + 1;
          cholmanager->ndims = mNull_tmp;
          nVars = 1;
          if (mNull_tmp > 1) {
            ix = 0;
            smax = std::abs(cholmanager->FMat[0]);
            for (idx_row = 2; idx_row <= mNull_tmp; idx_row++) {
              double s;
              ix += idx_col;
              s = std::abs(cholmanager->FMat[ix]);
              if (s > smax) {
                nVars = idx_row;
                smax = s;
              }
            }
          }

          smax = std::abs(cholmanager->FMat[nVars * (cholmanager->ldm + 1) - 1])
            * 2.2204460492503131E-16;
          if (!(smax > SCALED_REG_PRIMAL)) {
            smax = SCALED_REG_PRIMAL;
          }

          cholmanager->regTol_ = smax;
          if (mNull_tmp > 128) {
            boolean_T exitg1;
            idx_row = 0;
            exitg1 = false;
            while ((!exitg1) && (idx_row < mNull_tmp)) {
              nVars = idx_col * idx_row + 1;
              ldw = mNull_tmp - idx_row;
              if (idx_row + 48 <= mNull_tmp) {
                partialColLDL3_(cholmanager, nVars, ldw, SCALED_REG_PRIMAL);
                idx_row += 48;
              } else {
                fullColLDL2_(cholmanager, nVars, ldw, SCALED_REG_PRIMAL);
                exitg1 = true;
              }
            }
          } else {
            fullColLDL2_(cholmanager, 1, mNull_tmp, SCALED_REG_PRIMAL);
          }

          if (cholmanager->ConvexCheck) {
            nVars = 0;
            int exitg2;
            do {
              exitg2 = 0;
              if (nVars <= mNull_tmp - 1) {
                if (cholmanager->FMat[nVars + cholmanager->FMat.size(0) * nVars]
                    <= 0.0) {
                  cholmanager->info = -nVars - 1;
                  exitg2 = 1;
                } else {
                  nVars++;
                }
              } else {
                cholmanager->ConvexCheck = false;
                exitg2 = 1;
              }
            } while (exitg2 == 0);
          }

          if (cholmanager->info != 0) {
            solution->state = -6;
          } else {
            int ia;
            nVars = qrmanager->ldq;
            if (qrmanager->mrows != 0) {
              for (idx_col = 0; idx_col < mNull_tmp; idx_col++) {
                memspace->workspace_double[idx_col] = 0.0;
              }

              idx_col = 0;
              i = nullStartIdx_tmp + qrmanager->ldq * (mNull_tmp - 1);
              for (idx_row = nullStartIdx_tmp; nVars < 0 ? idx_row >= i :
                   idx_row <= i; idx_row += nVars) {
                ix = 0;
                smax = 0.0;
                ldw = idx_row + nVar_tmp;
                for (ia = idx_row; ia <= ldw; ia++) {
                  smax += qrmanager->Q[ia - 1] * objective->grad[ix];
                  ix++;
                }

                memspace->workspace_double[idx_col] = memspace->
                  workspace_double[idx_col] + -smax;
                idx_col++;
              }
            }

            r.set_size(memspace->workspace_double.size(0),
                       memspace->workspace_double.size(1));
            nVars = memspace->workspace_double.size(0) *
              memspace->workspace_double.size(1);
            for (i = 0; i < nVars; i++) {
              r[i] = memspace->workspace_double[i];
            }

            idx_col = cholmanager->ndims - 2;
            if (cholmanager->ndims != 0) {
              for (idx_row = 0; idx_row <= idx_col + 1; idx_row++) {
                nVars = idx_row + idx_row * cholmanager->ldm;
                i = idx_col - idx_row;
                for (ia = 0; ia <= i; ia++) {
                  ldw = ia + 1;
                  ix = idx_row + ldw;
                  r[ix] = r[ix] - r[idx_row] * cholmanager->FMat[nVars + ldw];
                }
              }
            }

            i = cholmanager->ndims;
            for (nVars = 0; nVars < i; nVars++) {
              r[nVars] = r[nVars] / cholmanager->FMat[nVars +
                cholmanager->FMat.size(0) * nVars];
            }

            idx_col = cholmanager->ndims;
            if (cholmanager->ndims != 0) {
              for (idx_row = idx_col; idx_row >= 1; idx_row--) {
                nVars = (idx_row - 1) * cholmanager->ldm;
                smax = r[idx_row - 1];
                i = idx_row + 1;
                for (ia = idx_col; ia >= i; ia--) {
                  smax -= cholmanager->FMat[(nVars + ia) - 1] * r[ia - 1];
                }

                r[idx_row - 1] = smax;
              }
            }

            memspace->workspace_double.set_size(r.size(0), r.size(1));
            nVars = r.size(0) * r.size(1);
            for (i = 0; i < nVars; i++) {
              memspace->workspace_double[i] = r[i];
            }

            nVars = qrmanager->ldq;
            if (qrmanager->mrows != 0) {
              for (idx_col = 0; idx_col <= nVar_tmp; idx_col++) {
                solution->searchDir[idx_col] = 0.0;
              }

              ix = 0;
              i = nullStartIdx_tmp + qrmanager->ldq * (mNull_tmp - 1);
              for (idx_row = nullStartIdx_tmp; nVars < 0 ? idx_row >= i :
                   idx_row <= i; idx_row += nVars) {
                idx_col = 0;
                ldw = idx_row + nVar_tmp;
                for (ia = idx_row; ia <= ldw; ia++) {
                  solution->searchDir[idx_col] = solution->searchDir[idx_col] +
                    qrmanager->Q[ia - 1] * r[ix];
                  idx_col++;
                }

                ix++;
              }
            }
          }
        }
        break;
      }
    }
  }
}

//
// File trailer for compute_deltax.cpp
//
// [EOF]
//

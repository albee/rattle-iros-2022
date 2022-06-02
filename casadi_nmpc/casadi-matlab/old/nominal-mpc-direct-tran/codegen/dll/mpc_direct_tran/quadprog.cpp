//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quadprog.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "quadprog.h"
#include "RemoveDependentEq_.h"
#include "computeFval.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <cmath>

// Function Definitions

//
// Arguments    : const coder::array<double, 2U> *H
//                const coder::array<double, 1U> *f
//                const coder::array<double, 2U> *Aineq
//                const coder::array<double, 1U> *bineq
//                const coder::array<double, 2U> *Aeq
//                const coder::array<double, 1U> *beq
//                const coder::array<double, 1U> *x0
//                coder::array<double, 1U> *x
// Return Type  : void
//
void quadprog(const coder::array<double, 2U> &H, const coder::array<double, 1U>
              &f, const coder::array<double, 2U> &Aineq, const coder::array<
              double, 1U> &bineq, const coder::array<double, 2U> &Aeq, const
              coder::array<double, 1U> &beq, const coder::array<double, 1U> &x0,
              coder::array<double, 1U> &x)
{
  int nVar;
  int mConstrMax;
  b_struct_T solution;
  int i;
  int idxFillStart;
  d_struct_T QPObjective;
  int maxDims;
  e_struct_T QRManager;
  f_struct_T CholRegManager;
  g_struct_T WorkingSet;
  c_struct_T memspace;
  double tol;
  double colSum;
  double b_tol;
  double H_infnrm;
  double f_infnrm;
  struct_T expl_temp;
  nVar = x0.size(0) - 1;
  mConstrMax = (bineq.size(0) + beq.size(0)) + 1;
  solution.xstar.set_size((x0.size(0) + 1));
  solution.fstar = 0.0;
  solution.firstorderopt = 0.0;
  solution.lambda.set_size(mConstrMax);
  for (i = 0; i < mConstrMax; i++) {
    solution.lambda[i] = 0.0;
  }

  solution.state = 0;
  solution.maxConstr = 0.0;
  solution.iterations = 0;
  solution.searchDir.set_size((x0.size(0) + 1));
  idxFillStart = x0.size(0);
  for (i = 0; i <= idxFillStart; i++) {
    solution.searchDir[i] = 0.0;
  }

  for (idxFillStart = 0; idxFillStart <= nVar; idxFillStart++) {
    solution.xstar[idxFillStart] = x0[idxFillStart];
  }

  QPObjective.grad.set_size((x0.size(0) + 1));
  QPObjective.Hx.set_size(x0.size(0));
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
  maxDims = x0.size(0) + 1;
  if (maxDims <= mConstrMax) {
    maxDims = mConstrMax;
  }

  QRManager.ldq = maxDims;
  QRManager.QR.set_size(maxDims, maxDims);
  QRManager.Q.set_size(maxDims, maxDims);
  idxFillStart = maxDims * maxDims;
  for (i = 0; i < idxFillStart; i++) {
    QRManager.Q[i] = 0.0;
  }

  QRManager.jpvt.set_size(maxDims);
  for (i = 0; i < maxDims; i++) {
    QRManager.jpvt[i] = 0;
  }

  QRManager.mrows = 0;
  QRManager.ncols = 0;
  QRManager.tau.set_size(maxDims);
  QRManager.minRowCol = 0;
  QRManager.usedPivoting = false;
  CholRegManager.FMat.set_size(maxDims, maxDims);
  CholRegManager.ldm = maxDims;
  CholRegManager.ndims = 0;
  CholRegManager.info = 0;
  CholRegManager.ConvexCheck = true;
  CholRegManager.workspace_.set_size(maxDims, 48);
  CholRegManager.workspace2_.set_size(maxDims, 48);
  CholRegManager.scaleFactor = 100.0;
  factoryConstruct(bineq.size(0), bineq.size(0), Aineq, bineq, beq.size(0),
                   beq.size(0), Aeq, beq, x0.size(0), x0.size(0) + 1, mConstrMax,
                   &WorkingSet);
  setProblemType(&WorkingSet, 3);
  idxFillStart = WorkingSet.isActiveIdx[2];
  i = WorkingSet.mConstrMax;
  for (mConstrMax = idxFillStart; mConstrMax <= i; mConstrMax++) {
    WorkingSet.isActiveConstr[mConstrMax - 1] = false;
  }

  WorkingSet.nWConstr[0] = 0;
  WorkingSet.nWConstr[1] = WorkingSet.sizes[1];
  WorkingSet.nWConstr[2] = 0;
  WorkingSet.nWConstr[3] = 0;
  WorkingSet.nWConstr[4] = 0;
  WorkingSet.nActiveConstr = WorkingSet.nWConstr[1];
  idxFillStart = WorkingSet.sizes[1];
  for (int idx_local = 0; idx_local < idxFillStart; idx_local++) {
    WorkingSet.Wid[idx_local] = 2;
    WorkingSet.Wlocalidx[idx_local] = idx_local + 1;
    WorkingSet.isActiveConstr[idx_local] = true;
    mConstrMax = WorkingSet.ldA * idx_local;
    i = WorkingSet.nVar - 1;
    for (int b_i = 0; b_i <= i; b_i++) {
      WorkingSet.ATwset[mConstrMax + b_i] = WorkingSet.Aeq[mConstrMax + b_i];
    }

    WorkingSet.bwset[idx_local] = WorkingSet.beq[idx_local];
  }

  WorkingSet.SLACK0 = 0.0;
  if (2 < x0.size(0) + 1) {
    i = x0.size(0) + 1;
  } else {
    i = 2;
  }

  memspace.workspace_double.set_size(maxDims, i);
  memspace.workspace_int.set_size(maxDims);
  memspace.workspace_sort.set_size(maxDims);
  idxFillStart = WorkingSet.nVarOrig - 1;
  tol = 1.0;
  i = WorkingSet.sizes[1];
  for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
    colSum = 0.0;
    for (maxDims = 0; maxDims <= idxFillStart; maxDims++) {
      colSum += std::abs(WorkingSet.Aeq[maxDims + WorkingSet.Aeq.size(0) *
                         mConstrMax]);
    }

    if ((!(tol > colSum)) && (!rtIsNaN(colSum))) {
      tol = colSum;
    }
  }

  i = WorkingSet.sizes[2];
  for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
    colSum = 0.0;
    for (maxDims = 0; maxDims <= idxFillStart; maxDims++) {
      colSum += std::abs(WorkingSet.Aineq[maxDims + WorkingSet.Aineq.size(0) *
                         mConstrMax]);
    }

    if ((!(tol > colSum)) && (!rtIsNaN(colSum))) {
      tol = colSum;
    }
  }

  b_tol = tol;
  H_infnrm = 0.0;
  f_infnrm = 0.0;
  if (f.size(0) != 0) {
    i = H.size(1);
    for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
      colSum = 0.0;
      idxFillStart = H.size(0);
      for (maxDims = 0; maxDims < idxFillStart; maxDims++) {
        colSum += std::abs(H[maxDims + H.size(0) * mConstrMax]);
      }

      if ((!(H_infnrm > colSum)) && (!rtIsNaN(colSum))) {
        H_infnrm = colSum;
      }

      colSum = std::abs(f[mConstrMax]);
      if ((!(f_infnrm > colSum)) && (!rtIsNaN(colSum))) {
        f_infnrm = colSum;
      }
    }
  } else {
    i = H.size(1);
    for (mConstrMax = 0; mConstrMax < i; mConstrMax++) {
      colSum = 0.0;
      idxFillStart = H.size(0);
      for (maxDims = 0; maxDims < idxFillStart; maxDims++) {
        colSum += std::abs(H[maxDims + H.size(0) * mConstrMax]);
      }

      if ((!(H_infnrm > colSum)) && (!rtIsNaN(colSum))) {
        H_infnrm = colSum;
      }
    }
  }

  if ((!(tol > f_infnrm)) && (!rtIsNaN(f_infnrm))) {
    b_tol = f_infnrm;
  }

  if ((!(b_tol > H_infnrm)) && (!rtIsNaN(H_infnrm))) {
    b_tol = H_infnrm;
  }

  if ((1.0 > b_tol) || rtIsNaN(b_tol)) {
    b_tol = 1.0;
  }

  expl_temp.RemainFeasible = false;
  expl_temp.ProbRelTolFactor = b_tol;
  expl_temp.ConstrRelTolFactor = tol;
  expl_temp.MaxIterations = 10 * ((x0.size(0) + beq.size(0)) + bineq.size(0));
  driver(H, f, &solution, &memspace, &WorkingSet, &QRManager, &CholRegManager,
         &QPObjective, expl_temp);
  x.set_size(x0.size(0));
  for (idxFillStart = 0; idxFillStart <= nVar; idxFillStart++) {
    x[idxFillStart] = solution.xstar[idxFillStart];
  }
}

//
// File trailer for quadprog.cpp
//
// [EOF]
//

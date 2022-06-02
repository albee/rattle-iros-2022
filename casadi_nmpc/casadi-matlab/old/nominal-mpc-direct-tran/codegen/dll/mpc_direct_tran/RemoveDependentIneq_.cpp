//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: RemoveDependentIneq_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "RemoveDependentIneq_.h"
#include "computeFval.h"
#include "countsort.h"
#include "factorQRE.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : g_struct_T *workingset
//                e_struct_T *qrmanager
//                c_struct_T *memspace
// Return Type  : void
//
void RemoveDependentIneq_(g_struct_T *workingset, e_struct_T *qrmanager,
  c_struct_T *memspace)
{
  int nActiveConstr;
  int nFixedConstr;
  nActiveConstr = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[1] + workingset->nWConstr[0];
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    double tol;
    int idx;
    int i;
    tol = 100.0 * static_cast<double>(workingset->nVar) * 2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }

    i = nFixedConstr + 1;
    for (idx = i; idx <= nActiveConstr; idx++) {
      qrmanager->jpvt[idx - 1] = 0;
    }

    factorQRE(qrmanager, workingset->ATwset, workingset->nVar,
              workingset->nActiveConstr);
    nActiveConstr = 0;
    for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx--) {
      nActiveConstr++;
      memspace->workspace_int[nActiveConstr - 1] = qrmanager->jpvt[idx - 1];
    }

    if (idx <= workingset->nVar) {
      while ((idx > nFixedConstr) && (std::abs(qrmanager->QR[(idx +
                qrmanager->QR.size(0) * (idx - 1)) - 1]) < tol)) {
        nActiveConstr++;
        memspace->workspace_int[nActiveConstr - 1] = qrmanager->jpvt[idx - 1];
        idx--;
      }
    }

    countsort(memspace->workspace_int, nActiveConstr, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nActiveConstr; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
}

//
// Arguments    : g_struct_T *workingset
//                e_struct_T *qrmanager
//                c_struct_T *memspace
// Return Type  : void
//
void b_RemoveDependentIneq_(g_struct_T *workingset, e_struct_T *qrmanager,
  c_struct_T *memspace)
{
  int nActiveConstr;
  int nFixedConstr;
  nActiveConstr = workingset->nActiveConstr;
  nFixedConstr = workingset->nWConstr[1] + workingset->nWConstr[0];
  if ((workingset->nWConstr[2] + workingset->nWConstr[3]) + workingset->
      nWConstr[4] > 0) {
    double tol;
    int idx;
    int i;
    tol = 1000.0 * static_cast<double>(workingset->nVar) *
      2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager->jpvt[idx] = 1;
    }

    i = nFixedConstr + 1;
    for (idx = i; idx <= nActiveConstr; idx++) {
      qrmanager->jpvt[idx - 1] = 0;
    }

    factorQRE(qrmanager, workingset->ATwset, workingset->nVar,
              workingset->nActiveConstr);
    nActiveConstr = 0;
    for (idx = workingset->nActiveConstr; idx > workingset->nVar; idx--) {
      nActiveConstr++;
      memspace->workspace_int[nActiveConstr - 1] = qrmanager->jpvt[idx - 1];
    }

    if (idx <= workingset->nVar) {
      while ((idx > nFixedConstr) && (std::abs(qrmanager->QR[(idx +
                qrmanager->QR.size(0) * (idx - 1)) - 1]) < tol)) {
        nActiveConstr++;
        memspace->workspace_int[nActiveConstr - 1] = qrmanager->jpvt[idx - 1];
        idx--;
      }
    }

    countsort(memspace->workspace_int, nActiveConstr, memspace->workspace_sort,
              nFixedConstr + 1, workingset->nActiveConstr);
    for (idx = nActiveConstr; idx >= 1; idx--) {
      removeConstr(workingset, memspace->workspace_int[idx - 1]);
    }
  }
}

//
// File trailer for RemoveDependentIneq_.cpp
//
// [EOF]
//

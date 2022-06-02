//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: removeConstr.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "removeConstr.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : g_struct_T *obj
//                int idx_global
// Return Type  : void
//
void removeConstr(g_struct_T *obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  TYPE_tmp = obj->Wid[idx_global - 1] - 1;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE_tmp] + obj->Wlocalidx[idx_global -
                       1]) - 2] = false;
  obj->Wid[idx_global - 1] = obj->Wid[obj->nActiveConstr - 1];
  obj->Wlocalidx[idx_global - 1] = obj->Wlocalidx[obj->nActiveConstr - 1];
  i = obj->nVar;
  for (int idx = 0; idx < i; idx++) {
    obj->ATwset[idx + obj->ATwset.size(0) * (idx_global - 1)] = obj->ATwset[idx
      + obj->ATwset.size(0) * (obj->nActiveConstr - 1)];
  }

  obj->bwset[idx_global - 1] = obj->bwset[obj->nActiveConstr - 1];
  obj->nActiveConstr--;
  obj->nWConstr[TYPE_tmp]--;
}

//
// File trailer for removeConstr.cpp
//
// [EOF]
//

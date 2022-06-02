//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: addBoundToActiveSetMatrix_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "addBoundToActiveSetMatrix_.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : g_struct_T *obj
//                int TYPE
//                int idx_local
// Return Type  : void
//
void addBoundToActiveSetMatrix_(g_struct_T *obj, int TYPE, int idx_local)
{
  int i;
  int idx_bnd_local;
  int idx;
  int i1;
  obj->nWConstr[TYPE - 1]++;
  obj->isActiveConstr[(obj->isActiveIdx[TYPE - 1] + idx_local) - 2] = true;
  obj->nActiveConstr++;
  obj->Wid[obj->nActiveConstr - 1] = TYPE;
  obj->Wlocalidx[obj->nActiveConstr - 1] = idx_local;
  i = obj->nActiveConstr - 1;
  switch (TYPE) {
   case 5:
    idx_bnd_local = 2147483646;
    obj->bwset[obj->nActiveConstr - 1] = 1.7976931348623157E+308;
    break;

   default:
    idx_bnd_local = obj->indexLB[idx_local - 1] - 1;
    obj->bwset[obj->nActiveConstr - 1] = obj->lb[idx_bnd_local];
    break;
  }

  for (idx = 0; idx < idx_bnd_local; idx++) {
    obj->ATwset[idx + obj->ATwset.size(0) * i] = 0.0;
  }

  obj->ATwset[idx_bnd_local + obj->ATwset.size(0) * (obj->nActiveConstr - 1)] =
    2.0 * static_cast<double>(TYPE == 5) - 1.0;
  idx_bnd_local += 2;
  i1 = obj->nVar;
  for (idx = idx_bnd_local; idx <= i1; idx++) {
    obj->ATwset[(idx + obj->ATwset.size(0) * i) - 1] = 0.0;
  }

  switch (obj->probType) {
   case 3:
   case 2:
    break;

   default:
    obj->ATwset[(obj->nVar + obj->ATwset.size(0) * (obj->nActiveConstr - 1)) - 1]
      = -1.0;
    break;
  }
}

//
// File trailer for addBoundToActiveSetMatrix_.cpp
//
// [EOF]
//

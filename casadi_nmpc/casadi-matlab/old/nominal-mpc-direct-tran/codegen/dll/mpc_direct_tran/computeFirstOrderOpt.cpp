//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFirstOrderOpt.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "computeFirstOrderOpt.h"
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions

//
// Arguments    : b_struct_T *solution
//                const d_struct_T *objective
//                int workingset_nVar
//                int workingset_ldA
//                const coder::array<double, 2U> *workingset_ATwset
//                int workingset_nActiveConstr
//                coder::array<double, 2U> *workspace
// Return Type  : void
//
void computeFirstOrderOpt(b_struct_T *solution, const d_struct_T *objective, int
  workingset_nVar, int workingset_ldA, const coder::array<double, 2U>
  &workingset_ATwset, int workingset_nActiveConstr, coder::array<double, 2U>
  &workspace)
{
  int iy;
  int ix;
  int idxmax;
  int iac;
  for (iy = 0; iy < workingset_nVar; iy++) {
    workspace[iy] = objective->grad[iy];
  }

  if (workingset_nActiveConstr != 0) {
    ix = 0;
    idxmax = workingset_ldA * (workingset_nActiveConstr - 1) + 1;
    for (iac = 1; workingset_ldA < 0 ? iac >= idxmax : iac <= idxmax; iac +=
         workingset_ldA) {
      int i;
      iy = 0;
      i = (iac + workingset_nVar) - 1;
      for (int ia = iac; ia <= i; ia++) {
        workspace[iy] = workspace[iy] + workingset_ATwset[ia - 1] *
          solution->lambda[ix];
        iy++;
      }

      ix++;
    }
  }

  if (workingset_nVar < 1) {
    idxmax = 0;
  } else {
    idxmax = 1;
    if (workingset_nVar > 1) {
      double smax;
      ix = 0;
      smax = std::abs(workspace[0]);
      for (iy = 2; iy <= workingset_nVar; iy++) {
        double s;
        ix++;
        s = std::abs(workspace[ix]);
        if (s > smax) {
          idxmax = iy;
          smax = s;
        }
      }
    }
  }

  solution->firstorderopt = std::abs(workspace[idxmax - 1]);
}

//
// File trailer for computeFirstOrderOpt.cpp
//
// [EOF]
//

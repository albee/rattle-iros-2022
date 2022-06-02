//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: linearForm_.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "linearForm_.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : boolean_T obj_hasLinear
//                int obj_nvar
//                coder::array<double, 2U> *workspace
//                const coder::array<double, 2U> *H
//                const coder::array<double, 1U> *f
//                const coder::array<double, 1U> *x
// Return Type  : void
//
void linearForm_(boolean_T obj_hasLinear, int obj_nvar, coder::array<double, 2U>
                 &workspace, const coder::array<double, 2U> &H, const coder::
                 array<double, 1U> &f, const coder::array<double, 1U> &x)
{
  int iy;
  int ix;
  int i;
  int iac;
  if (obj_hasLinear) {
    for (ix = 0; ix < obj_nvar; ix++) {
      workspace[ix] = f[ix];
    }
  } else {
    for (iy = 0; iy < obj_nvar; iy++) {
      workspace[iy] = 0.0;
    }
  }

  ix = 0;
  i = obj_nvar * (obj_nvar - 1) + 1;
  for (iac = 1; obj_nvar < 0 ? iac >= i : iac <= i; iac += obj_nvar) {
    double c;
    int i1;
    c = 0.5 * x[ix];
    iy = 0;
    i1 = (iac + obj_nvar) - 1;
    for (int ia = iac; ia <= i1; ia++) {
      workspace[iy] = workspace[iy] + H[ia - 1] * c;
      iy++;
    }

    ix++;
  }
}

//
// File trailer for linearForm_.cpp
//
// [EOF]
//

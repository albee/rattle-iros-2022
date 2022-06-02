//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeFval_ReuseHx.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "computeFval_ReuseHx.h"
#include "computeFval.h"
#include "linearForm_.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const d_struct_T *obj
//                coder::array<double, 2U> *workspace
//                const coder::array<double, 1U> *f
//                const coder::array<double, 1U> *x
// Return Type  : double
//
double computeFval_ReuseHx(const d_struct_T *obj, coder::array<double, 2U>
  &workspace, const coder::array<double, 1U> &f, const coder::array<double, 1U>
  &x)
{
  double val;
  val = 0.0;
  switch (obj->objtype) {
   case 5:
    val = obj->gammaScalar * x[obj->nvar - 1];
    break;

   case 3:
    {
      if (obj->hasLinear) {
        int ixlast;
        int k;
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          workspace[k] = 0.5 * obj->Hx[k] + f[k];
        }

        if (obj->nvar >= 1) {
          ixlast = obj->nvar;
          for (k = 0; k < ixlast; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        if (obj->nvar >= 1) {
          int ixlast;
          ixlast = obj->nvar;
          for (int k = 0; k < ixlast; k++) {
            val += x[k] * obj->Hx[k];
          }
        }

        val *= 0.5;
      }
    }
    break;

   case 4:
    {
      int maxRegVar_tmp;
      maxRegVar_tmp = obj->maxVar - 1;
      if (obj->hasLinear) {
        int ixlast;
        int k;
        ixlast = obj->nvar;
        for (k = 0; k < ixlast; k++) {
          workspace[k] = f[k];
        }

        ixlast = obj->maxVar - obj->nvar;
        for (k = 0; k <= ixlast - 2; k++) {
          workspace[obj->nvar + k] = 0.0;
        }

        for (k = 0; k < maxRegVar_tmp; k++) {
          workspace[k] = workspace[k] + 0.5 * obj->Hx[k];
        }

        if (maxRegVar_tmp >= 1) {
          ixlast = obj->maxVar;
          for (k = 0; k <= ixlast - 2; k++) {
            val += x[k] * workspace[k];
          }
        }
      } else {
        int ixlast;
        int k;
        if (maxRegVar_tmp >= 1) {
          ixlast = obj->maxVar;
          for (k = 0; k <= ixlast - 2; k++) {
            val += x[k] * obj->Hx[k];
          }
        }

        val *= 0.5;
        ixlast = obj->nvar + 1;
        for (k = ixlast; k <= maxRegVar_tmp; k++) {
          val += x[k - 1] * 0.0;
        }
      }
    }
    break;
  }

  return val;
}

//
// File trailer for computeFval_ReuseHx.cpp
//
// [EOF]
//

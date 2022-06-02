//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: computeGrad_StoreHx.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "computeGrad_StoreHx.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : d_struct_T *obj
//                const coder::array<double, 2U> *H
//                const coder::array<double, 1U> *f
//                const coder::array<double, 1U> *x
// Return Type  : void
//
void computeGrad_StoreHx(d_struct_T *obj, const coder::array<double, 2U> &H,
  const coder::array<double, 1U> &f, const coder::array<double, 1U> &x)
{
  int iac;
  switch (obj->objtype) {
   case 5:
    {
      int i;
      i = obj->nvar;
      for (int b_i = 0; b_i <= i - 2; b_i++) {
        obj->grad[b_i] = 0.0;
      }

      obj->grad[obj->nvar - 1] = obj->gammaScalar;
    }
    break;

   case 3:
    {
      int i;
      int b_i;
      int lda;
      b_i = obj->nvar - 1;
      lda = obj->nvar;
      if (obj->nvar != 0) {
        int iy;
        int ix;
        for (iy = 0; iy <= b_i; iy++) {
          obj->Hx[iy] = 0.0;
        }

        ix = 0;
        i = obj->nvar * (obj->nvar - 1) + 1;
        for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
          int i1;
          iy = 0;
          i1 = iac + b_i;
          for (int ia = iac; ia <= i1; ia++) {
            obj->Hx[iy] = obj->Hx[iy] + H[ia - 1] * x[ix];
            iy++;
          }

          ix++;
        }
      }

      i = obj->nvar;
      for (b_i = 0; b_i < i; b_i++) {
        obj->grad[b_i] = obj->Hx[b_i];
      }

      if (obj->hasLinear && (obj->nvar >= 1)) {
        i = obj->nvar - 1;
        for (b_i = 0; b_i <= i; b_i++) {
          obj->grad[b_i] = obj->grad[b_i] + f[b_i];
        }
      }
    }
    break;

   case 4:
    {
      int i;
      int b_i;
      int maxRegVar;
      int lda;
      maxRegVar = obj->maxVar - 1;
      b_i = obj->nvar - 1;
      lda = obj->nvar;
      if (obj->nvar != 0) {
        int iy;
        int ix;
        for (iy = 0; iy <= b_i; iy++) {
          obj->Hx[iy] = 0.0;
        }

        ix = 0;
        i = obj->nvar * (obj->nvar - 1) + 1;
        for (iac = 1; lda < 0 ? iac >= i : iac <= i; iac += lda) {
          int i1;
          iy = 0;
          i1 = iac + b_i;
          for (int ia = iac; ia <= i1; ia++) {
            obj->Hx[iy] = obj->Hx[iy] + H[ia - 1] * x[ix];
            iy++;
          }

          ix++;
        }
      }

      i = obj->nvar + 1;
      for (b_i = i; b_i <= maxRegVar; b_i++) {
        obj->Hx[b_i - 1] = 0.0 * x[b_i - 1];
      }

      for (b_i = 0; b_i < maxRegVar; b_i++) {
        obj->grad[b_i] = obj->Hx[b_i];
      }

      if (obj->hasLinear && (obj->nvar >= 1)) {
        i = obj->nvar - 1;
        for (b_i = 0; b_i <= i; b_i++) {
          obj->grad[b_i] = obj->grad[b_i] + f[b_i];
        }
      }
    }
    break;
  }
}

//
// File trailer for computeGrad_StoreHx.cpp
//
// [EOF]
//

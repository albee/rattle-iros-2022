//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: repmat.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "repmat.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const double a[6]
//                int varargin_1
//                coder::array<double, 1U> *b
// Return Type  : void
//
void b_repmat(const double a[6], int varargin_1, coder::array<double, 1U> &b)
{
  b.set_size((6 * varargin_1));
  for (int itilerow = 0; itilerow < varargin_1; itilerow++) {
    int ibcol;
    ibcol = itilerow * 6;
    for (int k = 0; k < 6; k++) {
      b[ibcol + k] = a[k];
    }
  }
}

//
// Arguments    : const double a[3]
//                int varargin_1
//                coder::array<double, 1U> *b
// Return Type  : void
//
void repmat(const double a[3], int varargin_1, coder::array<double, 1U> &b)
{
  b.set_size((3 * varargin_1));
  for (int itilerow = 0; itilerow < varargin_1; itilerow++) {
    int ibcol;
    ibcol = itilerow * 3;
    b[ibcol] = a[0];
    b[ibcol + 1] = a[1];
    b[ibcol + 2] = a[2];
  }
}

//
// File trailer for repmat.cpp
//
// [EOF]
//

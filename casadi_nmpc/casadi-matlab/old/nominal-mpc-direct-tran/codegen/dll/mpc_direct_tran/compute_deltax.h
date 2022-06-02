//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: compute_deltax.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void compute_deltax(const coder::array<double, 2U> &H, b_struct_T
  *solution, c_struct_T *memspace, const e_struct_T *qrmanager, f_struct_T
  *cholmanager, const d_struct_T *objective);

#endif

//
// File trailer for compute_deltax.h
//
// [EOF]
//

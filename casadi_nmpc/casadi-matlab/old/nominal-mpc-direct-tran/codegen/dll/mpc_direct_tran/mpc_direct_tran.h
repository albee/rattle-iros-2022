//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_direct_tran.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef MPC_DIRECT_TRAN_H
#define MPC_DIRECT_TRAN_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
MPC_DIRECT_TRAN_DLL_EXPORT extern void mpc_direct_tran(const double x_cur[6],
  int t_idx, double dt, int N, int Nf, const double x_ref_hist[60000], double
  u_mag, double v_mag, double Q_mag, double R_mag, double mass, const double
  w_bounds[6], double u_t_idx[3]);

#endif

//
// File trailer for mpc_direct_tran.h
//
// [EOF]
//

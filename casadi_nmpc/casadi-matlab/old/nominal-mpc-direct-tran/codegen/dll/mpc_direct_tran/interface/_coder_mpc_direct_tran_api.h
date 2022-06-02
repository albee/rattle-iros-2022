/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_mpc_direct_tran_api.h
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 08-Jul-2020 19:43:17
 */

#ifndef _CODER_MPC_DIRECT_TRAN_API_H
#define _CODER_MPC_DIRECT_TRAN_API_H

/* Include Files */
#include <stddef.h>
#include <stdlib.h>
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void mpc_direct_tran(real_T x_cur[6], int32_T t_idx, real_T dt, int32_T N,
  int32_T Nf, real_T x_ref_hist[60000], real_T u_mag, real_T v_mag, real_T Q_mag,
  real_T R_mag, real_T mass, real_T w_bounds[6], real_T u_t_idx[3]);
extern void mpc_direct_tran_api(const mxArray * const prhs[12], int32_T nlhs,
  const mxArray *plhs[1]);
extern void mpc_direct_tran_atexit(void);
extern void mpc_direct_tran_initialize(void);
extern void mpc_direct_tran_terminate(void);
extern void mpc_direct_tran_xil_shutdown(void);
extern void mpc_direct_tran_xil_terminate(void);

#endif

/*
 * File trailer for _coder_mpc_direct_tran_api.h
 *
 * [EOF]
 */

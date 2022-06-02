//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  get_dynamics_for_dt.cpp
//
//  Code generation for function 'get_dynamics_for_dt'
//


// Include files
#include "get_dynamics_for_dt.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Function Definitions
void get_dynamics_for_dt(real_T dt, real_T A[36], real_T B[18])
{
  static const int8_T iv[6] = { 0, 0, 0, 1, 0, 0 };

  static const int8_T iv1[6] = { 0, 0, 0, 0, 1, 0 };

  static const int8_T iv2[6] = { 0, 0, 0, 0, 0, 1 };

  covrtLogFcn(&emlrtCoverageInstance, 1, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 1, 0);

  //  Set dynamics based on dt zero-order hold
  A[0] = 1.0;
  A[1] = 0.0;
  A[2] = 0.0;
  A[3] = dt;
  A[4] = 0.0;
  A[5] = 0.0;
  A[6] = 0.0;
  A[7] = 1.0;
  A[8] = 0.0;
  A[9] = 0.0;
  A[10] = dt;
  A[11] = 0.0;
  A[12] = 0.0;
  A[13] = 0.0;
  A[14] = 1.0;
  A[15] = 0.0;
  A[16] = 0.0;
  A[17] = dt;
  for (int32_T i = 0; i < 6; i++) {
    A[i + 18] = iv[i];
    A[i + 24] = iv1[i];
    A[i + 30] = iv2[i];
  }

  real_T B_tmp;
  B_tmp = dt * dt / 19.16;
  B[0] = B_tmp;
  B[1] = 0.0;
  B[2] = 0.0;
  B[3] = 0.0;
  B[4] = B_tmp;
  B[5] = 0.0;
  B[6] = 0.0;
  B[7] = 0.0;
  B[8] = B_tmp;
  B[9] = dt / 9.58;
  B[10] = 0.0;
  B[11] = 0.0;
  B[12] = 0.0;
  B[13] = dt / 9.58;
  B[14] = 0.0;
  B[15] = 0.0;
  B[16] = 0.0;
  B[17] = dt / 9.58;
}

// End of code generation (get_dynamics_for_dt.cpp)

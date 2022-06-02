//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  get_dynamics_for_dt.h
//
//  Code generation for function 'get_dynamics_for_dt'
//


#pragma once

// Include files
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "mex.h"
#include "emlrt.h"
#include "covrt.h"
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
void get_dynamics_for_dt(real_T dt, real_T A[36], real_T B[18]);

// End of code generation (get_dynamics_for_dt.h)

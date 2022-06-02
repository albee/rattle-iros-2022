//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  PresolveWorkingSet.h
//
//  Code generation for function 'PresolveWorkingSet'
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
void PresolveWorkingSet(const emlrtStack *sp, b_struct_T *solution, c_struct_T
  *memspace, g_struct_T *workingset, e_struct_T *qrmanager, h_struct_T *options);
void b_PresolveWorkingSet(const emlrtStack *sp, b_struct_T *solution, c_struct_T
  *memspace, g_struct_T *workingset, e_struct_T *qrmanager);

// End of code generation (PresolveWorkingSet.h)

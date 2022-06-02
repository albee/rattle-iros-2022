//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  RemoveDependentIneq_.h
//
//  Code generation for function 'RemoveDependentIneq_'
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
void RemoveDependentIneq_(const emlrtStack *sp, g_struct_T *workingset,
  e_struct_T *qrmanager, c_struct_T *memspace);
void b_RemoveDependentIneq_(const emlrtStack *sp, g_struct_T *workingset,
  e_struct_T *qrmanager, c_struct_T *memspace);

// End of code generation (RemoveDependentIneq_.h)

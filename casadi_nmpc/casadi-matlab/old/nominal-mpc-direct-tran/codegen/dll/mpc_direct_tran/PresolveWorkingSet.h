//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: PresolveWorkingSet.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef PRESOLVEWORKINGSET_H
#define PRESOLVEWORKINGSET_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void PresolveWorkingSet(b_struct_T *solution, c_struct_T *memspace,
  g_struct_T *workingset, e_struct_T *qrmanager, h_struct_T *options);
extern void b_PresolveWorkingSet(b_struct_T *solution, c_struct_T *memspace,
  g_struct_T *workingset, e_struct_T *qrmanager);

#endif

//
// File trailer for PresolveWorkingSet.h
//
// [EOF]
//

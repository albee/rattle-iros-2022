//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: quadprog.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef QUADPROG_H
#define QUADPROG_H

// Include Files
#include <cstddef>
#include <cstdlib>
#include "rtwtypes.h"
#include "mpc_direct_tran_types.h"

// Function Declarations
extern void quadprog(const coder::array<double, 2U> &H, const coder::array<
                     double, 1U> &f, const coder::array<double, 2U> &Aineq,
                     const coder::array<double, 1U> &bineq, const coder::array<
                     double, 2U> &Aeq, const coder::array<double, 1U> &beq,
                     const coder::array<double, 1U> &x0, coder::array<double, 1U>
                     &x);

#endif

//
// File trailer for quadprog.h
//
// [EOF]
//

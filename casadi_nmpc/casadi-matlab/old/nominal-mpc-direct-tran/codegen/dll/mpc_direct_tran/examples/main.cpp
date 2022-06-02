//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: main.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

//***********************************************************************
// This automatically generated example C++ main file shows how to call
// entry-point functions that MATLAB Coder generated. You must customize
// this file for your application. Do not modify this file directly.
// Instead, make a copy of this file, modify it, and integrate it into
// your development environment.
//
// This file initializes entry-point function arguments to a default
// size and value before calling the entry-point functions. It does
// not store or use any values returned from the entry-point functions.
// If necessary, it does pre-allocate memory for returned values.
// You can use this file as a starting point for a main function that
// you can deploy in your application.
//
// After you copy the file, and before you deploy it, you must make the
// following changes:
// * For variable-size function arguments, change the example sizes to
// the sizes that your application requires.
// * Change the example values of function arguments to the values that
// your application requires.
// * If the entry-point functions return values, store these values or
// otherwise use them as required by your application.
//
//***********************************************************************

// Include Files
#include "main.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_terminate.h"
#include "rt_nonfinite.h"

// Function Declarations
static void argInit_10000x6_real_T(double result[60000]);
static void argInit_6x1_real_T(double result[6]);
static int argInit_int32_T();
static double argInit_real_T();
static void main_mpc_direct_tran();

// Function Definitions

//
// Arguments    : double result[60000]
// Return Type  : void
//
static void argInit_10000x6_real_T(double result[60000])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 10000; idx0++) {
    for (int idx1 = 0; idx1 < 6; idx1++) {
      // Set the value of the array element.
      // Change this value to the value that the application requires.
      result[idx0 + 10000 * idx1] = argInit_real_T();
    }
  }
}

//
// Arguments    : double result[6]
// Return Type  : void
//
static void argInit_6x1_real_T(double result[6])
{
  // Loop over the array to initialize each element.
  for (int idx0 = 0; idx0 < 6; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

//
// Arguments    : void
// Return Type  : int
//
static int argInit_int32_T()
{
  return 0;
}

//
// Arguments    : void
// Return Type  : double
//
static double argInit_real_T()
{
  return 0.0;
}

//
// Arguments    : void
// Return Type  : void
//
static void main_mpc_direct_tran()
{
  double x_cur_tmp[6];
  int t_idx_tmp;
  double dt_tmp;
  static double dv[60000];
  double u_t_idx[3];

  // Initialize function 'mpc_direct_tran' input arguments.
  // Initialize function input argument 'x_cur'.
  argInit_6x1_real_T(x_cur_tmp);
  t_idx_tmp = argInit_int32_T();
  dt_tmp = argInit_real_T();

  // Initialize function input argument 'x_ref_hist'.
  // Initialize function input argument 'w_bounds'.
  // Call the entry-point 'mpc_direct_tran'.
  argInit_10000x6_real_T(dv);
  mpc_direct_tran(x_cur_tmp, t_idx_tmp, dt_tmp, t_idx_tmp, t_idx_tmp, dv, dt_tmp,
                  dt_tmp, dt_tmp, dt_tmp, dt_tmp, x_cur_tmp, u_t_idx);
}

//
// Arguments    : int argc
//                const char * const argv[]
// Return Type  : int
//
int main(int, const char * const [])
{
  // The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. 
  // Invoke the entry-point functions.
  // You can call entry-point functions multiple times.
  main_mpc_direct_tran();

  // Terminate the application.
  // You do not need to do this more than one time.
  mpc_direct_tran_terminate();
  return 0;
}

//
// File trailer for main.cpp
//
// [EOF]
//

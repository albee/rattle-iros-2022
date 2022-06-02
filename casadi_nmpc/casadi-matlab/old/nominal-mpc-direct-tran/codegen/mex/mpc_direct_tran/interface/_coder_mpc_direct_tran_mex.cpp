//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  _coder_mpc_direct_tran_mex.cpp
//
//  Code generation for function '_coder_mpc_direct_tran_mex'
//


// Include files
#include "_coder_mpc_direct_tran_mex.h"
#include "_coder_mpc_direct_tran_api.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mpc_direct_tran_initialize.h"
#include "mpc_direct_tran_terminate.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mpc_direct_tran_mexFunction(mpc_direct_tranStackData
  *SD, int32_T nlhs, mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[6]);

// Function Definitions
void mpc_direct_tran_mexFunction(mpc_direct_tranStackData *SD, int32_T nlhs,
  mxArray *plhs[1], int32_T nrhs, const mxArray *prhs[6])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              // site
    NULL,                              // tls
    NULL                               // prev
  };

  st.tls = emlrtRootTLSGlobal;

  // Check for proper number of arguments.
  if (nrhs != 6) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 6, 4,
                        15, "mpc_direct_tran");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 15,
                        "mpc_direct_tran");
  }

  // Call the function.
  mpc_direct_tran_api(SD, prhs, nlhs, outputs);

  // Copy over outputs to the caller.
  emlrtReturnArrays(1, plhs, &outputs[0]);
}

void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mpc_direct_tranStackData *mpc_direct_tranStackDataGlobal = NULL;
  mpc_direct_tranStackDataGlobal = new mpc_direct_tranStackData;
  mexAtExit(&mpc_direct_tran_atexit);

  // Module initialization.
  mpc_direct_tran_initialize();

  // Dispatch the entry-point.
  mpc_direct_tran_mexFunction(mpc_direct_tranStackDataGlobal, nlhs, plhs, nrhs,
    prhs);

  // Module termination.
  mpc_direct_tran_terminate();
  delete mpc_direct_tranStackDataGlobal;
}

emlrtCTX mexFunctionCreateRootTLS()
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

// End of code generation (_coder_mpc_direct_tran_mex.cpp)

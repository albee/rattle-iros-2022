/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_mpc_direct_tran_mex.c
 *
 * MATLAB Coder version            : 5.0
 * C/C++ source code generated on  : 08-Jul-2020 19:43:17
 */

/* Include Files */
#include "_coder_mpc_direct_tran_mex.h"
#include "_coder_mpc_direct_tran_api.h"

/* Function Declarations */
MEXFUNCTION_LINKAGE void mpc_direct_tran_mexFunction(int32_T nlhs, mxArray *
  plhs[1], int32_T nrhs, const mxArray *prhs[12]);

/* Function Definitions */

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[1]
 *                int32_T nrhs
 *                const mxArray *prhs[12]
 * Return Type  : void
 */
void mpc_direct_tran_mexFunction(int32_T nlhs, mxArray *plhs[1], int32_T nrhs,
  const mxArray *prhs[12])
{
  const mxArray *outputs[1];
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;

  /* Check for proper number of arguments. */
  if (nrhs != 12) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:WrongNumberOfInputs", 5, 12, 12, 4,
                        15, "mpc_direct_tran");
  }

  if (nlhs > 1) {
    emlrtErrMsgIdAndTxt(&st, "EMLRT:runTime:TooManyOutputArguments", 3, 4, 15,
                        "mpc_direct_tran");
  }

  /* Call the function. */
  mpc_direct_tran_api(prhs, nlhs, outputs);

  /* Copy over outputs to the caller. */
  emlrtReturnArrays(1, plhs, outputs);
}

/*
 * Arguments    : int32_T nlhs
 *                mxArray *plhs[]
 *                int32_T nrhs
 *                const mxArray *prhs[]
 * Return Type  : void
 */
void mexFunction(int32_T nlhs, mxArray *plhs[], int32_T nrhs, const mxArray
                 *prhs[])
{
  mexAtExit(&mpc_direct_tran_atexit);

  /* Module initialization. */
  mpc_direct_tran_initialize();

  /* Dispatch the entry-point. */
  mpc_direct_tran_mexFunction(nlhs, plhs, nrhs, prhs);

  /* Module termination. */
  mpc_direct_tran_terminate();
}

/*
 * Arguments    : void
 * Return Type  : emlrtCTX
 */
emlrtCTX mexFunctionCreateRootTLS(void)
{
  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  return emlrtRootTLSGlobal;
}

/*
 * File trailer for _coder_mpc_direct_tran_mex.c
 *
 * [EOF]
 */

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mpc_direct_tran_initialize.cpp
//
//  Code generation for function 'mpc_direct_tran_initialize'
//


// Include files
#include "mpc_direct_tran_initialize.h"
#include "_coder_mpc_direct_tran_mex.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static void mpc_direct_tran_once();

// Function Definitions
static void mpc_direct_tran_once()
{
  mex_InitInfAndNan();

  // Allocate instance data
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  // Initialize Coverage Information
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",
                  0U, 1U, 7U, 1U, 0U, 0U, 0U, 2U, 0U, 0U, 0U);

  // Initialize Function Information
  covrtFcnInit(&emlrtCoverageInstance, 0U, 0U, "mpc_direct_tran", 1662, -1, 4983);

  // Initialize Basic Block Information
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 6U, 3822, -1, 4951);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 5U, 3541, -1, 3776);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 4U, 3126, -1, 3445);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 3U, 2963, -1, 3000);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 2U, 2883, -1, 2915);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 1U, 2803, -1, 2830);
  covrtBasicBlockInit(&emlrtCoverageInstance, 0U, 0U, 1801, -1, 2619);

  // Initialize If Information
  covrtIfInit(&emlrtCoverageInstance, 0U, 0U, 2677, 2692, 2866, 2954);

  // Initialize MCDC Information
  // Initialize For Information
  covrtForInit(&emlrtCoverageInstance, 0U, 0U, 2626, 2642, 3073);
  covrtForInit(&emlrtCoverageInstance, 0U, 1U, 3494, 3510, 3785);

  // Initialize While Information
  // Initialize Switch Information
  // Start callback for coverage engine
  covrtScriptStart(&emlrtCoverageInstance, 0U);

  // Allocate instance data
  covrtAllocateInstanceData(&emlrtCoverageInstance);

  // Initialize Coverage Information
  covrtScriptInit(&emlrtCoverageInstance,
                  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/get_dynamics_for_dt.m",
                  1U, 1U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U);

  // Initialize Function Information
  covrtFcnInit(&emlrtCoverageInstance, 1U, 0U, "get_dynamics_for_dt", 43, -1,
               536);

  // Initialize Basic Block Information
  covrtBasicBlockInit(&emlrtCoverageInstance, 1U, 0U, 93, -1, 531);

  // Initialize If Information
  // Initialize MCDC Information
  // Initialize For Information
  // Initialize While Information
  // Initialize Switch Information
  // Start callback for coverage engine
  covrtScriptStart(&emlrtCoverageInstance, 1U);
}

void mpc_direct_tran_initialize()
{
  emlrtStack st = { NULL,              // site
    NULL,                              // tls
    NULL                               // prev
  };

  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtLicenseCheckR2012b(&st, "optimization_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    mpc_direct_tran_once();
  }
}

// End of code generation (mpc_direct_tran_initialize.cpp)

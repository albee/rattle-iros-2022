//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mpc_direct_tran_data.cpp
//
//  Code generation for function 'mpc_direct_tran_data'
//


// Include files
#include "mpc_direct_tran_data.h"
#include "mpc_direct_tran.h"
#include "rt_nonfinite.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal = NULL;
const volatile char_T *emlrtBreakCheckR2012bFlagVar = NULL;
emlrtContext emlrtContextGlobal = { true,// bFirstTime
  false,                               // bInitialized
  131594U,                             // fVersionInfo
  NULL,                                // fErrorFunction
  "mpc_direct_tran",                   // fFunctionName
  NULL,                                // fRTCallStack
  false,                               // bDebugMode
  { 2666790369U, 2630951428U, 3350295197U, 1643587045U },// fSigWrd
  NULL                                 // fSigMem
};

emlrtRSInfo o_emlrtRSI = { 10,         // lineNo
  "get_dynamics_for_dt",               // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/get_dynamics_for_dt.m"// pathName 
};

emlrtRSInfo p_emlrtRSI = { 11,         // lineNo
  "get_dynamics_for_dt",               // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/get_dynamics_for_dt.m"// pathName 
};

emlrtRSInfo q_emlrtRSI = { 12,         // lineNo
  "get_dynamics_for_dt",               // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/get_dynamics_for_dt.m"// pathName 
};

emlrtRSInfo r_emlrtRSI = { 45,         // lineNo
  "mpower",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/ops/mpower.m"// pathName
};

emlrtRSInfo s_emlrtRSI = { 70,         // lineNo
  "power",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/ops/power.m"// pathName
};

emlrtRSInfo u_emlrtRSI = { 21,         // lineNo
  "eml_int_forloop_overflow_check",    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/eml/eml_int_forloop_overflow_check.m"// pathName 
};

emlrtRSInfo lb_emlrtRSI = { 33,        // lineNo
  "ixamax",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/ixamax.m"// pathName 
};

emlrtRSInfo nb_emlrtRSI = { 51,        // lineNo
  "xcopy",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xcopy.m"// pathName 
};

emlrtRSInfo ob_emlrtRSI = { 50,        // lineNo
  "xcopy",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xcopy.m"// pathName 
};

emlrtRSInfo ac_emlrtRSI = { 9,         // lineNo
  "int",                               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/int.m"// pathName 
};

emlrtRSInfo fc_emlrtRSI = { 1,         // lineNo
  "factorQRE",                         // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/factorQRE.p"// pathName 
};

emlrtRSInfo tc_emlrtRSI = { 8,         // lineNo
  "majority",                          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/majority.m"// pathName 
};

emlrtRSInfo uc_emlrtRSI = { 31,        // lineNo
  "infocheck",                         // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/infocheck.m"// pathName 
};

emlrtRSInfo xc_emlrtRSI = { 45,        // lineNo
  "xscal",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xscal.m"// pathName 
};

emlrtRSInfo yc_emlrtRSI = { 86,        // lineNo
  "xgemv",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m"// pathName 
};

emlrtRSInfo ad_emlrtRSI = { 87,        // lineNo
  "xgemv",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m"// pathName 
};

emlrtRSInfo bd_emlrtRSI = { 62,        // lineNo
  "xger",                              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xger.m"// pathName 
};

emlrtRSInfo dd_emlrtRSI = { 1,         // lineNo
  "computeSquareQ",                    // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/computeSquareQ.p"// pathName 
};

emlrtRSInfo gd_emlrtRSI = { 51,        // lineNo
  "ceval_xorgqr",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m"// pathName 
};

emlrtRSInfo hd_emlrtRSI = { 46,        // lineNo
  "ceval_xorgqr",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m"// pathName 
};

emlrtRSInfo id_emlrtRSI = { 38,        // lineNo
  "ceval_xorgqr",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/xorgqr.m"// pathName 
};

emlrtRSInfo od_emlrtRSI = { 1,         // lineNo
  "removeConstr",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeConstr.p"// pathName 
};

emlrtRSInfo sd_emlrtRSI = { 1,         // lineNo
  "factorQR",                          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/factorQR.p"// pathName 
};

emlrtRSInfo ee_emlrtRSI = { 88,        // lineNo
  "xgemm",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemm.m"// pathName 
};

emlrtRSInfo je_emlrtRSI = { 61,        // lineNo
  "xaxpy",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xaxpy.m"// pathName 
};

emlrtRSInfo ke_emlrtRSI = { 60,        // lineNo
  "xaxpy",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xaxpy.m"// pathName 
};

emlrtRSInfo ye_emlrtRSI = { 28,        // lineNo
  "xrotg",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xrotg.m"// pathName 
};

emlrtRSInfo af_emlrtRSI = { 27,        // lineNo
  "xrotg",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xrotg.m"// pathName 
};

emlrtRSInfo cf_emlrtRSI = { 47,        // lineNo
  "xrot",                              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xrot.m"// pathName 
};

emlrtRSInfo ff_emlrtRSI = { 1,         // lineNo
  "factor",                            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/factor.p"// pathName 
};

emlrtRSInfo if_emlrtRSI = { 1,         // lineNo
  "solve",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/solve.p"// pathName 
};

emlrtRSInfo jf_emlrtRSI = { 71,        // lineNo
  "xtrsv",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsv.m"// pathName 
};

emlrtRSInfo kf_emlrtRSI = { 70,        // lineNo
  "xtrsv",                             // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsv.m"// pathName 
};

emlrtRTEInfo e_emlrtRTEI = { 58,       // lineNo
  23,                                  // colNo
  "assertValidSizeArg",                // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/assertValidSizeArg.m"// pName 
};

emlrtRTEInfo f_emlrtRTEI = { 64,       // lineNo
  15,                                  // colNo
  "assertValidSizeArg",                // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/assertValidSizeArg.m"// pName 
};

emlrtRTEInfo g_emlrtRTEI = { 1,        // lineNo
  1,                                   // colNo
  "quadprog",                          // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/eml/quadprog.p"// pName
};

emlrtRTEInfo h_emlrtRTEI = { 47,       // lineNo
  19,                                  // colNo
  "allOrAny",                          // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/allOrAny.m"// pName 
};

emlrtBCInfo w_emlrtBCI = { 1,          // iFirst
  6,                                   // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeConstr",                      // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeConstr.p",// pName 
  0                                    // checkKind
};

emlrtBCInfo x_emlrtBCI = { -1,         // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeConstr",                      // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeConstr.p",// pName 
  0                                    // checkKind
};

emlrtBCInfo fb_emlrtBCI = { 1,         // iFirst
  5,                                   // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "removeConstr",                      // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/removeConstr.p",// pName 
  0                                    // checkKind
};

emlrtRTEInfo m_emlrtRTEI = { 48,       // lineNo
  13,                                  // colNo
  "infocheck",                         // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/infocheck.m"// pName 
};

emlrtRTEInfo n_emlrtRTEI = { 45,       // lineNo
  13,                                  // colNo
  "infocheck",                         // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+lapack/infocheck.m"// pName 
};

emlrtBCInfo kb_emlrtBCI = { -1,        // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "factorQR",                          // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+QRManager/factorQR.p",// pName 
  0                                    // checkKind
};

emlrtBCInfo qb_emlrtBCI = { -1,        // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "isActive",                          // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/isActive.p",// pName 
  0                                    // checkKind
};

emlrtBCInfo xb_emlrtBCI = { -1,        // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "addConstrUpdateRecords_",           // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/addConstrUpdateRecords_.p",// pName 
  0                                    // checkKind
};

emlrtBCInfo ec_emlrtBCI = { -1,        // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "solve",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/solve.p",// pName 
  0                                    // checkKind
};

emlrtBCInfo fc_emlrtBCI = { -1,        // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "factor",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/factor.p",// pName 
  0                                    // checkKind
};

emlrtRTEInfo gb_emlrtRTEI = { 15,      // lineNo
  13,                                  // colNo
  "isinf",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/isinf.m"// pName
};

emlrtRTEInfo hb_emlrtRTEI = { 15,      // lineNo
  13,                                  // colNo
  "isnan",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/lib/matlab/elmat/isnan.m"// pName
};

emlrtRTEInfo ib_emlrtRTEI = { 92,      // lineNo
  33,                                  // colNo
  "ixamax",                            // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/ixamax.m"// pName 
};

emlrtRTEInfo lb_emlrtRTEI = { 122,     // lineNo
  26,                                  // colNo
  "xcopy",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xcopy.m"// pName 
};

emlrtRTEInfo vb_emlrtRTEI = { 120,     // lineNo
  26,                                  // colNo
  "xcopy",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xcopy.m"// pName 
};

emlrtRTEInfo wb_emlrtRTEI = { 311,     // lineNo
  18,                                  // colNo
  "xgemv",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m"// pName 
};

emlrtRTEInfo xb_emlrtRTEI = { 316,     // lineNo
  18,                                  // colNo
  "xgemv",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m"// pName 
};

emlrtRTEInfo cc_emlrtRTEI = { 202,     // lineNo
  22,                                  // colNo
  "xgemm",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemm.m"// pName 
};

emlrtRTEInfo dc_emlrtRTEI = { 204,     // lineNo
  22,                                  // colNo
  "xgemm",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemm.m"// pName 
};

emlrtRTEInfo ec_emlrtRTEI = { 207,     // lineNo
  22,                                  // colNo
  "xgemm",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemm.m"// pName 
};

emlrtRTEInfo jc_emlrtRTEI = { 313,     // lineNo
  18,                                  // colNo
  "xgemv",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xgemv.m"// pName 
};

emlrtRTEInfo kc_emlrtRTEI = { 101,     // lineNo
  26,                                  // colNo
  "xdot",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xdot.m"// pName 
};

emlrtRTEInfo pc_emlrtRTEI = { 135,     // lineNo
  28,                                  // colNo
  "xtrsv",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsv.m"// pName 
};

emlrtRTEInfo qc_emlrtRTEI = { 137,     // lineNo
  27,                                  // colNo
  "xtrsv",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xtrsv.m"// pName 
};

emlrtRTEInfo tc_emlrtRTEI = { 102,     // lineNo
  27,                                  // colNo
  "xscal",                             // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xscal.m"// pName 
};

covrtInstance emlrtCoverageInstance;

// End of code generation (mpc_direct_tran_data.cpp)

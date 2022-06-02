//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mpc_direct_tran.cpp
//
//  Code generation for function 'mpc_direct_tran'
//


// Include files
#include "mpc_direct_tran.h"
#include "computeFval.h"
#include "diag.h"
#include "eye.h"
#include "get_dynamics_for_dt.h"
#include "indexShapeCheck.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "repmat.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Type Definitions
struct cell_wrap_0
{
  coder::array<real_T, 2U> f1;
};

// Variable Definitions
static emlrtRSInfo emlrtRSI = { 63,    // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo b_emlrtRSI = { 77,  // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo c_emlrtRSI = { 114, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo d_emlrtRSI = { 120, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo e_emlrtRSI = { 121, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo f_emlrtRSI = { 122, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo g_emlrtRSI = { 125, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo h_emlrtRSI = { 126, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo i_emlrtRSI = { 127, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo j_emlrtRSI = { 130, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo k_emlrtRSI = { 131, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo l_emlrtRSI = { 146, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo m_emlrtRSI = { 147, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo n_emlrtRSI = { 148, // lineNo
  "mpc_direct_tran",                   // fcnName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pathName 
};

static emlrtRSInfo v_emlrtRSI = { 27,  // lineNo
  "cat",                               // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pathName
};

static emlrtRSInfo w_emlrtRSI = { 102, // lineNo
  "cat_impl",                          // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pathName
};

static emlrtDCInfo emlrtDCI = { 74,    // lineNo
  18,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  4                                    // checkKind
};

static emlrtDCInfo b_emlrtDCI = { 74,  // lineNo
  18,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo c_emlrtDCI = { 75,  // lineNo
  18,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  4                                    // checkKind
};

static emlrtDCInfo d_emlrtDCI = { 75,  // lineNo
  18,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtRTEInfo emlrtRTEI = { 81,  // lineNo
  13,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo b_emlrtRTEI = { 104,// lineNo
  13,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtBCInfo emlrtBCI = { -1,    // iFirst
  -1,                                  // iLast
  107,                                 // lineNo
  41,                                  // colNo
  "Aeq_state",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo b_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  107,                                 // lineNo
  57,                                  // colNo
  "Aeq_state",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtECInfo emlrtECI = { -1,    // nDims
  107,                                 // lineNo
  9,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtDCInfo e_emlrtDCI = { 130, // lineNo
  21,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo f_emlrtDCI = { 130, // lineNo
  39,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo g_emlrtDCI = { 131, // lineNo
  34,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo h_emlrtDCI = { 131, // lineNo
  54,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo i_emlrtDCI = { 145, // lineNo
  16,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtBCInfo c_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  147,                                 // lineNo
  17,                                  // colNo
  "dec_vec",                           // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo d_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  147,                                 // lineNo
  40,                                  // colNo
  "dec_vec",                           // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo e_emlrtBCI = { 1,   // iFirst
  10000,                               // iLast
  86,                                  // lineNo
  21,                                  // colNo
  "x_ref_hist",                        // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtDCInfo j_emlrtDCI = { 86,  // lineNo
  21,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtBCInfo f_emlrtBCI = { 1,   // iFirst
  10000,                               // iLast
  84,                                  // lineNo
  21,                                  // colNo
  "x_ref_hist",                        // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtDCInfo k_emlrtDCI = { 84,  // lineNo
  21,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtRTEInfo c_emlrtRTEI = { 283,// lineNo
  27,                                  // colNo
  "check_non_axis_size",               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pName
};

static emlrtDCInfo l_emlrtDCI = { 94,  // lineNo
  39,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo m_emlrtDCI = { 94,  // lineNo
  23,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo n_emlrtDCI = { 95,  // lineNo
  39,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo o_emlrtDCI = { 95,  // lineNo
  23,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo p_emlrtDCI = { 79,  // lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo q_emlrtDCI = { 79,  // lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  4                                    // checkKind
};

static emlrtDCInfo r_emlrtDCI = { 94,  // lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtBCInfo g_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  88,                                  // lineNo
  9,                                   // colNo
  "f",                                 // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtDCInfo s_emlrtDCI = { 95,  // lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtDCInfo t_emlrtDCI = { 96,  // lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  1                                    // checkKind
};

static emlrtBCInfo h_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  100,                                 // lineNo
  5,                                   // colNo
  "Aeq_state",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo i_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  106,                                 // lineNo
  9,                                   // colNo
  "Aeq_state",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo j_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  107,                                 // lineNo
  19,                                  // colNo
  "Aeq_state",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo k_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  102,                                 // lineNo
  5,                                   // colNo
  "beq",                               // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo l_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  101,                                 // lineNo
  5,                                   // colNo
  "Aeq_input",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo m_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  110,                                 // lineNo
  9,                                   // colNo
  "Aeq_input",                         // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtBCInfo n_emlrtBCI = { -1,  // iFirst
  -1,                                  // iLast
  148,                                 // lineNo
  15,                                  // colNo
  "U",                                 // aName
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo o_emlrtRTEI = { 76,// lineNo
  14,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo p_emlrtRTEI = { 79,// lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo q_emlrtRTEI = { 94,// lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo r_emlrtRTEI = { 95,// lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo s_emlrtRTEI = { 96,// lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo t_emlrtRTEI = { 114,// lineNo
  23,                                  // colNo
  "cat",                               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pName
};

static emlrtRTEInfo u_emlrtRTEI = { 114,// lineNo
  9,                                   // colNo
  "cat",                               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pName
};

static emlrtRTEInfo v_emlrtRTEI = { 120,// lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo w_emlrtRTEI = { 125,// lineNo
  5,                                   // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo x_emlrtRTEI = { 27,// lineNo
  9,                                   // colNo
  "cat",                               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pName
};

static emlrtRTEInfo y_emlrtRTEI = { 303,// lineNo
  14,                                  // colNo
  "cat",                               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pName
};

static emlrtRTEInfo ab_emlrtRTEI = { 132,// lineNo
  13,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

static emlrtRTEInfo bb_emlrtRTEI = { 305,// lineNo
  14,                                  // colNo
  "cat",                               // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/cat.m"// pName
};

static emlrtRTEInfo cb_emlrtRTEI = { 145,// lineNo
  10,                                  // colNo
  "mpc_direct_tran",                   // fName
  "/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/tube_mpc/matlab/export_edition/mpc_direct_tran.m"// pName 
};

// Function Definitions
void mpc_direct_tran(const emlrtStack *sp, const real_T x_cur[6], real_T t_idx,
                     real_T dt, real_T N, real_T Nf, const real_T x_ref_hist
                     [60000], real_T u_t_idx[3])
{
  real_T A[36];
  real_T B[18];
  real_T state_horz_dim;
  real_T input_horz_dim;
  real_T dec_dim;
  int32_T i;
  int32_T i1;
  int32_T input_horz_dim_tmp;
  int32_T b_input_horz_dim[2];
  int32_T loop_ub;
  coder::array<real_T, 2U> Aeq_state;
  int32_T i2;
  coder::array<real_T, 2U> H;
  real_T d;
  int32_T b_loop_ub;
  coder::array<real_T, 1U> f;
  int32_T b_i;
  real_T d1;
  int32_T input_sizes_idx_1;
  int32_T i3;
  real_T x_des[6];
  int32_T i4;
  coder::array<real_T, 2U> Aeq_input;
  int32_T i5;
  coder::array<real_T, 1U> beq;
  real_T Aeq_state_tmp[36];
  real_T b_A[36];
  int32_T i6;
  boolean_T empty_non_axis_sizes;
  int32_T b_state_horz_dim[2];
  cell_wrap_0 reshapes[2];
  cell_wrap_0 b_reshapes[2];
  coder::array<real_T, 2U> Aineq_input;
  coder::array<real_T, 1U> bineq_input;
  cell_wrap_0 c_reshapes[2];
  coder::array<real_T, 1U> bineq_state;
  uint32_T u;
  cell_wrap_0 d_reshapes[2];
  cell_wrap_0 e_reshapes[2];
  cell_wrap_0 f_reshapes[2];
  coder::array<real_T, 1U> b_bineq_input;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  covrtLogFcn(&emlrtCoverageInstance, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0, 0);

  // {
  // Perform robust MPC, direct transcription.
  // }
  //          K_lqr;  % ancillary controller gain
  //          w_mag;  % max magnitude of uncertainty
  //          x_ref;  % reference value
  //      function prepare_ancillary(self)
  //          %--- Ancillary controller ---%
  //          Q = self.Q_mag*eye(self.n);
  //          R = self.R_mag*eye(self.m);
  //          [K, ~, ~] = dlqr(self.A, self.B, Q, R, zeros(self.n, self.m));
  //          self.K_lqr = -K;
  //
  //          self.shrink_constraints();  % shrink constraints for tube MPC
  //      end
  //
  //      % Shrink the constraints for the selected disturbance rejection term
  //      function um_shrunk = shrink_constraints(self)
  //          max_w = self.w_mag*ones(self.n,1);
  //          K_dr = self.K_lqr;
  //          Zmax = K_dr*max_w;
  //          self.u_constraint = self.u_constraint - abs(Zmax);
  //      end
  //      %% Ancillary controller
  //      %{
  //      Given the state estimate and goal, give the desired forces and
  //      torques
  //      %}
  //      function F_T = lqr_calc_input(self, t_idx)
  //          n = self.n;
  //
  //          x_ref = self.x_ref_hist(t_idx,1:n)';
  //          x_err = self.x - x_ref;  % predicted reference state
  //          F_T = self.K_lqr*x_err;
  //      end
  //      %{
  //      Calculate nominal MPC with tightened constraints, then add ancillary
  //      controller correction.
  //      %}
  //      function u_t_idx = mpc_robust_tube(self, t_idx)
  //          u_t_idx = mpc_direct_tran(self, t_idx);
  //          u_t_idx = u_t_idx + lqr_calc_input(self, t_idx);
  //      end
  //     %% Direct transcription
  //     %{
  //     Compute MPC inputs by direct transcription.
  //     Assumes the state estimate has been set!
  //     %}
  //  time index, dt, horizon length, ref traj length, ref traj
  //  state size
  //  input size
  //  mass, kg
  st.site = &emlrtRSI;
  get_dynamics_for_dt(dt, A, B);
  state_horz_dim = 6.0 * N;

  //  size of state decision variable vector
  input_horz_dim = 3.0 * N;

  //  size of input decision variable vector
  dec_dim = state_horz_dim + input_horz_dim;

  //  size of total decision variable vector
  //  decision vector is of form [x_horz ; u_horz]
  // ---Cost function---
  //  min [x-x_ref]'*Q*[x-x_ref] + u'*R*u
  //  min 0.5*x'*H*x + f'x
  if (!(state_horz_dim >= 0.0)) {
    emlrtNonNegativeCheckR2012b(state_horz_dim, &emlrtDCI, sp);
  }

  i = static_cast<int32_T>(muDoubleScalarFloor(state_horz_dim));
  if (state_horz_dim != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &b_emlrtDCI, sp);
  }

  //  penalize states
  if (!(input_horz_dim >= 0.0)) {
    emlrtNonNegativeCheckR2012b(input_horz_dim, &c_emlrtDCI, sp);
  }

  i1 = static_cast<int32_T>(muDoubleScalarFloor(input_horz_dim));
  if (input_horz_dim != i1) {
    emlrtIntegerCheckR2012b(input_horz_dim, &d_emlrtDCI, sp);
  }

  //  penalize inputs
  input_horz_dim_tmp = static_cast<int32_T>(input_horz_dim);
  b_input_horz_dim[0] = static_cast<int32_T>(input_horz_dim);
  b_input_horz_dim[1] = 1;
  loop_ub = static_cast<int32_T>(state_horz_dim);
  Aeq_state.set_size((&o_emlrtRTEI), sp, 1, (static_cast<int32_T>(state_horz_dim)
    + static_cast<int32_T>(input_horz_dim)));
  for (i2 = 0; i2 < loop_ub; i2++) {
    Aeq_state[i2] = 10000.0;
  }

  for (i2 = 0; i2 < input_horz_dim_tmp; i2++) {
    Aeq_state[i2 + static_cast<int32_T>(state_horz_dim)] = 1.0;
  }

  st.site = &b_emlrtRSI;
  diag(&st, Aeq_state, H);
  if (!(dec_dim >= 0.0)) {
    emlrtNonNegativeCheckR2012b(dec_dim, &q_emlrtDCI, sp);
  }

  d = static_cast<int32_T>(muDoubleScalarFloor(dec_dim));
  if (dec_dim != d) {
    emlrtIntegerCheckR2012b(dec_dim, &p_emlrtDCI, sp);
  }

  b_loop_ub = static_cast<int32_T>(dec_dim);
  f.set_size((&p_emlrtRTEI), sp, b_loop_ub);
  if (dec_dim != d) {
    emlrtIntegerCheckR2012b(dec_dim, &p_emlrtDCI, sp);
  }

  for (i2 = 0; i2 < b_loop_ub; i2++) {
    f[i2] = 0.0;
  }

  i2 = static_cast<int32_T>((N - 1.0) + 1.0);
  emlrtForLoopVectorCheckR2012b(0.0, 1.0, N - 1.0, mxDOUBLE_CLASS,
    static_cast<int32_T>((N - 1.0) + 1.0), &emlrtRTEI, sp);
  for (b_i = 0; b_i < i2; b_i++) {
    covrtLogFor(&emlrtCoverageInstance, 0, 0, 0, 1);

    //  for the full horizon...
    d1 = static_cast<real_T>(b_i) + t_idx;
    if (covrtLogIf(&emlrtCoverageInstance, 0, 0, 0, d1 > Nf)) {
      covrtLogBasicBlock(&emlrtCoverageInstance, 0, 1);

      //  (use final state if horizon exceeds ref traj)
      //                  fprintf("%d %d\n", i, t0_idx);
      if (Nf != static_cast<int32_T>(muDoubleScalarFloor(Nf))) {
        emlrtIntegerCheckR2012b(Nf, &k_emlrtDCI, sp);
      }

      i3 = static_cast<int32_T>(Nf);
      if ((i3 < 1) || (i3 > 10000)) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, 10000, &f_emlrtBCI, sp);
      }

      for (i4 = 0; i4 < 6; i4++) {
        x_des[i4] = x_ref_hist[i4 + 6 * (i3 - 1)];
      }

      //  get state reference...
    } else {
      covrtLogBasicBlock(&emlrtCoverageInstance, 0, 2);
      if (d1 != static_cast<int32_T>(muDoubleScalarFloor(d1))) {
        emlrtIntegerCheckR2012b(d1, &j_emlrtDCI, sp);
      }

      input_sizes_idx_1 = static_cast<int32_T>(d1);
      if ((input_sizes_idx_1 < 1) || (input_sizes_idx_1 > 10000)) {
        emlrtDynamicBoundsCheckR2012b(input_sizes_idx_1, 1, 10000, &e_emlrtBCI,
          sp);
      }

      for (i3 = 0; i3 < 6; i3++) {
        x_des[i3] = x_ref_hist[i3 + 6 * (input_sizes_idx_1 - 1)];
      }

      //  get state reference...
    }

    covrtLogBasicBlock(&emlrtCoverageInstance, 0, 3);
    for (i3 = 0; i3 < 6; i3++) {
      x_des[i3] = -x_des[i3] * 10000.0;
    }

    d1 = 6.0 * static_cast<real_T>(b_i);
    i3 = f.size(0);
    for (i4 = 0; i4 < 6; i4++) {
      i5 = static_cast<int32_T>(d1 + (static_cast<real_T>(i4) + 1.0));
      if ((i5 < 1) || (i5 > i3)) {
        emlrtDynamicBoundsCheckR2012b(i5, 1, i3, &g_emlrtBCI, sp);
      }

      f[i5 - 1] = x_des[i4];
    }

    //  ...and set a penalty for missing this reference (state only)
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  covrtLogFor(&emlrtCoverageInstance, 0, 0, 0, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0, 4);

  // ---Equality constraints---
  //  A*x = b
  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &l_emlrtDCI, sp);
  }

  Aeq_state.set_size((&q_emlrtRTEI), sp, Aeq_state.size(0), loop_ub);
  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &m_emlrtDCI, sp);
  }

  Aeq_state.set_size((&q_emlrtRTEI), sp, loop_ub, Aeq_state.size(1));
  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &r_emlrtDCI, sp);
  }

  input_sizes_idx_1 = static_cast<int32_T>(state_horz_dim) * static_cast<int32_T>
    (state_horz_dim);
  for (i2 = 0; i2 < input_sizes_idx_1; i2++) {
    Aeq_state[i2] = 0.0;
  }

  if (static_cast<int32_T>(input_horz_dim) != i1) {
    emlrtIntegerCheckR2012b(input_horz_dim, &n_emlrtDCI, sp);
  }

  Aeq_input.set_size((&r_emlrtRTEI), sp, Aeq_input.size(0), input_horz_dim_tmp);
  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &o_emlrtDCI, sp);
  }

  Aeq_input.set_size((&r_emlrtRTEI), sp, loop_ub, Aeq_input.size(1));
  if (static_cast<int32_T>(input_horz_dim) != i1) {
    emlrtIntegerCheckR2012b(input_horz_dim, &s_emlrtDCI, sp);
  }

  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &s_emlrtDCI, sp);
  }

  input_sizes_idx_1 = static_cast<int32_T>(input_horz_dim) * static_cast<int32_T>
    (state_horz_dim);
  for (i2 = 0; i2 < input_sizes_idx_1; i2++) {
    Aeq_input[i2] = 0.0;
  }

  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &t_emlrtDCI, sp);
  }

  beq.set_size((&s_emlrtRTEI), sp, loop_ub);
  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &t_emlrtDCI, sp);
  }

  for (i2 = 0; i2 < loop_ub; i2++) {
    beq[i2] = 0.0;
  }

  //  Dynamics constraints
  //  First constraint is set
  //  -x_1 + B*u_0 = -A*x_0
  eye(Aeq_state_tmp);
  for (i2 = 0; i2 < 36; i2++) {
    Aeq_state_tmp[i2] = -Aeq_state_tmp[i2];
  }

  for (i2 = 0; i2 < 6; i2++) {
    i3 = i2 + 1;
    for (i4 = 0; i4 < 6; i4++) {
      i5 = i4 + 1;
      if (i5 > static_cast<int32_T>(state_horz_dim)) {
        emlrtDynamicBoundsCheckR2012b(i5, 1, static_cast<int32_T>(state_horz_dim),
          &h_emlrtBCI, sp);
      }

      if (i3 > static_cast<int32_T>(state_horz_dim)) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, static_cast<int32_T>(state_horz_dim),
          &h_emlrtBCI, sp);
      }

      Aeq_state[(i5 + Aeq_state.size(1) * (i3 - 1)) - 1] = Aeq_state_tmp[i4 + 6 *
        i2];
    }
  }

  for (i2 = 0; i2 < 6; i2++) {
    if (1 > static_cast<int32_T>(input_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(1, 1, 0, &l_emlrtBCI, sp);
    }

    i3 = i2 + 1;
    if (i3 > static_cast<int32_T>(state_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, static_cast<int32_T>(state_horz_dim),
        &l_emlrtBCI, sp);
    }

    Aeq_input[Aeq_input.size(1) * (i3 - 1)] = B[3 * i2];
    if (2 > static_cast<int32_T>(input_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(2, 1, 1, &l_emlrtBCI, sp);
    }

    i3 = i2 + 1;
    if (i3 > static_cast<int32_T>(state_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, static_cast<int32_T>(state_horz_dim),
        &l_emlrtBCI, sp);
    }

    Aeq_input[Aeq_input.size(1) * (i3 - 1) + 1] = B[3 * i2 + 1];
    if (3 > static_cast<int32_T>(input_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(3, 1, 2, &l_emlrtBCI, sp);
    }

    i3 = i2 + 1;
    if (i3 > static_cast<int32_T>(state_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, static_cast<int32_T>(state_horz_dim),
        &l_emlrtBCI, sp);
    }

    Aeq_input[Aeq_input.size(1) * (i3 - 1) + 2] = B[3 * i2 + 2];
  }

  for (i2 = 0; i2 < 36; i2++) {
    b_A[i2] = -A[i2];
  }

  for (i2 = 0; i2 < 6; i2++) {
    d1 = 0.0;
    for (i3 = 0; i3 < 6; i3++) {
      d1 += x_cur[i3] * b_A[i3 + 6 * i2];
    }

    i3 = i2 + 1;
    if (i3 > static_cast<int32_T>(state_horz_dim)) {
      emlrtDynamicBoundsCheckR2012b(i3, 1, static_cast<int32_T>(state_horz_dim),
        &k_emlrtBCI, sp);
    }

    beq[i3 - 1] = d1;
  }

  //  These constraints depend on dec vars
  i2 = static_cast<int32_T>(N - 1.0);
  emlrtForLoopVectorCheckR2012b(1.0, 1.0, N - 1.0, mxDOUBLE_CLASS,
    static_cast<int32_T>(N - 1.0), &b_emlrtRTEI, sp);
  if (0 <= i2 - 1) {
    b_input_horz_dim[0] = 6;
    b_input_horz_dim[1] = 6;
  }

  for (b_i = 0; b_i < i2; b_i++) {
    covrtLogFor(&emlrtCoverageInstance, 0, 0, 1, 1);
    covrtLogBasicBlock(&emlrtCoverageInstance, 0, 5);

    //  x_n+ = A*x_n
    d1 = 6.0 * (static_cast<real_T>(b_i) + 1.0);
    for (i3 = 0; i3 < 6; i3++) {
      x_des[i3] = d1 + (static_cast<real_T>(i3) + 1.0);
    }

    d1 = 6.0 * ((static_cast<real_T>(b_i) + 1.0) - 1.0);
    i3 = Aeq_state.size(1);
    i4 = Aeq_state.size(0);
    for (i5 = 0; i5 < 6; i5++) {
      i6 = static_cast<int32_T>(x_des[i5]);
      for (input_horz_dim_tmp = 0; input_horz_dim_tmp < 6; input_horz_dim_tmp++)
      {
        loop_ub = static_cast<int32_T>(d1 + (static_cast<real_T>
          (input_horz_dim_tmp) + 1.0));
        if ((loop_ub < 1) || (loop_ub > i3)) {
          emlrtDynamicBoundsCheckR2012b(loop_ub, 1, i3, &i_emlrtBCI, sp);
        }

        if ((i6 < 1) || (i6 > i4)) {
          emlrtDynamicBoundsCheckR2012b(i6, 1, i4, &i_emlrtBCI, sp);
        }

        Aeq_state[(loop_ub + Aeq_state.size(1) * (i6 - 1)) - 1] =
          A[input_horz_dim_tmp + 6 * i5];
      }
    }

    if ((d1 + 1.0) + 6.0 > (d1 + 6.0) + 6.0) {
      i3 = 0;
      i4 = 0;
    } else {
      i3 = static_cast<int32_T>((d1 + 1.0) + 6.0);
      if ((i3 < 1) || (i3 > Aeq_state.size(1))) {
        emlrtDynamicBoundsCheckR2012b(i3, 1, Aeq_state.size(1), &emlrtBCI, sp);
      }

      i3--;
      i4 = static_cast<int32_T>((d1 + 6.0) + 6.0);
      if ((i4 < 1) || (i4 > Aeq_state.size(1))) {
        emlrtDynamicBoundsCheckR2012b(i4, 1, Aeq_state.size(1), &b_emlrtBCI, sp);
      }
    }

    for (i5 = 0; i5 < 6; i5++) {
      i6 = static_cast<int32_T>(x_des[i5]);
      if ((i6 < 1) || (i6 > Aeq_state.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i6, 1, Aeq_state.size(0), &j_emlrtBCI, sp);
      }
    }

    b_state_horz_dim[0] = 6;
    input_sizes_idx_1 = i4 - i3;
    b_state_horz_dim[1] = input_sizes_idx_1;
    emlrtSubAssignSizeCheckR2012b(&b_state_horz_dim[0], 2, &b_input_horz_dim[0],
      2, &emlrtECI, sp);
    for (i4 = 0; i4 < 6; i4++) {
      for (i5 = 0; i5 < input_sizes_idx_1; i5++) {
        Aeq_state[(i3 + i5) + Aeq_state.size(1) * (static_cast<int32_T>(x_des[i4])
          - 1)] = Aeq_state_tmp[i5 + input_sizes_idx_1 * i4];
      }
    }

    //  +B*u_n
    d1 = 3.0 * (static_cast<real_T>(b_i) + 1.0);
    i3 = Aeq_input.size(1);
    i4 = Aeq_input.size(0);
    i5 = static_cast<int32_T>(d1 + 1.0);
    i6 = static_cast<int32_T>(d1 + 2.0);
    input_horz_dim_tmp = static_cast<int32_T>(d1 + 3.0);
    for (loop_ub = 0; loop_ub < 6; loop_ub++) {
      if ((i5 < 1) || (i5 > i3)) {
        emlrtDynamicBoundsCheckR2012b(i5, 1, i3, &m_emlrtBCI, sp);
      }

      input_sizes_idx_1 = static_cast<int32_T>(x_des[loop_ub]);
      if ((input_sizes_idx_1 < 1) || (input_sizes_idx_1 > i4)) {
        emlrtDynamicBoundsCheckR2012b(input_sizes_idx_1, 1, i4, &m_emlrtBCI, sp);
      }

      Aeq_input[(i5 + Aeq_input.size(1) * (input_sizes_idx_1 - 1)) - 1] = B[3 *
        loop_ub];
      if ((i6 < 1) || (i6 > i3)) {
        emlrtDynamicBoundsCheckR2012b(i6, 1, i3, &m_emlrtBCI, sp);
      }

      if (input_sizes_idx_1 > i4) {
        emlrtDynamicBoundsCheckR2012b(input_sizes_idx_1, 1, i4, &m_emlrtBCI, sp);
      }

      Aeq_input[(i6 + Aeq_input.size(1) * (input_sizes_idx_1 - 1)) - 1] = B[3 *
        loop_ub + 1];
      if ((input_horz_dim_tmp < 1) || (input_horz_dim_tmp > i3)) {
        emlrtDynamicBoundsCheckR2012b(input_horz_dim_tmp, 1, i3, &m_emlrtBCI, sp);
      }

      if (input_sizes_idx_1 > i4) {
        emlrtDynamicBoundsCheckR2012b(input_sizes_idx_1, 1, i4, &m_emlrtBCI, sp);
      }

      Aeq_input[(input_horz_dim_tmp + Aeq_input.size(1) * (input_sizes_idx_1 - 1))
        - 1] = B[3 * loop_ub + 2];
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(sp);
    }
  }

  covrtLogFor(&emlrtCoverageInstance, 0, 0, 1, 0);
  covrtLogBasicBlock(&emlrtCoverageInstance, 0, 6);

  //  Assemble equality matrix
  st.site = &c_emlrtRSI;
  b_st.site = &v_emlrtRSI;
  if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_state.size(0);
  } else if ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_input.size(0);
  } else {
    if (Aeq_state.size(0) > 0) {
      input_horz_dim_tmp = Aeq_state.size(0);
    } else {
      input_horz_dim_tmp = 0;
    }

    if (Aeq_input.size(0) > input_horz_dim_tmp) {
      input_horz_dim_tmp = Aeq_input.size(0);
    }
  }

  c_st.site = &w_emlrtRSI;
  if ((Aeq_state.size(0) != input_horz_dim_tmp) && ((Aeq_state.size(0) != 0) &&
       (Aeq_state.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if ((Aeq_input.size(0) != input_horz_dim_tmp) && ((Aeq_input.size(0) != 0) &&
       (Aeq_input.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (input_horz_dim_tmp == 0) {
    empty_non_axis_sizes = true;
    input_sizes_idx_1 = Aeq_state.size(1);
  } else {
    empty_non_axis_sizes = false;
    if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
      input_sizes_idx_1 = Aeq_state.size(1);
    } else {
      input_sizes_idx_1 = 0;
    }
  }

  if ((input_sizes_idx_1 == Aeq_state.size(1)) && (input_horz_dim_tmp ==
       Aeq_state.size(0))) {
    reshapes[0].f1.set_size((&u_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    loop_ub = input_sizes_idx_1 * input_horz_dim_tmp;
    for (i2 = 0; i2 < loop_ub; i2++) {
      reshapes[0].f1[i2] = Aeq_state[i2];
    }
  } else {
    i2 = 0;
    i3 = 0;
    i4 = 0;
    i5 = 0;
    reshapes[0].f1.set_size((&t_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    for (i6 = 0; i6 < input_horz_dim_tmp * input_sizes_idx_1; i6++) {
      reshapes[0].f1[i3 + reshapes[0].f1.size(1) * i2] = Aeq_state[i5 +
        Aeq_state.size(1) * i4];
      i2++;
      i4++;
      if (i2 > reshapes[0].f1.size(0) - 1) {
        i2 = 0;
        i3++;
      }

      if (i4 > Aeq_state.size(0) - 1) {
        i4 = 0;
        i5++;
      }
    }
  }

  if (empty_non_axis_sizes || ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) !=
        0))) {
    input_sizes_idx_1 = Aeq_input.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((input_sizes_idx_1 == Aeq_input.size(1)) && (input_horz_dim_tmp ==
       Aeq_input.size(0))) {
    reshapes[1].f1.set_size((&u_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    loop_ub = input_sizes_idx_1 * input_horz_dim_tmp;
    for (i2 = 0; i2 < loop_ub; i2++) {
      reshapes[1].f1[i2] = Aeq_input[i2];
    }
  } else {
    i2 = 0;
    i3 = 0;
    i4 = 0;
    i5 = 0;
    reshapes[1].f1.set_size((&t_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    for (i6 = 0; i6 < input_horz_dim_tmp * input_sizes_idx_1; i6++) {
      reshapes[1].f1[i3 + reshapes[1].f1.size(1) * i2] = Aeq_input[i5 +
        Aeq_input.size(1) * i4];
      i2++;
      i4++;
      if (i2 > reshapes[1].f1.size(0) - 1) {
        i2 = 0;
        i3++;
      }

      if (i4 > Aeq_input.size(0) - 1) {
        i4 = 0;
        i5++;
      }
    }
  }

  // ---Inequality constraints---
  //  A*x <= b
  //  Actuator saturation inequality constraint
  st.site = &d_emlrtRSI;
  b_st.site = &d_emlrtRSI;
  b_eye(&b_st, input_horz_dim, Aeq_state);
  loop_ub = Aeq_state.size(1) * Aeq_state.size(0);
  for (i2 = 0; i2 < loop_ub; i2++) {
    Aeq_state[i2] = -Aeq_state[i2];
  }

  b_st.site = &e_emlrtRSI;
  b_eye(&b_st, input_horz_dim, Aeq_input);
  b_st.site = &v_emlrtRSI;
  if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_state.size(1);
  } else if ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_input.size(1);
  } else {
    if (Aeq_state.size(1) > 0) {
      input_horz_dim_tmp = Aeq_state.size(1);
    } else {
      input_horz_dim_tmp = 0;
    }

    if (Aeq_input.size(1) > input_horz_dim_tmp) {
      input_horz_dim_tmp = Aeq_input.size(1);
    }
  }

  c_st.site = &w_emlrtRSI;
  if ((Aeq_state.size(1) != input_horz_dim_tmp) && ((Aeq_state.size(0) != 0) &&
       (Aeq_state.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if ((Aeq_input.size(1) != input_horz_dim_tmp) && ((Aeq_input.size(0) != 0) &&
       (Aeq_input.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (input_horz_dim_tmp == 0) {
    empty_non_axis_sizes = true;
    input_sizes_idx_1 = Aeq_state.size(0);
  } else {
    empty_non_axis_sizes = false;
    if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
      input_sizes_idx_1 = Aeq_state.size(0);
    } else {
      input_sizes_idx_1 = 0;
    }
  }

  if ((input_horz_dim_tmp == Aeq_state.size(1)) && (input_sizes_idx_1 ==
       Aeq_state.size(0))) {
    b_reshapes[0].f1.set_size((&u_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    loop_ub = input_horz_dim_tmp * input_sizes_idx_1;
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_reshapes[0].f1[i2] = Aeq_state[i2];
    }
  } else {
    i2 = 0;
    i3 = 0;
    i4 = 0;
    i5 = 0;
    b_reshapes[0].f1.set_size((&t_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    for (i6 = 0; i6 < input_sizes_idx_1 * input_horz_dim_tmp; i6++) {
      b_reshapes[0].f1[i3 + b_reshapes[0].f1.size(1) * i2] = Aeq_state[i5 +
        Aeq_state.size(1) * i4];
      i2++;
      i4++;
      if (i2 > b_reshapes[0].f1.size(0) - 1) {
        i2 = 0;
        i3++;
      }

      if (i4 > Aeq_state.size(0) - 1) {
        i4 = 0;
        i5++;
      }
    }
  }

  if (empty_non_axis_sizes || ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) !=
        0))) {
    input_sizes_idx_1 = Aeq_input.size(0);
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((input_horz_dim_tmp == Aeq_input.size(1)) && (input_sizes_idx_1 ==
       Aeq_input.size(0))) {
    b_reshapes[1].f1.set_size((&u_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    loop_ub = input_horz_dim_tmp * input_sizes_idx_1;
    for (i2 = 0; i2 < loop_ub; i2++) {
      b_reshapes[1].f1[i2] = Aeq_input[i2];
    }
  } else {
    i2 = 0;
    i3 = 0;
    i4 = 0;
    i5 = 0;
    b_reshapes[1].f1.set_size((&t_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    for (i6 = 0; i6 < input_sizes_idx_1 * input_horz_dim_tmp; i6++) {
      b_reshapes[1].f1[i3 + b_reshapes[1].f1.size(1) * i2] = Aeq_input[i5 +
        Aeq_input.size(1) * i4];
      i2++;
      i4++;
      if (i2 > b_reshapes[1].f1.size(0) - 1) {
        i2 = 0;
        i3++;
      }

      if (i4 > Aeq_input.size(0) - 1) {
        i4 = 0;
        i5++;
      }
    }
  }

  Aineq_input.set_size((&v_emlrtRTEI), (&b_st), (b_reshapes[0].f1.size(0) +
    b_reshapes[1].f1.size(0)), b_reshapes[0].f1.size(1));
  loop_ub = b_reshapes[0].f1.size(0);
  for (i2 = 0; i2 < loop_ub; i2++) {
    input_sizes_idx_1 = b_reshapes[0].f1.size(1);
    for (i3 = 0; i3 < input_sizes_idx_1; i3++) {
      Aineq_input[i3 + Aineq_input.size(1) * i2] = b_reshapes[0].f1[i3 +
        b_reshapes[0].f1.size(1) * i2];
    }
  }

  loop_ub = b_reshapes[1].f1.size(0);
  for (i2 = 0; i2 < loop_ub; i2++) {
    input_sizes_idx_1 = b_reshapes[1].f1.size(1);
    for (i3 = 0; i3 < input_sizes_idx_1; i3++) {
      Aineq_input[i3 + Aineq_input.size(1) * (i2 + b_reshapes[0].f1.size(0))] =
        b_reshapes[1].f1[i3 + b_reshapes[1].f1.size(1) * i2];
    }
  }

  st.site = &f_emlrtRSI;
  repmat(&st, N * 2.0, bineq_input);

  //  Half-plane inequality constraint
  st.site = &g_emlrtRSI;
  b_st.site = &g_emlrtRSI;
  b_eye(&b_st, state_horz_dim, Aeq_state);
  b_st.site = &h_emlrtRSI;
  b_eye(&b_st, state_horz_dim, Aeq_input);
  loop_ub = Aeq_input.size(1) * Aeq_input.size(0);
  for (i2 = 0; i2 < loop_ub; i2++) {
    Aeq_input[i2] = -Aeq_input[i2];
  }

  b_st.site = &v_emlrtRSI;
  if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_state.size(1);
  } else if ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_input.size(1);
  } else {
    if (Aeq_state.size(1) > 0) {
      input_horz_dim_tmp = Aeq_state.size(1);
    } else {
      input_horz_dim_tmp = 0;
    }

    if (Aeq_input.size(1) > input_horz_dim_tmp) {
      input_horz_dim_tmp = Aeq_input.size(1);
    }
  }

  c_st.site = &w_emlrtRSI;
  if ((Aeq_state.size(1) != input_horz_dim_tmp) && ((Aeq_state.size(0) != 0) &&
       (Aeq_state.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if ((Aeq_input.size(1) != input_horz_dim_tmp) && ((Aeq_input.size(0) != 0) &&
       (Aeq_input.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (input_horz_dim_tmp == 0) {
    empty_non_axis_sizes = true;
    input_sizes_idx_1 = Aeq_state.size(0);
  } else {
    empty_non_axis_sizes = false;
    if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
      input_sizes_idx_1 = Aeq_state.size(0);
    } else {
      input_sizes_idx_1 = 0;
    }
  }

  if ((input_horz_dim_tmp == Aeq_state.size(1)) && (input_sizes_idx_1 ==
       Aeq_state.size(0))) {
    c_reshapes[0].f1.set_size((&u_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    loop_ub = input_horz_dim_tmp * input_sizes_idx_1;
    for (i2 = 0; i2 < loop_ub; i2++) {
      c_reshapes[0].f1[i2] = Aeq_state[i2];
    }
  } else {
    i2 = 0;
    i3 = 0;
    i4 = 0;
    i5 = 0;
    c_reshapes[0].f1.set_size((&t_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    for (i6 = 0; i6 < input_sizes_idx_1 * input_horz_dim_tmp; i6++) {
      c_reshapes[0].f1[i3 + c_reshapes[0].f1.size(1) * i2] = Aeq_state[i5 +
        Aeq_state.size(1) * i4];
      i2++;
      i4++;
      if (i2 > c_reshapes[0].f1.size(0) - 1) {
        i2 = 0;
        i3++;
      }

      if (i4 > Aeq_state.size(0) - 1) {
        i4 = 0;
        i5++;
      }
    }
  }

  if (empty_non_axis_sizes || ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) !=
        0))) {
    input_sizes_idx_1 = Aeq_input.size(0);
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((input_horz_dim_tmp == Aeq_input.size(1)) && (input_sizes_idx_1 ==
       Aeq_input.size(0))) {
    c_reshapes[1].f1.set_size((&u_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    loop_ub = input_horz_dim_tmp * input_sizes_idx_1;
    for (i2 = 0; i2 < loop_ub; i2++) {
      c_reshapes[1].f1[i2] = Aeq_input[i2];
    }
  } else {
    i2 = 0;
    i3 = 0;
    i4 = 0;
    i5 = 0;
    c_reshapes[1].f1.set_size((&t_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    for (i6 = 0; i6 < input_sizes_idx_1 * input_horz_dim_tmp; i6++) {
      c_reshapes[1].f1[i3 + c_reshapes[1].f1.size(1) * i2] = Aeq_input[i5 +
        Aeq_input.size(1) * i4];
      i2++;
      i4++;
      if (i2 > c_reshapes[1].f1.size(0) - 1) {
        i2 = 0;
        i3++;
      }

      if (i4 > Aeq_input.size(0) - 1) {
        i4 = 0;
        i5++;
      }
    }
  }

  Aeq_state.set_size((&w_emlrtRTEI), (&b_st), (c_reshapes[0].f1.size(0) +
    c_reshapes[1].f1.size(0)), c_reshapes[0].f1.size(1));
  loop_ub = c_reshapes[0].f1.size(0);
  for (i2 = 0; i2 < loop_ub; i2++) {
    input_sizes_idx_1 = c_reshapes[0].f1.size(1);
    for (i3 = 0; i3 < input_sizes_idx_1; i3++) {
      Aeq_state[i3 + Aeq_state.size(1) * i2] = c_reshapes[0].f1[i3 + c_reshapes
        [0].f1.size(1) * i2];
    }
  }

  loop_ub = c_reshapes[1].f1.size(0);
  for (i2 = 0; i2 < loop_ub; i2++) {
    input_sizes_idx_1 = c_reshapes[1].f1.size(1);
    for (i3 = 0; i3 < input_sizes_idx_1; i3++) {
      Aeq_state[i3 + Aeq_state.size(1) * (i2 + c_reshapes[0].f1.size(0))] =
        c_reshapes[1].f1[i3 + c_reshapes[1].f1.size(1) * i2];
    }
  }

  st.site = &i_emlrtRSI;
  b_repmat(&st, N * 2.0, bineq_state);

  //  Assemble inequality matrix
  st.site = &j_emlrtRSI;
  if (static_cast<int32_T>(state_horz_dim) != i) {
    emlrtIntegerCheckR2012b(state_horz_dim, &f_emlrtDCI, &st);
  }

  u = static_cast<uint32_T>(input_horz_dim) * 2U;
  if (static_cast<real_T>(u) != static_cast<int32_T>(u)) {
    emlrtIntegerCheckR2012b(static_cast<real_T>(u), &e_emlrtDCI, &st);
  }

  b_st.site = &v_emlrtRSI;
  i = static_cast<int32_T>(input_horz_dim) * 2;
  if ((i != 0) && (static_cast<int32_T>(state_horz_dim) != 0)) {
    input_horz_dim_tmp = i;
  } else if ((Aineq_input.size(0) != 0) && (Aineq_input.size(1) != 0)) {
    input_horz_dim_tmp = Aineq_input.size(0);
  } else {
    if (i > 0) {
      input_horz_dim_tmp = i;
    } else {
      input_horz_dim_tmp = 0;
    }

    if (Aineq_input.size(0) > input_horz_dim_tmp) {
      input_horz_dim_tmp = Aineq_input.size(0);
    }
  }

  c_st.site = &w_emlrtRSI;
  if ((i != input_horz_dim_tmp) && ((i != 0) && (static_cast<int32_T>
        (state_horz_dim) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if ((Aineq_input.size(0) != input_horz_dim_tmp) && ((Aineq_input.size(0) != 0)
       && (Aineq_input.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (input_horz_dim_tmp == 0) {
    empty_non_axis_sizes = true;
    input_sizes_idx_1 = static_cast<int32_T>(state_horz_dim);
  } else {
    empty_non_axis_sizes = false;
    if ((i != 0) && (static_cast<int32_T>(state_horz_dim) != 0)) {
      input_sizes_idx_1 = static_cast<int32_T>(state_horz_dim);
    } else {
      input_sizes_idx_1 = 0;
    }
  }

  if ((input_sizes_idx_1 == static_cast<int32_T>(state_horz_dim)) &&
      (input_horz_dim_tmp == i)) {
    d_reshapes[0].f1.set_size((&u_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    loop_ub = input_sizes_idx_1 * input_horz_dim_tmp;
    for (i = 0; i < loop_ub; i++) {
      d_reshapes[0].f1[i] = 0.0;
    }
  } else {
    i = 0;
    i2 = 0;
    d_reshapes[0].f1.set_size((&t_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    for (i3 = 0; i3 < input_horz_dim_tmp * input_sizes_idx_1; i3++) {
      d_reshapes[0].f1[i2 + d_reshapes[0].f1.size(1) * i] = 0.0;
      i++;
      if (i > d_reshapes[0].f1.size(0) - 1) {
        i = 0;
        i2++;
      }
    }
  }

  if (empty_non_axis_sizes || ((Aineq_input.size(0) != 0) && (Aineq_input.size(1)
        != 0))) {
    input_sizes_idx_1 = Aineq_input.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((input_sizes_idx_1 == Aineq_input.size(1)) && (input_horz_dim_tmp ==
       Aineq_input.size(0))) {
    d_reshapes[1].f1.set_size((&u_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    loop_ub = input_sizes_idx_1 * input_horz_dim_tmp;
    for (i = 0; i < loop_ub; i++) {
      d_reshapes[1].f1[i] = Aineq_input[i];
    }
  } else {
    i = 0;
    i2 = 0;
    i3 = 0;
    i4 = 0;
    d_reshapes[1].f1.set_size((&t_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    for (i5 = 0; i5 < input_horz_dim_tmp * input_sizes_idx_1; i5++) {
      d_reshapes[1].f1[i2 + d_reshapes[1].f1.size(1) * i] = Aineq_input[i4 +
        Aineq_input.size(1) * i3];
      i++;
      i3++;
      if (i > d_reshapes[1].f1.size(0) - 1) {
        i = 0;
        i2++;
      }

      if (i3 > Aineq_input.size(0) - 1) {
        i3 = 0;
        i4++;
      }
    }
  }

  Aeq_input.set_size((&x_emlrtRTEI), (&b_st), d_reshapes[0].f1.size(0),
                     (d_reshapes[0].f1.size(1) + d_reshapes[1].f1.size(1)));
  loop_ub = d_reshapes[0].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = d_reshapes[0].f1.size(1);
    for (i2 = 0; i2 < input_sizes_idx_1; i2++) {
      Aeq_input[i2 + Aeq_input.size(1) * i] = d_reshapes[0].f1[i2 + d_reshapes[0]
        .f1.size(1) * i];
    }
  }

  loop_ub = d_reshapes[1].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = d_reshapes[1].f1.size(1);
    for (i2 = 0; i2 < input_sizes_idx_1; i2++) {
      Aeq_input[(i2 + d_reshapes[0].f1.size(1)) + Aeq_input.size(1) * i] =
        d_reshapes[1].f1[i2 + d_reshapes[1].f1.size(1) * i];
    }
  }

  st.site = &k_emlrtRSI;
  if (static_cast<int32_T>(input_horz_dim) != i1) {
    emlrtIntegerCheckR2012b(input_horz_dim, &h_emlrtDCI, &st);
  }

  u = static_cast<uint32_T>(state_horz_dim) * 2U;
  if (static_cast<real_T>(u) != static_cast<int32_T>(u)) {
    emlrtIntegerCheckR2012b(static_cast<real_T>(u), &g_emlrtDCI, &st);
  }

  b_st.site = &v_emlrtRSI;
  if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_state.size(0);
  } else {
    i = static_cast<int32_T>(state_horz_dim) * 2;
    if ((i != 0) && (static_cast<int32_T>(input_horz_dim) != 0)) {
      input_horz_dim_tmp = i;
    } else {
      if (Aeq_state.size(0) > 0) {
        input_horz_dim_tmp = Aeq_state.size(0);
      } else {
        input_horz_dim_tmp = 0;
      }

      if (i > input_horz_dim_tmp) {
        input_horz_dim_tmp = i;
      }
    }
  }

  c_st.site = &w_emlrtRSI;
  if ((Aeq_state.size(0) != input_horz_dim_tmp) && ((Aeq_state.size(0) != 0) &&
       (Aeq_state.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  i = static_cast<int32_T>(state_horz_dim) * 2;
  if ((i != input_horz_dim_tmp) && ((i != 0) && (static_cast<int32_T>
        (input_horz_dim) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (input_horz_dim_tmp == 0) {
    empty_non_axis_sizes = true;
    input_sizes_idx_1 = Aeq_state.size(1);
  } else {
    empty_non_axis_sizes = false;
    if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
      input_sizes_idx_1 = Aeq_state.size(1);
    } else {
      input_sizes_idx_1 = 0;
    }
  }

  if ((input_sizes_idx_1 == Aeq_state.size(1)) && (input_horz_dim_tmp ==
       Aeq_state.size(0))) {
    e_reshapes[0].f1.set_size((&u_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    loop_ub = input_sizes_idx_1 * input_horz_dim_tmp;
    for (i1 = 0; i1 < loop_ub; i1++) {
      e_reshapes[0].f1[i1] = Aeq_state[i1];
    }
  } else {
    i1 = 0;
    i2 = 0;
    i3 = 0;
    i4 = 0;
    e_reshapes[0].f1.set_size((&t_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    for (i5 = 0; i5 < input_horz_dim_tmp * input_sizes_idx_1; i5++) {
      e_reshapes[0].f1[i2 + e_reshapes[0].f1.size(1) * i1] = Aeq_state[i4 +
        Aeq_state.size(1) * i3];
      i1++;
      i3++;
      if (i1 > e_reshapes[0].f1.size(0) - 1) {
        i1 = 0;
        i2++;
      }

      if (i3 > Aeq_state.size(0) - 1) {
        i3 = 0;
        i4++;
      }
    }
  }

  if (empty_non_axis_sizes || ((i != 0) && (static_cast<int32_T>(input_horz_dim)
        != 0))) {
    input_sizes_idx_1 = static_cast<int32_T>(input_horz_dim);
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((input_sizes_idx_1 == static_cast<int32_T>(input_horz_dim)) &&
      (input_horz_dim_tmp == i)) {
    e_reshapes[1].f1.set_size((&u_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    loop_ub = input_sizes_idx_1 * input_horz_dim_tmp;
    for (i = 0; i < loop_ub; i++) {
      e_reshapes[1].f1[i] = 0.0;
    }
  } else {
    i = 0;
    i1 = 0;
    e_reshapes[1].f1.set_size((&t_emlrtRTEI), (&b_st), input_horz_dim_tmp,
      input_sizes_idx_1);
    for (i2 = 0; i2 < input_horz_dim_tmp * input_sizes_idx_1; i2++) {
      e_reshapes[1].f1[i1 + e_reshapes[1].f1.size(1) * i] = 0.0;
      i++;
      if (i > e_reshapes[1].f1.size(0) - 1) {
        i = 0;
        i1++;
      }
    }
  }

  Aeq_state.set_size((&x_emlrtRTEI), (&b_st), e_reshapes[0].f1.size(0),
                     (e_reshapes[0].f1.size(1) + e_reshapes[1].f1.size(1)));
  loop_ub = e_reshapes[0].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = e_reshapes[0].f1.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      Aeq_state[i1 + Aeq_state.size(1) * i] = e_reshapes[0].f1[i1 + e_reshapes[0]
        .f1.size(1) * i];
    }
  }

  loop_ub = e_reshapes[1].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = e_reshapes[1].f1.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      Aeq_state[(i1 + e_reshapes[0].f1.size(1)) + Aeq_state.size(1) * i] =
        e_reshapes[1].f1[i1 + e_reshapes[1].f1.size(1) * i];
    }
  }

  st.site = &j_emlrtRSI;
  b_st.site = &v_emlrtRSI;
  if ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_input.size(1);
  } else if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
    input_horz_dim_tmp = Aeq_state.size(1);
  } else {
    if (Aeq_input.size(1) > 0) {
      input_horz_dim_tmp = Aeq_input.size(1);
    } else {
      input_horz_dim_tmp = 0;
    }

    if (Aeq_state.size(1) > input_horz_dim_tmp) {
      input_horz_dim_tmp = Aeq_state.size(1);
    }
  }

  c_st.site = &w_emlrtRSI;
  if ((Aeq_input.size(1) != input_horz_dim_tmp) && ((Aeq_input.size(0) != 0) &&
       (Aeq_input.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if ((Aeq_state.size(1) != input_horz_dim_tmp) && ((Aeq_state.size(0) != 0) &&
       (Aeq_state.size(1) != 0))) {
    emlrtErrorWithMessageIdR2018a(&c_st, &c_emlrtRTEI,
      "MATLAB:catenate:matrixDimensionMismatch",
      "MATLAB:catenate:matrixDimensionMismatch", 0);
  }

  if (input_horz_dim_tmp == 0) {
    empty_non_axis_sizes = true;
    input_sizes_idx_1 = Aeq_input.size(0);
  } else {
    empty_non_axis_sizes = false;
    if ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) != 0)) {
      input_sizes_idx_1 = Aeq_input.size(0);
    } else {
      input_sizes_idx_1 = 0;
    }
  }

  if ((input_horz_dim_tmp == Aeq_input.size(1)) && (input_sizes_idx_1 ==
       Aeq_input.size(0))) {
    f_reshapes[0].f1.set_size((&u_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    loop_ub = input_horz_dim_tmp * input_sizes_idx_1;
    for (i = 0; i < loop_ub; i++) {
      f_reshapes[0].f1[i] = Aeq_input[i];
    }
  } else {
    i = 0;
    i1 = 0;
    i2 = 0;
    i3 = 0;
    f_reshapes[0].f1.set_size((&t_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    for (i4 = 0; i4 < input_sizes_idx_1 * input_horz_dim_tmp; i4++) {
      f_reshapes[0].f1[i1 + f_reshapes[0].f1.size(1) * i] = Aeq_input[i3 +
        Aeq_input.size(1) * i2];
      i++;
      i2++;
      if (i > f_reshapes[0].f1.size(0) - 1) {
        i = 0;
        i1++;
      }

      if (i2 > Aeq_input.size(0) - 1) {
        i2 = 0;
        i3++;
      }
    }
  }

  if (empty_non_axis_sizes || ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) !=
        0))) {
    input_sizes_idx_1 = Aeq_state.size(0);
  } else {
    input_sizes_idx_1 = 0;
  }

  if ((input_horz_dim_tmp == Aeq_state.size(1)) && (input_sizes_idx_1 ==
       Aeq_state.size(0))) {
    f_reshapes[1].f1.set_size((&u_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    loop_ub = input_horz_dim_tmp * input_sizes_idx_1;
    for (i = 0; i < loop_ub; i++) {
      f_reshapes[1].f1[i] = Aeq_state[i];
    }
  } else {
    i = 0;
    i1 = 0;
    i2 = 0;
    i3 = 0;
    f_reshapes[1].f1.set_size((&t_emlrtRTEI), (&b_st), input_sizes_idx_1,
      input_horz_dim_tmp);
    for (i4 = 0; i4 < input_sizes_idx_1 * input_horz_dim_tmp; i4++) {
      f_reshapes[1].f1[i1 + f_reshapes[1].f1.size(1) * i] = Aeq_state[i3 +
        Aeq_state.size(1) * i2];
      i++;
      i2++;
      if (i > f_reshapes[1].f1.size(0) - 1) {
        i = 0;
        i1++;
      }

      if (i2 > Aeq_state.size(0) - 1) {
        i2 = 0;
        i3++;
      }
    }
  }

  //      options =  optimoptions(@quadprog,'Display','off','Algorithm','interior-point-convex'); 
  //     %{
  //     min 0.5 x'*H*x + f'*x
  //     s.t.
  //     A*x = b
  //     A*x <= b
  //     %}
  if (dec_dim != d) {
    emlrtIntegerCheckR2012b(dec_dim, &i_emlrtDCI, sp);
  }

  //  active set initial guess
  Aeq_state.set_size((&y_emlrtRTEI), sp, (f_reshapes[0].f1.size(0) + f_reshapes
    [1].f1.size(0)), f_reshapes[0].f1.size(1));
  loop_ub = f_reshapes[0].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = f_reshapes[0].f1.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      Aeq_state[i1 + Aeq_state.size(1) * i] = f_reshapes[0].f1[i1 + f_reshapes[0]
        .f1.size(1) * i];
    }
  }

  loop_ub = f_reshapes[1].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = f_reshapes[1].f1.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      Aeq_state[i1 + Aeq_state.size(1) * (i + f_reshapes[0].f1.size(0))] =
        f_reshapes[1].f1[i1 + f_reshapes[1].f1.size(1) * i];
    }
  }

  b_bineq_input.set_size((&ab_emlrtRTEI), sp, (bineq_input.size(0) +
    bineq_state.size(0)));
  loop_ub = bineq_input.size(0);
  for (i = 0; i < loop_ub; i++) {
    b_bineq_input[i] = bineq_input[i];
  }

  loop_ub = bineq_state.size(0);
  for (i = 0; i < loop_ub; i++) {
    b_bineq_input[i + bineq_input.size(0)] = bineq_state[i];
  }

  Aeq_input.set_size((&bb_emlrtRTEI), sp, reshapes[0].f1.size(0), (reshapes[0].
    f1.size(1) + reshapes[1].f1.size(1)));
  loop_ub = reshapes[0].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = reshapes[0].f1.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      Aeq_input[i1 + Aeq_input.size(1) * i] = reshapes[0].f1[i1 + reshapes[0].
        f1.size(1) * i];
    }
  }

  loop_ub = reshapes[1].f1.size(0);
  for (i = 0; i < loop_ub; i++) {
    input_sizes_idx_1 = reshapes[1].f1.size(1);
    for (i1 = 0; i1 < input_sizes_idx_1; i1++) {
      Aeq_input[(i1 + reshapes[0].f1.size(1)) + Aeq_input.size(1) * i] =
        reshapes[1].f1[i1 + reshapes[1].f1.size(1) * i];
    }
  }

  bineq_state.set_size((&cb_emlrtRTEI), sp, b_loop_ub);
  for (i = 0; i < b_loop_ub; i++) {
    bineq_state[i] = 0.0;
  }

  st.site = &l_emlrtRSI;
  quadprog(&st, H, f, Aeq_state, b_bineq_input, Aeq_input, beq, bineq_state,
           bineq_input);

  //  [x0; x_horz; u_horz]
  d = (static_cast<real_T>(bineq_input.size(0)) + 1.0) - input_horz_dim;
  if (d > bineq_input.size(0)) {
    i = 0;
    i1 = 0;
  } else {
    i = static_cast<int32_T>(d);
    if ((i < 1) || (i > bineq_input.size(0))) {
      emlrtDynamicBoundsCheckR2012b(i, 1, bineq_input.size(0), &c_emlrtBCI, sp);
    }

    i--;
    if (bineq_input.size(0) < 1) {
      emlrtDynamicBoundsCheckR2012b(bineq_input.size(0), 1, bineq_input.size(0),
        &d_emlrtBCI, sp);
    }

    i1 = bineq_input.size(0);
  }

  b_state_horz_dim[0] = 1;
  input_sizes_idx_1 = i1 - i;
  b_state_horz_dim[1] = input_sizes_idx_1;
  st.site = &m_emlrtRSI;
  indexShapeCheck(&st, bineq_input.size(0), b_state_horz_dim);
  st.site = &n_emlrtRSI;
  b_indexShapeCheck(&st, i1 - i);
  if (1 > input_sizes_idx_1) {
    emlrtDynamicBoundsCheckR2012b(1, 1, input_sizes_idx_1, &n_emlrtBCI, sp);
  }

  u_t_idx[0] = bineq_input[i];
  if (2 > input_sizes_idx_1) {
    emlrtDynamicBoundsCheckR2012b(2, 1, 1, &n_emlrtBCI, sp);
  }

  u_t_idx[1] = bineq_input[i + 1];
  if (3 > input_sizes_idx_1) {
    emlrtDynamicBoundsCheckR2012b(3, 1, 2, &n_emlrtBCI, sp);
  }

  u_t_idx[2] = bineq_input[i + 2];

  //  inputs for this timestep
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (mpc_direct_tran.cpp)

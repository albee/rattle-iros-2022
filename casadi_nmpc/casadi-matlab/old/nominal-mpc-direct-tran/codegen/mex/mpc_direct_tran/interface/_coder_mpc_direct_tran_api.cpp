//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  _coder_mpc_direct_tran_api.cpp
//
//  Code generation for function '_coder_mpc_direct_tran_api'
//


// Include files
#include "_coder_mpc_direct_tran_api.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "rt_nonfinite.h"

// Function Declarations
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x_cur, const
  char_T *identifier, real_T y[6]);
static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6]);
static void e_emlrt_marshallIn(const emlrtStack *sp, mpc_direct_tranStackData
  *SD, const mxArray *x_ref_hist, const char_T *identifier, real_T y[60000]);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *a__output_of_feval_, const char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[3]);
static void f_emlrt_marshallIn(const emlrtStack *sp, mpc_direct_tranStackData
  *SD, const mxArray *u, const emlrtMsgIdentifier *parentId, real_T y[60000]);
static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6]);
static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[60000]);

// Function Definitions
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x_cur, const
  char_T *identifier, real_T y[6])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  d_emlrt_marshallIn(sp, emlrtAlias(x_cur), &thisId, y);
  emlrtDestroyArray(&x_cur);
}

static void d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId, real_T y[6])
{
  h_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static void e_emlrt_marshallIn(const emlrtStack *sp, mpc_direct_tranStackData
  *SD, const mxArray *x_ref_hist, const char_T *identifier, real_T y[60000])
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  f_emlrt_marshallIn(sp, SD, emlrtAlias(x_ref_hist), &thisId, y);
  emlrtDestroyArray(&x_ref_hist);
}

static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *a__output_of_feval_, const char_T *identifier)
{
  real_T y;
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char *>(identifier);
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(a__output_of_feval_), &thisId);
  emlrtDestroyArray(&a__output_of_feval_);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[3])
{
  const mxArray *y;
  const mxArray *m;
  static const int32_T iv[1] = { 3 };

  real_T *pData;
  y = NULL;
  m = emlrtCreateNumericArray(1, &iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u[0];
  pData[1] = u[1];
  pData[2] = u[2];
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack *sp, mpc_direct_tranStackData
  *SD, const mxArray *u, const emlrtMsgIdentifier *parentId, real_T y[60000])
{
  i_emlrt_marshallIn(sp, emlrtAlias(u), parentId, SD->f0.dv);
  for (int32_T i = 0; i < 10000; i++) {
    for (int32_T i1 = 0; i1 < 6; i1++) {
      y[i1 + 6 * i] = SD->f0.dv[i + 10000 * i1];
    }
  }

  emlrtDestroyArray(&u);
}

static real_T g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  real_T ret;
  static const int32_T dims = 0;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[6])
{
  static const int32_T dims[1] = { 6 };

  real_T (*r)[6];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 1U, dims);
  r = (real_T (*)[6])emlrtMxGetData(src);
  for (int32_T i = 0; i < 6; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

static void i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId, real_T ret[60000])
{
  static const int32_T dims[2] = { 10000, 6 };

  real_T (*r)[60000];
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, dims);
  r = (real_T (*)[60000])emlrtMxGetData(src);
  for (int32_T i = 0; i < 60000; i++) {
    ret[i] = (*r)[i];
  }

  emlrtDestroyArray(&src);
}

void mpc_direct_tran_api(mpc_direct_tranStackData *SD, const mxArray * const
  prhs[6], int32_T, const mxArray *plhs[1])
{
  real_T x_cur[6];
  real_T t_idx;
  real_T dt;
  real_T N;
  real_T Nf;
  real_T u_t_idx[3];
  emlrtStack st = { NULL,              // site
    NULL,                              // tls
    NULL                               // prev
  };

  st.tls = emlrtRootTLSGlobal;

  // Marshall function inputs
  c_emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "x_cur", x_cur);
  t_idx = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "t_idx");
  dt = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]), "dt");
  N = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "N");
  Nf = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "Nf");
  e_emlrt_marshallIn(&st, SD, emlrtAliasP(prhs[5]), "x_ref_hist",
                     SD->f1.x_ref_hist);

  // Invoke the target function
  mpc_direct_tran(&st, x_cur, t_idx, dt, N, Nf, SD->f1.x_ref_hist, u_t_idx);

  // Marshall function outputs
  plhs[0] = emlrt_marshallOut(u_t_idx);
}

// End of code generation (_coder_mpc_direct_tran_api.cpp)

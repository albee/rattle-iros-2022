//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  computeFirstOrderOpt.cpp
//
//  Code generation for function 'computeFirstOrderOpt'
//


// Include files
#include "computeFirstOrderOpt.h"
#include "blas.h"
#include "computeFval.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo xf_emlrtRSI = { 1,  // lineNo
  "computeFirstOrderOpt",              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+parseoutput/computeFirstOrderOpt.p"// pathName 
};

static emlrtBCInfo qc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "computeFirstOrderOpt",              // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+parseoutput/computeFirstOrderOpt.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo cd_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "computeFirstOrderOpt",              // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+parseoutput/computeFirstOrderOpt.p"// pName 
};

// Function Definitions
void computeFirstOrderOpt(const emlrtStack *sp, b_struct_T *solution, const
  d_struct_T *objective, int32_T workingset_nVar, int32_T workingset_ldA, const
  coder::array<real_T, 2U> &workingset_ATwset, int32_T workingset_nActiveConstr,
  coder::array<real_T, 2U> &workspace)
{
  real_T alpha1;
  real_T beta1;
  ptrdiff_t n_t;
  char_T TRANSA;
  ptrdiff_t incx_t;
  ptrdiff_t m_t;
  coder::array<real_T, 2U> r;
  int32_T loop_ub;
  ptrdiff_t lda_t;
  int32_T i;
  ptrdiff_t incy_t;
  int32_T b_loop_ub;
  int32_T i1;
  coder::array<real_T, 2U> r1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  st.site = &xf_emlrtRSI;
  xcopy(&st, workingset_nVar, objective->grad, workspace);
  st.site = &xf_emlrtRSI;
  if (workingset_nActiveConstr >= 1) {
    b_st.site = &ad_emlrtRSI;
    alpha1 = 1.0;
    beta1 = 1.0;
    TRANSA = 'N';
    m_t = (ptrdiff_t)workingset_nVar;
    n_t = (ptrdiff_t)workingset_nActiveConstr;
    lda_t = (ptrdiff_t)workingset_ldA;
    incx_t = (ptrdiff_t)1;
    incy_t = (ptrdiff_t)1;
    r.set_size((&wb_emlrtRTEI), (&b_st), workingset_ATwset.size(1),
               workingset_ATwset.size(0));
    loop_ub = workingset_ATwset.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = workingset_ATwset.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r[i1 + r.size(1) * i] = workingset_ATwset[i + workingset_ATwset.size(1) *
          i1];
      }
    }

    r1.set_size((&xb_emlrtRTEI), (&b_st), workspace.size(1), workspace.size(0));
    loop_ub = workspace.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = workspace.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        r1[i1 + r1.size(1) * i] = workspace[i + workspace.size(1) * i1];
      }
    }

    dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t,
          &(solution->lambda.data())[0], &incx_t, &beta1, &(r1.data())[0],
          &incy_t);
    workspace.set_size((&cd_emlrtRTEI), (&b_st), r1.size(1), r1.size(0));
    loop_ub = r1.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = r1.size(0);
      for (i1 = 0; i1 < b_loop_ub; i1++) {
        workspace[i1 + workspace.size(1) * i] = r1[i + r1.size(1) * i1];
      }
    }
  }

  st.site = &xf_emlrtRSI;
  n_t = (ptrdiff_t)workingset_nVar;
  incx_t = (ptrdiff_t)1;
  r.set_size((&ib_emlrtRTEI), (&st), workspace.size(1), workspace.size(0));
  loop_ub = workspace.size(1);
  for (i = 0; i < loop_ub; i++) {
    b_loop_ub = workspace.size(0);
    for (i1 = 0; i1 < b_loop_ub; i1++) {
      r[i1 + r.size(1) * i] = workspace[i + workspace.size(1) * i1];
    }
  }

  m_t = idamax(&n_t, &(r.data())[0], &incx_t);
  i = workspace.size(0) * workspace.size(1);
  if (((int32_T)m_t < 1) || ((int32_T)m_t > i)) {
    emlrtDynamicBoundsCheckR2012b((int32_T)m_t, 1, i, &qc_emlrtBCI, sp);
  }

  i = workspace.size(0);
  solution->firstorderopt = muDoubleScalarAbs(workspace[((int32_T)m_t - 1) % i *
    workspace.size(1) + ((int32_T)m_t - 1) / i]);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (computeFirstOrderOpt.cpp)

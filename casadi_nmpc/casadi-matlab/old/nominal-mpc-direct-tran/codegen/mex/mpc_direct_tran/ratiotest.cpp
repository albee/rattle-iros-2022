//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  ratiotest.cpp
//
//  Code generation for function 'ratiotest'
//


// Include files
#include "ratiotest.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemv.h"

// Variable Definitions
static emlrtRSInfo qf_emlrtRSI = { 1,  // lineNo
  "ratiotest",                         // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/ratiotest.p"// pathName 
};

static emlrtBCInfo nc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "ratiotest",                         // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/ratiotest.p",// pName 
  0                                    // checkKind
};

// Function Definitions
void ratiotest(const emlrtStack *sp, const coder::array<real_T, 1U>
               &solution_xstar, const coder::array<real_T, 1U>
               &solution_searchDir, coder::array<real_T, 2U> &workspace, int32_T
               workingset_nVar, int32_T workingset_ldA, const coder::array<
               real_T, 2U> &workingset_Aineq, const coder::array<real_T, 2U>
               &workingset_bineq, const coder::array<real_T, 1U> &workingset_lb,
               const coder::array<int32_T, 1U> &workingset_indexLB, const
               int32_T workingset_sizes[5], const int32_T
               workingset_isActiveIdx[6], const coder::array<boolean_T, 1U>
               &workingset_isActiveConstr, const int32_T workingset_nWConstr[5],
               real_T *toldelta, real_T *alpha, boolean_T *newBlocking, int32_T *
               constrType, int32_T *constrIdx)
{
  int32_T totalIneq;
  real_T p_max;
  real_T alphaTemp;
  ptrdiff_t n_t;
  real_T denomTol;
  ptrdiff_t incx_t;
  int32_T idx;
  int32_T i;
  int32_T i1;
  int32_T i2;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  totalIneq = workingset_sizes[2];
  *alpha = 1.0E+30;
  *newBlocking = false;
  *constrType = 0;
  *constrIdx = 0;
  p_max = 0.0;
  st.site = &qf_emlrtRSI;
  if (workingset_nVar < 1) {
    alphaTemp = 0.0;
  } else {
    n_t = (ptrdiff_t)workingset_nVar;
    incx_t = (ptrdiff_t)1;
    alphaTemp = dnrm2(&n_t, &(((coder::array<real_T, 1U> *)&solution_searchDir
      )->data())[0], &incx_t);
  }

  denomTol = 2.2204460492503131E-13 * alphaTemp;
  if (workingset_nWConstr[2] < workingset_sizes[2]) {
    int32_T ldw;
    boolean_T overflow;
    st.site = &qf_emlrtRSI;
    e_xcopy(&st, workingset_sizes[2], workingset_bineq, workspace);
    st.site = &qf_emlrtRSI;
    f_xgemv(&st, workingset_nVar, workingset_sizes[2], workingset_Aineq,
            workingset_ldA, solution_xstar, workspace);
    ldw = workspace.size(0);
    st.site = &qf_emlrtRSI;
    g_xgemv(&st, workingset_nVar, workingset_sizes[2], workingset_Aineq,
            workingset_ldA, solution_searchDir, workspace, workspace.size(0) + 1);
    st.site = &qf_emlrtRSI;
    overflow = ((1 <= workingset_sizes[2]) && (workingset_sizes[2] > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < totalIneq; idx++) {
      i = workspace.size(0) * workspace.size(1);
      i1 = ldw + idx;
      i2 = i1 + 1;
      if ((i2 < 1) || (i2 > i)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i, &nc_emlrtBCI, sp);
      }

      i = workspace.size(0);
      if (workspace[i1 % i * workspace.size(1) + i1 / i] > denomTol) {
        st.site = &qf_emlrtRSI;
        i = workingset_isActiveIdx[2] + idx;
        if ((i < 1) || (i > workingset_isActiveConstr.size(0))) {
          emlrtDynamicBoundsCheckR2012b(i, 1, workingset_isActiveConstr.size(0),
            &qb_emlrtBCI, &st);
        }

        if (!workingset_isActiveConstr[i - 1]) {
          int32_T i3;
          int32_T i4;
          i = workspace.size(0) * workspace.size(1);
          i3 = idx + 1;
          if ((i3 < 1) || (i3 > i)) {
            emlrtDynamicBoundsCheckR2012b(i3, 1, i, &nc_emlrtBCI, sp);
          }

          i = workspace.size(0) * workspace.size(1);
          i3 = idx + 1;
          if ((i3 < 1) || (i3 > i)) {
            emlrtDynamicBoundsCheckR2012b(i3, 1, i, &nc_emlrtBCI, sp);
          }

          i = workspace.size(0) * workspace.size(1);
          if (i2 > i) {
            emlrtDynamicBoundsCheckR2012b(i2, 1, i, &nc_emlrtBCI, sp);
          }

          i = workspace.size(0);
          i3 = workspace.size(0);
          i4 = workspace.size(0);
          alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(workspace[idx % i *
            workspace.size(1) + idx / i] - *toldelta), (1.0E-8 - workspace[idx %
            i3 * workspace.size(1) + idx / i3]) + *toldelta) / workspace[i1 % i4
            * workspace.size(1) + i1 / i4];
          if (alphaTemp <= *alpha) {
            i = workspace.size(0) * workspace.size(1);
            if (i2 > i) {
              emlrtDynamicBoundsCheckR2012b(i2, 1, i, &nc_emlrtBCI, sp);
            }

            i = workspace.size(0);
            if (muDoubleScalarAbs(workspace[i1 % i * workspace.size(1) + i1 / i])
                > p_max) {
              *alpha = alphaTemp;
              *constrType = 3;
              *constrIdx = idx + 1;
              *newBlocking = true;
            }
          }

          i = workspace.size(0) * workspace.size(1);
          i3 = idx + 1;
          if ((i3 < 1) || (i3 > i)) {
            emlrtDynamicBoundsCheckR2012b(i3, 1, i, &nc_emlrtBCI, sp);
          }

          i = workspace.size(0) * workspace.size(1);
          i3 = idx + 1;
          if ((i3 < 1) || (i3 > i)) {
            emlrtDynamicBoundsCheckR2012b(i3, 1, i, &nc_emlrtBCI, sp);
          }

          i = workspace.size(0) * workspace.size(1);
          if (i2 > i) {
            emlrtDynamicBoundsCheckR2012b(i2, 1, i, &nc_emlrtBCI, sp);
          }

          i = workspace.size(0);
          i3 = workspace.size(0);
          i4 = workspace.size(0);
          alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(workspace[idx % i *
            workspace.size(1) + idx / i]), 1.0E-8 - workspace[idx % i3 *
            workspace.size(1) + idx / i3]) / workspace[i1 % i4 * workspace.size
            (1) + i1 / i4];
          if (alphaTemp < *alpha) {
            *alpha = alphaTemp;
            *constrType = 3;
            *constrIdx = idx + 1;
            *newBlocking = true;
            i = workspace.size(0) * workspace.size(1);
            if (i2 > i) {
              emlrtDynamicBoundsCheckR2012b(i2, 1, i, &nc_emlrtBCI, sp);
            }

            i = workspace.size(0);
            p_max = muDoubleScalarAbs(workspace[i1 % i * workspace.size(1) + i1 /
              i]);
          }
        }
      }
    }
  }

  if (workingset_nWConstr[3] < workingset_sizes[3]) {
    real_T phaseOneCorrectionX;
    real_T phaseOneCorrectionP;
    if ((workingset_nVar < 1) || (workingset_nVar > solution_xstar.size(0))) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_xstar.size(0),
        &nc_emlrtBCI, sp);
    }

    phaseOneCorrectionX = 0.0 * solution_xstar[workingset_nVar - 1];
    if (workingset_nVar > solution_searchDir.size(0)) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_searchDir.size
        (0), &nc_emlrtBCI, sp);
    }

    phaseOneCorrectionP = 0.0 * solution_searchDir[workingset_nVar - 1];
    totalIneq = workingset_sizes[3];
    st.site = &qf_emlrtRSI;
    for (idx = 0; idx <= totalIneq - 2; idx++) {
      real_T pk_corrected;
      i = idx + 1;
      if ((i < 1) || (i > workingset_indexLB.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i, 1, workingset_indexLB.size(0),
          &nc_emlrtBCI, sp);
      }

      if ((workingset_indexLB[idx] < 1) || (workingset_indexLB[idx] >
           solution_searchDir.size(0))) {
        emlrtDynamicBoundsCheckR2012b(workingset_indexLB[idx], 1,
          solution_searchDir.size(0), &nc_emlrtBCI, sp);
      }

      pk_corrected = -solution_searchDir[workingset_indexLB[idx] - 1] -
        phaseOneCorrectionP;
      if (pk_corrected > denomTol) {
        st.site = &qf_emlrtRSI;
        i = workingset_isActiveIdx[3] + idx;
        if ((i < 1) || (i > workingset_isActiveConstr.size(0))) {
          emlrtDynamicBoundsCheckR2012b(i, 1, workingset_isActiveConstr.size(0),
            &qb_emlrtBCI, &st);
        }

        if (!workingset_isActiveConstr[i - 1]) {
          real_T ratio_tmp;
          if ((workingset_indexLB[idx] < 1) || (workingset_indexLB[idx] >
               solution_xstar.size(0))) {
            emlrtDynamicBoundsCheckR2012b(workingset_indexLB[idx], 1,
              solution_xstar.size(0), &nc_emlrtBCI, sp);
          }

          if ((workingset_indexLB[idx] < 1) || (workingset_indexLB[idx] >
               workingset_lb.size(0))) {
            emlrtDynamicBoundsCheckR2012b(workingset_indexLB[idx], 1,
              workingset_lb.size(0), &nc_emlrtBCI, sp);
          }

          ratio_tmp = -solution_xstar[workingset_indexLB[idx] - 1] -
            workingset_lb[workingset_indexLB[idx] - 1];
          alphaTemp = (ratio_tmp - *toldelta) - phaseOneCorrectionX;
          alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(alphaTemp), 1.0E-8 -
            alphaTemp) / pk_corrected;
          if ((alphaTemp <= *alpha) && (muDoubleScalarAbs(pk_corrected) > p_max))
          {
            *alpha = alphaTemp;
            *constrType = 4;
            *constrIdx = idx + 1;
            *newBlocking = true;
          }

          if ((workingset_indexLB[idx] < 1) || (workingset_indexLB[idx] >
               solution_xstar.size(0))) {
            emlrtDynamicBoundsCheckR2012b(workingset_indexLB[idx], 1,
              solution_xstar.size(0), &nc_emlrtBCI, sp);
          }

          if ((workingset_indexLB[idx] < 1) || (workingset_indexLB[idx] >
               workingset_lb.size(0))) {
            emlrtDynamicBoundsCheckR2012b(workingset_indexLB[idx], 1,
              workingset_lb.size(0), &nc_emlrtBCI, sp);
          }

          alphaTemp = ratio_tmp - phaseOneCorrectionX;
          alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(alphaTemp), 1.0E-8 -
            alphaTemp) / pk_corrected;
          if (alphaTemp < *alpha) {
            *alpha = alphaTemp;
            *constrType = 4;
            *constrIdx = idx + 1;
            *newBlocking = true;
            p_max = muDoubleScalarAbs(pk_corrected);
          }
        }
      }
    }

    if ((workingset_sizes[3] < 1) || (workingset_sizes[3] >
         workingset_indexLB.size(0))) {
      emlrtDynamicBoundsCheckR2012b(workingset_sizes[3], 1,
        workingset_indexLB.size(0), &nc_emlrtBCI, sp);
    }

    i = workingset_indexLB[workingset_sizes[3] - 1];
    if ((i < 1) || (i > solution_searchDir.size(0))) {
      emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir.size(0),
        &nc_emlrtBCI, sp);
    }

    if (-solution_searchDir[i - 1] > denomTol) {
      st.site = &qf_emlrtRSI;
      i1 = workingset_isActiveIdx[3] + workingset_sizes[3];
      i2 = i1 - 1;
      if ((i2 < 1) || (i2 > workingset_isActiveConstr.size(0))) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, workingset_isActiveConstr.size(0),
          &qb_emlrtBCI, &st);
      }

      if (!workingset_isActiveConstr[i1 - 2]) {
        if (i > solution_xstar.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, solution_xstar.size(0),
            &nc_emlrtBCI, sp);
        }

        if (i > workingset_lb.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, workingset_lb.size(0),
            &nc_emlrtBCI, sp);
        }

        alphaTemp = (-solution_xstar[i - 1] - workingset_lb[i - 1]) - *toldelta;
        if (i > solution_searchDir.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir.size(0),
            &nc_emlrtBCI, sp);
        }

        alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(alphaTemp), 1.0E-8 -
          alphaTemp) / -solution_searchDir[i - 1];
        if (alphaTemp <= *alpha) {
          if (i > solution_searchDir.size(0)) {
            emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir.size(0),
              &nc_emlrtBCI, sp);
          }

          if (muDoubleScalarAbs
              (solution_searchDir[workingset_indexLB[workingset_sizes[3] - 1] -
               1]) > p_max) {
            *alpha = alphaTemp;
            *constrType = 4;
            *constrIdx = workingset_sizes[3];
            *newBlocking = true;
          }
        }

        if (i > solution_xstar.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, solution_xstar.size(0),
            &nc_emlrtBCI, sp);
        }

        if (i > workingset_lb.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, workingset_lb.size(0),
            &nc_emlrtBCI, sp);
        }

        alphaTemp = -solution_xstar[i - 1] - workingset_lb[i - 1];
        if (i > solution_searchDir.size(0)) {
          emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir.size(0),
            &nc_emlrtBCI, sp);
        }

        alphaTemp = muDoubleScalarMin(muDoubleScalarAbs(alphaTemp), 1.0E-8 -
          alphaTemp) / -solution_searchDir[i - 1];
        if (alphaTemp < *alpha) {
          *alpha = alphaTemp;
          *constrType = 4;
          *constrIdx = workingset_sizes[3];
          *newBlocking = true;
          if (i > solution_searchDir.size(0)) {
            emlrtDynamicBoundsCheckR2012b(i, 1, solution_searchDir.size(0),
              &nc_emlrtBCI, sp);
          }

          p_max = muDoubleScalarAbs
            (solution_searchDir[workingset_indexLB[workingset_sizes[3] - 1] - 1]);
        }
      }
    }
  }

  if (workingset_nWConstr[4] < 0) {
    if ((workingset_nVar < 1) || (workingset_nVar > solution_xstar.size(0))) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_xstar.size(0),
        &nc_emlrtBCI, sp);
    }

    if (workingset_nVar > solution_searchDir.size(0)) {
      emlrtDynamicBoundsCheckR2012b(workingset_nVar, 1, solution_searchDir.size
        (0), &nc_emlrtBCI, sp);
    }

    st.site = &qf_emlrtRSI;
  }

  *toldelta += 6.608625846508183E-7;
  if (p_max > 0.0) {
    *alpha = muDoubleScalarMax(*alpha, 6.608625846508183E-7 / p_max);
  }

  if ((*newBlocking) && (*alpha > 1.0)) {
    *newBlocking = false;
  }

  *alpha = muDoubleScalarMin(*alpha, 1.0);
}

// End of code generation (ratiotest.cpp)

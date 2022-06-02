//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  factor.cpp
//
//  Code generation for function 'factor'
//


// Include files
#include "factor.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "fullColLDL2_.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "partialColLDL3_.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Function Definitions
void factor(const emlrtStack *sp, f_struct_T *obj, const coder::array<real_T, 2U>
            &A, int32_T ndims, int32_T ldA)
{
  real_T SCALED_REG_PRIMAL;
  int32_T LDimSizeP1_tmp_tmp;
  boolean_T overflow;
  int32_T A_maxDiag_idx;
  int32_T LD_diagOffset;
  ptrdiff_t n_t;
  int32_T i;
  ptrdiff_t incx_t;
  int32_T order;
  coder::array<real_T, 2U> r;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  SCALED_REG_PRIMAL = 1.4901161193847656E-6 * static_cast<real_T>(ndims);
  LDimSizeP1_tmp_tmp = obj->ldm + 1;
  obj->ndims = ndims;
  if (A.size(1) != 0) {
    st.site = &ff_emlrtRSI;
    overflow = ((1 <= ndims) && (ndims > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (LD_diagOffset = 0; LD_diagOffset < ndims; LD_diagOffset++) {
      st.site = &ff_emlrtRSI;
      b_xcopy(&st, ndims, A, ldA * LD_diagOffset + 1, obj->FMat, obj->ldm *
              LD_diagOffset + 1);
    }
  }

  st.site = &ff_emlrtRSI;
  if (ndims < 1) {
    A_maxDiag_idx = 0;
  } else {
    n_t = (ptrdiff_t)ndims;
    incx_t = (ptrdiff_t)(obj->ldm + 1);
    r.set_size((&ib_emlrtRTEI), (&st), obj->FMat.size(1), obj->FMat.size(0));
    A_maxDiag_idx = obj->FMat.size(1);
    for (i = 0; i < A_maxDiag_idx; i++) {
      LD_diagOffset = obj->FMat.size(0);
      for (order = 0; order < LD_diagOffset; order++) {
        r[order + r.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * order];
      }
    }

    n_t = idamax(&n_t, &(r.data())[0], &incx_t);
    A_maxDiag_idx = (int32_T)n_t;
  }

  i = obj->FMat.size(0) * obj->FMat.size(1);
  order = A_maxDiag_idx * (obj->ldm + 1);
  if ((order < 1) || (order > i)) {
    emlrtDynamicBoundsCheckR2012b(order, 1, i, &fc_emlrtBCI, sp);
  }

  i = order - 1;
  order = obj->FMat.size(0);
  obj->regTol_ = muDoubleScalarMax(muDoubleScalarAbs(obj->FMat[i % order *
    obj->FMat.size(1) + i / order]) * 2.2204460492503131E-16, muDoubleScalarAbs
    (SCALED_REG_PRIMAL));
  if (ndims > 128) {
    boolean_T exitg1;
    A_maxDiag_idx = 0;
    exitg1 = false;
    while ((!exitg1) && (A_maxDiag_idx < ndims)) {
      LD_diagOffset = LDimSizeP1_tmp_tmp * A_maxDiag_idx + 1;
      order = ndims - A_maxDiag_idx;
      if (A_maxDiag_idx + 48 <= ndims) {
        st.site = &ff_emlrtRSI;
        partialColLDL3_(&st, obj, LD_diagOffset, order, SCALED_REG_PRIMAL);
        A_maxDiag_idx += 48;
      } else {
        st.site = &ff_emlrtRSI;
        fullColLDL2_(&st, obj, LD_diagOffset, order, SCALED_REG_PRIMAL);
        exitg1 = true;
      }
    }
  } else {
    st.site = &ff_emlrtRSI;
    fullColLDL2_(&st, obj, 1, ndims, SCALED_REG_PRIMAL);
  }

  if (obj->ConvexCheck) {
    st.site = &ff_emlrtRSI;
    overflow = ((1 <= ndims) && (ndims > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    LD_diagOffset = 0;
    int32_T exitg2;
    do {
      exitg2 = 0;
      if (LD_diagOffset <= ndims - 1) {
        i = obj->FMat.size(1);
        order = LD_diagOffset + 1;
        if ((order < 1) || (order > i)) {
          emlrtDynamicBoundsCheckR2012b(order, 1, i, &fc_emlrtBCI, sp);
        }

        i = obj->FMat.size(0);
        A_maxDiag_idx = LD_diagOffset + 1;
        if ((A_maxDiag_idx < 1) || (A_maxDiag_idx > i)) {
          emlrtDynamicBoundsCheckR2012b(A_maxDiag_idx, 1, i, &fc_emlrtBCI, sp);
        }

        if (obj->FMat[(order + obj->FMat.size(1) * (A_maxDiag_idx - 1)) - 1] <=
            0.0) {
          obj->info = -(LD_diagOffset + 1);
          exitg2 = 1;
        } else {
          LD_diagOffset++;
        }
      } else {
        obj->ConvexCheck = false;
        exitg2 = 1;
      }
    } while (exitg2 == 0);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (factor.cpp)

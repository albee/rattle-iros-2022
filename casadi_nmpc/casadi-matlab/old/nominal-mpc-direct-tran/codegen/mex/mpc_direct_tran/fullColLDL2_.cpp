//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  fullColLDL2_.cpp
//
//  Code generation for function 'fullColLDL2_'
//


// Include files
#include "fullColLDL2_.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"

// Variable Definitions
static emlrtRSInfo cd_emlrtRSI = { 63, // lineNo
  "xger",                              // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xger.m"// pathName 
};

static emlrtRSInfo hf_emlrtRSI = { 1,  // lineNo
  "fullColLDL2_",                      // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/fullColLDL2_.p"// pathName 
};

static emlrtBCInfo jc_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "fullColLDL2_",                      // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/fullColLDL2_.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo uc_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "fullColLDL2_",                      // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/fullColLDL2_.p"// pName 
};

static emlrtRTEInfo vc_emlrtRTEI = { 209,// lineNo
  26,                                  // colNo
  "xger",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xger.m"// pName 
};

static emlrtRTEInfo wc_emlrtRTEI = { 211,// lineNo
  26,                                  // colNo
  "xger",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xger.m"// pName 
};

static emlrtRTEInfo xc_emlrtRTEI = { 213,// lineNo
  26,                                  // colNo
  "xger",                              // fName
  "/usr/local/MATLAB/R2020a/toolbox/eml/eml/+coder/+internal/+blas/xger.m"// pName 
};

// Function Definitions
void fullColLDL2_(const emlrtStack *sp, f_struct_T *obj, int32_T LD_offset,
                  int32_T NColsRemain, real_T REG_PRIMAL)
{
  int32_T LDimSizeP1;
  boolean_T overflow;
  int32_T i;
  real_T alpha1;
  int32_T i1;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  ptrdiff_t m_t;
  coder::array<real_T, 2U> r1;
  ptrdiff_t lda_t;
  coder::array<real_T, 2U> r2;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  LDimSizeP1 = obj->ldm;
  st.site = &hf_emlrtRSI;
  overflow = ((1 <= NColsRemain) && (NColsRemain > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (int32_T k = 0; k < NColsRemain; k++) {
    int32_T LD_diagOffset;
    int32_T subMatrixDim;
    int32_T loop_ub;
    int32_T b_loop_ub;
    LD_diagOffset = LD_offset + (LDimSizeP1 + 1) * k;
    i = obj->FMat.size(0) * obj->FMat.size(1);
    if ((LD_diagOffset < 1) || (LD_diagOffset > i)) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &jc_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    if (muDoubleScalarAbs(obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1)
                          + (LD_diagOffset - 1) / i]) <= obj->regTol_) {
      i = obj->FMat.size(0) * obj->FMat.size(1);
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &jc_emlrtBCI, sp);
      }

      i = obj->FMat.size(0) * obj->FMat.size(1);
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &jc_emlrtBCI, sp);
      }

      i = obj->FMat.size(0);
      i1 = obj->FMat.size(0);
      obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1) + (LD_diagOffset - 1)
        / i] = obj->FMat[(LD_diagOffset - 1) % i1 * obj->FMat.size(1) +
        (LD_diagOffset - 1) / i1] + REG_PRIMAL;
    }

    i = obj->FMat.size(0) * obj->FMat.size(1);
    if (LD_diagOffset > i) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &jc_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    alpha1 = -1.0 / obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1) +
      (LD_diagOffset - 1) / i];
    subMatrixDim = (NColsRemain - k) - 1;
    st.site = &hf_emlrtRSI;
    if (subMatrixDim >= 1) {
      b_st.site = &nb_emlrtRSI;
      n_t = (ptrdiff_t)subMatrixDim;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      r.set_size((&vb_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
      loop_ub = obj->FMat.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = obj->FMat.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
        }
      }

      r1.set_size((&lb_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      loop_ub = obj->workspace_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r1[i1 + r1.size(1) * i] = obj->workspace_[i + 48 * i1];
        }
      }

      dcopy(&n_t, &r[LD_diagOffset], &incx_t, &(r1.data())[0], &incy_t);
      obj->workspace_.set_size((&uc_emlrtRTEI), (&b_st), r1.size(1), 48);
      loop_ub = r1.size(1);
      for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 48; i1++) {
          obj->workspace_[i1 + 48 * i] = r1[i + r1.size(1) * i1];
        }
      }
    }

    st.site = &hf_emlrtRSI;
    if (subMatrixDim >= 1) {
      b_st.site = &cd_emlrtRSI;
      m_t = (ptrdiff_t)subMatrixDim;
      n_t = (ptrdiff_t)subMatrixDim;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      lda_t = (ptrdiff_t)obj->ldm;
      r1.set_size((&vc_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      r2.set_size((&wc_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      loop_ub = obj->workspace_.size(0);
      b_loop_ub = obj->workspace_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r1[i1 + r1.size(1) * i] = obj->workspace_[i + 48 * i1];
        }

        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r2[i1 + r2.size(1) * i] = obj->workspace_[i + 48 * i1];
        }
      }

      r.set_size((&xc_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
      loop_ub = obj->FMat.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = obj->FMat.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
        }
      }

      dger(&m_t, &n_t, &alpha1, &(r1.data())[0], &incx_t, &(r2.data())[0],
           &incy_t, &r[LD_diagOffset + LDimSizeP1], &lda_t);
      obj->FMat.set_size((&uc_emlrtRTEI), (&b_st), r.size(1), r.size(0));
      loop_ub = r.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = r.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          obj->FMat[i1 + obj->FMat.size(1) * i] = r[i + r.size(1) * i1];
        }
      }
    }

    i = obj->FMat.size(0) * obj->FMat.size(1);
    if (LD_diagOffset > i) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &jc_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    alpha1 = 1.0 / obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1) +
      (LD_diagOffset - 1) / i];
    st.site = &hf_emlrtRSI;
    if (subMatrixDim >= 1) {
      b_st.site = &xc_emlrtRSI;
      n_t = (ptrdiff_t)subMatrixDim;
      incx_t = (ptrdiff_t)1;
      r.set_size((&tc_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
      loop_ub = obj->FMat.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = obj->FMat.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
        }
      }

      dscal(&n_t, &alpha1, &r[LD_diagOffset], &incx_t);
      obj->FMat.set_size((&uc_emlrtRTEI), (&b_st), r.size(1), r.size(0));
      loop_ub = r.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = r.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          obj->FMat[i1 + obj->FMat.size(1) * i] = r[i + r.size(1) * i1];
        }
      }
    }
  }

  LDimSizeP1 = LD_offset + (obj->ldm + 1) * (NColsRemain - 1);
  i = obj->FMat.size(0) * obj->FMat.size(1);
  if ((LDimSizeP1 < 1) || (LDimSizeP1 > i)) {
    emlrtDynamicBoundsCheckR2012b(LDimSizeP1, 1, i, &jc_emlrtBCI, sp);
  }

  i = obj->FMat.size(0);
  if (muDoubleScalarAbs(obj->FMat[(LDimSizeP1 - 1) % i * obj->FMat.size(1) +
                        (LDimSizeP1 - 1) / i]) <= obj->regTol_) {
    i = obj->FMat.size(0) * obj->FMat.size(1);
    if (LDimSizeP1 > i) {
      emlrtDynamicBoundsCheckR2012b(LDimSizeP1, 1, i, &jc_emlrtBCI, sp);
    }

    i = obj->FMat.size(0) * obj->FMat.size(1);
    if (LDimSizeP1 > i) {
      emlrtDynamicBoundsCheckR2012b(LDimSizeP1, 1, i, &jc_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    i1 = obj->FMat.size(0);
    obj->FMat[(LDimSizeP1 - 1) % i * obj->FMat.size(1) + (LDimSizeP1 - 1) / i] =
      obj->FMat[(LDimSizeP1 - 1) % i1 * obj->FMat.size(1) + (LDimSizeP1 - 1) /
      i1] + REG_PRIMAL;
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (fullColLDL2_.cpp)

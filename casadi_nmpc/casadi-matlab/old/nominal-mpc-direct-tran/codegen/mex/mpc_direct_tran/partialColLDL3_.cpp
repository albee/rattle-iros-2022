//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  partialColLDL3_.cpp
//
//  Code generation for function 'partialColLDL3_'
//


// Include files
#include "partialColLDL3_.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo gf_emlrtRSI = { 1,  // lineNo
  "partialColLDL3_",                   // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/partialColLDL3_.p"// pathName 
};

static emlrtBCInfo ic_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "partialColLDL3_",                   // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/partialColLDL3_.p",// pName 
  0                                    // checkKind
};

static emlrtRTEInfo sc_emlrtRTEI = { 1,// lineNo
  1,                                   // colNo
  "partialColLDL3_",                   // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+DynamicRegCholManager/partialColLDL3_.p"// pName 
};

// Function Definitions
void partialColLDL3_(const emlrtStack *sp, f_struct_T *obj, int32_T LD_offset,
                     int32_T NColsRemain, real_T REG_PRIMAL)
{
  int32_T LDimSizeP1;
  int32_T k;
  int32_T b;
  int32_T subRows;
  int32_T LD_diagOffset_tmp;
  boolean_T overflow;
  int32_T LD_diagOffset;
  int32_T offsetColK;
  int32_T j;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  real_T alpha1;
  coder::array<real_T, 2U> r1;
  real_T beta1;
  int32_T loop_ub;
  char_T TRANSA;
  int32_T b_loop_ub;
  ptrdiff_t m_t;
  int32_T i;
  ptrdiff_t lda_t;
  int32_T i1;
  coder::array<real_T, 2U> r2;
  char_T TRANSA1;
  ptrdiff_t ldc_t;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  LDimSizeP1 = obj->ldm + 1;
  for (k = 0; k < 48; k++) {
    subRows = NColsRemain - k;
    LD_diagOffset_tmp = LDimSizeP1 * k;
    LD_diagOffset = LD_offset + LD_diagOffset_tmp;
    st.site = &gf_emlrtRSI;
    d_xcopy(&st, subRows, obj->FMat, LD_diagOffset, obj->workspace_,
            LD_diagOffset_tmp + 1);
    offsetColK = obj->ldm * k;
    st.site = &gf_emlrtRSI;
    if (NColsRemain >= 1) {
      b_st.site = &nb_emlrtRSI;
      n_t = (ptrdiff_t)NColsRemain;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      r.set_size((&vb_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      r1.set_size((&lb_emlrtRTEI), (&b_st), 48, obj->workspace2_.size(0));
      loop_ub = obj->workspace_.size(0);
      b_loop_ub = obj->workspace2_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->workspace_[i + 48 * i1];
        }

        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r1[i1 + r1.size(1) * i] = obj->workspace2_[i + 48 * i1];
        }
      }

      dcopy(&n_t, &r[offsetColK], &incx_t, &(r1.data())[0], &incy_t);
      obj->workspace2_.set_size((&sc_emlrtRTEI), (&b_st), r1.size(1), 48);
      loop_ub = r1.size(1);
      for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 48; i1++) {
          obj->workspace2_[i1 + 48 * i] = r1[i + r1.size(1) * i1];
        }
      }
    }

    st.site = &gf_emlrtRSI;
    if ((NColsRemain >= 1) && (k >= 1)) {
      b_st.site = &ad_emlrtRSI;
      alpha1 = -1.0;
      beta1 = 1.0;
      TRANSA = 'N';
      m_t = (ptrdiff_t)NColsRemain;
      n_t = (ptrdiff_t)k;
      lda_t = (ptrdiff_t)obj->ldm;
      incx_t = (ptrdiff_t)obj->ldm;
      incy_t = (ptrdiff_t)1;
      r.set_size((&wb_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      loop_ub = obj->workspace_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->workspace_[i + 48 * i1];
        }
      }

      r2.set_size((&jc_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
      loop_ub = obj->FMat.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = obj->FMat.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r2[i1 + r2.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
        }
      }

      r1.set_size((&xb_emlrtRTEI), (&b_st), 48, obj->workspace2_.size(0));
      loop_ub = obj->workspace2_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r1[i1 + r1.size(1) * i] = obj->workspace2_[i + 48 * i1];
        }
      }

      dgemv(&TRANSA, &m_t, &n_t, &alpha1, &(r.data())[0], &lda_t, &r2[(LD_offset
             + k) - 1], &incx_t, &beta1, &(r1.data())[0], &incy_t);
      obj->workspace2_.set_size((&sc_emlrtRTEI), (&b_st), r1.size(1), 48);
      loop_ub = r1.size(1);
      for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 48; i1++) {
          obj->workspace2_[i1 + 48 * i] = r1[i + r1.size(1) * i1];
        }
      }
    }

    st.site = &gf_emlrtRSI;
    if (NColsRemain >= 1) {
      b_st.site = &nb_emlrtRSI;
      n_t = (ptrdiff_t)NColsRemain;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      r.set_size((&vb_emlrtRTEI), (&b_st), 48, obj->workspace2_.size(0));
      r1.set_size((&lb_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      loop_ub = obj->workspace2_.size(0);
      b_loop_ub = obj->workspace_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->workspace2_[i + 48 * i1];
        }

        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r1[i1 + r1.size(1) * i] = obj->workspace_[i + 48 * i1];
        }
      }

      dcopy(&n_t, &(r.data())[0], &incx_t, &r1[offsetColK], &incy_t);
      obj->workspace_.set_size((&sc_emlrtRTEI), (&b_st), r1.size(1), 48);
      loop_ub = r1.size(1);
      for (i = 0; i < loop_ub; i++) {
        for (i1 = 0; i1 < 48; i1++) {
          obj->workspace_[i1 + 48 * i] = r1[i + r1.size(1) * i1];
        }
      }
    }

    st.site = &gf_emlrtRSI;
    if (subRows >= 1) {
      b_st.site = &nb_emlrtRSI;
      n_t = (ptrdiff_t)subRows;
      incx_t = (ptrdiff_t)1;
      incy_t = (ptrdiff_t)1;
      r.set_size((&vb_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
      loop_ub = obj->workspace_.size(0);
      for (i = 0; i < 48; i++) {
        for (i1 = 0; i1 < loop_ub; i1++) {
          r[i1 + r.size(1) * i] = obj->workspace_[i + 48 * i1];
        }
      }

      r2.set_size((&lb_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
      loop_ub = obj->FMat.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = obj->FMat.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r2[i1 + r2.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
        }
      }

      dcopy(&n_t, &r[LD_diagOffset_tmp], &incx_t, &r2[LD_diagOffset - 1],
            &incy_t);
      obj->FMat.set_size((&sc_emlrtRTEI), (&b_st), r2.size(1), r2.size(0));
      loop_ub = r2.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = r2.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          obj->FMat[i1 + obj->FMat.size(1) * i] = r2[i + r2.size(1) * i1];
        }
      }
    }

    i = obj->FMat.size(0) * obj->FMat.size(1);
    if ((LD_diagOffset < 1) || (LD_diagOffset > i)) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ic_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    if (muDoubleScalarAbs(obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1)
                          + (LD_diagOffset - 1) / i]) <= obj->regTol_) {
      i = obj->FMat.size(0) * obj->FMat.size(1);
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ic_emlrtBCI, sp);
      }

      i = obj->FMat.size(0) * obj->FMat.size(1);
      if (LD_diagOffset > i) {
        emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ic_emlrtBCI, sp);
      }

      i = obj->FMat.size(0);
      i1 = obj->FMat.size(0);
      obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1) + (LD_diagOffset - 1)
        / i] = obj->FMat[(LD_diagOffset - 1) % i1 * obj->FMat.size(1) +
        (LD_diagOffset - 1) / i1] + REG_PRIMAL;
    }

    i = obj->FMat.size(0) * obj->FMat.size(1);
    if (LD_diagOffset > i) {
      emlrtDynamicBoundsCheckR2012b(LD_diagOffset, 1, i, &ic_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    alpha1 = 1.0 / obj->FMat[(LD_diagOffset - 1) % i * obj->FMat.size(1) +
      (LD_diagOffset - 1) / i];
    st.site = &gf_emlrtRSI;
    if (subRows - 1 >= 1) {
      b_st.site = &xc_emlrtRSI;
      n_t = (ptrdiff_t)(subRows - 1);
      incx_t = (ptrdiff_t)1;
      r2.set_size((&tc_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
      loop_ub = obj->FMat.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = obj->FMat.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          r2[i1 + r2.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
        }
      }

      dscal(&n_t, &alpha1, &r2[LD_diagOffset], &incx_t);
      obj->FMat.set_size((&sc_emlrtRTEI), (&b_st), r2.size(1), r2.size(0));
      loop_ub = r2.size(1);
      for (i = 0; i < loop_ub; i++) {
        b_loop_ub = r2.size(0);
        for (i1 = 0; i1 < b_loop_ub; i1++) {
          obj->FMat[i1 + obj->FMat.size(1) * i] = r2[i + r2.size(1) * i1];
        }
      }
    }
  }

  b = NColsRemain - 1;
  st.site = &gf_emlrtRSI;
  overflow = ((48 <= NColsRemain - 1) && (NColsRemain - 1 > 2147483599));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (j = 48; j <= b; j += 48) {
    int32_T subBlockSize;
    int32_T b_tmp;
    int32_T b_b;
    offsetColK = NColsRemain - j;
    subBlockSize = muIntScalarMin_sint32(48, offsetColK);
    b_tmp = j + subBlockSize;
    b_b = b_tmp - 1;
    st.site = &gf_emlrtRSI;
    overflow = ((j <= b_b) && (b_b > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (k = j; k <= b_b; k++) {
      LD_diagOffset = b_tmp - k;
      subRows = LD_offset + k;
      for (loop_ub = 0; loop_ub < 48; loop_ub++) {
        i = obj->workspace2_.size(0);
        i1 = (subRows + loop_ub * obj->ldm) - 1;
        LD_diagOffset_tmp = obj->FMat.size(0);
        obj->workspace2_[loop_ub % i * 48 + loop_ub / i] = obj->FMat[i1 %
          LD_diagOffset_tmp * obj->FMat.size(1) + i1 / LD_diagOffset_tmp];
      }

      st.site = &gf_emlrtRSI;
      if (LD_diagOffset >= 1) {
        b_st.site = &ad_emlrtRSI;
        alpha1 = -1.0;
        beta1 = 1.0;
        TRANSA = 'N';
        m_t = (ptrdiff_t)LD_diagOffset;
        n_t = (ptrdiff_t)48;
        lda_t = (ptrdiff_t)obj->ldm;
        incx_t = (ptrdiff_t)1;
        incy_t = (ptrdiff_t)1;
        r.set_size((&wb_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
        r1.set_size((&jc_emlrtRTEI), (&b_st), 48, obj->workspace2_.size(0));
        loop_ub = obj->workspace_.size(0);
        b_loop_ub = obj->workspace2_.size(0);
        for (i = 0; i < 48; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            r[i1 + r.size(1) * i] = obj->workspace_[i + 48 * i1];
          }

          for (i1 = 0; i1 < b_loop_ub; i1++) {
            r1[i1 + r1.size(1) * i] = obj->workspace2_[i + 48 * i1];
          }
        }

        r2.set_size((&xb_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size
                    (0));
        loop_ub = obj->FMat.size(1);
        for (i = 0; i < loop_ub; i++) {
          b_loop_ub = obj->FMat.size(0);
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            r2[i1 + r2.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
          }
        }

        dgemv(&TRANSA, &m_t, &n_t, &alpha1, &r[k], &lda_t, &(r1.data())[0],
              &incx_t, &beta1, &r2[(LD_offset + LDimSizeP1 * k) - 1], &incy_t);
        obj->FMat.set_size((&sc_emlrtRTEI), (&b_st), r2.size(1), r2.size(0));
        loop_ub = r2.size(1);
        for (i = 0; i < loop_ub; i++) {
          b_loop_ub = r2.size(0);
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            obj->FMat[i1 + obj->FMat.size(1) * i] = r2[i + r2.size(1) * i1];
          }
        }
      }
    }

    if (b_tmp < NColsRemain) {
      LD_diagOffset = offsetColK - subBlockSize;
      for (subRows = 0; subRows < 48; subRows++) {
        st.site = &gf_emlrtRSI;
        d_xcopy(&st, subBlockSize, obj->FMat, (LD_offset + j) + subRows *
                obj->ldm, obj->workspace2_, subRows * obj->ldm + 1);
      }

      st.site = &gf_emlrtRSI;
      if ((LD_diagOffset >= 1) && (subBlockSize >= 1)) {
        b_st.site = &ee_emlrtRSI;
        alpha1 = -1.0;
        beta1 = 1.0;
        TRANSA = 'T';
        TRANSA1 = 'N';
        m_t = (ptrdiff_t)LD_diagOffset;
        n_t = (ptrdiff_t)subBlockSize;
        incx_t = (ptrdiff_t)48;
        lda_t = (ptrdiff_t)obj->ldm;
        incy_t = (ptrdiff_t)obj->ldm;
        ldc_t = (ptrdiff_t)obj->ldm;
        r.set_size((&cc_emlrtRTEI), (&b_st), 48, obj->workspace_.size(0));
        r1.set_size((&dc_emlrtRTEI), (&b_st), 48, obj->workspace2_.size(0));
        loop_ub = obj->workspace_.size(0);
        b_loop_ub = obj->workspace2_.size(0);
        for (i = 0; i < 48; i++) {
          for (i1 = 0; i1 < loop_ub; i1++) {
            r[i1 + r.size(1) * i] = obj->workspace_[i + 48 * i1];
          }

          for (i1 = 0; i1 < b_loop_ub; i1++) {
            r1[i1 + r1.size(1) * i] = obj->workspace2_[i + 48 * i1];
          }
        }

        r2.set_size((&ec_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size
                    (0));
        loop_ub = obj->FMat.size(1);
        for (i = 0; i < loop_ub; i++) {
          b_loop_ub = obj->FMat.size(0);
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            r2[i1 + r2.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * i1];
          }
        }

        dgemm(&TRANSA1, &TRANSA, &m_t, &n_t, &incx_t, &alpha1, &r[j +
              subBlockSize], &lda_t, &(r1.data())[0], &incy_t, &beta1, &r2
              [((LD_offset + subBlockSize) + LDimSizeP1 * j) - 1], &ldc_t);
        obj->FMat.set_size((&sc_emlrtRTEI), (&b_st), r2.size(1), r2.size(0));
        loop_ub = r2.size(1);
        for (i = 0; i < loop_ub; i++) {
          b_loop_ub = r2.size(0);
          for (i1 = 0; i1 < b_loop_ub; i1++) {
            obj->FMat[i1 + obj->FMat.size(1) * i] = r2[i + r2.size(1) * i1];
          }
        }
      }
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (partialColLDL3_.cpp)

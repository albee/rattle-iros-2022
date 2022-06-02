//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  maxConstraintViolation.cpp
//
//  Code generation for function 'maxConstraintViolation'
//


// Include files
#include "maxConstraintViolation.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"
#include "xgemv.h"

// Variable Definitions
static emlrtRSInfo le_emlrtRSI = { 1,  // lineNo
  "maxConstraintViolation",            // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/maxConstraintViolation.p"// pathName 
};

static emlrtRSInfo me_emlrtRSI = { 1,  // lineNo
  "maxConstraintViolation_AMats_regularized_",// fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/maxConstraintViolation_AMats_regularized_.p"// pathName 
};

static emlrtRSInfo ne_emlrtRSI = { 1,  // lineNo
  "maxConstraintViolation_AMats_nonregularized_",// fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/maxConstraintViolation_AMats_nonregularized_.p"// pathName 
};

static emlrtBCInfo lb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "maxConstraintViolation",            // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/maxConstraintViolation.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo mb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "maxConstraintViolation_AMats_nonregularized_",// fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/maxConstraintViolation_AMats_nonregularized_.p",// pName 
  0                                    // checkKind
};

static emlrtBCInfo nb_emlrtBCI = { -1, // iFirst
  -1,                                  // iLast
  1,                                   // lineNo
  1,                                   // colNo
  "",                                  // aName
  "maxConstraintViolation_AMats_regularized_",// fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+qpactiveset/+WorkingSet/maxConstraintViolation_AMats_regularized_.p",// pName 
  0                                    // checkKind
};

// Function Definitions
real_T b_maxConstraintViolation(const emlrtStack *sp, g_struct_T *obj, const
  coder::array<real_T, 2U> &x, int32_T ix0)
{
  real_T v;
  int32_T mLB;
  boolean_T overflow;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  int32_T idx;
  int32_T i;
  int32_T i1;
  int32_T i2;
  int32_T i3;
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
  mLB = obj->sizes[3];
  switch (obj->probType) {
   case 2:
    {
      int32_T mIneq;
      int32_T mEq;
      int32_T offsetEq1;
      int32_T offsetEq2;
      st.site = &le_emlrtRSI;
      v = 0.0;
      mIneq = obj->sizes[2];
      mEq = obj->sizes[1];
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        b_st.site = &me_emlrtRSI;
        if (obj->sizes[2] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[2];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->bineq.size(1),
                     obj->bineq.size(0));
          offsetEq1 = obj->bineq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->bineq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->bineq[i + obj->bineq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &me_emlrtRSI;
        b_xgemv(&b_st, obj->nVarOrig, obj->sizes[2], obj->Aineq, obj->ldA, x,
                ix0, obj->maxConstrWorkspace);
        b_st.site = &me_emlrtRSI;
        overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mIneq; idx++) {
          i = x.size(0) * x.size(1);
          i1 = (ix0 + obj->nVarOrig) + idx;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, &st);
          }

          i = i1 - 1;
          i1 = obj->maxConstrWorkspace.size(0);
          i2 = idx + 1;
          if ((i2 < 1) || (i2 > i1)) {
            emlrtDynamicBoundsCheckR2012b(i2, 1, i1, &nb_emlrtBCI, &st);
          }

          i1 = obj->maxConstrWorkspace.size(0);
          i3 = idx + 1;
          if ((i3 < 1) || (i3 > i1)) {
            emlrtDynamicBoundsCheckR2012b(i3, 1, i1, &nb_emlrtBCI, &st);
          }

          obj->maxConstrWorkspace[i3 - 1] = obj->maxConstrWorkspace[i2 - 1] -
            x[i % x.size(0) * x.size(1) + i / x.size(0)];
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, obj->maxConstrWorkspace[idx]);
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        boolean_T b;
        boolean_T b1;
        boolean_T b2;
        boolean_T b3;
        b_st.site = &me_emlrtRSI;
        if (obj->sizes[1] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[1];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->beq.size(1), obj->beq.size(0));
          offsetEq1 = obj->beq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->beq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->beq[i + obj->beq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &me_emlrtRSI;
        b_xgemv(&b_st, obj->nVarOrig, obj->sizes[1], obj->Aeq, obj->ldA, x, ix0,
                obj->maxConstrWorkspace);
        offsetEq1 = (obj->nVarOrig + obj->sizes[2]) - 2;
        offsetEq2 = offsetEq1 + obj->sizes[1];
        b_st.site = &me_emlrtRSI;
        overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        b = true;
        b1 = ((x.size(1) <= 0) || (x.size(0) <= 0));
        i = x.size(1) * x.size(0);
        i1 = 0;
        b2 = true;
        b3 = ((x.size(1) <= 0) || (x.size(0) <= 0));
        i2 = x.size(1) * x.size(0);
        i3 = 0;
        for (idx = 0; idx < mEq; idx++) {
          int32_T i4;
          int32_T i5;
          int32_T i6;
          mIneq = idx + 1;
          i4 = (ix0 + offsetEq2) + mIneq;
          if (b3 || (i4 < 0) || (i4 >= i2)) {
            i3 = 0;
            b2 = true;
          } else if (b2) {
            b2 = false;
            i3 = i4 % x.size(0) * x.size(1) + i4 / x.size(0);
          } else {
            i5 = x.size(1) * x.size(0) - 1;
            if (i3 > MAX_int32_T - x.size(1)) {
              i3 = i4 % x.size(0) * x.size(1) + i4 / x.size(0);
            } else {
              i3 += x.size(1);
              if (i3 > i5) {
                i3 -= i5;
              }
            }
          }

          i5 = (ix0 + offsetEq1) + mIneq;
          if (b1 || (i5 < 0) || (i5 >= i)) {
            i1 = 0;
            b = true;
          } else if (b) {
            b = false;
            i1 = i5 % x.size(0) * x.size(1) + i5 / x.size(0);
          } else {
            i6 = x.size(1) * x.size(0) - 1;
            if (i1 > MAX_int32_T - x.size(1)) {
              i1 = i5 % x.size(0) * x.size(1) + i5 / x.size(0);
            } else {
              i1 += x.size(1);
              if (i1 > i6) {
                i1 -= i6;
              }
            }
          }

          i6 = x.size(0) * x.size(1);
          i5++;
          if ((i5 < 1) || (i5 > i6)) {
            emlrtDynamicBoundsCheckR2012b(i5, 1, i6, &nb_emlrtBCI, &st);
          }

          i5 = x.size(0) * x.size(1);
          i4++;
          if ((i4 < 1) || (i4 > i5)) {
            emlrtDynamicBoundsCheckR2012b(i4, 1, i5, &nb_emlrtBCI, &st);
          }

          i4 = obj->maxConstrWorkspace.size(0);
          if ((mIneq < 1) || (mIneq > i4)) {
            emlrtDynamicBoundsCheckR2012b(mIneq, 1, i4, &nb_emlrtBCI, &st);
          }

          i4 = obj->maxConstrWorkspace.size(0);
          if (mIneq > i4) {
            emlrtDynamicBoundsCheckR2012b(mIneq, 1, i4, &nb_emlrtBCI, &st);
          }

          obj->maxConstrWorkspace[mIneq - 1] = (obj->maxConstrWorkspace[mIneq -
            1] - x[i1]) + x[i3];
          i4 = obj->maxConstrWorkspace.size(0);
          if (mIneq > i4) {
            emlrtDynamicBoundsCheckR2012b(mIneq, 1, i4, &nb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->
            maxConstrWorkspace[mIneq - 1]));
        }
      }
    }
    break;

   default:
    {
      int32_T mIneq;
      int32_T mEq;
      int32_T offsetEq1;
      int32_T offsetEq2;
      st.site = &le_emlrtRSI;
      v = 0.0;
      mIneq = obj->sizes[2];
      mEq = obj->sizes[1];
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        b_st.site = &ne_emlrtRSI;
        if (obj->sizes[2] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[2];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->bineq.size(1),
                     obj->bineq.size(0));
          offsetEq1 = obj->bineq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->bineq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->bineq[i + obj->bineq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &ne_emlrtRSI;
        b_xgemv(&b_st, obj->nVar, obj->sizes[2], obj->Aineq, obj->ldA, x, ix0,
                obj->maxConstrWorkspace);
        b_st.site = &ne_emlrtRSI;
        overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mIneq; idx++) {
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, obj->maxConstrWorkspace[idx]);
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        b_st.site = &ne_emlrtRSI;
        if (obj->sizes[1] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[1];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->beq.size(1), obj->beq.size(0));
          offsetEq1 = obj->beq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->beq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->beq[i + obj->beq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &ne_emlrtRSI;
        b_xgemv(&b_st, obj->nVar, obj->sizes[1], obj->Aeq, obj->ldA, x, ix0,
                obj->maxConstrWorkspace);
        b_st.site = &ne_emlrtRSI;
        overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mEq; idx++) {
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->maxConstrWorkspace[i1
            - 1]));
        }
      }
    }
    break;
  }

  if (obj->sizes[3] > 0) {
    st.site = &le_emlrtRSI;
    overflow = ((1 <= obj->sizes[3]) && (obj->sizes[3] > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < mLB; idx++) {
      i = obj->indexLB.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &lb_emlrtBCI, sp);
      }

      i = x.size(0) * x.size(1);
      i1 = ix0 + obj->indexLB[idx];
      i2 = i1 - 1;
      if ((i2 < 1) || (i2 > i)) {
        emlrtDynamicBoundsCheckR2012b(i2, 1, i, &lb_emlrtBCI, sp);
      }

      i = i1 - 2;
      i1 = obj->lb.size(0);
      if ((obj->indexLB[idx] < 1) || (obj->indexLB[idx] > i1)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB[idx], 1, i1, &lb_emlrtBCI, sp);
      }

      v = muDoubleScalarMax(v, -x[i % x.size(0) * x.size(1) + i / x.size(0)] -
                            obj->lb[obj->indexLB[idx] - 1]);
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return v;
}

real_T c_maxConstraintViolation(const emlrtStack *sp, g_struct_T *obj, const
  coder::array<real_T, 1U> &x)
{
  real_T v;
  int32_T mLB;
  boolean_T overflow;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  int32_T offsetEq1;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  int32_T idx;
  int32_T i;
  int32_T i1;
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
  mLB = obj->sizes[3];
  switch (obj->probType) {
   case 2:
    {
      int32_T mIneq;
      int32_T mEq;
      int32_T offsetEq2;
      int32_T b_idx;
      st.site = &le_emlrtRSI;
      v = 0.0;
      mIneq = obj->sizes[2];
      mEq = obj->sizes[1];
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        b_st.site = &me_emlrtRSI;
        if (obj->sizes[2] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[2];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->bineq.size(1),
                     obj->bineq.size(0));
          offsetEq1 = obj->bineq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->bineq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->bineq[i + obj->bineq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &me_emlrtRSI;
        c_xgemv(&b_st, obj->nVarOrig, obj->sizes[2], obj->Aineq, obj->ldA, x,
                obj->maxConstrWorkspace);
        b_st.site = &me_emlrtRSI;
        overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mIneq; idx++) {
          b_idx = idx + 1;
          i = obj->maxConstrWorkspace.size(0);
          if ((b_idx < 1) || (b_idx > i)) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &nb_emlrtBCI, &st);
          }

          i = obj->nVarOrig + b_idx;
          if ((i < 1) || (i > x.size(0))) {
            emlrtDynamicBoundsCheckR2012b(i, 1, x.size(0), &nb_emlrtBCI, &st);
          }

          i1 = obj->maxConstrWorkspace.size(0);
          if (b_idx > i1) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i1, &nb_emlrtBCI, &st);
          }

          obj->maxConstrWorkspace[b_idx - 1] = obj->maxConstrWorkspace[b_idx - 1]
            - x[i - 1];
          i = obj->maxConstrWorkspace.size(0);
          if (b_idx > i) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &nb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, obj->maxConstrWorkspace[b_idx - 1]);
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        b_st.site = &me_emlrtRSI;
        if (obj->sizes[1] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[1];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->beq.size(1), obj->beq.size(0));
          offsetEq1 = obj->beq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->beq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->beq[i + obj->beq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &me_emlrtRSI;
        c_xgemv(&b_st, obj->nVarOrig, obj->sizes[1], obj->Aeq, obj->ldA, x,
                obj->maxConstrWorkspace);
        offsetEq1 = obj->nVarOrig + obj->sizes[2];
        offsetEq2 = offsetEq1 + obj->sizes[1];
        b_st.site = &me_emlrtRSI;
        overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mEq; idx++) {
          b_idx = idx + 1;
          i = obj->maxConstrWorkspace.size(0);
          if ((b_idx < 1) || (b_idx > i)) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &nb_emlrtBCI, &st);
          }

          i = offsetEq1 + b_idx;
          if ((i < 1) || (i > x.size(0))) {
            emlrtDynamicBoundsCheckR2012b(i, 1, x.size(0), &nb_emlrtBCI, &st);
          }

          i1 = offsetEq2 + b_idx;
          if ((i1 < 1) || (i1 > x.size(0))) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, x.size(0), &nb_emlrtBCI, &st);
          }

          mIneq = obj->maxConstrWorkspace.size(0);
          if (b_idx > mIneq) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, mIneq, &nb_emlrtBCI, &st);
          }

          obj->maxConstrWorkspace[b_idx - 1] = (obj->maxConstrWorkspace[b_idx -
            1] - x[i - 1]) + x[i1 - 1];
          i = obj->maxConstrWorkspace.size(0);
          if (b_idx > i) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &nb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->
            maxConstrWorkspace[b_idx - 1]));
        }
      }
    }
    break;

   default:
    {
      int32_T mIneq;
      int32_T mEq;
      int32_T offsetEq2;
      st.site = &le_emlrtRSI;
      v = 0.0;
      mIneq = obj->sizes[2];
      mEq = obj->sizes[1];
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        b_st.site = &ne_emlrtRSI;
        if (obj->sizes[2] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[2];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->bineq.size(1),
                     obj->bineq.size(0));
          offsetEq1 = obj->bineq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->bineq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->bineq[i + obj->bineq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &ne_emlrtRSI;
        c_xgemv(&b_st, obj->nVar, obj->sizes[2], obj->Aineq, obj->ldA, x,
                obj->maxConstrWorkspace);
        b_st.site = &ne_emlrtRSI;
        overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mIneq; idx++) {
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, obj->maxConstrWorkspace[idx]);
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        b_st.site = &ne_emlrtRSI;
        if (obj->sizes[1] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[1];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->beq.size(1), obj->beq.size(0));
          offsetEq1 = obj->beq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->beq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->beq[i + obj->beq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &ne_emlrtRSI;
        c_xgemv(&b_st, obj->nVar, obj->sizes[1], obj->Aeq, obj->ldA, x,
                obj->maxConstrWorkspace);
        b_st.site = &ne_emlrtRSI;
        overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mEq; idx++) {
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->maxConstrWorkspace[i1
            - 1]));
        }
      }
    }
    break;
  }

  if (obj->sizes[3] > 0) {
    st.site = &le_emlrtRSI;
    overflow = ((1 <= obj->sizes[3]) && (obj->sizes[3] > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < mLB; idx++) {
      i = obj->indexLB.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &lb_emlrtBCI, sp);
      }

      offsetEq1 = obj->indexLB[i1 - 1] - 1;
      if ((obj->indexLB[idx] < 1) || (obj->indexLB[idx] > x.size(0))) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB[idx], 1, x.size(0),
          &lb_emlrtBCI, sp);
      }

      i = obj->lb.size(0);
      if ((obj->indexLB[idx] < 1) || (obj->indexLB[idx] > i)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB[idx], 1, i, &lb_emlrtBCI, sp);
      }

      v = muDoubleScalarMax(v, -x[offsetEq1] - obj->lb[offsetEq1]);
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return v;
}

real_T maxConstraintViolation(const emlrtStack *sp, g_struct_T *obj, const coder::
  array<real_T, 2U> &x)
{
  real_T v;
  int32_T mLB;
  boolean_T overflow;
  ptrdiff_t n_t;
  ptrdiff_t incx_t;
  int32_T offsetEq1;
  ptrdiff_t incy_t;
  coder::array<real_T, 2U> r;
  int32_T idx;
  int32_T i;
  int32_T i1;
  int32_T i2;
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
  mLB = obj->sizes[3];
  switch (obj->probType) {
   case 2:
    {
      int32_T mIneq;
      int32_T mEq;
      int32_T offsetEq2;
      int32_T b_idx;
      st.site = &le_emlrtRSI;
      v = 0.0;
      mIneq = obj->sizes[2];
      mEq = obj->sizes[1];
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        b_st.site = &me_emlrtRSI;
        if (obj->sizes[2] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[2];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->bineq.size(1),
                     obj->bineq.size(0));
          offsetEq1 = obj->bineq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->bineq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->bineq[i + obj->bineq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &me_emlrtRSI;
        xgemv(&b_st, obj->nVarOrig, obj->sizes[2], obj->Aineq, obj->ldA, x,
              obj->maxConstrWorkspace);
        b_st.site = &me_emlrtRSI;
        overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mIneq; idx++) {
          b_idx = idx + 1;
          i = x.size(0) * x.size(1);
          i1 = obj->nVarOrig + b_idx;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &nb_emlrtBCI, &st);
          }

          i = i1 - 1;
          i1 = obj->maxConstrWorkspace.size(0);
          if ((b_idx < 1) || (b_idx > i1)) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i1, &nb_emlrtBCI, &st);
          }

          i1 = obj->maxConstrWorkspace.size(0);
          if (b_idx > i1) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i1, &nb_emlrtBCI, &st);
          }

          obj->maxConstrWorkspace[b_idx - 1] = obj->maxConstrWorkspace[b_idx - 1]
            - x[i % x.size(0) * x.size(1) + i / x.size(0)];
          i = obj->maxConstrWorkspace.size(0);
          if (b_idx > i) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i, &nb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, obj->maxConstrWorkspace[b_idx - 1]);
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        boolean_T b;
        boolean_T b1;
        boolean_T b2;
        boolean_T b3;
        b_st.site = &me_emlrtRSI;
        if (obj->sizes[1] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[1];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->beq.size(1), obj->beq.size(0));
          offsetEq1 = obj->beq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->beq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->beq[i + obj->beq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &me_emlrtRSI;
        xgemv(&b_st, obj->nVarOrig, obj->sizes[1], obj->Aeq, obj->ldA, x,
              obj->maxConstrWorkspace);
        offsetEq1 = (obj->nVarOrig + obj->sizes[2]) - 1;
        offsetEq2 = offsetEq1 + obj->sizes[1];
        b_st.site = &me_emlrtRSI;
        overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        b = true;
        b1 = ((x.size(1) <= 0) || (x.size(0) <= 0));
        i = x.size(1) * x.size(0);
        i1 = 0;
        b2 = true;
        b3 = ((x.size(1) <= 0) || (x.size(0) <= 0));
        mIneq = x.size(1) * x.size(0);
        i2 = 0;
        for (idx = 0; idx < mEq; idx++) {
          int32_T i3;
          int32_T i4;
          int32_T i5;
          b_idx = idx + 1;
          i3 = offsetEq2 + b_idx;
          if (b3 || (i3 < 0) || (i3 >= mIneq)) {
            i2 = 0;
            b2 = true;
          } else if (b2) {
            b2 = false;
            i2 = i3 % x.size(0) * x.size(1) + i3 / x.size(0);
          } else {
            i4 = x.size(1) * x.size(0) - 1;
            if (i2 > MAX_int32_T - x.size(1)) {
              i2 = i3 % x.size(0) * x.size(1) + i3 / x.size(0);
            } else {
              i2 += x.size(1);
              if (i2 > i4) {
                i2 -= i4;
              }
            }
          }

          i4 = offsetEq1 + b_idx;
          if (b1 || (i4 < 0) || (i4 >= i)) {
            i1 = 0;
            b = true;
          } else if (b) {
            b = false;
            i1 = i4 % x.size(0) * x.size(1) + i4 / x.size(0);
          } else {
            i5 = x.size(1) * x.size(0) - 1;
            if (i1 > MAX_int32_T - x.size(1)) {
              i1 = i4 % x.size(0) * x.size(1) + i4 / x.size(0);
            } else {
              i1 += x.size(1);
              if (i1 > i5) {
                i1 -= i5;
              }
            }
          }

          i5 = x.size(0) * x.size(1);
          i4++;
          if ((i4 < 1) || (i4 > i5)) {
            emlrtDynamicBoundsCheckR2012b(i4, 1, i5, &nb_emlrtBCI, &st);
          }

          i4 = x.size(0) * x.size(1);
          i3++;
          if ((i3 < 1) || (i3 > i4)) {
            emlrtDynamicBoundsCheckR2012b(i3, 1, i4, &nb_emlrtBCI, &st);
          }

          i3 = obj->maxConstrWorkspace.size(0);
          if ((b_idx < 1) || (b_idx > i3)) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i3, &nb_emlrtBCI, &st);
          }

          i3 = obj->maxConstrWorkspace.size(0);
          if (b_idx > i3) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i3, &nb_emlrtBCI, &st);
          }

          obj->maxConstrWorkspace[b_idx - 1] = (obj->maxConstrWorkspace[b_idx -
            1] - x[i1]) + x[i2];
          i3 = obj->maxConstrWorkspace.size(0);
          if (b_idx > i3) {
            emlrtDynamicBoundsCheckR2012b(b_idx, 1, i3, &nb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->
            maxConstrWorkspace[b_idx - 1]));
        }
      }
    }
    break;

   default:
    {
      int32_T mIneq;
      int32_T mEq;
      int32_T offsetEq2;
      st.site = &le_emlrtRSI;
      v = 0.0;
      mIneq = obj->sizes[2];
      mEq = obj->sizes[1];
      if ((obj->Aineq.size(0) != 0) && (obj->Aineq.size(1) != 0)) {
        b_st.site = &ne_emlrtRSI;
        if (obj->sizes[2] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[2];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->bineq.size(1),
                     obj->bineq.size(0));
          offsetEq1 = obj->bineq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->bineq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->bineq[i + obj->bineq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &ne_emlrtRSI;
        xgemv(&b_st, obj->nVar, obj->sizes[2], obj->Aineq, obj->ldA, x,
              obj->maxConstrWorkspace);
        b_st.site = &ne_emlrtRSI;
        overflow = ((1 <= obj->sizes[2]) && (obj->sizes[2] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mIneq; idx++) {
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, obj->maxConstrWorkspace[idx]);
        }
      }

      if ((obj->Aeq.size(0) != 0) && (obj->Aeq.size(1) != 0)) {
        b_st.site = &ne_emlrtRSI;
        if (obj->sizes[1] >= 1) {
          c_st.site = &nb_emlrtRSI;
          n_t = (ptrdiff_t)obj->sizes[1];
          incx_t = (ptrdiff_t)1;
          incy_t = (ptrdiff_t)1;
          r.set_size((&vb_emlrtRTEI), (&c_st), obj->beq.size(1), obj->beq.size(0));
          offsetEq1 = obj->beq.size(1);
          for (i = 0; i < offsetEq1; i++) {
            offsetEq2 = obj->beq.size(0);
            for (i1 = 0; i1 < offsetEq2; i1++) {
              r[i1 + r.size(1) * i] = obj->beq[i + obj->beq.size(1) * i1];
            }
          }

          dcopy(&n_t, &(r.data())[0], &incx_t, &(obj->maxConstrWorkspace.data())
                [0], &incy_t);
        }

        b_st.site = &ne_emlrtRSI;
        xgemv(&b_st, obj->nVar, obj->sizes[1], obj->Aeq, obj->ldA, x,
              obj->maxConstrWorkspace);
        b_st.site = &ne_emlrtRSI;
        overflow = ((1 <= obj->sizes[1]) && (obj->sizes[1] > 2147483646));
        if (overflow) {
          c_st.site = &u_emlrtRSI;
          check_forloop_overflow_error(&c_st);
        }

        for (idx = 0; idx < mEq; idx++) {
          i = obj->maxConstrWorkspace.size(0);
          i1 = idx + 1;
          if ((i1 < 1) || (i1 > i)) {
            emlrtDynamicBoundsCheckR2012b(i1, 1, i, &mb_emlrtBCI, &st);
          }

          v = muDoubleScalarMax(v, muDoubleScalarAbs(obj->maxConstrWorkspace[i1
            - 1]));
        }
      }
    }
    break;
  }

  if (obj->sizes[3] > 0) {
    st.site = &le_emlrtRSI;
    overflow = ((1 <= obj->sizes[3]) && (obj->sizes[3] > 2147483646));
    if (overflow) {
      b_st.site = &u_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (idx = 0; idx < mLB; idx++) {
      i = obj->indexLB.size(0);
      i1 = idx + 1;
      if ((i1 < 1) || (i1 > i)) {
        emlrtDynamicBoundsCheckR2012b(i1, 1, i, &lb_emlrtBCI, sp);
      }

      offsetEq1 = obj->indexLB[i1 - 1] - 1;
      i = x.size(0) * x.size(1);
      if ((obj->indexLB[idx] < 1) || (obj->indexLB[idx] > i)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB[idx], 1, i, &lb_emlrtBCI, sp);
      }

      i = obj->lb.size(0);
      if ((obj->indexLB[idx] < 1) || (obj->indexLB[idx] > i)) {
        emlrtDynamicBoundsCheckR2012b(obj->indexLB[idx], 1, i, &lb_emlrtBCI, sp);
      }

      v = muDoubleScalarMax(v, -x[offsetEq1 % x.size(0) * x.size(1) + offsetEq1 /
                            x.size(0)] - obj->lb[offsetEq1]);
    }
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
  return v;
}

// End of code generation (maxConstraintViolation.cpp)

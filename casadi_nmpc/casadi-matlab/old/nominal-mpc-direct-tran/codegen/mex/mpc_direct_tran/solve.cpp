//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  solve.cpp
//
//  Code generation for function 'solve'
//


// Include files
#include "solve.h"
#include "blas.h"
#include "computeFval.h"
#include "eml_int_forloop_overflow_check.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Function Definitions
void solve(const emlrtStack *sp, const f_struct_T *obj, coder::array<real_T, 1U>
           &rhs)
{
  int32_T loop_ub;
  boolean_T overflow;
  char_T DIAGA1;
  char_T TRANSA1;
  char_T UPLO1;
  ptrdiff_t n_t;
  int32_T idx;
  ptrdiff_t lda_t;
  ptrdiff_t incx_t;
  int32_T b_loop_ub;
  coder::array<real_T, 2U> r;
  int32_T i;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  st.site = &if_emlrtRSI;
  if (obj->ndims >= 1) {
    b_st.site = &jf_emlrtRSI;
    DIAGA1 = 'U';
    TRANSA1 = 'N';
    UPLO1 = 'L';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    r.set_size((&pc_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
    loop_ub = obj->FMat.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = obj->FMat.size(0);
      for (idx = 0; idx < b_loop_ub; idx++) {
        r[idx + r.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * idx];
      }
    }

    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &(r.data())[0], &lda_t, &(rhs.data())
          [0], &incx_t);
  }

  loop_ub = obj->ndims;
  st.site = &if_emlrtRSI;
  overflow = ((1 <= obj->ndims) && (obj->ndims > 2147483646));
  if (overflow) {
    b_st.site = &u_emlrtRSI;
    check_forloop_overflow_error(&b_st);
  }

  for (idx = 0; idx < loop_ub; idx++) {
    b_loop_ub = idx + 1;
    i = rhs.size(0);
    if ((b_loop_ub < 1) || (b_loop_ub > i)) {
      emlrtDynamicBoundsCheckR2012b(b_loop_ub, 1, i, &ec_emlrtBCI, sp);
    }

    i = obj->FMat.size(1);
    if (b_loop_ub > i) {
      emlrtDynamicBoundsCheckR2012b(b_loop_ub, 1, i, &ec_emlrtBCI, sp);
    }

    i = obj->FMat.size(0);
    if (b_loop_ub > i) {
      emlrtDynamicBoundsCheckR2012b(b_loop_ub, 1, i, &ec_emlrtBCI, sp);
    }

    i = rhs.size(0);
    if (b_loop_ub > i) {
      emlrtDynamicBoundsCheckR2012b(b_loop_ub, 1, i, &ec_emlrtBCI, sp);
    }

    rhs[b_loop_ub - 1] = rhs[b_loop_ub - 1] / obj->FMat[(b_loop_ub +
      obj->FMat.size(1) * (b_loop_ub - 1)) - 1];
  }

  st.site = &if_emlrtRSI;
  if (obj->ndims >= 1) {
    b_st.site = &jf_emlrtRSI;
    DIAGA1 = 'U';
    TRANSA1 = 'T';
    UPLO1 = 'L';
    n_t = (ptrdiff_t)obj->ndims;
    lda_t = (ptrdiff_t)obj->ldm;
    incx_t = (ptrdiff_t)1;
    r.set_size((&pc_emlrtRTEI), (&b_st), obj->FMat.size(1), obj->FMat.size(0));
    loop_ub = obj->FMat.size(1);
    for (i = 0; i < loop_ub; i++) {
      b_loop_ub = obj->FMat.size(0);
      for (idx = 0; idx < b_loop_ub; idx++) {
        r[idx + r.size(1) * i] = obj->FMat[i + obj->FMat.size(1) * idx];
      }
    }

    dtrsv(&UPLO1, &TRANSA1, &DIAGA1, &n_t, &(r.data())[0], &lda_t, &(rhs.data())
          [0], &incx_t);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (solve.cpp)

//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  checkLinearInputs.cpp
//
//  Code generation for function 'checkLinearInputs'
//


// Include files
#include "checkLinearInputs.h"
#include "all.h"
#include "computeFval.h"
#include "factoryConstruct.h"
#include "feasibleratiotest.h"
#include "iterate.h"
#include "mpc_direct_tran.h"
#include "mpc_direct_tran_data.h"
#include "mwmathutil.h"
#include "quadprog.h"
#include "rt_nonfinite.h"
#include "xcopy.h"

// Variable Definitions
static emlrtRSInfo mb_emlrtRSI = { 1,  // lineNo
  "checkLinearInputs",                 // fcnName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+validate/checkLinearInputs.p"// pathName 
};

static emlrtRTEInfo i_emlrtRTEI = { 1, // lineNo
  1,                                   // colNo
  "checkLinearInputs",                 // fName
  "/usr/local/MATLAB/R2020a/toolbox/optim/+optim/+coder/+validate/checkLinearInputs.p"// pName 
};

// Function Definitions
void checkLinearInputs(const emlrtStack *sp, const coder::array<real_T, 1U> &x0,
  const coder::array<real_T, 2U> &Aineq, const coder::array<real_T, 1U> &bineq,
  const coder::array<real_T, 2U> &Aeq, const coder::array<real_T, 1U> &beq)
{
  int32_T Aineq_idx_0;
  int32_T i;
  coder::array<boolean_T, 1U> r;
  int32_T i1;
  int32_T i2;
  coder::array<real_T, 1U> r1;
  int32_T i3;
  coder::array<boolean_T, 1U> r2;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  if ((Aineq.size(0) != 0) && (Aineq.size(1) != 0)) {
    Aineq_idx_0 = Aineq.size(0) * Aineq.size(1);
    i = 0;
    i1 = 0;
    i2 = 0;
    r1.set_size((&g_emlrtRTEI), sp, Aineq_idx_0);
    for (i3 = 0; i3 < Aineq_idx_0; i3++) {
      r1[i] = Aineq[i2 + Aineq.size(1) * i1];
      i++;
      i1++;
      if (i1 > Aineq.size(0) - 1) {
        i1 = 0;
        i2++;
      }
    }

    r.set_size((&gb_emlrtRTEI), sp, r1.size(0));
    Aineq_idx_0 = r1.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = muDoubleScalarIsInf(r1[i]);
    }

    r2.set_size((&hb_emlrtRTEI), sp, r1.size(0));
    Aineq_idx_0 = r1.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r2[i] = muDoubleScalarIsNaN(r1[i]);
    }

    Aineq_idx_0 = r.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = ((!r[i]) && (!r2[i]));
    }

    st.site = &mb_emlrtRSI;
    if (!all(&st, r)) {
      emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
        "optim_codegen:common:InfNaNComplexDetected",
        "optim_codegen:common:InfNaNComplexDetected", 3, 4, 1, "A");
    }
  }

  if (bineq.size(0) != 0) {
    r.set_size((&gb_emlrtRTEI), sp, bineq.size(0));
    Aineq_idx_0 = bineq.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = muDoubleScalarIsInf(bineq[i]);
    }

    r2.set_size((&hb_emlrtRTEI), sp, bineq.size(0));
    Aineq_idx_0 = bineq.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r2[i] = muDoubleScalarIsNaN(bineq[i]);
    }

    Aineq_idx_0 = r.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = ((!r[i]) && (!r2[i]));
    }

    st.site = &mb_emlrtRSI;
    if (!all(&st, r)) {
      emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
        "optim_codegen:common:InfNaNComplexDetected",
        "optim_codegen:common:InfNaNComplexDetected", 3, 4, 1, "B");
    }
  }

  if ((Aeq.size(0) != 0) && (Aeq.size(1) != 0)) {
    Aineq_idx_0 = Aeq.size(0) * Aeq.size(1);
    i = 0;
    i1 = 0;
    i2 = 0;
    r1.set_size((&g_emlrtRTEI), sp, Aineq_idx_0);
    for (i3 = 0; i3 < Aineq_idx_0; i3++) {
      r1[i] = Aeq[i2 + Aeq.size(1) * i1];
      i++;
      i1++;
      if (i1 > Aeq.size(0) - 1) {
        i1 = 0;
        i2++;
      }
    }

    r.set_size((&gb_emlrtRTEI), sp, r1.size(0));
    Aineq_idx_0 = r1.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = muDoubleScalarIsInf(r1[i]);
    }

    r2.set_size((&hb_emlrtRTEI), sp, r1.size(0));
    Aineq_idx_0 = r1.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r2[i] = muDoubleScalarIsNaN(r1[i]);
    }

    Aineq_idx_0 = r.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = ((!r[i]) && (!r2[i]));
    }

    st.site = &mb_emlrtRSI;
    if (!all(&st, r)) {
      emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
        "optim_codegen:common:InfNaNComplexDetected",
        "optim_codegen:common:InfNaNComplexDetected", 3, 4, 3, "Aeq");
    }
  }

  if (beq.size(0) != 0) {
    r.set_size((&gb_emlrtRTEI), sp, beq.size(0));
    Aineq_idx_0 = beq.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = muDoubleScalarIsInf(beq[i]);
    }

    r2.set_size((&hb_emlrtRTEI), sp, beq.size(0));
    Aineq_idx_0 = beq.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r2[i] = muDoubleScalarIsNaN(beq[i]);
    }

    Aineq_idx_0 = r.size(0);
    for (i = 0; i < Aineq_idx_0; i++) {
      r[i] = ((!r[i]) && (!r2[i]));
    }

    st.site = &mb_emlrtRSI;
    if (!all(&st, r)) {
      emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
        "optim_codegen:common:InfNaNComplexDetected",
        "optim_codegen:common:InfNaNComplexDetected", 3, 4, 3, "Beq");
    }
  }

  r.set_size((&gb_emlrtRTEI), sp, x0.size(0));
  Aineq_idx_0 = x0.size(0);
  for (i = 0; i < Aineq_idx_0; i++) {
    r[i] = muDoubleScalarIsInf(x0[i]);
  }

  r2.set_size((&hb_emlrtRTEI), sp, x0.size(0));
  Aineq_idx_0 = x0.size(0);
  for (i = 0; i < Aineq_idx_0; i++) {
    r2[i] = muDoubleScalarIsNaN(x0[i]);
  }

  Aineq_idx_0 = r.size(0);
  for (i = 0; i < Aineq_idx_0; i++) {
    r[i] = ((!r[i]) && (!r2[i]));
  }

  st.site = &mb_emlrtRSI;
  if (!all(&st, r)) {
    emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
      "optim_codegen:common:InfNaNComplexDetected",
      "optim_codegen:common:InfNaNComplexDetected", 3, 4, 2, "X0");
  }

  if ((Aineq.size(0) != 0) && (Aineq.size(1) != 0) && (Aineq.size(1) != x0.size
       (0))) {
    emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
      "optim_codegen:common:WrongNumberOfColumnsInA",
      "optim_codegen:common:WrongNumberOfColumnsInA", 2, 6, static_cast<real_T>
      (x0.size(0)));
  }

  if ((Aineq.size(0) != 0) && (Aineq.size(1) != 0) && (Aineq.size(0) !=
       bineq.size(0))) {
    emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
      "optim_codegen:common:AAndBinInconsistent",
      "optim_codegen:common:AAndBinInconsistent", 0);
  }

  if ((Aeq.size(0) != 0) && (Aeq.size(1) != 0) && (Aeq.size(1) != x0.size(0))) {
    emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
      "optim_codegen:common:WrongNumberOfColumnsInAeq",
      "optim_codegen:common:WrongNumberOfColumnsInAeq", 2, 6, static_cast<real_T>
      (x0.size(0)));
  }

  if ((Aeq.size(0) != 0) && (Aeq.size(1) != 0) && (Aeq.size(0) != beq.size(0)))
  {
    emlrtErrorWithMessageIdR2018a(sp, &i_emlrtRTEI,
      "optim_codegen:common:AeqAndBeqInconsistent",
      "optim_codegen:common:AeqAndBeqInconsistent", 0);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

// End of code generation (checkLinearInputs.cpp)

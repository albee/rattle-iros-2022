//
//  Academic License - for use in teaching, academic research, and meeting
//  course requirements at degree granting institutions only.  Not for
//  government, commercial, or other organizational use.
//
//  mpc_direct_tran_types.h
//
//  Code generation for function 'mpc_direct_tran_types'
//


#pragma once

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4251)

#endif

// Type Definitions
struct e_struct_T
{
  int32_T ldq;
  coder::array<real_T, 2U> QR;
  coder::array<real_T, 2U> Q;
  coder::array<int32_T, 1U> jpvt;
  int32_T mrows;
  int32_T ncols;
  coder::array<real_T, 1U> tau;
  int32_T minRowCol;
  boolean_T usedPivoting;
};

struct f_struct_T
{
  coder::array<real_T, 2U> FMat;
  int32_T ldm;
  int32_T ndims;
  int32_T info;
  real_T scaleFactor;
  boolean_T ConvexCheck;
  real_T regTol_;
  coder::array<real_T, 2U> workspace_;
  coder::array<real_T, 2U> workspace2_;
};

struct g_struct_T
{
  int32_T mConstr;
  int32_T mConstrOrig;
  int32_T mConstrMax;
  int32_T nVar;
  int32_T nVarOrig;
  int32_T nVarMax;
  int32_T ldA;
  coder::array<real_T, 2U> Aineq;
  coder::array<real_T, 2U> bineq;
  coder::array<real_T, 2U> Aeq;
  coder::array<real_T, 2U> beq;
  coder::array<real_T, 1U> lb;
  coder::array<real_T, 1U> ub;
  coder::array<int32_T, 1U> indexLB;
  coder::array<int32_T, 1U> indexUB;
  coder::array<int32_T, 1U> indexFixed;
  int32_T mEqRemoved;
  coder::array<int32_T, 1U> indexEqRemoved;
  coder::array<real_T, 2U> ATwset;
  coder::array<real_T, 1U> bwset;
  int32_T nActiveConstr;
  coder::array<real_T, 1U> maxConstrWorkspace;
  int32_T sizes[5];
  int32_T sizesNormal[5];
  int32_T sizesPhaseOne[5];
  int32_T sizesRegularized[5];
  int32_T sizesRegPhaseOne[5];
  int32_T isActiveIdx[6];
  int32_T isActiveIdxNormal[6];
  int32_T isActiveIdxPhaseOne[6];
  int32_T isActiveIdxRegularized[6];
  int32_T isActiveIdxRegPhaseOne[6];
  coder::array<boolean_T, 1U> isActiveConstr;
  coder::array<int32_T, 1U> Wid;
  coder::array<int32_T, 1U> Wlocalidx;
  int32_T nWConstr[5];
  int32_T probType;
  real_T SLACK0;
};

struct mpc_direct_tranStackData
{
  struct {
    real_T dv[60000];
  } f0;

  struct {
    real_T x_ref_hist[60000];
  } f1;
};

struct h_struct_T
{
  char_T FiniteDifferenceType[7];
  boolean_T ScaleProblem;
  boolean_T SpecifyConstraintGradient;
  boolean_T SpecifyObjectiveGradient;
  boolean_T NonFiniteSupport;
  boolean_T IterDisplaySQP;
  real_T FiniteDifferenceStepSize;
  real_T MaxFunctionEvaluations;
  boolean_T IterDisplayQP;
  real_T PricingTolerance;
  char_T Algorithm[10];
  real_T ObjectiveLimit;
  real_T ConstraintTolerance;
  real_T OptimalityTolerance;
  real_T StepTolerance;
  real_T MaxIterations;
  char_T SolverName[8];
  boolean_T CheckGradients;
  char_T Diagnostics[3];
  real_T DiffMaxChange;
  real_T DiffMinChange;
  char_T Display[3];
  char_T FunValCheck[3];
  boolean_T UseParallel;
  real_T FunctionTolerance;
  char_T LinearSolver[4];
  char_T SubproblemAlgorithm[2];
};

struct d_struct_T
{
  coder::array<real_T, 1U> grad;
  coder::array<real_T, 1U> Hx;
  boolean_T hasLinear;
  int32_T nvar;
  int32_T maxVar;
  real_T beta;
  real_T rho;
  int32_T objtype;
  int32_T prev_objtype;
  int32_T prev_nvar;
  boolean_T prev_hasLinear;
  real_T gammaScalar;
};

struct struct_T
{
  int32_T MaxIterations;
  real_T ConstrRelTolFactor;
  real_T ProbRelTolFactor;
  boolean_T RemainFeasible;
};

struct b_struct_T
{
  coder::array<real_T, 1U> xstar;
  real_T fstar;
  real_T firstorderopt;
  coder::array<real_T, 1U> lambda;
  int32_T state;
  real_T maxConstr;
  int32_T iterations;
  coder::array<real_T, 1U> searchDir;
};

struct c_struct_T
{
  coder::array<real_T, 2U> workspace_double;
  coder::array<int32_T, 1U> workspace_int;
  coder::array<int32_T, 1U> workspace_sort;
};

#ifdef _MSC_VER

#pragma warning(pop)

#endif

// End of code generation (mpc_direct_tran_types.h)

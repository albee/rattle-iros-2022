//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_direct_tran_types.h
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//
#ifndef MPC_DIRECT_TRAN_TYPES_H
#define MPC_DIRECT_TRAN_TYPES_H

// Include Files
#include "rtwtypes.h"
#include "coder_array.h"
#ifdef MPC_DIRECT_TRAN_XIL_BUILD
#if defined(_MSC_VER) || defined(__LCC__)
#define MPC_DIRECT_TRAN_DLL_EXPORT     __declspec(dllimport)
#else
#define MPC_DIRECT_TRAN_DLL_EXPORT
#endif

#elif defined(BUILDING_MPC_DIRECT_TRAN)
#if defined(_MSC_VER) || defined(__LCC__)
#define MPC_DIRECT_TRAN_DLL_EXPORT     __declspec(dllexport)
#else
#define MPC_DIRECT_TRAN_DLL_EXPORT     __attribute__ ((visibility("default")))
#endif

#else
#define MPC_DIRECT_TRAN_DLL_EXPORT
#endif

#ifdef _MSC_VER

#pragma warning(push)
#pragma warning(disable : 4251)

#endif

// Type Definitions
struct d_struct_T
{
  coder::array<double, 1U> grad;
  coder::array<double, 1U> Hx;
  boolean_T hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  boolean_T prev_hasLinear;
  double gammaScalar;
};

struct h_struct_T
{
  char FiniteDifferenceType[7];
  boolean_T ScaleProblem;
  boolean_T SpecifyConstraintGradient;
  boolean_T SpecifyObjectiveGradient;
  boolean_T NonFiniteSupport;
  boolean_T IterDisplaySQP;
  double FiniteDifferenceStepSize;
  double MaxFunctionEvaluations;
  boolean_T IterDisplayQP;
  double PricingTolerance;
  char Algorithm[10];
  double ObjectiveLimit;
  double ConstraintTolerance;
  double OptimalityTolerance;
  double StepTolerance;
  double MaxIterations;
  char SolverName[8];
  boolean_T CheckGradients;
  char Diagnostics[3];
  double DiffMaxChange;
  double DiffMinChange;
  char Display[3];
  char FunValCheck[3];
  boolean_T UseParallel;
  double FunctionTolerance;
  char LinearSolver[4];
  char SubproblemAlgorithm[2];
};

struct f_struct_T
{
  coder::array<double, 2U> FMat;
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  coder::array<double, 2U> workspace_;
  coder::array<double, 2U> workspace2_;
};

struct e_struct_T
{
  int ldq;
  coder::array<double, 2U> QR;
  coder::array<double, 2U> Q;
  coder::array<int, 1U> jpvt;
  int mrows;
  int ncols;
  coder::array<double, 1U> tau;
  int minRowCol;
  boolean_T usedPivoting;
};

struct g_struct_T
{
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  coder::array<double, 2U> Aineq;
  coder::array<double, 2U> bineq;
  coder::array<double, 2U> Aeq;
  coder::array<double, 2U> beq;
  coder::array<double, 1U> lb;
  coder::array<double, 1U> ub;
  coder::array<int, 1U> indexLB;
  coder::array<int, 1U> indexUB;
  coder::array<int, 1U> indexFixed;
  int mEqRemoved;
  coder::array<int, 1U> indexEqRemoved;
  coder::array<double, 2U> ATwset;
  coder::array<double, 1U> bwset;
  int nActiveConstr;
  coder::array<double, 1U> maxConstrWorkspace;
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  coder::array<boolean_T, 1U> isActiveConstr;
  coder::array<int, 1U> Wid;
  coder::array<int, 1U> Wlocalidx;
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct c_struct_T
{
  coder::array<double, 2U> workspace_double;
  coder::array<int, 1U> workspace_int;
  coder::array<int, 1U> workspace_sort;
};

struct b_struct_T
{
  coder::array<double, 1U> xstar;
  double fstar;
  double firstorderopt;
  coder::array<double, 1U> lambda;
  int state;
  double maxConstr;
  int iterations;
  coder::array<double, 1U> searchDir;
};

struct struct_T
{
  int MaxIterations;
  double ConstrRelTolFactor;
  double ProbRelTolFactor;
  boolean_T RemainFeasible;
};

#ifdef _MSC_VER

#pragma warning(pop)

#endif
#endif

//
// File trailer for mpc_direct_tran_types.h
//
// [EOF]
//

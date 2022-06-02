//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: setProblemType.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "setProblemType.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran.h"
#include "quadprog.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : g_struct_T *obj
//                int PROBLEM_TYPE
// Return Type  : void
//
void setProblemType(g_struct_T *obj, int PROBLEM_TYPE)
{
  int i1;
  switch (PROBLEM_TYPE) {
   case 3:
    {
      int i;
      obj->nVar = obj->nVarOrig;
      obj->mConstr = obj->mConstrOrig;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesNormal[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxNormal[i];
      }
    }
    break;

   case 1:
    {
      int i;
      int mIneq;
      int idxStartIneq;
      obj->nVar = obj->nVarOrig + 1;
      obj->mConstr = obj->mConstrOrig + 1;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxPhaseOne[i];
      }

      i = obj->sizes[1];
      for (mIneq = 0; mIneq < i; mIneq++) {
        obj->Aeq[(obj->nVar + obj->Aeq.size(0) * mIneq) - 1] = 0.0;
        obj->ATwset[(obj->nVar + obj->ATwset.size(0) * ((obj->isActiveIdx[1] +
          mIneq) - 1)) - 1] = 0.0;
      }

      i = obj->sizes[2];
      for (mIneq = 0; mIneq < i; mIneq++) {
        obj->Aineq[(obj->nVar + obj->Aineq.size(0) * mIneq) - 1] = -1.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = obj->nVar;
      obj->lb[obj->nVar - 1] = obj->SLACK0;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (mIneq = idxStartIneq; mIneq <= i; mIneq++) {
        obj->ATwset[(obj->nVar + obj->ATwset.size(0) * (mIneq - 1)) - 1] = -1.0;
      }
    }
    break;

   case 2:
    {
      int i;
      obj->nVar = obj->nVarMax - 1;
      obj->mConstr = obj->mConstrMax - 1;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegularized[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegularized[i];
      }

      if (obj->probType != 4) {
        int mIneq;
        int mEq;
        int offsetIneq_tmp_tmp;
        int idxGlobalColStart;
        int offsetEq1;
        int offsetEq2;
        int idx_col;
        int idxStartIneq;
        int idx_row;
        mIneq = obj->sizes[2];
        mEq = obj->sizes[1];
        offsetIneq_tmp_tmp = obj->nVarOrig + 1;
        idxGlobalColStart = obj->nVarOrig + obj->sizes[2];
        offsetEq1 = idxGlobalColStart - 1;
        offsetEq2 = (idxGlobalColStart + obj->sizes[1]) - 1;
        for (idx_col = 0; idx_col < mIneq; idx_col++) {
          i = offsetIneq_tmp_tmp + idx_col;
          i1 = i - 1;
          for (idx_row = offsetIneq_tmp_tmp; idx_row <= i1; idx_row++) {
            obj->Aineq[(idx_row + obj->Aineq.size(0) * idx_col) - 1] = 0.0;
          }

          obj->Aineq[i1 + obj->Aineq.size(0) * idx_col] = -1.0;
          i++;
          i1 = obj->nVar;
          for (idx_row = i; idx_row <= i1; idx_row++) {
            obj->Aineq[(idx_row + obj->Aineq.size(0) * idx_col) - 1] = 0.0;
          }
        }

        idxGlobalColStart = obj->isActiveIdx[1] - 2;
        for (idx_col = 0; idx_col < mEq; idx_col++) {
          idxStartIneq = idx_col + 1;
          for (idx_row = offsetIneq_tmp_tmp; idx_row <= offsetEq1 + 1; idx_row++)
          {
            obj->Aeq[(idx_row + obj->Aeq.size(0) * (idxStartIneq - 1)) - 1] =
              0.0;
            obj->ATwset[(idx_row + obj->ATwset.size(0) * (idxGlobalColStart +
              idxStartIneq)) - 1] = 0.0;
          }

          i = offsetEq1 + 2;
          i1 = offsetEq1 + idxStartIneq;
          for (idx_row = i; idx_row <= i1; idx_row++) {
            obj->Aeq[(idx_row + obj->Aeq.size(0) * (idxStartIneq - 1)) - 1] =
              0.0;
            obj->ATwset[(idx_row + obj->ATwset.size(0) * (idxGlobalColStart +
              idxStartIneq)) - 1] = 0.0;
          }

          obj->Aeq[i1 + obj->Aeq.size(0) * (idxStartIneq - 1)] = -1.0;
          i = idxGlobalColStart + idxStartIneq;
          obj->ATwset[i1 + obj->ATwset.size(0) * i] = -1.0;
          i1 += 2;
          for (idx_row = i1; idx_row <= offsetEq2 + 1; idx_row++) {
            obj->Aeq[(idx_row + obj->Aeq.size(0) * (idxStartIneq - 1)) - 1] =
              0.0;
            obj->ATwset[(idx_row + obj->ATwset.size(0) * i) - 1] = 0.0;
          }

          i1 = offsetEq2 + 2;
          mIneq = offsetEq2 + idxStartIneq;
          for (idx_row = i1; idx_row <= mIneq; idx_row++) {
            obj->Aeq[(idx_row + obj->Aeq.size(0) * (idxStartIneq - 1)) - 1] =
              0.0;
            obj->ATwset[(idx_row + obj->ATwset.size(0) * i) - 1] = 0.0;
          }

          obj->Aeq[mIneq + obj->Aeq.size(0) * (idxStartIneq - 1)] = 1.0;
          obj->ATwset[mIneq + obj->ATwset.size(0) * i] = 1.0;
          i1 = mIneq + 2;
          mIneq = obj->nVar;
          for (idx_row = i1; idx_row <= mIneq; idx_row++) {
            obj->Aeq[(idx_row + obj->Aeq.size(0) * (idxStartIneq - 1)) - 1] =
              0.0;
            obj->ATwset[(idx_row + obj->ATwset.size(0) * i) - 1] = 0.0;
          }
        }

        idxGlobalColStart = obj->nVarOrig;
        i = obj->sizesRegularized[3];
        for (mIneq = 1; mIneq <= i; mIneq++) {
          idxGlobalColStart++;
          obj->indexLB[mIneq - 1] = idxGlobalColStart;
        }

        i = obj->nVarOrig + 1;
        i1 = (obj->nVarOrig + obj->sizes[2]) + (obj->sizes[1] << 1);
        for (mIneq = i; mIneq <= i1; mIneq++) {
          obj->lb[mIneq - 1] = 0.0;
        }

        idxStartIneq = obj->isActiveIdx[2];
        i = obj->nActiveConstr;
        for (idx_col = idxStartIneq; idx_col <= i; idx_col++) {
          switch (obj->Wid[idx_col - 1]) {
           case 3:
            idxGlobalColStart = obj->Wlocalidx[idx_col - 1];
            i1 = (offsetIneq_tmp_tmp + idxGlobalColStart) - 2;
            for (idx_row = offsetIneq_tmp_tmp; idx_row <= i1; idx_row++) {
              obj->ATwset[(idx_row + obj->ATwset.size(0) * (idx_col - 1)) - 1] =
                0.0;
            }

            obj->ATwset[((offsetIneq_tmp_tmp + idxGlobalColStart) +
                         obj->ATwset.size(0) * (idx_col - 1)) - 2] = -1.0;
            i1 = offsetIneq_tmp_tmp + idxGlobalColStart;
            mIneq = obj->nVar;
            for (idx_row = i1; idx_row <= mIneq; idx_row++) {
              obj->ATwset[(idx_row + obj->ATwset.size(0) * (idx_col - 1)) - 1] =
                0.0;
            }
            break;

           default:
            i1 = obj->nVar;
            for (idx_row = offsetIneq_tmp_tmp; idx_row <= i1; idx_row++) {
              obj->ATwset[(idx_row + obj->ATwset.size(0) * (idx_col - 1)) - 1] =
                0.0;
            }
            break;
          }
        }
      }
    }
    break;

   default:
    {
      int i;
      int mIneq;
      int idxStartIneq;
      obj->nVar = obj->nVarMax;
      obj->mConstr = obj->mConstrMax;
      for (i = 0; i < 5; i++) {
        obj->sizes[i] = obj->sizesRegPhaseOne[i];
      }

      for (i = 0; i < 6; i++) {
        obj->isActiveIdx[i] = obj->isActiveIdxRegPhaseOne[i];
      }

      i = obj->sizes[1];
      for (mIneq = 0; mIneq < i; mIneq++) {
        obj->Aeq[(obj->nVar + obj->Aeq.size(0) * mIneq) - 1] = 0.0;
        obj->ATwset[(obj->nVar + obj->ATwset.size(0) * ((obj->isActiveIdx[1] +
          mIneq) - 1)) - 1] = 0.0;
      }

      i = obj->sizes[2];
      for (mIneq = 0; mIneq < i; mIneq++) {
        obj->Aineq[(obj->nVar + obj->Aineq.size(0) * mIneq) - 1] = -1.0;
      }

      obj->indexLB[obj->sizes[3] - 1] = obj->nVar;
      obj->lb[obj->nVar - 1] = obj->SLACK0;
      idxStartIneq = obj->isActiveIdx[2];
      i = obj->nActiveConstr;
      for (mIneq = idxStartIneq; mIneq <= i; mIneq++) {
        obj->ATwset[(obj->nVar + obj->ATwset.size(0) * (mIneq - 1)) - 1] = -1.0;
      }
    }
    break;
  }

  obj->probType = PROBLEM_TYPE;
}

//
// File trailer for setProblemType.cpp
//
// [EOF]
//

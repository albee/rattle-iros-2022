//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: mpc_direct_tran.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 08-Jul-2020 19:43:17
//

// Include Files
#include "mpc_direct_tran.h"
#include "computeFval.h"
#include "feasibleX0ForWorkingSet.h"
#include "linearForm_.h"
#include "maxConstraintViolation.h"
#include "mpc_direct_tran_data.h"
#include "mpc_direct_tran_initialize.h"
#include "quadprog.h"
#include "repmat.h"
#include "rt_nonfinite.h"

// Function Definitions

//
// Arguments    : const double x_cur[6]
//                int t_idx
//                double dt
//                int N
//                int Nf
//                const double x_ref_hist[60000]
//                double u_mag
//                double v_mag
//                double Q_mag
//                double R_mag
//                double mass
//                const double w_bounds[6]
//                double u_t_idx[3]
// Return Type  : void
//
void mpc_direct_tran(const double x_cur[6], int t_idx, double dt, int N, int Nf,
                     const double x_ref_hist[60000], double u_mag, double v_mag,
                     double Q_mag, double R_mag, double mass, const double [6],
                     double u_t_idx[3])
{
  double A[36];
  int i;
  double B_tmp;
  static const signed char iv[6] = { 0, 0, 0, 1, 0, 0 };

  double B[18];
  static const signed char iv1[6] = { 0, 0, 0, 0, 1, 0 };

  static const signed char iv2[6] = { 0, 0, 0, 0, 0, 1 };

  long i1;
  int state_horz_dim;
  int input_horz_dim;
  int qY;
  coder::array<double, 1U> f;
  int b_qY;
  int b_i;
  coder::array<double, 2U> Aeq_state;
  int sizes_idx_1;
  int loop_ub_tmp;
  coder::array<double, 2U> Aeq_input;
  int loop_ub;
  double x_des[6];
  coder::array<double, 1U> beq;
  int input_sizes_idx_1;
  signed char b_I[36];
  int nv;
  int i2;
  double b_A[36];
  int c_i;
  boolean_T empty_non_axis_sizes;
  coder::array<signed char, 2U> varargin_1_tmp;
  int input_sizes_idx_0;
  coder::array<double, 2U> varargin_1;
  int sizes_idx_0;
  coder::array<signed char, 2U> Aineq_input;
  int iv3[6];
  double dv[3];
  coder::array<double, 1U> bineq_input;
  coder::array<signed char, 2U> Aineq_state;
  coder::array<double, 1U> bineq_state;
  coder::array<signed char, 2U> result;
  coder::array<double, 2U> v;
  coder::array<double, 1U> b_bineq_input;
  coder::array<double, 2U> b_Aeq_state;
  if (!isInitialized_libmpc_direct_tran) {
    mpc_direct_tran_initialize();
  }

  // {
  // Perform robust MPC, direct transcription.
  // }
  //          K_lqr;  % ancillary controller gain
  //          w_mag;  % max magnitude of uncertainty
  //          x_ref;  % reference value
  //      function prepare_ancillary(self)
  //          %--- Ancillary controller ---%
  //          Q = self.Q_mag*eye(self.n);
  //          R = self.R_mag*eye(self.m);
  //          [K, ~, ~] = dlqr(self.A, self.B, Q, R, zeros(self.n, self.m));
  //          self.K_lqr = -K;
  //
  //          self.shrink_constraints();  % shrink constraints for tube MPC
  //      end
  //
  //      % Shrink the constraints for the selected disturbance rejection term
  //      function um_shrunk = shrink_constraints(self)
  //          max_w = self.w_mag*ones(self.n,1);
  //          K_dr = self.K_lqr;
  //          Zmax = K_dr*max_w;
  //          self.u_constraint = self.u_constraint - abs(Zmax);
  //      end
  //      %% Ancillary controller
  //      %{
  //      Given the state estimate and goal, give the desired forces and
  //      torques
  //      %}
  //      function F_T = lqr_calc_input(self, t_idx)
  //          n = self.n;
  //
  //          x_ref = self.x_ref_hist(t_idx,1:n)';
  //          x_err = self.x - x_ref;  % predicted reference state
  //          F_T = self.K_lqr*x_err;
  //      end
  //      %{
  //      Calculate nominal MPC with tightened constraints, then add ancillary
  //      controller correction.
  //      %}
  //      function u_t_idx = mpc_robust_tube(self, t_idx)
  //          u_t_idx = mpc_direct_tran(self, t_idx);
  //          u_t_idx = u_t_idx + lqr_calc_input(self, t_idx);
  //      end
  //     %% Direct transcription
  //     %{
  //     Compute MPC inputs by direct transcription.
  //     Assumes the state estimate has been set!
  //
  //     Inputs:
  //     [current state, time index, dt, horizon length, ref traj length, ref
  //     traj, input magnitude, velocity magnitude, state weight, input weight,
  //     mass, uncertainty bounds]
  //     %}
  //  state size
  //  input size
  //  mass, kg
  //  are these actually being enforced?
  //  Set dynamics based on dt zero-order hold
  A[0] = 1.0;
  A[6] = 0.0;
  A[12] = 0.0;
  A[18] = dt;
  A[24] = 0.0;
  A[30] = 0.0;
  A[1] = 0.0;
  A[7] = 1.0;
  A[13] = 0.0;
  A[19] = 0.0;
  A[25] = dt;
  A[31] = 0.0;
  A[2] = 0.0;
  A[8] = 0.0;
  A[14] = 1.0;
  A[20] = 0.0;
  A[26] = 0.0;
  A[32] = dt;
  for (i = 0; i < 6; i++) {
    A[6 * i + 3] = iv[i];
    A[6 * i + 4] = iv1[i];
    A[6 * i + 5] = iv2[i];
  }

  B_tmp = dt * dt / (2.0 * mass);
  B[0] = B_tmp;
  B[6] = 0.0;
  B[12] = 0.0;
  B[1] = 0.0;
  B[7] = B_tmp;
  B[13] = 0.0;
  B[2] = 0.0;
  B[8] = 0.0;
  B[14] = B_tmp;
  B_tmp = dt / mass;
  B[3] = B_tmp;
  B[9] = 0.0;
  B[15] = 0.0;
  B[4] = 0.0;
  B[10] = B_tmp;
  B[16] = 0.0;
  B[5] = 0.0;
  B[11] = 0.0;
  B[17] = B_tmp;
  i1 = 6L * N;
  if (i1 > 2147483647L) {
    i1 = 2147483647L;
  } else {
    if (i1 < -2147483648L) {
      i1 = -2147483648L;
    }
  }

  state_horz_dim = static_cast<int>(i1);

  //  size of state decision variable vector
  i1 = 3L * N;
  if (i1 > 2147483647L) {
    i1 = 2147483647L;
  } else {
    if (i1 < -2147483648L) {
      i1 = -2147483648L;
    }
  }

  input_horz_dim = static_cast<int>(i1);

  //  size of input decision variable vector
  if ((state_horz_dim < 0) && (input_horz_dim < MIN_int32_T - state_horz_dim)) {
    qY = MIN_int32_T;
  } else if ((state_horz_dim > 0) && (input_horz_dim > MAX_int32_T
              - state_horz_dim)) {
    qY = MAX_int32_T;
  } else {
    qY = state_horz_dim + input_horz_dim;
  }

  //  size of total decision variable vector
  //  decision vector is of form [x_horz ; u_horz]
  // ---Cost function---
  //  min [x-x_ref]'*Q*[x-x_ref] + u'*R*u
  //  min 0.5*x'*H*x + f'x
  //  penalize states
  //  penalize inputs
  f.set_size(qY);
  for (i = 0; i < qY; i++) {
    f[i] = 0.0;
  }

  if (N < -2147483647) {
    b_qY = MIN_int32_T;
  } else {
    b_qY = N - 1;
  }

  for (b_i = 0; b_i <= b_qY; b_i++) {
    //  for the full horizon...
    if ((b_i > 0) && (t_idx > MAX_int32_T - b_i)) {
      sizes_idx_1 = MAX_int32_T;
    } else {
      sizes_idx_1 = b_i + t_idx;
    }

    if (sizes_idx_1 > Nf) {
      //  (use final state if horizon exceeds ref traj)
      //                  fprintf("%d %d\n", i, t0_idx);
      for (i = 0; i < 6; i++) {
        x_des[i] = x_ref_hist[(Nf + 10000 * i) - 1];
      }

      //  get state reference...
    } else {
      if ((b_i > 0) && (t_idx > MAX_int32_T - b_i)) {
        sizes_idx_1 = MAX_int32_T;
      } else {
        sizes_idx_1 = b_i + t_idx;
      }

      for (i = 0; i < 6; i++) {
        x_des[i] = x_ref_hist[(sizes_idx_1 + 10000 * i) - 1];
      }

      //  get state reference...
    }

    i1 = 6L * b_i;
    if (i1 > 2147483647L) {
      i1 = 2147483647L;
    } else {
      if (i1 < -2147483648L) {
        i1 = -2147483648L;
      }
    }

    input_sizes_idx_1 = static_cast<int>(i1);
    for (i = 0; i < 6; i++) {
      B_tmp = -x_des[i] * Q_mag;
      x_des[i] = B_tmp;
      nv = i + 1;
      if ((input_sizes_idx_1 < 0) && (nv < MIN_int32_T - input_sizes_idx_1)) {
        sizes_idx_1 = MIN_int32_T;
      } else if ((input_sizes_idx_1 > 0) && (nv > MAX_int32_T
                  - input_sizes_idx_1)) {
        sizes_idx_1 = MAX_int32_T;
      } else {
        sizes_idx_1 = input_sizes_idx_1 + nv;
      }

      f[sizes_idx_1 - 1] = B_tmp;
    }

    //  ...and set a penalty for missing this reference (state only)
  }

  // ---Equality constraints---
  //  A*x = b
  Aeq_state.set_size(state_horz_dim, state_horz_dim);
  loop_ub_tmp = state_horz_dim * state_horz_dim;
  for (i = 0; i < loop_ub_tmp; i++) {
    Aeq_state[i] = 0.0;
  }

  Aeq_input.set_size(state_horz_dim, input_horz_dim);
  loop_ub = state_horz_dim * input_horz_dim;
  for (i = 0; i < loop_ub; i++) {
    Aeq_input[i] = 0.0;
  }

  beq.set_size(state_horz_dim);
  for (i = 0; i < state_horz_dim; i++) {
    beq[i] = 0.0;
  }

  //  Dynamics constraints
  //  First constraint is set
  //  -x_1 + B*u_0 = -A*x_0
  for (i = 0; i < 36; i++) {
    b_I[i] = 0;
  }

  for (sizes_idx_1 = 0; sizes_idx_1 < 6; sizes_idx_1++) {
    b_I[sizes_idx_1 + 6 * sizes_idx_1] = 1;
  }

  for (i = 0; i < 36; i++) {
    b_I[i] = static_cast<signed char>(-b_I[i]);
  }

  for (i = 0; i < 6; i++) {
    for (i2 = 0; i2 < 6; i2++) {
      Aeq_state[i2 + Aeq_state.size(0) * i] = b_I[i2 + 6 * i];
    }
  }

  for (i = 0; i < 3; i++) {
    for (i2 = 0; i2 < 6; i2++) {
      Aeq_input[i2 + Aeq_input.size(0) * i] = B[i2 + 6 * i];
    }
  }

  for (i = 0; i < 36; i++) {
    b_A[i] = -A[i];
  }

  for (i = 0; i < 6; i++) {
    B_tmp = 0.0;
    for (i2 = 0; i2 < 6; i2++) {
      B_tmp += b_A[i + 6 * i2] * x_cur[i2];
    }

    beq[i] = B_tmp;
  }

  //  These constraints depend on dec vars
  if (N < -2147483647) {
    b_qY = MIN_int32_T;
  } else {
    b_qY = N - 1;
  }

  for (b_i = 0; b_i < b_qY; b_i++) {
    c_i = b_i + 1;

    //  x_n+ = A*x_n
    i1 = 6L * c_i;
    if (i1 > 2147483647L) {
      i1 = 2147483647L;
    } else {
      if (i1 < -2147483648L) {
        i1 = -2147483648L;
      }
    }

    input_sizes_idx_1 = static_cast<int>(i1);
    if (c_i < -2147483647) {
      sizes_idx_1 = MIN_int32_T;
    } else {
      sizes_idx_1 = c_i - 1;
    }

    i1 = 6L * sizes_idx_1;
    if (i1 > 2147483647L) {
      i1 = 2147483647L;
    } else {
      if (i1 < -2147483648L) {
        i1 = -2147483648L;
      }
    }

    input_sizes_idx_0 = static_cast<int>(i1);
    for (i = 0; i < 6; i++) {
      nv = i + 1;
      if ((input_sizes_idx_0 < 0) && (nv < MIN_int32_T - input_sizes_idx_0)) {
        sizes_idx_1 = MIN_int32_T;
      } else if ((input_sizes_idx_0 > 0) && (nv > MAX_int32_T
                  - input_sizes_idx_0)) {
        sizes_idx_1 = MAX_int32_T;
      } else {
        sizes_idx_1 = input_sizes_idx_0 + nv;
      }

      for (i2 = 0; i2 < 6; i2++) {
        nv = i2 + 1;
        if ((input_sizes_idx_1 < 0) && (nv < MIN_int32_T - input_sizes_idx_1)) {
          sizes_idx_0 = MIN_int32_T;
        } else if ((input_sizes_idx_1 > 0) && (nv > MAX_int32_T
                    - input_sizes_idx_1)) {
          sizes_idx_0 = MAX_int32_T;
        } else {
          sizes_idx_0 = input_sizes_idx_1 + nv;
        }

        Aeq_state[(sizes_idx_0 + Aeq_state.size(0) * (sizes_idx_1 - 1)) - 1] =
          A[i2 + 6 * i];
      }
    }

    if (input_sizes_idx_0 > 2147483646) {
      sizes_idx_1 = MAX_int32_T;
    } else {
      sizes_idx_1 = input_sizes_idx_0 + 1;
    }

    if (sizes_idx_1 > 2147483641) {
      sizes_idx_1 = MAX_int32_T;
    } else {
      sizes_idx_1 += 6;
    }

    if (input_sizes_idx_0 > 2147483641) {
      sizes_idx_0 = MAX_int32_T;
    } else {
      sizes_idx_0 = input_sizes_idx_0 + 6;
    }

    if (sizes_idx_0 > 2147483641) {
      sizes_idx_0 = MAX_int32_T;
    } else {
      sizes_idx_0 += 6;
    }

    if (sizes_idx_1 > sizes_idx_0) {
      i = 0;
      sizes_idx_0 = 0;
    } else {
      i = sizes_idx_1 - 1;
    }

    for (i2 = 0; i2 < 6; i2++) {
      nv = i2 + 1;
      if ((input_sizes_idx_1 < 0) && (nv < MIN_int32_T - input_sizes_idx_1)) {
        sizes_idx_1 = MIN_int32_T;
      } else if ((input_sizes_idx_1 > 0) && (nv > MAX_int32_T
                  - input_sizes_idx_1)) {
        sizes_idx_1 = MAX_int32_T;
      } else {
        sizes_idx_1 = input_sizes_idx_1 + nv;
      }

      iv3[i2] = sizes_idx_1 - 1;
    }

    loop_ub = sizes_idx_0 - i;
    for (i2 = 0; i2 < loop_ub; i2++) {
      for (sizes_idx_1 = 0; sizes_idx_1 < 6; sizes_idx_1++) {
        Aeq_state[iv3[sizes_idx_1] + Aeq_state.size(0) * (i + i2)] =
          b_I[sizes_idx_1 + 6 * i2];
      }
    }

    //  +B*u_n
    i1 = 3L * c_i;
    if (i1 > 2147483647L) {
      i1 = 2147483647L;
    } else {
      if (i1 < -2147483648L) {
        i1 = -2147483648L;
      }
    }

    input_sizes_idx_0 = static_cast<int>(i1);
    for (i = 0; i < 3; i++) {
      nv = i + 1;
      if ((input_sizes_idx_0 < 0) && (nv < MIN_int32_T - input_sizes_idx_0)) {
        sizes_idx_1 = MIN_int32_T;
      } else if ((input_sizes_idx_0 > 0) && (nv > MAX_int32_T
                  - input_sizes_idx_0)) {
        sizes_idx_1 = MAX_int32_T;
      } else {
        sizes_idx_1 = input_sizes_idx_0 + nv;
      }

      for (i2 = 0; i2 < 6; i2++) {
        nv = i2 + 1;
        if ((input_sizes_idx_1 < 0) && (nv < MIN_int32_T - input_sizes_idx_1)) {
          sizes_idx_0 = MIN_int32_T;
        } else if ((input_sizes_idx_1 > 0) && (nv > MAX_int32_T
                    - input_sizes_idx_1)) {
          sizes_idx_0 = MAX_int32_T;
        } else {
          sizes_idx_0 = input_sizes_idx_1 + nv;
        }

        Aeq_input[(sizes_idx_0 + Aeq_input.size(0) * (sizes_idx_1 - 1)) - 1] =
          B[i2 + 6 * i];
      }
    }
  }

  //  Assemble equality matrix
  if ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) != 0)) {
    b_i = Aeq_state.size(0);
  } else if ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) != 0)) {
    b_i = Aeq_input.size(0);
  } else {
    b_i = Aeq_state.size(0);
    if (b_i <= 0) {
      b_i = 0;
    }

    if (Aeq_input.size(0) > b_i) {
      b_i = Aeq_input.size(0);
    }
  }

  empty_non_axis_sizes = (b_i == 0);
  if (empty_non_axis_sizes || ((Aeq_state.size(0) != 0) && (Aeq_state.size(1) !=
        0))) {
    input_sizes_idx_1 = Aeq_state.size(1);
  } else {
    input_sizes_idx_1 = 0;
  }

  if (empty_non_axis_sizes || ((Aeq_input.size(0) != 0) && (Aeq_input.size(1) !=
        0))) {
    c_i = Aeq_input.size(1);
  } else {
    c_i = 0;
  }

  // ---Inequality constraints---
  //  A*x <= b
  //  Actuator saturation inequality constraint
  varargin_1_tmp.set_size(input_horz_dim, input_horz_dim);
  loop_ub = input_horz_dim * input_horz_dim;
  for (i = 0; i < loop_ub; i++) {
    varargin_1_tmp[i] = 0;
  }

  if (input_horz_dim > 0) {
    for (sizes_idx_1 = 0; sizes_idx_1 < input_horz_dim; sizes_idx_1++) {
      varargin_1_tmp[sizes_idx_1 + varargin_1_tmp.size(0) * sizes_idx_1] = 1;
    }
  }

  varargin_1.set_size(varargin_1_tmp.size(0), varargin_1_tmp.size(1));
  loop_ub = varargin_1_tmp.size(0) * varargin_1_tmp.size(1);
  for (i = 0; i < loop_ub; i++) {
    varargin_1[i] = -static_cast<double>(varargin_1_tmp[i]);
  }

  if ((varargin_1.size(0) != 0) && (varargin_1.size(1) != 0)) {
    b_qY = varargin_1.size(1);
  } else if ((varargin_1_tmp.size(0) != 0) && (varargin_1_tmp.size(1) != 0)) {
    b_qY = varargin_1_tmp.size(1);
  } else {
    b_qY = varargin_1.size(1);
    if (b_qY <= 0) {
      b_qY = 0;
    }

    if (varargin_1_tmp.size(1) > b_qY) {
      b_qY = varargin_1_tmp.size(1);
    }
  }

  empty_non_axis_sizes = (b_qY == 0);
  if (empty_non_axis_sizes || ((varargin_1.size(0) != 0) && (varargin_1.size(1)
        != 0))) {
    input_sizes_idx_0 = varargin_1.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }

  if (empty_non_axis_sizes || ((varargin_1_tmp.size(0) != 0) &&
       (varargin_1_tmp.size(1) != 0))) {
    sizes_idx_0 = varargin_1_tmp.size(0);
  } else {
    sizes_idx_0 = 0;
  }

  Aineq_input.set_size((input_sizes_idx_0 + sizes_idx_0), b_qY);
  for (i = 0; i < b_qY; i++) {
    for (i2 = 0; i2 < input_sizes_idx_0; i2++) {
      Aineq_input[i2 + Aineq_input.size(0) * i] = static_cast<signed char>
        (varargin_1[i2 + input_sizes_idx_0 * i]);
    }
  }

  for (i = 0; i < b_qY; i++) {
    for (i2 = 0; i2 < sizes_idx_0; i2++) {
      Aineq_input[(i2 + input_sizes_idx_0) + Aineq_input.size(0) * i] =
        varargin_1_tmp[i2 + sizes_idx_0 * i];
    }
  }

  dv[0] = u_mag;
  dv[1] = u_mag;
  dv[2] = u_mag;
  if (N > 1073741823) {
    i = MAX_int32_T;
  } else if (N <= -1073741824) {
    i = MIN_int32_T;
  } else {
    i = N << 1;
  }

  repmat(dv, i, bineq_input);

  //  Half-plane inequality constraint
  varargin_1_tmp.set_size(state_horz_dim, state_horz_dim);
  for (i = 0; i < loop_ub_tmp; i++) {
    varargin_1_tmp[i] = 0;
  }

  if (state_horz_dim > 0) {
    for (sizes_idx_1 = 0; sizes_idx_1 < state_horz_dim; sizes_idx_1++) {
      varargin_1_tmp[sizes_idx_1 + varargin_1_tmp.size(0) * sizes_idx_1] = 1;
    }
  }

  varargin_1.set_size(varargin_1_tmp.size(0), varargin_1_tmp.size(1));
  loop_ub = varargin_1_tmp.size(0) * varargin_1_tmp.size(1);
  for (i = 0; i < loop_ub; i++) {
    varargin_1[i] = -static_cast<double>(varargin_1_tmp[i]);
  }

  if ((varargin_1_tmp.size(0) != 0) && (varargin_1_tmp.size(1) != 0)) {
    b_qY = varargin_1_tmp.size(1);
  } else if ((varargin_1.size(0) != 0) && (varargin_1.size(1) != 0)) {
    b_qY = varargin_1.size(1);
  } else {
    b_qY = varargin_1_tmp.size(1);
    if (b_qY <= 0) {
      b_qY = 0;
    }

    if (varargin_1.size(1) > b_qY) {
      b_qY = varargin_1.size(1);
    }
  }

  empty_non_axis_sizes = (b_qY == 0);
  if (empty_non_axis_sizes || ((varargin_1_tmp.size(0) != 0) &&
       (varargin_1_tmp.size(1) != 0))) {
    input_sizes_idx_0 = varargin_1_tmp.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }

  if (empty_non_axis_sizes || ((varargin_1.size(0) != 0) && (varargin_1.size(1)
        != 0))) {
    sizes_idx_0 = varargin_1.size(0);
  } else {
    sizes_idx_0 = 0;
  }

  Aineq_state.set_size((input_sizes_idx_0 + sizes_idx_0), b_qY);
  for (i = 0; i < b_qY; i++) {
    for (i2 = 0; i2 < input_sizes_idx_0; i2++) {
      Aineq_state[i2 + Aineq_state.size(0) * i] = varargin_1_tmp[i2 +
        input_sizes_idx_0 * i];
    }
  }

  for (i = 0; i < b_qY; i++) {
    for (i2 = 0; i2 < sizes_idx_0; i2++) {
      Aineq_state[(i2 + input_sizes_idx_0) + Aineq_state.size(0) * i] =
        static_cast<signed char>(varargin_1[i2 + sizes_idx_0 * i]);
    }
  }

  x_des[0] = 100.0;
  x_des[1] = 100.0;
  x_des[2] = 100.0;
  x_des[3] = v_mag;
  x_des[4] = v_mag;
  x_des[5] = v_mag;
  if (N > 1073741823) {
    i = MAX_int32_T;
  } else if (N <= -1073741824) {
    i = MIN_int32_T;
  } else {
    i = N << 1;
  }

  b_repmat(x_des, i, bineq_state);

  //  Assemble inequality matrix
  if (input_horz_dim > 1073741823) {
    sizes_idx_1 = MAX_int32_T;
  } else if (input_horz_dim <= -1073741824) {
    sizes_idx_1 = MIN_int32_T;
  } else {
    sizes_idx_1 = input_horz_dim << 1;
  }

  if ((sizes_idx_1 != 0) && (state_horz_dim != 0)) {
    nv = sizes_idx_1;
  } else if ((Aineq_input.size(0) != 0) && (Aineq_input.size(1) != 0)) {
    nv = Aineq_input.size(0);
  } else {
    if (sizes_idx_1 > 0) {
      nv = sizes_idx_1;
    } else {
      nv = 0;
    }

    if (Aineq_input.size(0) > nv) {
      nv = Aineq_input.size(0);
    }
  }

  empty_non_axis_sizes = (nv == 0);
  if (empty_non_axis_sizes || ((sizes_idx_1 != 0) && (state_horz_dim != 0))) {
    input_sizes_idx_0 = state_horz_dim;
  } else {
    input_sizes_idx_0 = 0;
  }

  if (empty_non_axis_sizes || ((Aineq_input.size(0) != 0) && (Aineq_input.size(1)
        != 0))) {
    sizes_idx_1 = Aineq_input.size(1);
  } else {
    sizes_idx_1 = 0;
  }

  result.set_size(nv, (input_sizes_idx_0 + sizes_idx_1));
  for (i = 0; i < input_sizes_idx_0; i++) {
    for (i2 = 0; i2 < nv; i2++) {
      result[i2 + result.size(0) * i] = 0;
    }
  }

  for (i = 0; i < sizes_idx_1; i++) {
    for (i2 = 0; i2 < nv; i2++) {
      result[i2 + result.size(0) * (i + input_sizes_idx_0)] = Aineq_input[i2 +
        nv * i];
    }
  }

  if (state_horz_dim > 1073741823) {
    sizes_idx_1 = MAX_int32_T;
  } else if (state_horz_dim <= -1073741824) {
    sizes_idx_1 = MIN_int32_T;
  } else {
    sizes_idx_1 = state_horz_dim << 1;
  }

  if ((Aineq_state.size(0) != 0) && (Aineq_state.size(1) != 0)) {
    b_qY = Aineq_state.size(0);
  } else if ((sizes_idx_1 != 0) && (input_horz_dim != 0)) {
    b_qY = sizes_idx_1;
  } else {
    b_qY = Aineq_state.size(0);
    if (b_qY <= 0) {
      b_qY = 0;
    }

    if (sizes_idx_1 > b_qY) {
      b_qY = sizes_idx_1;
    }
  }

  empty_non_axis_sizes = (b_qY == 0);
  if (empty_non_axis_sizes || ((Aineq_state.size(0) != 0) && (Aineq_state.size(1)
        != 0))) {
    input_sizes_idx_0 = Aineq_state.size(1);
  } else {
    input_sizes_idx_0 = 0;
  }

  if (empty_non_axis_sizes || ((sizes_idx_1 != 0) && (input_horz_dim != 0))) {
    sizes_idx_1 = input_horz_dim;
  } else {
    sizes_idx_1 = 0;
  }

  varargin_1_tmp.set_size(b_qY, (input_sizes_idx_0 + sizes_idx_1));
  for (i = 0; i < input_sizes_idx_0; i++) {
    for (i2 = 0; i2 < b_qY; i2++) {
      varargin_1_tmp[i2 + varargin_1_tmp.size(0) * i] = Aineq_state[i2 + b_qY *
        i];
    }
  }

  for (i = 0; i < sizes_idx_1; i++) {
    for (i2 = 0; i2 < b_qY; i2++) {
      varargin_1_tmp[i2 + varargin_1_tmp.size(0) * (i + input_sizes_idx_0)] = 0;
    }
  }

  if ((result.size(0) != 0) && (result.size(1) != 0)) {
    b_qY = result.size(1);
  } else if ((varargin_1_tmp.size(0) != 0) && (varargin_1_tmp.size(1) != 0)) {
    b_qY = varargin_1_tmp.size(1);
  } else {
    b_qY = result.size(1);
    if (b_qY <= 0) {
      b_qY = 0;
    }

    if (varargin_1_tmp.size(1) > b_qY) {
      b_qY = varargin_1_tmp.size(1);
    }
  }

  empty_non_axis_sizes = (b_qY == 0);
  if (empty_non_axis_sizes || ((result.size(0) != 0) && (result.size(1) != 0)))
  {
    input_sizes_idx_0 = result.size(0);
  } else {
    input_sizes_idx_0 = 0;
  }

  if (empty_non_axis_sizes || ((varargin_1_tmp.size(0) != 0) &&
       (varargin_1_tmp.size(1) != 0))) {
    sizes_idx_0 = varargin_1_tmp.size(0);
  } else {
    sizes_idx_0 = 0;
  }

  //      options =  optimoptions(@quadprog,'Display','off','Algorithm','interior-point-convex'); 
  //     %{
  //     min 0.5 x'*H*x + f'*x
  //     s.t.
  //     A*x = b
  //     A*x <= b
  //     %}
  //  active set initial guess
  v.set_size(1, (state_horz_dim + input_horz_dim));
  for (i = 0; i < state_horz_dim; i++) {
    v[i] = Q_mag;
  }

  for (i = 0; i < input_horz_dim; i++) {
    v[i + state_horz_dim] = R_mag;
  }

  nv = v.size(1);
  varargin_1.set_size(v.size(1), v.size(1));
  loop_ub = v.size(1) * v.size(1);
  for (i = 0; i < loop_ub; i++) {
    varargin_1[i] = 0.0;
  }

  for (sizes_idx_1 = 0; sizes_idx_1 < nv; sizes_idx_1++) {
    varargin_1[sizes_idx_1 + varargin_1.size(0) * sizes_idx_1] = v[sizes_idx_1];
  }

  v.set_size((input_sizes_idx_0 + sizes_idx_0), b_qY);
  for (i = 0; i < b_qY; i++) {
    for (i2 = 0; i2 < input_sizes_idx_0; i2++) {
      v[i2 + v.size(0) * i] = result[i2 + input_sizes_idx_0 * i];
    }
  }

  for (i = 0; i < b_qY; i++) {
    for (i2 = 0; i2 < sizes_idx_0; i2++) {
      v[(i2 + input_sizes_idx_0) + v.size(0) * i] = varargin_1_tmp[i2 +
        sizes_idx_0 * i];
    }
  }

  b_bineq_input.set_size((bineq_input.size(0) + bineq_state.size(0)));
  loop_ub = bineq_input.size(0);
  for (i = 0; i < loop_ub; i++) {
    b_bineq_input[i] = bineq_input[i];
  }

  loop_ub = bineq_state.size(0);
  for (i = 0; i < loop_ub; i++) {
    b_bineq_input[i + bineq_input.size(0)] = bineq_state[i];
  }

  b_Aeq_state.set_size(b_i, (input_sizes_idx_1 + c_i));
  for (i = 0; i < input_sizes_idx_1; i++) {
    for (i2 = 0; i2 < b_i; i2++) {
      b_Aeq_state[i2 + b_Aeq_state.size(0) * i] = Aeq_state[i2 + b_i * i];
    }
  }

  for (i = 0; i < c_i; i++) {
    for (i2 = 0; i2 < b_i; i2++) {
      b_Aeq_state[i2 + b_Aeq_state.size(0) * (i + input_sizes_idx_1)] =
        Aeq_input[i2 + b_i * i];
    }
  }

  bineq_state.set_size(qY);
  for (i = 0; i < qY; i++) {
    bineq_state[i] = 0.0;
  }

  quadprog(varargin_1, f, v, b_bineq_input, b_Aeq_state, beq, bineq_state,
           bineq_input);

  //  [x0; x_horz; u_horz]
  B_tmp = (static_cast<double>(bineq_input.size(0)) + 1.0) - static_cast<double>
    (input_horz_dim);
  if (B_tmp < 2.147483648E+9) {
    if (B_tmp >= -2.147483648E+9) {
      i = static_cast<int>(B_tmp);
    } else {
      i = MIN_int32_T;
    }
  } else {
    i = MAX_int32_T;
  }

  if (i > bineq_input.size(0)) {
    i = 1;
  }

  u_t_idx[0] = bineq_input[i - 1];
  u_t_idx[1] = bineq_input[i];
  u_t_idx[2] = bineq_input[i + 1];

  //  inputs for this timestep
}

//
// File trailer for mpc_direct_tran.cpp
//
// [EOF]
//

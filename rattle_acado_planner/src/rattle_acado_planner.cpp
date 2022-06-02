/*
A C++ wrapper for calling the ACADO trajectory optimization software.

Monica Ekal and Keenan Albee, 2021.
Credit to ALG Prasad and Sambaran Ghoshal for ACADO integration details.
*/
#include "rattle_acado_planner/rattle_acado_planner.h"


RattlePlanner::RattlePlanner() {
  std::cout << "Model parameter values: " << params_model_ << std::endl;
  acado_startup();  // set initial values for solver
  acado_tic(&t);
}


ACADOvariables RattlePlanner::create_local_plan() {
  /* Run the planning loop.
  */
  // std::cout << "In local plan..." << std::endl;
  double tic = ros::Time::now().toSec();
  time_elapsed = (double)acado_toc(&t);  // note down time elapsed to use in the exponential decay

  // more iterations can improve the solution quality, but can't be too slow
  double kkt_value = 1.0e10;
  double last_kkt_value = 2*kkt_value;
  double max_kkt_tol = 1e-2;
  int iter = 1;
  while (kkt_value >max_kkt_tol && iter <= max_RTI_iter_) {
    kkt_value = run_acado_RTI_iteration();
    printf("RTI Iteration %d:  KKT Tolerance = %.3e Objective = %.3e\n", iter, acado_getKKT(), acado_getObjective());
    if ( abs(kkt_value - last_kkt_value) < 1e-2){  // quit iteration if no improvement
      break;
    }

    last_kkt_value = kkt_value;
    iter++;
  }
  
  iter_++;

  double toc = ros::Time::now().toSec();
  if (VERBOSE) {
    // std::cout << "Inputs: " << acadoVariables.u[0] << " " << acadoVariables.u[1] << " " << acadoVariables.u[2] << std::endl;
    std::cout << "Computation time: " << toc - tic << std::endl;
    // printf("Setpoint %d:  KKT Tolerance = %.3e Objective = %.3e\n", iter_, acado_getKKT(),
            // acado_getObjective());
    // printf("Info weights (m, cx, cy, ixx, iyy, izz): %f %f %f %f %F %F\n", INFO_WEIGHT_M, INFO_WEIGHT_CX, INFO_WEIGHT_CY, INFO_WEIGHT_IXX, INFO_WEIGHT_IYY, INFO_WEIGHT_IZZ);
    // std::cout << "Model parameters: " << params_model_ << std::endl;
    print_data();

    // if (shift_global_setpoints) {
    //   shift_start_point(); // the goal point in the last local plan becomes the start point for the next one
    //   update_psi();
    // }
  }

  // acado_shiftStates(1, 0, 0);  // shift state for next initial guess--speeds up Acado

  return acadoVariables;
}


double RattlePlanner::run_acado_RTI_iteration() {
  /* Run an RTI iteration of Acado
  */
  acado_preparationStep();   // Prepare for solving
  acado_feedbackStep();  // where the magic happens
  acado_shiftStates(1, 0, 0);  // helps with the initialization
  acado_shiftControls(0);

  return acado_getKKT();
}


void RattlePlanner::acado_startup() {
  /* Required setters to prepare Acado
  */
  acado_initializeSolver();  // Initialize the solver
  initialize_mpc();          // Initialize MPC-related ACADO variables
  get_cost_matrix();           // Initialize cost function weights
  if ( VERBOSE ) {
    // acado_printHeader();
  }
}


void RattlePlanner::initialize_mpc() {
  /* Initialize to zero-ed start state, inputs, etc.
  */

  int i;
  /* Initialize the states and controls. */
  for (i = 0; i <  (N + 1)*NX; ++i)  acadoVariables.x[i] = 0.0;
  for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.1;  // required to get info solutions!!

  /* Initialize the measurements/reference. */
  for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0;
  for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = 0.0;
  
  /* Initialize the current state. */
  for (i = 0; i < NX; ++i) acadoVariables.x0[i] = 0.0;
  acadoVariables.x0[9] = 1.0;  // quaternion
  update_psi();  // set psi variables to non-zero

  /* Initialize the online data values to zero---make sure they get real values before use!*/
  for (i = 0; i < NOD * (N+1); ++i) acadoVariables.od[i] = 0.0;
}


void RattlePlanner::get_cost_matrix() {
  /* ACADO updating and initialization
  */
  int i, j;
  for (i = 0; i < NY; i++) {
    for (j = 0; j < NY; j++) {
      if (i == j) {  // diagonal terms only
        switch (j) {
          case 0:
            acadoVariables.W[i * NY + j] = 0.0;  // weight for mass info gain
            break;
          case 1:
            acadoVariables.W[i * NY + j] = 0.0;  // weight for cx info gain
            break;
          case 2:
            acadoVariables.W[i * NY + j] = 0.0;  // weight for cy info gain
            break;
          case 3:
            acadoVariables.W[i * NY + j] = 0.0;  // weight for izz info gain
            break;
          case 4:
            acadoVariables.W[i * NY + j] = Wx_[0];  // weight for error_pos x
            break;
          case 5:
            acadoVariables.W[i * NY + j] = Wx_[1];  // weight for error_pos y
            break;
          case 6:
            acadoVariables.W[i * NY + j] = Wx_[2];  // weight for error_pos z
            break;
          case 7:
            acadoVariables.W[i * NY + j] = Wx_[3];  // 8.0;  // weight for error_vel x
            break;
          case 8:
            acadoVariables.W[i * NY + j] = Wx_[4];  // 8.0;  // weight for error_vel y
            break;
          case 9:
            acadoVariables.W[i * NY + j] = Wx_[5];  // 8.0;  // weight for error vel z
            break;
          case 10:
            acadoVariables.W[i * NY + j] = Wx_[6];  // weight for orientation eul error x
            break;
          case 11:
            acadoVariables.W[i * NY + j] = Wx_[7];  // weight for orientation eul error y
            break;
          case 12:
            acadoVariables.W[i * NY + j] = Wx_[8];  // weight for orientation eul error z
            break;
          case 13:
            acadoVariables.W[i * NY + j] = Wx_[9];  // weight for w_x
            break;
          case 14:
            acadoVariables.W[i * NY + j] = Wx_[10];  // weight for w_y
            break;
          case 15:
            acadoVariables.W[i * NY + j] = Wx_[11];  // weight for w_z
            break;
          case 16:
            acadoVariables.W[i * NY + j] = Wu_[0];  // fx
            break;
          case 17:
            acadoVariables.W[i * NY + j] = Wu_[1];  // fy
            break;
          case 18:
            acadoVariables.W[i * NY + j] = Wu_[2];  // fz
            break;
          case 19:
            acadoVariables.W[i * NY + j] = Wu_[3];  // tx
            break;
          case 20:
            acadoVariables.W[i * NY + j] = Wu_[4];  // ty
            break;
          case 21:
            acadoVariables.W[i * NY + j] = Wu_[5];  // tz
            break;
          default:
            acadoVariables.W[i * NY + j] = 0.2;  // other?
        }
      } else {
        acadoVariables.W[i * NY + j] = 0.0;  // off-diagonal terms
      }
    }
  }

  // set terminal weighting
  for (i = 0; i < NYN; i++) {
    for (j = 0; j < NYN; j++) {
      if (i == j) {  // diagonal terms
        acadoVariables.WN[i*NYN+j] = WN_[i];
      } else {
        acadoVariables.WN[i * NYN + j] = 0;  // off-diagonal terms
      }
    }
  }
}


void RattlePlanner::update_online_data(Params* param_ptr) {
  /* Update the reference parameters and goal state for ACADO
  */
  int i, j;

  // [mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3)]
  online_data_vector[0] = param_ptr->mass;
  online_data_vector[1] = param_ptr->ixx;
  online_data_vector[2] = param_ptr->iyy;
  online_data_vector[3] = param_ptr->izz;
  online_data_vector[4] = param_ptr->ixy;
  online_data_vector[5] = param_ptr->iyz;
  online_data_vector[6] = param_ptr->ixz;
  online_data_vector[7] = param_ptr->cx;
  online_data_vector[8] = param_ptr->cy;
  online_data_vector[9] = param_ptr->cz;
  online_data_vector[10] = lin_des[0];
  online_data_vector[11] = lin_des[1];
  online_data_vector[12] = lin_des[2];
  online_data_vector[13] = lin_des[3];
  online_data_vector[14] = lin_des[4];
  online_data_vector[15] = lin_des[5];
  online_data_vector[16] = orientation_des[0];
  online_data_vector[17] = orientation_des[1];
  online_data_vector[18] = orientation_des[2];
  online_data_vector[19] = orientation_des[3];
  online_data_vector[20] = orientation_des[4];
  online_data_vector[21] = orientation_des[5];
  online_data_vector[22] = orientation_des[6];

  // assign the repeating vector to acado variables od (online data)
  for (i = 0; i < N + 1; i++) {  // horizon
    for (j = 0; j < NOD; j++) {  // number of online data variables
      acadoVariables.od[i*NOD + j] = online_data_vector[j];
    }
  }
}


void RattlePlanner::assignWeightInfoGain(double time_elapsed) {
  /* Assign a weighting to information gain (optional)
  */
  int tau = 8;  // 10;//3;
  int W0 = 10;
  int W0_izz = 1;
  int W0_cy = 0.5;
  int W0_cx = 0.1;
  INFO_WEIGHT_M = W0*std::exp(-1.0/tau*time_elapsed);  // mass
  INFO_WEIGHT_IZZ = W0_izz*std::exp(-1.0/tau*time_elapsed);  // MoI
  INFO_WEIGHT_CX = W0_cx*std::exp(-1.0/tau*time_elapsed);  // mass
  INFO_WEIGHT_CY = W0_cy*std::exp(-1.0/tau*time_elapsed);  // MoI
}


void RattlePlanner::use_latest_info_gain() {
  /* Set the info gain weight in the cost function.
  */
  int i, j;
  for (i = 0; i < NY; i++) {  // number of measurements (matches number of W terms)
    for (j = 0; j < 4; j++) {  // number of terms to update
      if (i == j) {
        switch (j) {
          case 0:
            acadoVariables.W[i * NY + j] = INFO_WEIGHT_M;  // weight for mass info gain
            break;
          case 1:
            if (ground_.compare("true") == 0) {
              acadoVariables.W[i * NY + j] = INFO_WEIGHT_CX;  // weight for cx info gain
            }
            else {
              acadoVariables.W[i * NY + j] = INFO_WEIGHT_IXX;  // weight for ixx info gain
            }
            break;
          case 2:
            if (ground_.compare("true") == 0) {
              acadoVariables.W[i * NY + j] = INFO_WEIGHT_CY;  // weight for cy info gain
            }
            else {
              acadoVariables.W[i * NY + j] = INFO_WEIGHT_IYY;  // weight for iyy info gain
            }
            break;
          case 3:
            acadoVariables.W[i * NY + j] = INFO_WEIGHT_IZZ;  // weight for izz info gain
            break;
        }
      }
    }
  }
}


void RattlePlanner::shift_start_point() {
  /* As the local plan finds a path between two global plan setpoints, the goal point for the previous local plan
  becomes the start point for the next one
  */
  acadoVariables.x0[0] = acadoVariables.od[10];
  acadoVariables.x0[1] = acadoVariables.od[11];
  acadoVariables.x0[2] = acadoVariables.od[12];
  acadoVariables.x0[3] = acadoVariables.od[13];
  acadoVariables.x0[4] = acadoVariables.od[14];
  acadoVariables.x0[5] = acadoVariables.od[15];
  acadoVariables.x0[6] = acadoVariables.od[16];
  acadoVariables.x0[7] = acadoVariables.od[17];
  acadoVariables.x0[8] = acadoVariables.od[18];
  acadoVariables.x0[9] = acadoVariables.od[19];
  acadoVariables.x0[10] = acadoVariables.od[20];
  acadoVariables.x0[11] = acadoVariables.od[21];
  acadoVariables.x0[12] = acadoVariables.od[22];
}


void RattlePlanner::update_psi() {
  /* Update the psi integration variables
  */
  for (int i = 13; i < NX; i++) {
    acadoVariables.x0[i] = 0.0;
  }
}


void RattlePlanner::print_data() {
  /* Print acadoVariables
  */
  std::cout << "ACADO x0: " << acadoVariables.x0[0] << " " << acadoVariables.x0[1] << " "
    << acadoVariables.x0[2] << " " << acadoVariables.x0[3] << " " << acadoVariables.x0[4] << " "
    << acadoVariables.x0[5] << " " << acadoVariables.x0[6] << " " << acadoVariables.x0[7] << " "
    << acadoVariables.x0[8] << " " << acadoVariables.x0[9] << " " << acadoVariables.x0[10] << " "
    << acadoVariables.x0[11] << " " << acadoVariables.x0[12] << " " << std::endl;

  std::cout << "ACADO x_des: " << acadoVariables.od[10] << " " << acadoVariables.od[11] << " "
        << acadoVariables.od[12] << " " << acadoVariables.od[13] << " " << acadoVariables.od[14] << " "
        << acadoVariables.od[15] << " " << acadoVariables.od[16] << " " << acadoVariables.od[17] << " "
        << acadoVariables.od[18] << " " << acadoVariables.od[19] << " " << acadoVariables.od[20] << " "
        << acadoVariables.od[21] << " " << acadoVariables.od[22] << " " << std::endl;

  std::cout << "ACADO theta (params): " << acadoVariables.od[0] << " " << acadoVariables.od[1] << " "
      << acadoVariables.od[2] << " " << acadoVariables.od[3] << " " << acadoVariables.od[4] << " "
      << acadoVariables.od[5] << " " << acadoVariables.od[6] << " " << acadoVariables.od[7] << " "
      << acadoVariables.od[8] << " " << acadoVariables.od[9] << std::endl;

  std::cout << "ACADO W: ";
  for (int i = 0; i < NY; i++) {
    for (int j = 0; j < NY; j++) {
      if (i == j) {  // diagonal terms only
        std::cout << acadoVariables.W[i * NY + j] << " ";
      }
    }
  }
  std::cout << std::endl;

  // std::cout << "ACADO WN: ";
  // for (int i = 0; i < NYN; i++) {
  //   for (int j = 0; j < NYN; j++) {
  //     if (i == j) {  // diagonal terms
  //       std::cout << acadoVariables.WN[i*NYN+j] << " ";
  //     }
  //   }
  // }
  std::cout << std::endl;
}

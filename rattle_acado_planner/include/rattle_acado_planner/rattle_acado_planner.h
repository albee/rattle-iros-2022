#pragma once

// ACADO
#include <acado_common.h>
#include <acado_auxiliary_functions.h>

// C++ STL inclues
#include <sstream>
#include <string>
#include <memory>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

#include "rattle_acado_planner/params.h"

// Eigen for matrix mult
#include <Eigen/Dense>

#include <ros/ros.h>  // for time

#define NX ACADO_NX    // number of differential variables, [r(3) v(3) q(4) w(3) psi]
#define NOD ACADO_NOD  // number of online_data variables
#define NU ACADO_NU    // number of system inputs
#define NY ACADO_NY    // number of measurements per timestep
#define NYN ACADO_NYN  // number of measurements, N-th timestep
#define N ACADO_N      // number of timesteps
#define VERBOSE 1
#define GRANITE 1      // change to receive this parameter from the config file

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/*
OnlineData format, [N x 23] (body frame?):
  [mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3)]
u format [6 x 1] (body frame):
  [F_x F_y F_z T_x T_y T_z]
x format [13+31 x 1] (body frame):
  [r(3) v(3) q(4) w(3) psi(31)]
 */


class RattlePlanner {
 public:
  enum InitialModelMode{ground_truth_iss, ground_truth_ground, incorrect_low, incorrect_high};  // initial system model to use
  int initial_model_mode_ = ground_truth_iss;
  bool use_params_{false};  // {true, false}: use estimated param update callback?
  bool use_global_setpoints = true;  // if true, do NOT use EKF updates
  int max_RTI_iter_ = 5;  // based off estimated hardware runtime
  std::vector<double> Wx_{10, 10, 10, 5, 5, 5, 1, 1, 1, 1, 1, 1};  // x weighting
  std::vector<double> Wu_{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};  // u weighting
  std::vector<double> WN_{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};  // terminal weighting

  const int n_psi = NX - 13;
  static const int n_acado_online_data = 23;  // size of [mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3)]
  double online_data_vector[n_acado_online_data];
  
  // model parameters
  std::vector<Params> initial_mode_vec = {Params{9.58, 0.153, 0.143, 0.162}, Params{18.9715, 0.2517, 0.2517, 0.2517},
                                          Params{5.0, 0.075, 0.075, 0.075}, Params{20.0, 0.30, 0.30, 0.30}};
  Params params_model_ = initial_mode_vec[initial_model_mode_];  // mass Ixx Iyy Izz, use by system mode
  Params params_est_{0.0, 0.0, 0.0, 0.0};  // mass Ixx Iyy Izz, considered for model updates
  double mass_lb_ = 7.0;  // kg
  double inertia_lb_ = 0.10;  // N-m
  int iter_ = 0;

  std::string ground_ = "false";

  // TODO: fix trailing underscore
  std::vector<double> lin_des = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // [x y z vx vy vz]
  std::vector<double> orientation_des = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};  // [qx qy qz qw wx wy wz]
  int STATUS = 2;  // indicator of whether ACADO computations should be performed, {2=wait}
  
  double INFO_WEIGHT_M = 0.0;  // info weightings
  double INFO_WEIGHT_IXX = 0.0;
  double INFO_WEIGHT_IYY = 0.0;
  double INFO_WEIGHT_IZZ = 0.0;
  double INFO_WEIGHT_IXY = 0.0;
  double INFO_WEIGHT_IYZ = 0.0;
  double INFO_WEIGHT_IXZ = 0.0;
  double INFO_WEIGHT_CX = 0.0;
  double INFO_WEIGHT_CY = 0.0;
  double INFO_WEIGHT_CZ = 0.0;
  
  double time_elapsed;
  acado_timer t;

  RattlePlanner();
  ~RattlePlanner() {};

  ACADOvariables create_local_plan();
  void initialize_mpc();
  void print_data();
  void shift_start_point();
  void use_latest_info_gain();
  void assignWeightInfoGain(double time_elapsed);
  void update_online_data(Params* params);
  void get_cost_matrix();
  void acado_startup();
  void update_psi();
  double run_acado_RTI_iteration();
};

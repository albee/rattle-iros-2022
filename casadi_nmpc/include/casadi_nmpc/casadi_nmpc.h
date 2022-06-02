#ifndef CASADI_NMPC_NODE_H_
#define CASADI_NMPC_NODE_H_

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>

// Msgs
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/GraphState.h>
#include <ff_msgs/FamCommand.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/MultiArrayDimension.h>
#include <msg_conversions/msg_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <reswarm_msgs/ReswarmMsgMRPI.h>
#include <reswarm_msgs/ReswarmSrvMRPI.h>
#include <reswarm_msgs/ReswarmStatusPrimary.h>
#include <reswarm_msgs/ReswarmCasadiDebug.h>
#include <reswarm_msgs/ReswarmCasadiStatus.h>
#include <reswarm_msgs/RattleTestInstruct.h>
#include <param_est/Params.h>
#include <casadi_nmpc/params.h>

// CasADi
#include <casadi/casadi.hpp>
#include <cstddef>  // Includes of codegen shared library
#include <cstdlib>

// Custom includes
#include "casadi_nmpc/pd_attitude_controller.h"
#include "casadi_nmpc/eigen_msg.h"
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <string.h>
#include <iostream>

using namespace std; // sorry Pedro
using namespace Eigen;

namespace casadi_nmpc {
struct gains {
  // Standard MPC
  double Q_pos_factor;
  double Q_vel_factor;
  double R_factor;
  double QN_pos_factor;
  double QN_vel_factor;

  // Tube MPC
  double Q_pos_tube_factor;
  double Q_vel_tube_factor;
  double R_tube_factor;
  double QN_pos_tube_factor;
  double QN_vel_tube_factor;

  // ancillary controller
  double Q_pos_anc_factor;
  double Q_vel_anc_factor;
  double R_anc_factor;
};

static std::string CASADI_MPC_LIB = "libcasadi_mpc.so";
static std::string CASADI_ROBUST_TUBE_MPC_LIB = "libcasadi_robust_tube_mpc.so";

static std::string UC_BOUND_TOPIC = "/reswarm/uc_bound/uc_bound";  // always global
static std::string TOPIC_RESWARM_TUBE_MPC_MRPI = "/reswarm/tube_mpc/mrpi";  // always global

static std::string TOPIC_PARAM_EST = "mob/inertia_est";

static std::string TOPIC_RESWARM_STATUS = "reswarm/status";
static std::string TOPIC_RESWARM_TUBE_MPC_TRAJ = "reswarm/tube_mpc/traj";
static std::string TOPIC_RESWARM_TUBE_MPC_DEBUG = "reswarm/tube_mpc/debug";
static std::string TOPIC_RESWARM_CASADI_STATUS = "reswarm/casadi_nmpc/status";
static std::string TOPIC_RESWARM_TUBE_MPC_REG_SETPOINT = "reswarm/tube_mpc/reg_setpoint";
static std::string TOPIC_RATTLE_TEST_INSTRUCT = "/rattle/test_instruct";  // always global

class CasadiNMPCNodelet : public ff_util::FreeFlyerNodelet {
 public:
  CasadiNMPCNodelet() : ff_util::FreeFlyerNodelet(true) {}  // initialization of FreeFlyerNodelet
  ~CasadiNMPCNodelet() {}

  void Run();  // starts the main ROS loop

  // parameters and status info
  std::string control_mode_ = "inactive";  // standard: {track, track_tube, regulate, inactive}, debug: {debug, unit_test, unit_test_pd}
  std::string ground_ = "false";
  std::string sim_ = "false";

  bool use_params_{false};  // {true, false}: use estimated param update callback?
  enum GainMode{yaml_default, permissive_u, strict_u, cautious_tube_mpc};
  int gain_mode_ = yaml_default;  // {0, 1, 2, 3, ...}. 0 is always the YAML values.
  enum InitialModelMode{ground_truth_iss, ground_truth_ground, incorrect_low, incorrect_high};  // initial system model to use
  int initial_model_mode_ = ground_truth_iss;

  bool coord_ok_ = true;
  bool mrpi_finished_ = false;
  bool traj_finished_ = false;
  bool unit_test_complete_ = false;
  bool using_fallback_mrpi_ = false;
  bool regulate_lockout_ = false;  // stops rewswarm/status from updating control_mode_

  // ROS pubs and subs
  ros::Publisher pub_ctl_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_casadi_status_;
  ros::Publisher pub_mrpi_;

  ros::Subscriber sub_ekf_;
  ros::Subscriber sub_x_des_traj_;
  ros::Subscriber sub_uc_bound_;
  ros::Subscriber sub_status_;
  ros::Subscriber sub_reg_setpoint_;
  ros::Subscriber sub_param_est_;
  ros::Subscriber sub_rattle_instruct_;  // RATTLE configuration options
  std::shared_ptr<std::thread> thread_;  // for subscriber management

  // rates and timing: must match CasADi export!
  double T; // MPC time horizon, [s]
  int N; // MPC control intervals per time horizon (|x| is 1 greater)
  double MPC_rate_;  // MPC model's rate (N/T) [Hz]
  double MPC_dt_;  // MPC model's timestep period (1/MPC_rate_) [s]

  double traj_rate_;  // rate at which the trajectory is fed, [Hz]
  double control_dt_;  // rate at which the controller expects to be called---cannot be changed and should match traj_rate_, [s]
  double command_rate_ = 62.5;  // rate at which Astrobee firmware expects control inputs, met using zero-order hold [Hz]
  double tube_update_dt_ = 10.0; // rate at which tube MPC updates are requested with newest params[s]

  double casadi_comp_time_;  // dt of latest casadi computation
  double total_comp_time_;

  // model parameters
  std::vector<Params> initial_mode_vec = {Params{9.58, 0.153, 0.143, 0.162}, Params{18.9715, 0.2517, 0.2517, 0.2517},
                                          Params{5.0, 0.075, 0.075, 0.075}, Params{20.0, 0.30, 0.30, 0.30}};
  Params params_model_ = initial_mode_vec[initial_model_mode_];  // mass Ixx Iyy Izz, use by system mode
  Params params_est_{0.0, 0.0, 0.0, 0.0};  // mass Ixx Iyy Izz, considered for model updates
  double mass_lb_ = 7.0;  // kg
  double inertia_lb = 0.05;  // N-m

  // inputs
  pd::BackupController pd_control_;  // PD controller class
  Matrix<double, 3, 1> u_mag_; // [N]
  Matrix<double, 3, 1> torque_mag_;  // [N-m]
  double torque_x_ = 0.0;  // torque from ctl [N-m]
  double torque_y_ = 0.0;
  double torque_z_ = 0.0;
  Matrix<double, 6, 1> u_opt_;  // optimal control input [Fx, Fy, Fz, Tx, Ty, Tz]

  // trajectory book-keeping
  int HAVE_TRAJ = 0;
  int WAS_TRACK = 0;
  int N_traj_ = 0; // length of trajectory
  double dt_traj_ = 0.0;  // dt of trajecory
  int traj_idx_ = 0;  // current index of the trajectory to track
  ros::Time t_start_ ;
  double t_elapsed_;
  double t_prev_;  // rostime of previous
  Eigen::Vector3d r_RI_;  // TVR (reference) frame wrt inertial

  // estimates and x_des
  Matrix<double, 6, 1> x_real_;  // current translational state position [x1, x2, x3, x1d, x2d, x3d].T
  Matrix<double, 16, 1> x_real_complete_;  // entire x vector supplied by estimator [x y z  qx qy qz qw  vx vy vz  wx wy wz  ax ay az].T
  MatrixXd x_des_traj_N_;  // length of des traj
  // x_des := [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]
  MatrixXd eigen_x_des_traj_;        // trajectory in inertial frame. Updated with target attitude estimates.
  MatrixXd eigen_x_des_traj_init_;  // initial inertial frame trajectory

  // regulation
  Vector3f x0_;  // regulation translational state
  Vector4f a0_;  // regulation attitude state
  Matrix<double, 50, 14> eigen_x_des_traj_reg_; // regulation desired trajectory

  // mRPI
  Matrix<double, 6, 1> w_bound_ = MatrixXd::Zero(6, 1);  // uncertainty bound
  MatrixXd K_dr_ = MatrixXd::Zero(3, 6);  // default initialization
  MatrixXd Au_;
  MatrixXd bu_;
  MatrixXd AZ_;
  MatrixXd bZ_;

  // Tube MPC and MPC vars
  // QN indicate terminal weight. Q is state; R is input.
  std::vector<gains> gains_list_ground_;
  std::vector<gains> gains_list_iss_;

  // individual input weights
  double Q1;  // standard MPC
  double Q2;
  double Q3;
  double Q4;
  double Q5;
  double Q6;
  double R1;
  double R2;
  double R3;
  double QN1;
  double QN2;
  double QN3;
  double QN4;
  double QN5;
  double QN6;
  double Q1_T_;  // tube MPC
  double Q2_T_;
  double Q3_T_;
  double Q4_T_;
  double Q5_T_;
  double Q6_T_;
  double R1_T_;
  double R2_T_;
  double R3_T_;
  double QN1_T_;
  double QN2_T_;
  double QN3_T_;
  double QN4_T_;
  double QN5_T_;
  double QN6_T_;

  double Q_pos_anc_factor_;  // ancillary controller
  double Q_vel_anc_factor_;
  double R_anc_factor_;

  // CasADi variables
  casadi::DM dm_u_mag_;
  casadi::DM dm_m_;
  casadi::DM dm_Au_;
  casadi::DM dm_bu_;
  casadi::DM dm_AZ_;
  casadi::DM dm_bZ_;
  casadi::DM dm_K_dr_;
  casadi::DM dm_Q1_;
  casadi::DM dm_Q2_;
  casadi::DM dm_Q3_;
  casadi::DM dm_Q4_;
  casadi::DM dm_Q5_;
  casadi::DM dm_Q6_;
  casadi::DM dm_R1_;
  casadi::DM dm_R2_;
  casadi::DM dm_R3_;
  casadi::DM dm_QN1_;
  casadi::DM dm_QN2_;
  casadi::DM dm_QN3_;
  casadi::DM dm_QN4_;
  casadi::DM dm_QN5_;
  casadi::DM dm_QN6_;
  casadi::DM dm_Q1_T_;
  casadi::DM dm_Q2_T_;
  casadi::DM dm_Q3_T_;
  casadi::DM dm_Q4_T_;
  casadi::DM dm_Q5_T_;
  casadi::DM dm_Q6_T_;
  casadi::DM dm_R1_T_;
  casadi::DM dm_R2_T_;
  casadi::DM dm_R3_T_;
  casadi::DM dm_QN1_T_;
  casadi::DM dm_QN2_T_;
  casadi::DM dm_QN3_T_;
  casadi::DM dm_QN4_T_;
  casadi::DM dm_QN5_T_;
  casadi::DM dm_QN6_T_;

  // CasADi functions
  casadi::Function mpc_func_casadi_;  // C versions
  casadi::Function tube_mpc_func_casadi_;
  casadi::Function tube_mpc_func_serialized_;  // serialized graph versions
  casadi::Function mpc_func_serialized_;
  casadi::Function dynamics_func_serialized_;


  void Initialize(ros::NodeHandle* nh);
  void update_u_opt_(std::string control_mode_);
  void select_closest_setpoint();
  Vector3d calc_torques_PD(bool regulate);
  Vector3d call_pd_and_print();

  // timers
  void MPC_timer_callback(const ros::TimerEvent&);
  void command_timer_callback(const ros::TimerEvent&);
  void tube_update_timer_callback(const ros::TimerEvent&);

  // tests
  void run_debug();
  void unit_test_mpc();
  void unit_test_pd();
  void run_unit_test();

  // mpc calls
  std::tuple<Vector3d, Vector3d, Vector3d, Matrix<double, 6, 1>> call_tube_mpc_func_casadi();
  Vector3d call_mpc_func_casadi(bool regulate);
  Eigen::MatrixXd get_setpoints();
  void prepare_mrpi(Eigen::MatrixXd w, Eigen::MatrixXd u_max, double dt, double mass, double Q_pos_anc, double Q_vel_anc, double R_anc);

  // ROS pubs and subs
  void status_callback(const reswarm_msgs::ReswarmStatusPrimary::ConstPtr& msg);
  void ekf_callback(const ff_msgs::EkfState::ConstPtr msg);
  void w_bound_callback(const std_msgs::Float64MultiArray::ConstPtr uc_bound);
  void x_des_traj_callback(const std_msgs::Float64MultiArray::ConstPtr msg);
  void regulation_setpoint_callback(const geometry_msgs::Pose::ConstPtr& msg);
  void publish_casadi_status();
  void publish_mrpi();
  void publish_eigen_x_des_traj();
  void publish_debug(Matrix<double, 6, 1> u_t_idx, Vector3d u0_mpc, Vector3d u0_dr, Matrix<double, 6, 1> x_nom);
  void publish_ctl(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B);
  void update_parameters_callback(const param_est::Params::ConstPtr& msg);
  void rattle_instruct_callback(const reswarm_msgs::RattleTestInstruct::ConstPtr& msg);

  // utils
  void get_initial_regulation();
  void make_gains(gains gains_YAML);
  void switch_gains(int gain_mode);
  void set_mRPI_fallback_values();
  std::string quat2str(tf2::Quaternion q);
  void get_all_YAML_parameters();
  void update_regulation_setpoint(Vector3f x0, Vector4f a0);
  Eigen::Matrix3f q2dcm(const Vector4f &q);
  void read_traj_from_file(std::string traj_filename);
  casadi::DM eigen2dm(MatrixXd mat);
  void print_x_des_traj_(int idx);
  std::tuple <tf2::Vector3, tf2::Vector3> check_input_limits(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B);
  Eigen::MatrixXd evaluate_w(double m, double sigma_m);

  // interpolation
  Eigen::MatrixXd interpolate_traj(Eigen::MatrixXd x_traj, double t0, double t_step, double tf);
  Eigen::MatrixXd interpolate_state(Eigen::MatrixXd x0, Eigen::MatrixXd x1, double t0, double t1, double t_h);
  // quaternion
};
}  // end namespace casadi_nmpc
#endif  // CASADI_NMPC_NODE_H_

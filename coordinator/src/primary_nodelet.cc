/* primary_nodelet.cc

The primary coordinator, which derives from CoorindatorBase and adds methods for DMPC, RATTLE, and assembly tests.
*/

#include "coordinator/primary_nodelet.h"
#include "coordinator/primary_dmpc_methods.hpp"
#include "coordinator/primary_ooa_methods.hpp"
#include "coordinator/primary_rattle_methods.hpp"

/* ************************************************************************** */
void PrimaryNodelet::Initialize(ros::NodeHandle* nh) {
  /**
  * @brief This is called when the nodelet is loaded into the nodelet manager
  * 
  */
  // Create Multi-threaded NH
  MTNH = getMTNodeHandle();

  // Load Params
  load_params();

  // publishers
  pub_flight_mode_ = nh->advertise<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 1, true);  // FlightMode
  pub_status_ = nh->advertise<reswarm_msgs::ReswarmStatusPrimary>(TOPIC_RESWARM_STATUS, 5, true);
  pub_x_des_traj_ = nh->advertise<std_msgs::Float64MultiArray>(TOPIC_RESWARM_TUBE_MPC_TRAJ, 5, true);  // always global, full traj to send to tube MPC
  pub_uc_bound_ = nh->advertise<std_msgs::Float64MultiArray>(UC_BOUND_TOPIC, 5, true);  // always global
  pub_rattle_test_instruct_ = nh->advertise<reswarm_msgs::RattleTestInstruct>(RATTLE_TEST_INSTRUCT_TOPIC, 5, true);
  pub_reg_setpoint_ = nh->advertise<geometry_msgs::Pose>(TOPIC_RESWARM_TUBE_MPC_REG_SETPOINT, 5, true);
  
  // subscribers
  sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
    boost::bind(&PrimaryNodelet::flight_mode_callback, this, _1));  // flight mode getter
  sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
    boost::bind(&PrimaryNodelet::ekf_callback, this, _1));;
  sub_reswarm_test_number_ = nh->subscribe<reswarm_msgs::ReswarmTestNumber>(TOPIC_RESWARM_TEST_NUMBER, 5,
    boost::bind(&PrimaryNodelet::test_num_callback, this, _1));
  sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
    boost::bind(&PrimaryNodelet::flight_mode_callback, this, _1));  // flight mode setter
  sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
    boost::bind(&PrimaryNodelet::ekf_callback, this, _1));
  sub_info_status_ = nh->subscribe<reswarm_msgs::ReswarmInfoStatus>("reswarm/info_traj_status", 5,
    boost::bind(&PrimaryNodelet::info_status_callback, this, _1));
  sub_casadi_status_ = nh->subscribe<reswarm_msgs::ReswarmCasadiStatus>("reswarm/casadi_nmpc/status", 5,
    boost::bind(&PrimaryNodelet::casadi_status_callback, this, _1));
  sub_planner_status_ = nh->subscribe<reswarm_msgs::ReswarmPlannerStatus>("/reswarm/planner_lqrrrt/status", 5,
    boost::bind(&PrimaryNodelet::planner_status_callback, this, _1));
  sub_control_mode_ = nh->subscribe<std_msgs::String>(CONTROL_MODE_TOPIC, 5,
    boost::bind(&PrimaryNodelet::control_mode_callback, this, _1)); 
  sub_dmpc_status_ = nh->subscribe<reswarm_dmpc::DMPCTestStatusStamped>("reswarm/dmpc_status", 5,
    boost::bind(&PrimaryNodelet::dmpc_status_cb, this, _1));   
  sub_rattle_status_ = nh->subscribe<reswarm_msgs::ReswarmRattleStatus>("reswarm/rattle/status", 5,
    boost::bind(&PrimaryNodelet::rattle_status_callback, this, _1));     

  // services
  serv_ctl_enable_ = nh->serviceClient<std_srvs::SetBool>(SERVICE_GNC_CTL_ENABLE);

  // tracking points
  try{
    std::vector<double> param_data;
    nh->getParam("/reswarm/primary/point_a_granite", param_data);
    POINT_A_GRANITE = Eigen::Matrix<double, 7, 1>(param_data.data());  // init from std::vector param
    nh->getParam("/reswarm/primary/point_a_iss", param_data);
    POINT_A_ISS = Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/reswarm/primary/point_b_granite", param_data);
    POINT_B_GRANITE = Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/reswarm/primary/point_b_iss", param_data);
    POINT_B_ISS= Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/reswarm/primary/point_c_granite", param_data);
    POINT_C_GRANITE = Eigen::Matrix<double, 7, 1>(param_data.data());  
    nh->getParam("/reswarm/primary/point_c_iss", param_data);
    POINT_C_ISS = Eigen::Matrix<double, 7, 1>(param_data.data()); 
  }
  catch (const std::exception &exc) {
    std::cerr << exc.what() << std::endl;
    std::cout << "[PRIMARY_COORD]: Input parameters are invalid!" << std::endl;
  }

  // Pass control to Run method (activate timers and spin)
  NODELET_INFO_STREAM("[PRIMARY_COORD] Initialized.");
  thread_.reset(new std::thread(&CoordinatorBase::Run, this, nh));
}

/* ************************************************************************** */
void PrimaryNodelet::get_reswarm_status_msg(reswarm_msgs::ReswarmStatusPrimary& msg){
  /**
   * @brief Fills the ReswarmStatus message with the state of the private
   * variables. Published by a coordinator Timer.
   * Inputs: base_reswarm_status_ and primary_reswarm_status_
   * 
   */
  msg.stamp = ros::Time::now();
  msg.test_number = base_reswarm_status_.test_number;
  msg.default_control = base_reswarm_status_.default_control;
  msg.flight_mode = base_reswarm_status_.flight_mode;
  msg.test_finished = base_reswarm_status_.test_finished;
  msg.coord_ok = base_reswarm_status_.coord_ok;
  msg.control_mode = primary_reswarm_status_.control_mode;

  msg.regulate_finished = base_reswarm_status_.regulate_finished;

  // msg.uc_bound_activated = primary_reswarm_status_.reswarm_uc_bound_activated;
  msg.uc_bound_finished = primary_reswarm_status_.uc_bound_finished;

  msg.mrpi_finished = primary_reswarm_status_.mrpi_finished;
  msg.traj_finished = primary_reswarm_status_.traj_finished;
  msg.gain_mode = primary_reswarm_status_.reswarm_gain_mode;
  msg.activate_rattle = primary_reswarm_status_.activate_rattle;
  msg.lqrrrt_activated = primary_reswarm_status_.lqrrrt_activated;
  msg.lqrrrt_finished = primary_reswarm_status_.lqrrrt_finished;
  msg.traj_sent = primary_reswarm_status_.traj_sent;

  msg.info_traj_send = primary_reswarm_status_.info_traj_send;

  // DMPC status
  msg.solver_status = primary_reswarm_status_.solver_status;
  msg.cost_value = primary_reswarm_status_.cost_value;
  msg.kkt_value = primary_reswarm_status_.kkt_value;
  msg.sol_time = primary_reswarm_status_.sol_time;

   msg.description = primary_reswarm_status_.description;

  msg.rattle_use_params = primary_reswarm_status_.rattle_use_params;
  msg.rattle_weight_mode = primary_reswarm_status_.rattle_weight_mode;
  msg.rattle_initial_model_mode = primary_reswarm_status_.rattle_initial_model_mode;
}

/* ************************************************************************** */
void PrimaryNodelet::load_params(){
  // Get sim and ground flags
  std::string sim_str, ground_str;
  ros::param::get("/reswarm/sim", sim_str);
  sim_ = !std::strcmp(sim_str.c_str(), "true"); // convert to bool
  ros::param::get("/reswarm/ground", ground_str);
  ground_ = !std::strcmp(ground_str.c_str(), "true");  // convert to bool, 1 if it's true
  
  // regulation
  ros::param::getCached("/reswarm/primary/reg_time", reg_time_);
  ros::param::getCached("/reswarm/primary/x_start", x0_(0));
  ros::param::getCached("/reswarm/primary/y_start", x0_(1));
  ros::param::getCached("/reswarm/primary/z_start", x0_(2));
  ros::param::getCached("/reswarm/primary/qx_start", a0_(0));
  ros::param::getCached("/reswarm/primary/qy_start", a0_(1));
  ros::param::getCached("/reswarm/primary/qz_start", a0_(2));
  ros::param::getCached("/reswarm/primary/qw_start", a0_(3));
  ros::param::getCached("/reswarm/primary/pos_reg_thresh", pos_reg_thresh_);
  ros::param::getCached("/reswarm/primary/vel_reg_thresh", vel_reg_thresh_);
  ros::param::getCached("/reswarm/primary/att_reg_thresh", att_reg_thresh_);
  ros::param::getCached("/reswarm/primary/omega_reg_thresh", omega_reg_thresh_);

  // target pos IC (INERTIAL frame)
  ros::param::getCached("/reswarm/primary/targ_offset_x", targ_offset_(0));
  ros::param::getCached("/reswarm/primary/targ_offset_y", targ_offset_(1));
  ros::param::getCached("/reswarm/primary/targ_offset_z", targ_offset_(2));

  // translation of TVR frame w/r to ISS frame
  ros::param::getCached("/reswarm/primary/r_RI_ISS_x", r_RI_(0));
  ros::param::getCached("/reswarm/primary/r_RI_ISS_y", r_RI_(1));
  ros::param::getCached("/reswarm/primary/r_RI_ISS_z", r_RI_(2));

  // NODELET_INFO_STREAM("[PRIMARY COORD] Parameters Loaded...");
}

/* ************************************************************************** */
std::tuple<Eigen::MatrixXd, int> PrimaryNodelet::read_traj(std::string traj_path){
  /*
  Read in a trajectory from a file in DLR format:
  [t, position, velocity, linear accel, jerk]

  Params:
  traj_filename: the full path to the trajectory directory of interest

  Outputs:
  tuple of output_x Eigen::MatrixXd and and int for number of setpoints.
  Automatically publishes to controller
  */

  // ros::param::getCached("/td/chaser_traj_file", traj_path);  // filename to read in

  int num_setpoints;
  Eigen::MatrixXd output_x;
  Eigen::MatrixXd output_u;

  // Either choose trajectory from motion planner or from predetermined trajectory file
  output_x = get_traj_from_file_x<Eigen::MatrixXd>(traj_path);
  // output_u = dlr::get_dlr_output_u<MatrixXd>(traj_filename_);
  num_setpoints = output_x.rows();

  Eigen::MatrixXd eigen_x_des_traj = output_x;

  // translate from TVR frame to ISS frame (don't do this for tracking tests already in ISS coordinates!)
  // if (ground_.compare("true") != 0) {
  //   for (int i = 0; i < num_setpoints; i++) {
  //     eigen_x_des_traj_(i,1) = eigen_x_des_traj_(i,1) + r_RI_(0);
  //     eigen_x_des_traj_(i,2) = eigen_x_des_traj_(i,2) + r_RI_(1);
  //     eigen_x_des_traj_(i,3) = eigen_x_des_traj_(i,3) + r_RI_(2);
  //   }
  // }

  send_traj_to_controller(eigen_x_des_traj);  // send the full trajectory off to any controllers waiting for it

  return std::make_tuple(output_x, num_setpoints);
}


/* ************************************************************************** */
void PrimaryNodelet::send_traj_to_controller(Eigen::MatrixXd eigen_x_des_traj){
  /* Send a reference trajectory to the controller
  */
  // IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
  std::string sep = "\n----------------------------------------\n";
  NODELET_DEBUG_STREAM("Sending trajectory to casadi_nmpc... ");
  // NODELET_DEBUG_STREAM(sep << eigen_x_des_traj_ << sep);

  std_msgs::Float64MultiArray eigen_x_des_traj_msg;
  tf::matrixEigenToMsg(eigen_x_des_traj, eigen_x_des_traj_msg);

  pub_x_des_traj_.publish(eigen_x_des_traj_msg);
}


void PrimaryNodelet::pub_reg_setpoint(Eigen::MatrixXd reg_setpoint_){
  /* Convenience wrapper to generate msg from Eigen::MatrixXd
  */
  geometry_msgs::Pose msg;
  msg.position.x = reg_setpoint_(0);
  msg.position.y = reg_setpoint_(1);
  msg.position.z = reg_setpoint_(2);
  msg.orientation.x = reg_setpoint_(3);
  msg.orientation.y = reg_setpoint_(4);
  msg.orientation.z = reg_setpoint_(5);
  msg.orientation.w = reg_setpoint_(6);

  pub_reg_setpoint_.publish(msg);
}

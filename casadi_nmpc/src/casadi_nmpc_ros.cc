/*
# casadi_nmpc_ros.cc

ROS interface (pubs and subs)of casadi_nmpc.

Keenan Albee
MIT Space Systems Lab, 06.23.21
*/
#include "casadi_nmpc/casadi_nmpc.h"

namespace casadi_nmpc{
/* ************************************************************************** */
void CasadiNMPCNodelet::status_callback(const reswarm_msgs::ReswarmStatusPrimary::ConstPtr& msg) {
  /* Get /reswarm/status and decide and accpet it IF we have no forced regulation
  */
  // std::cout << regulate_lockout_ << " " << control_mode_ << std::endl;
  if (regulate_lockout_ != true){  // if we have not internally set control_mode_ to regulate
    control_mode_ = msg->control_mode;
    gain_mode_ = msg->gain_mode;
    switch_gains(gain_mode_);  // set the desired 
  }
}


/* ************************************************************************** */
void CasadiNMPCNodelet::rattle_instruct_callback(const reswarm_msgs::RattleTestInstruct::ConstPtr& msg)  {
    /* Configuration options for RATTLE.
    @msg USE_PARAMS: use paramter updates {0, 1}
    @msg INITIAL_MODEL_MODE: mode to use for calc, see enum
    */
    use_params_ = msg->USE_PARAMS;
    initial_model_mode_ = msg->INITIAL_MODEL_MODE;

    params_model_ = initial_mode_vec[initial_model_mode_];
}


/* ************************************************************************** */
void CasadiNMPCNodelet::regulation_setpoint_callback(const geometry_msgs::Pose::ConstPtr& msg)  {
    /* Get parameters specifying where to start the regulation
    */
    x0_(0) = msg->position.x;
    x0_(1) = msg->position.y;
    x0_(2) = msg->position.z;
    a0_(0) = msg->orientation.x;
    a0_(1) = msg->orientation.y;
    a0_(2) = msg->orientation.z;
    a0_(3) = msg->orientation.w;

    update_regulation_setpoint(x0_, a0_);
}


/* ************************************************************************** */
void CasadiNMPCNodelet::ekf_callback(const ff_msgs::EkfState::ConstPtr msg) {
  /* The `gnc/ekf` subscriber callback
  Called at 62.5 Hz; updates Chaser state.
  x_real_complete_ = [x y z  qx qy qz qw  vx vy vz  wx wy wz  ax ay az]
  */
  float qx = msg->pose.orientation.x;
  float qy = msg->pose.orientation.y;
  float qz = msg->pose.orientation.z;
  float qw = msg->pose.orientation.w;
  // check if EKF message is 0 before populating state (or just set it if regulate)
  if ((qx != 0 || qy != 0 || qz != 0 || qw != 0) || (control_mode_.compare("regulate") == 0)) {
    x_real_complete_(0) = msg->pose.position.x;
    x_real_complete_(1) = msg->pose.position.y;
    x_real_complete_(2) = msg->pose.position.z;
    x_real_complete_(3) = msg->pose.orientation.x;
    x_real_complete_(4) = msg->pose.orientation.y;
    x_real_complete_(5) = msg->pose.orientation.z;
    x_real_complete_(6) = msg->pose.orientation.w;
    x_real_complete_(7) = msg->velocity.x;
    x_real_complete_(8) = msg->velocity.y;
    x_real_complete_(9) = msg->velocity.z;
    x_real_complete_(10) = msg->omega.x;
    x_real_complete_(11) = msg->omega.y;
    x_real_complete_(12) = msg->omega.z;
    x_real_complete_(13) = 0.0;
    x_real_complete_(14) = 0.0;
    x_real_complete_(15) = 0.0;

    x_real_(0) = x_real_complete_(0);
    x_real_(1) = x_real_complete_(1);
    x_real_(2) = x_real_complete_(2);
    x_real_(3) = x_real_complete_(7);
    x_real_(4) = x_real_complete_(8);
    x_real_(5) = x_real_complete_(9);
  }
}


void CasadiNMPCNodelet::update_parameters_callback(const param_est::Params::ConstPtr& msg) {
  /* The `mob/inertia_est` subscriber callback.
  Keep updating the mass as long as it is feasible.
  */
  if (use_params_) {
    params_est_.mass = msg->inertia.m;
    params_est_.sigma[0] = msg->Cov[0];

    if (params_est_.mass > mass_lb_ ) {
      params_model_.mass = params_est_.mass;
    }
    params_model_.sigma[0] = msg->Cov[0];
  }
}


/* ************************************************************************** */
void CasadiNMPCNodelet::w_bound_callback(const std_msgs::Float64MultiArray::ConstPtr uc_bound) {
  /* Get the latest w_bound/uc_bound. Must be called before tube MPC starts! Makes a service call to z_poly_calc.
  @param uc_bound: contains float64multiarray of [4, 3], of maxes and mins on the uncertainty bound.
  */
  int i = uc_bound->layout.dim[0].size;
  int j = uc_bound->layout.dim[1].size;
  std::vector<double> data = uc_bound->data; // copy constructor of underlying data
  MatrixXd w_bound_charles_(4, 3);
  w_bound_charles_ = Eigen::Map<Eigen::MatrixXd>(data.data(), i, j);  // std::vector --> Eigen
  w_bound_ << w_bound_charles_(0), w_bound_charles_(1), w_bound_charles_(2), w_bound_charles_(6), w_bound_charles_(7), w_bound_charles_(8);

  prepare_mrpi(w_bound_, u_mag_, MPC_dt_, params_model_.mass, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr and Au and bu ready
}


/* ************************************************************************** */
void CasadiNMPCNodelet::x_des_traj_callback(const std_msgs::Float64MultiArray::ConstPtr msg) {
  /* The `reswarm/tube_mpc/traj` subscriber callback
  Called by coordinator_nodelet (or otherwise). This is the full reference trajectory.

  New: if called multiple times, the trajectory will be *appended* to the current ref traj!
  It is assumed these additional trajectories use the same rate.

  x_des_traj = [t, position, velocity, linear accel, jerk]
  eigen_x_des_traj_ = [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]
  */
  int i = msg->layout.dim[0].size;
  int j = msg->layout.dim[1].size;
  std::vector<double> data = msg->data; // copy constructor of underlying data

  Eigen::MatrixXd eigen_x_des_traj_tmp;
  Eigen::MatrixXd eigen_x_des_traj_append;
  int last_row;
  double t0;
  double tf;
  double dt;

  IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
  std::string sep = "\n----------------------------------------\n";

  if (HAVE_TRAJ == 0) {
    NODELET_INFO_STREAM("x_des_traj callback initiated...");
    // NOTE: default ROS message conversion flips row and column order!!!
    eigen_x_des_traj_tmp = Eigen::Map<Eigen::MatrixXd>(data.data(), j, i).transpose();  // std::vector --> Eigen
    last_row = eigen_x_des_traj_tmp.rows() - 1;
    t0 = eigen_x_des_traj_tmp.row(0)(0);
    tf = eigen_x_des_traj_tmp.row(last_row)(0);
    dt = control_dt_;  // nominally 0.2 [s]

    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_tmp.leftCols(4) << sep);

    eigen_x_des_traj_ = interpolate_traj(eigen_x_des_traj_tmp, t0, dt, tf);  // interpolate!

    eigen_x_des_traj_init_ = eigen_x_des_traj_;  // std::vector --> Eigen
    N_traj_ = eigen_x_des_traj_.rows();

    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_tmp.leftCols(4) << sep);
    // NODELET_INFO_STREAM(eigen_x_des_traj_.rows() << " " << eigen_x_des_traj_.cols());
    // IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    // std::string sep = "\n----------------------------------------\n";
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.row(0).format(HeavyFmt) << sep);
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.row(1).format(HeavyFmt) << sep);
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.row(2).format(HeavyFmt) << sep);
    NODELET_INFO_STREAM("...traj read-in complete.");

    dt_traj_ = eigen_x_des_traj_(1, 0) - eigen_x_des_traj_(0, 0);
    traj_rate_ = 1.0/dt_traj_;
    HAVE_TRAJ = 1;
  }
  else {
    NODELET_INFO_STREAM("x_des_traj callback appended...");
    // NOTE: default ROS message conversion flips row and column order!!!
    eigen_x_des_traj_tmp = Eigen::Map<Eigen::MatrixXd>(data.data(), j, i).transpose();  // std::vector --> Eigen
    last_row = eigen_x_des_traj_tmp.rows() - 1;
    t0 = eigen_x_des_traj_tmp.row(0)(0);
    tf = eigen_x_des_traj_tmp.row(last_row)(0);
    dt = control_dt_;  // nominally 0.2 [s]

    // NODELET_INFO_STREAM(eigen_x_des_traj_tmp.rows() << " " << eigen_x_des_traj_tmp.cols());
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_tmp.bottomRows(3).format(HeavyFmt) << sep);

    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_tmp.leftCols(4) << sep);

    eigen_x_des_traj_append = interpolate_traj(eigen_x_des_traj_tmp, t0, dt, tf);  // interpolate!

    eigen_x_des_traj_init_ = eigen_x_des_traj_append;
    eigen_x_des_traj_tmp = eigen_x_des_traj_;

    eigen_x_des_traj_.resize(eigen_x_des_traj_.rows()+eigen_x_des_traj_append.rows(), eigen_x_des_traj_.cols());
    eigen_x_des_traj_ << eigen_x_des_traj_tmp, eigen_x_des_traj_append;

    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.leftCols(4) << sep);

    // NODELET_INFO_STREAM(eigen_x_des_traj_.rows() << " " << eigen_x_des_traj_.cols());
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.bottomRows(3).format(HeavyFmt) << sep);

    N_traj_ += eigen_x_des_traj_append.rows();


    NODELET_INFO_STREAM("...traj read-in complete.");
  }

  regulate_lockout_ = false;  // we can accept new tracking modes again

  // std::cout << eigen_x_des_traj_.leftCols(4) << std::endl;
}


/* ************************************************************************** */
void CasadiNMPCNodelet::publish_casadi_status( ) {
  reswarm_msgs::ReswarmCasadiStatus msg;
  msg.stamp = ros::Time::now();
  msg.coord_ok = coord_ok_;
  msg.mrpi_finished = mrpi_finished_;
  msg.traj_finished = traj_finished_;
  msg.control_mode = control_mode_;
  pub_casadi_status_.publish(msg);
}


/* ************************************************************************** */
void CasadiNMPCNodelet::publish_mrpi( ) {
  reswarm_msgs::ReswarmMsgMRPI mrpi_msg;

  std_msgs::Float64MultiArray K_msg;
  tf::matrixEigenToMsg(K_dr_, K_msg);  // Eigen --> msg

  std_msgs::Float64MultiArray Au_msg;
  tf::matrixEigenToMsg(Au_, Au_msg);  // Eigen --> msg

  std_msgs::Float64MultiArray bu_msg;
  tf::matrixEigenToMsg(bu_, bu_msg);  // Eigen --> msg

  std_msgs::Float64MultiArray AZ_msg;
  tf::matrixEigenToMsg(AZ_, AZ_msg);  // Eigen --> msg

  std_msgs::Float64MultiArray bZ_msg;
  tf::matrixEigenToMsg(bZ_, bZ_msg);  // Eigen --> msg

  std_msgs::Float64MultiArray w_bound_msg;
  tf::matrixEigenToMsg(w_bound_, w_bound_msg);  // Eigen --> msg

  std_msgs::Bool fallback_msg;
  fallback_msg.data = using_fallback_mrpi_;

  mrpi_msg.K  = K_msg;
  mrpi_msg.Au = Au_msg;
  mrpi_msg.bu = bu_msg;
  mrpi_msg.AZ = AZ_msg;
  mrpi_msg.bZ = bZ_msg;
  mrpi_msg.w_bound = w_bound_msg;
  mrpi_msg.using_fallback_mrpi = fallback_msg;

  pub_mrpi_.publish(mrpi_msg);
}


/* ************************************************************************** */
void CasadiNMPCNodelet::publish_debug(Matrix<double, 6, 1> u_t_idx, Vector3d u0_mpc, Vector3d u0_dr,
  Matrix<double, 6, 1> x_nom) {
  reswarm_msgs::ReswarmCasadiDebug msg;
  msg.header.stamp = ros::Time::now();

  // Translation control (body frame)
  msg.wrench.force.x = u_t_idx[0];
  msg.wrench.force.y = u_t_idx[1];
  msg.wrench.force.z = u_t_idx[2];  // 3D only

  msg.wrench.torque.x = u_t_idx[3];  // 3D only
  msg.wrench.torque.y = u_t_idx[4];  // 3D only
  msg.wrench.torque.z = u_t_idx[5];

  msg.u0_mpc.x = u0_mpc[0];
  msg.u0_mpc.y = u0_mpc[1];
  msg.u0_mpc.z = u0_mpc[2];

  msg.u0_dr.x = u0_dr[0];
  msg.u0_dr.y = u0_dr[1];
  msg.u0_dr.z = u0_dr[2];

  std_msgs::Float64MultiArray x_nom_msg;
  tf::matrixEigenToMsg(x_nom, x_nom_msg);  // Eigen --> msg
  msg.x_nom = x_nom_msg;

  msg.casadi_comp_time.data = casadi_comp_time_;
  msg.total_comp_time.data = total_comp_time_;

  msg.control_mode.data = control_mode_;

  msg.Q_pos_factor = Q1;
  msg.Q_vel_factor = Q4;
  msg.R_factor = R1;
  msg.QN_pos_factor = QN1;
  msg.QN_vel_factor = QN4;

  msg.Q_pos_tube_factor = Q1_T_;
  msg.Q_vel_tube_factor = Q4_T_;
  msg.R_tube_factor = R1_T_;
  msg.QN_pos_tube_factor = QN1_T_;
  msg.QN_vel_tube_factor = QN4_T_;

  msg.Q_pos_anc_factor = Q_pos_anc_factor_;
  msg.Q_vel_anc_factor = Q_vel_anc_factor_;
  msg.R_anc_factor = R_anc_factor_;

  msg.T = T;
  msg.N = N;
  msg.control_dt = control_dt_;

  pub_debug_.publish(msg);
}

/* ************************************************************************** */
void CasadiNMPCNodelet::publish_ctl(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B) {
  /* Publishes caclulated forces/torques to the FAM.

  Inputs:
  F_xyz_B: translational forces in the BODY frame
  T_xyz_B: attitude torques in the BODY frame

  Outputs:
  Publishes forces/torques on gnc/ctl/command at 62.5 Hz BODY frame
  */
  ff_msgs::FamCommand famcmd;

  famcmd.header.stamp = ros::Time::now();

  // Translation control (body frame)
  famcmd.wrench.force.x = F_xyz_B[0];
  famcmd.wrench.force.y = F_xyz_B[1];
  famcmd.wrench.force.z = F_xyz_B[2];  // 3D only

  // Attitude control (body frame)
  famcmd.wrench.torque.x = T_xyz_B[0];  // 3D only
  famcmd.wrench.torque.y = T_xyz_B[1];  // 3D only
  famcmd.wrench.torque.z = T_xyz_B[2];

  pub_ctl_.publish(famcmd);
}
} // end namespace casadi_nmpc
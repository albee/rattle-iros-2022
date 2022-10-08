/*
# casadi_nmpc_nodelet.cc

Combines translational tube MPC output with PD attitude control provided by backup_controller.

control_mode_ is the most important parameter that defines what state the controller is in.
MPC can be in mode {track, track_tube, regulate, inactive, debug, unit_test, unit_test_pd}, see Run().

Note: Actuator saturation due to both controllers' computation is currently NOT considered!

Keenan Albee
MIT Space Systems Lab, 05.21.21
*/
#include "casadi_nmpc/casadi_nmpc.h"

typedef Matrix<float, 1, 7> state_type; // type for regulation state vector, [x y z qx qy qz qw]


namespace casadi_nmpc {
  /* ************************************************************************** */
  void CasadiNMPCNodelet::MPC_timer_callback(const ros::TimerEvent&) {
    /* Main control loop. Run MPC at 1/control_dt_.
    Assumes w_bound_ and x_des_traj_ have been set before running.

    control_mode_ for MPC can be: {track, track_tube, regulate, inactive}, or {debug}
    WAS_TRACK:= {0, 1}, 1 means was previously tracking
    HAVE_TRAJ:= {0, 1}, 1 means a trajectory is loaded in
    */

    // reset trajectory if stopped
    if ((control_mode_.compare("track") != 0 && control_mode_.compare("track_tube") != 0) && WAS_TRACK) {
      WAS_TRACK = 0;
      HAVE_TRAJ = 0;
      traj_idx_ = 0;
      traj_finished_ = false;
    }

    // check if debug mode
    if (control_mode_.compare("debug") == 0) {
      run_debug();
      control_mode_ = "inactive";
    }

    // main control loop
    if (control_mode_.compare("inactive") == 0) {
      // NODELET_INFO_STREAM("MPC is not active..." << control_mode_);
      u_opt_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    else if (control_mode_.compare("regulate") == 0) {  // regulate (assumes setpoints are set)
      // NODELET_INFO_STREAM("casadi_nmpc is regulating...");
      update_u_opt_(control_mode_);
    }
    else if (control_mode_.compare("track") == 0 || control_mode_.compare("track_tube") == 0) {  // Standard MPC or Tube MPC
      // NODELET_INFO_STREAM("final: " << eigen_x_des_traj_.block(N_traj_-1, 1, 1, 3).cast<float>());
      auto tic_total = std::chrono::high_resolution_clock::now();
      if (WAS_TRACK == 0) {  // start the time count
        t_start_ = ros::Time::now();
      }
      WAS_TRACK = 1;

      // set regulation to latest state estimate
      Vector3f x0;
      Vector4f a0;
      x0 << x_real_complete_(0), x_real_complete_(1), x_real_complete_(2);  // position
      a0 << x_real_complete_(3), x_real_complete_(4), x_real_complete_(5), x_real_complete_(6);  // attitude, qx qy qz qw
      update_regulation_setpoint(x0, a0);  // update regulation in case we swap over

      if (HAVE_TRAJ != 1) {  // do we actually have a trajectory?
        NODELET_INFO_STREAM("Trajectory has not been set!");
        control_mode_ = "inactive";
      }
      else if (traj_idx_ >= N_traj_) {  // are we finished with the trajectory?
        NODELET_INFO_STREAM("Trajectory complete! Swapping to regulation.");
        // set regulation to final point of traj
        control_mode_ = "regulate";
        Vector3f x0;
        x0 << float(eigen_x_des_traj_(N_traj_-1, 1)), float(eigen_x_des_traj_(N_traj_-1, 2)), float(eigen_x_des_traj_(N_traj_-1, 3));  // final [x, y, z]
        Vector4f a0;
        a0 << float(eigen_x_des_traj_(N_traj_-1, 7)), float(eigen_x_des_traj_(N_traj_-1, 8)), float(eigen_x_des_traj_(N_traj_-1, 9)),
              float(eigen_x_des_traj_(N_traj_-1, 10)); // final [qx, qy, qz, qw]
        update_regulation_setpoint(x0, a0);
        traj_finished_ = true;
        regulate_lockout_ = true;  // stop rattle/status from forcing an update! only okay if new traj received
      }
      else {  // use MPC or Tube MPC
        // Update inertial frame trajectory if getting target attitude estimates
        // NODELET_INFO_STREAM("Casadi model is:"  << params_model_);
        update_u_opt_(control_mode_);
        auto toc_total = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> tictoc = toc_total - tic_total;
        total_comp_time_ = tictoc.count();
        // NODELET_INFO_STREAM("total_comp_time_: " << total_comp_time_);
      }
    }

    // update /rattle/casadi_status once every loop
    publish_casadi_status();
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::command_timer_callback(const ros::TimerEvent&) {
    /* Publish the latest available u_opt_
    */
    // NODELET_INFO_STREAM("Command callback..." << u_opt_[0] << " " << u_opt_[1] << " " << u_opt_[2] );

    if (control_mode_.compare("inactive") != 0) {
      // Convert to body frame on every call
      tf2::Vector3 F_xyz_I(u_opt_[0], u_opt_[1], u_opt_[2]);  // must convert to Body frame
      tf2::Vector3 T_xyz_B(u_opt_[3], u_opt_[4], u_opt_[5]);

      tf2::Quaternion q_BI;  // convert Inertial to Body
      tf2::Quaternion q_IB(x_real_complete_(3), x_real_complete_(4), x_real_complete_(5), x_real_complete_(6));  // Body wrt Inertial
      q_BI = q_IB.inverse();
      q_BI.normalize();
      tf2::Vector3 F_xyz_B = tf2::Transform(q_BI)*F_xyz_I;

      std::tie(F_xyz_B, T_xyz_B) = check_input_limits(F_xyz_B, T_xyz_B);  // sanity check on input limits---should already be accomplished

      CasadiNMPCNodelet::publish_ctl(F_xyz_B, T_xyz_B);
    }
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::tube_update_timer_callback(const ros::TimerEvent&) {
    /* Update the tube MPC conservativeness based on latest model.
    Only update if we're using tube MPC and receiving parameter updates.
    */
    if (control_mode_.compare("track_tube") == 0 && use_params_) {
      w_bound_ = evaluate_w(params_model_.mass, params_model_.sigma[0]);
      prepare_mrpi(w_bound_, u_mag_, MPC_dt_, params_model_.mass, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr and Au and bu ready
    }
  }


  /* ************************************************************************** */
  Eigen::MatrixXd CasadiNMPCNodelet::evaluate_w(double m, double sigma_m) {
    /*Evaluate the approximate uncertainty bound given a mean mass value and its variance

    @param mass: \hat{mass}
    @param sigma_m: mass variance
    @return w_bound: updated w_bound
    */
    Eigen::MatrixXd w_bound;

    // select 2-sigma values for mass
    double m_low = m - 2*sqrt(sigma_m);
    if (m_low < 0.0){
      m_low = 1.0;
    }

    double m_high = m + 2*sqrt(sigma_m);

    // evaluate x+ for nominal, max inputs
    Eigen::MatrixXd x = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd u = u_mag_;

    // std::cout << m_low << " " << m_high << " " << u << " " << x;

    std::vector<casadi::DM> casadi_args = {eigen2dm(x), eigen2dm(u), casadi::DM{m}};  // input vector to CasADi function
    std::vector<casadi::DM> casadi_out = dynamics_func_serialized_(casadi_args);
    std::cout << casadi_out << std::endl;
    auto vector_out = static_cast<std::vector<double>>(casadi_out[0]);
    Eigen::MatrixXd x0 = Eigen::Matrix<double, 6, 1>(vector_out.data());
  
    // evaluate x+ for extreme values, max inputs
    casadi_args = {eigen2dm(x), eigen2dm(u), casadi::DM{m_low}};  // input vector to CasADi function
    casadi_out = dynamics_func_serialized_(casadi_args);
    vector_out = static_cast<std::vector<double>>(casadi_out[0]);
    Eigen::MatrixXd x_low = Eigen::Matrix<double, 6, 1>(vector_out.data());

    casadi_args = {eigen2dm(x), eigen2dm(u), casadi::DM{m_high}};  // input vector to CasADi function
    casadi_out = dynamics_func_serialized_(casadi_args);
    vector_out = static_cast<std::vector<double>>(casadi_out[0]);
    Eigen::MatrixXd x_high = Eigen::Matrix<double, 6, 1>(vector_out.data());

    Eigen::MatrixXd m1 = (x_high - x0).cwiseAbs();
    Eigen::MatrixXd m2 = (x_low - x0).cwiseAbs();

    w_bound = m1.cwiseMax(m2);
    // std::cout << "before: " << w_bound << std::endl;
    // w_bound(0) += 0.001;  // maintain a base level of uncertainty
    // w_bound(1) += 0.001;
    // w_bound(2) += 0.001;
    // w_bound(3) += 0.0005;
    // w_bound(4) += 0.0005;
    // w_bound(5) += 0.0005;
    // std::cout << "after: " << w_bound << std::endl;
    return w_bound;
  }


  /* ************************************************************************** */
  std::tuple <tf2::Vector3, tf2::Vector3> CasadiNMPCNodelet::check_input_limits(tf2::Vector3 F_xyz_B, tf2::Vector3 T_xyz_B) {
    /* Sanity check on commanded forces/torques. These constraints should already be enforced,
    but this step is a second check before commanded the FAM.

    F_xyz_B: force in body frame
    T_xyz_B: torque in body frame
    */
    // force check
    double F_TOL = 0.0;  // no tolerance

    for (int i = 0; i < 3; i++) {
      if (F_xyz_B[i] > (u_mag_(i) + F_TOL)) {
        F_xyz_B[i] = u_mag_(i);
      }
      else if (F_xyz_B[i] < (-u_mag_(i) - F_TOL)) {
        F_xyz_B[i] = -u_mag_(i);
      }
    }

    // torque check
    for (int i = 0; i < 3; i++) {
      if (T_xyz_B[i] > torque_mag_(i)) {
        T_xyz_B[i] = torque_mag_(i);
      }
      else if (T_xyz_B[i] < -torque_mag_(i))  {
        T_xyz_B[i] = -torque_mag_(i);
      }
    }

    return std::make_tuple(F_xyz_B, T_xyz_B);
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::update_u_opt_(std::string control_mode_) {
    /* Update u_opt_ with both CasADi translation and PD attitude control.
    */
    Vector3d u_forces;
    Vector3d u_torques;

    Vector3d u0_mpc = Vector3d::Zero();
    Vector3d u0_dr = Vector3d::Zero();
    Matrix<double, 6, 1> x_nom = Matrix<double, 6, 1>::Zero();

    // Tube MPC
    if (control_mode_.compare("track_tube") == 0) {
      /// (1) Compute the nominal MPC input INCLUDING ancillary
      std::tie(u_forces, u0_mpc, u0_dr, x_nom) = call_tube_mpc_func_casadi();
      u_torques = calc_torques_PD(false);
      u_opt_ << u_forces, u_torques;

      // NODELET_INFO_STREAM(
      //    "\n******************\n"
      // << "TUBE MPC:"
      // << "\ntraj_idx_: " << traj_idx_
      // << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_, 1, 1, 3)
      // << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
      // << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
      // << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      // << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
      // << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      // << "\nu_forces: " << u_forces.transpose()
      // << "\nu_torques: " << u_torques.transpose()
      // << "\n******************");
    }

    // Standard MPC
    else if (control_mode_.compare("track") == 0){
      u_forces = call_mpc_func_casadi(false);
      u_torques = calc_torques_PD(false);
      u_opt_ << u_forces, u_torques;

      // NODELET_INFO_STREAM(
      //    "\n******************\n"
      // << "STANDARD MPC:"
      // << "\ntraj_idx_: " << traj_idx_
      // << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_, 1, 1, 3)
      // << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
      // << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
      // << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      // << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
      // << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      // << "\nu_forces: " << u_forces.transpose()
      // << "\nu_torques: " << u_torques.transpose()
      // << "\n******************");
    }

    // Regulation (using Standard MPC)
    else if (control_mode_.compare("regulate") == 0) {
      u_forces = call_mpc_func_casadi(true);
      u_torques = calc_torques_PD(true);
      u_opt_ << u_forces, u_torques;

      // NODELET_INFO_STREAM(
      //      "\n******************\n"
      //   << "REGULATE:"
      //   << "traj_idx_: " << traj_idx_ << " N_traj_: " << N_traj_ << " t_elapsed_: " << t_elapsed_
      //   << "\npos_des: " << eigen_x_des_traj_reg_.block(traj_idx_, 1, 1, 3)
      //   << "\npos_real: " << x_real_complete_.segment(0, 3).transpose()
      //   << "\nquat_des: " << eigen_x_des_traj_reg_.block(traj_idx_, 7, 1, 4)
      //   << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      //   << "\nw_des: " << eigen_x_des_traj_reg_.block(traj_idx_, 11, 1, 3)
      //   << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      //   << "\nu_forces: " << u_forces.transpose()
      //   << "\nu_torques: " << u_torques.transpose()
      //   << "\n******************");
    }

    // note time, increment traj_idx_
    t_elapsed_ = ros::Time::now().toSec() - t_start_.toSec();

    // don't increment traj_idx_ if we're regulating; otherwise, increment
    if (control_mode_.compare("regulate") != 0) {
      traj_idx_ += 1;  
      // NODELET_INFO_STREAM("timing: " << t_elapsed_ << " " << traj_idx_*control_dt_ << " " << traj_idx_);
      if (casadi_comp_time_ > control_dt_) {
        select_closest_setpoint();  // select closest setpoint rather than use delayed one (if CasADi is slow)
      }
    }

    // publish post-processing info if we're tracking or regulating
    if (control_mode_.compare("track") == 0 || control_mode_.compare("track_tube") == 0 ||
      control_mode_.compare("regulate") == 0) {
      publish_debug(u_opt_, u0_mpc, u0_dr, x_nom);  // to get info at MPC rate
    }
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::select_closest_setpoint() {
    /* Select the closest setpoint in the event traj_idx is not updated on time.
    */
    traj_idx_ = round(t_elapsed_/control_dt_) - 1;
    NODELET_INFO_STREAM("new traj_idx: " << traj_idx_);
  }


  /* ************************************************************************** */
  Vector3d CasadiNMPCNodelet::calc_torques_PD(bool regulate) {
    /* Calc torques from Bong Wie PD attitude controller

    Inputs:
    regulate - true or false

    x_real_complete_ - full state, set globally from Ekf data
    eigen_x_des_traj_ - current trajectory being tracked (x_des taken)
    */
    Vector3d u_torques;

    ff_msgs::EkfState state;
    geometry_msgs::Pose pose;
    geometry_msgs::Point r;
    geometry_msgs::Quaternion q;
    geometry_msgs::Vector3 v;
    geometry_msgs::Vector3 w;

    // x_real_complete_ = [x y z qx qy qz qw vx vy vz wx wyz wz]
    r.x = x_real_complete_(0);
    r.y = x_real_complete_(1);
    r.z = x_real_complete_(2);
    q.x = x_real_complete_(3);
    q.y = x_real_complete_(4);
    q.z = x_real_complete_(5);
    q.w = x_real_complete_(6);
    v.x = x_real_complete_(7);
    v.y = x_real_complete_(8);
    v.z = x_real_complete_(9);
    w.x = x_real_complete_(10);
    w.y = x_real_complete_(11);
    w.z = x_real_complete_(12);
    pose.position = r;
    pose.orientation = q;
    state.pose = pose;
    state.velocity = v;
    state.omega = w;

    // eigen_x_des_traj_ needs to be converted to a ControlState from
    // eigen_x_des_traj_ = [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]
    if (regulate) {
      r.x = eigen_x_des_traj_reg_(traj_idx_, 1);
      r.y = eigen_x_des_traj_reg_(traj_idx_, 2);
      r.z = eigen_x_des_traj_reg_(traj_idx_, 3);
      v.x = eigen_x_des_traj_reg_(traj_idx_, 4);
      v.y = eigen_x_des_traj_reg_(traj_idx_, 5);
      v.z = eigen_x_des_traj_reg_(traj_idx_, 6);
      q.x = eigen_x_des_traj_reg_(traj_idx_, 7);
      q.y = eigen_x_des_traj_reg_(traj_idx_, 8);
      q.z = eigen_x_des_traj_reg_(traj_idx_, 9);
      q.w = eigen_x_des_traj_reg_(traj_idx_, 10);
      w.x = eigen_x_des_traj_reg_(traj_idx_, 11);
      w.y = eigen_x_des_traj_reg_(traj_idx_, 12);
      w.z = eigen_x_des_traj_reg_(traj_idx_, 13);
    }
    else {
      r.x = eigen_x_des_traj_(traj_idx_, 1);
      r.y = eigen_x_des_traj_(traj_idx_, 2);
      r.z = eigen_x_des_traj_(traj_idx_, 3);
      v.x = eigen_x_des_traj_(traj_idx_, 4);
      v.y = eigen_x_des_traj_(traj_idx_, 5);
      v.z = eigen_x_des_traj_(traj_idx_, 6);
      q.x = eigen_x_des_traj_(traj_idx_, 7);
      q.y = eigen_x_des_traj_(traj_idx_, 8);
      q.z = eigen_x_des_traj_(traj_idx_, 9);
      q.w = eigen_x_des_traj_(traj_idx_, 10);
      w.x = eigen_x_des_traj_(traj_idx_, 11);
      w.y = eigen_x_des_traj_(traj_idx_, 12);
      w.z = eigen_x_des_traj_(traj_idx_, 13);
    }

    ff_msgs::ControlState x_des;
    geometry_msgs::Twist twist;
    pose.position = r;
    pose.orientation = q;
    twist.linear = v;
    twist.angular = w;
    x_des.pose = pose;
    x_des.twist = twist;

    u_torques = pd_control_.controller_main(state, x_des, torque_mag_);  // also performs torque vector scaling
    return u_torques;
  }


  /* ************************************************************************** */
  std::tuple<Vector3d, Vector3d, Vector3d, Matrix<double, 6, 1>> CasadiNMPCNodelet::call_tube_mpc_func_casadi(){
    /* Call CasADi.
    Input:
    casadi_args (see below)

    Output:
    tuple of
    u_forces [3x1] forces including ancillary input
    x_nom - [6x1] nominal state (x0)
    */
    Vector3d u_forces;

    // debug info
    Vector3d u0_mpc;
    Vector3d u0_dr;
    Eigen::Matrix<double, 6, 1> x_nom;

    vector<casadi::DM> casadi_args = {};  // input vector to CasADi function

    casadi::DM dm_x0_ = eigen2dm(x_real_);  // latest state estimate
    casadi::DM dm_u_mag = eigen2dm(u_mag_);
    x_des_traj_N_ = get_setpoints();        // get setpoint values
    casadi::DM dm_x_des_traj_N = eigen2dm(x_des_traj_N_);  // ugly Eigen --> DM conversion

    // IOFormat HeavyFmt(FullPrecision, 0, ", ", ";\n", "[", "]", "[", "]");
    // std::string sep = "\n----------------------------------------\n";
    // NODELET_INFO_STREAM(sep << eigen_x_des_traj_.block(traj_idx_,0,1,7).format(HeavyFmt) << sep);
    // NODELET_INFO_STREAM(x_des_traj_N_ << "\n");

    // NODELET_INFO_STREAM("\nx0\n" << dm_x0_ << "\nx_des\n" << dm_x_des_traj_N << "\nu_mag\n" << dm_u_mag << "\nm\n"<< dm_m_ << "\nAu\n" << dm_Au_ << "\nbu\n" << dm_bu_ << "\nAz\n"
    // << dm_AZ_ << "\nbZ\n" << dm_bZ_ << "\nK_dr\n" << dm_K_dr_ << "\nQ1\n" << dm_Q1_T_ << "\nQ2\n" << dm_Q2_T_ << "\nQ3\n" << dm_Q3_T_ << "\nQ4\n" << dm_Q4_T_ << "\nQ5\n"
    // << dm_Q5_T_ << "\nQ6\n" << dm_Q6_T_ << "\nR1\n" << dm_R1_T_ << "\nR2\n"<< dm_R2_T_ << "\nR3\n" << dm_R3_T_ << "\nQN1\n" << dm_QN1_T_ << "\nQN2\n" << dm_QN2_T_ << "\nQN3\n"
    // << dm_QN3_T_ << "\nQN4\n" << dm_QN4_T_ << "\nQN5\n" << dm_QN5_T_ << "\nQN6\n" << dm_QN6_T_);

    //--- CasADi arguments
    casadi_args.push_back(dm_x0_); // for lasso constraint
    casadi_args.push_back(dm_x_des_traj_N);
    casadi_args.push_back(dm_u_mag);
    casadi_args.push_back(dm_m_);
    casadi_args.push_back(dm_Au_);
    casadi_args.push_back(dm_bu_);
    casadi_args.push_back(dm_AZ_);
    casadi_args.push_back(dm_bZ_);
    casadi_args.push_back(dm_K_dr_);
    casadi_args.push_back(dm_Q1_T_);
    casadi_args.push_back(dm_Q2_T_);
    casadi_args.push_back(dm_Q3_T_);
    casadi_args.push_back(dm_Q4_T_);
    casadi_args.push_back(dm_Q5_T_);
    casadi_args.push_back(dm_Q6_T_);
    casadi_args.push_back(dm_R1_T_);
    casadi_args.push_back(dm_R2_T_);
    casadi_args.push_back(dm_R3_T_);
    casadi_args.push_back(dm_QN1_T_);
    casadi_args.push_back(dm_QN2_T_);
    casadi_args.push_back(dm_QN3_T_);
    casadi_args.push_back(dm_QN4_T_);
    casadi_args.push_back(dm_QN5_T_);
    casadi_args.push_back(dm_QN6_T_);

    // Note: CasADi C++ interface takes std::vectors of CasADi type arguments! Make sure casadi::DM, casadi::MX, or casadi::SX are used
    // vector{x0, x_des, u_mag, m, Au, bu, AZ, bZ, K_dr, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6}
    // std::vector<casadi::DM> casadi_out = tube_mpc_func_casadi_(casadi_args);  // nominal MPC
    auto tic = std::chrono::high_resolution_clock::now();
    // std::vector<casadi::DM> casadi_out = tube_mpc_func_casadi_(casadi_args);
    std::vector<casadi::DM> casadi_out = tube_mpc_func_serialized_(casadi_args);  // nominal MPC
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tictoc = toc - tic;
    casadi_comp_time_ = tictoc.count();
    // NODELET_INFO_STREAM("debug " << casadi_comp_time_);

    // grab CasADi inputs
    u_forces(0) = double(casadi_out.at(0)(0));
    u_forces(1) = double(casadi_out.at(0)(1));
    u_forces(2) = double(casadi_out.at(0)(2));
    if (ground_.compare("true") == 0) { // Zero any z-axis control input for ground.
      u_forces(2) = 0.0;
    }

    u0_mpc(0) = double(casadi_out.at(1)(0));
    u0_mpc(1) = double(casadi_out.at(1)(1));
    u0_mpc(2) = double(casadi_out.at(1)(2));

    u0_dr(0) = double(casadi_out.at(2)(0));
    u0_dr(1) = double(casadi_out.at(2)(1));
    u0_dr(2) = double(casadi_out.at(2)(2));

    x_nom(0) = double(casadi_out.at(3)(0));
    x_nom(1) = double(casadi_out.at(3)(1));
    x_nom(2) = double(casadi_out.at(3)(2));
    x_nom(3) = double(casadi_out.at(3)(3));
    x_nom(4) = double(casadi_out.at(3)(4));
    x_nom(5) = double(casadi_out.at(3)(5));

    // NODELET_INFO_STREAM("traj_idx_: " << traj_idx_ << " N_traj_: " << N_traj_ << " t_elapsed_: " << t_elapsed_
    //              << "\nu_forces:\n" << u_forces);

    return std::make_tuple(u_forces, u0_mpc, u0_dr, x_nom);
  }

  /* ************************************************************************** */
  Vector3d CasadiNMPCNodelet::call_mpc_func_casadi(bool regulate){
    /* Call CasADi.
    Input:
    regulate: bool, true if regulating
    casadi_args (see below)

    Output:
    u_forces [3x1]
    */
    Vector3d u_forces;
    vector<casadi::DM> casadi_args = {};  // input vector to CasADi function

    // Gather remaining CasADi args
    casadi::DM dm_x0_ = eigen2dm(x_real_);  // latest state estimate
    casadi::DM dm_x_des_traj_N;
    casadi::DM dm_u_mag = eigen2dm(u_mag_);

    // Set x_des_traj_N based on either regulation or tracking
    if (regulate) {
      // NODELET_DEBUG_STREAM("[CASADI]: Des state horizon: \n" << eigen_x_des_traj_reg_.block(traj_idx_, 1, N+1, 10));
      x_des_traj_N_ = eigen_x_des_traj_reg_.block(traj_idx_, 1, N+1, 6);  // equivalent to MATLAB eigen_x_des_traj_(idx:idx+N+1, 0:6);
      dm_x_des_traj_N = eigen2dm(x_des_traj_N_);  // ugly Eigen --> DM conversion
    }
    else {
      if (traj_idx_ + (N+1) < N_traj_) {
        // NODELET_DEBUG_STREAM("[CASADI]: Des state horizon: \n" << eigen_x_des_traj_.block(traj_idx_, 1, N+1, 10));
      }
      x_des_traj_N_ = get_setpoints();
      dm_x_des_traj_N = eigen2dm(x_des_traj_N_);  // ugly Eigen --> DM conversion
    }
    // NODELET_DEBUG_STREAM("[CASADI]: Real state: \n" << x_real_complete_.block(0, 0, 1, 10));

    //--- CasADi arguments
    casadi_args.push_back(dm_x0_);
    casadi_args.push_back(dm_x_des_traj_N);
    casadi_args.push_back(dm_u_mag);
    casadi_args.push_back(dm_m_);
    casadi_args.push_back(dm_Q1_);
    casadi_args.push_back(dm_Q2_);
    casadi_args.push_back(dm_Q3_);
    casadi_args.push_back(dm_Q4_);
    casadi_args.push_back(dm_Q5_);
    casadi_args.push_back(dm_Q6_);
    casadi_args.push_back(dm_R1_);
    casadi_args.push_back(dm_R2_);
    casadi_args.push_back(dm_R3_);
    casadi_args.push_back(dm_QN1_);
    casadi_args.push_back(dm_QN2_);
    casadi_args.push_back(dm_QN3_);
    casadi_args.push_back(dm_QN4_);
    casadi_args.push_back(dm_QN5_);
    casadi_args.push_back(dm_QN6_);

    // Note: CasADi C++ interface takes std::vectors of CasADi type arguments! Make sure casadi::DM, casadi::MX, or casadi::SX are used
    // vector{x0, x_des_traj_N, u_mag, m, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6}
    auto tic = std::chrono::high_resolution_clock::now();
    // std::vector<casadi::DM> casadi_out = mpc_func_casadi_(casadi_args);
    std::vector<casadi::DM> casadi_out = mpc_func_serialized_(casadi_args);  // nominal MPC
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> tictoc = toc - tic;
    casadi_comp_time_ = tictoc.count();
    // NODELET_INFO_STREAM("debug " << casadi_comp_time_);

    // grab CasADi force output
    u_forces(0) = double(casadi_out.at(0)(0));
    u_forces(1) = double(casadi_out.at(0)(1));
    u_forces(2) = double(casadi_out.at(0)(2));

    // Zero any z-axis control input for ground.
    if (ground_.compare("true") == 0) {
      u_forces(2) = 0.0;
    }

    // NODELET_INFO_STREAM("\ntraj_idx_: " << traj_idx_ << " N_traj_: " << N_traj_ << " t_elapsed_: " << t_elapsed_
    //              << "\nu_forces:\n" << u_forces);

    return u_forces;
  }

  /* ************************************************************************** */
  Eigen::MatrixXd CasadiNMPCNodelet::get_setpoints() {
    /* Return N+1 setpoints from x_des_traj_. If traj_idx_ > N_traj_ - N, then
    start repeating the last setpoint. Account for setpoint stepping.
    */
    int setpoint_step = round(MPC_dt_/dt_traj_);  // should be a nice int: the amount that we skip forward
    int setpoints_required = setpoint_step*N+1;  // includes current time
    // NODELET_INFO_STREAM("setpoint checks: " << setpoint_step << " " << setpoints_required << " " << traj_idx_ << " " << N_traj_);

    Eigen::MatrixXd x_des_traj_N = Eigen::MatrixXd::Zero(N+1, 6);

    if (traj_idx_ + setpoints_required - 1 < N_traj_) {  // if there are enough setpoints, use them all
      for (int i = 0; i <= N; i++) {
        // NODELET_INFO_STREAM("idx : " << traj_idx_+i*setpoint_step);
        x_des_traj_N.row(i) = eigen_x_des_traj_.block(traj_idx_+i*setpoint_step, 1, 1, 6);  // equivalent to MATLAB eigen_x_des_traj_(idx:idx+N+1, 0:6);
      }
      // NODELET_INFO_STREAM("x_des_traj_N: " << x_des_traj_N);
    }
    else {
      int setpoints_available = ((N_traj_ - 1) - traj_idx_) + 1;  // includes current time
      int N_available = floor((setpoints_available - 1)/setpoint_step);  // setpoint steps forward available
      int N_rep = N+1 - (N_available); // number of repeated regulation points
      // x_des_traj_N.block(0, 0, N_available, 6) = eigen_x_des_traj_.block(traj_idx_, 1, N_available, 6);

      // available setpoints
      for (int i = 0; i <= N_available; i++) {
        // NODELET_INFO_STREAM("idx : " << traj_idx_+i*setpoint_step);
        x_des_traj_N.row(i) = eigen_x_des_traj_.block(traj_idx_+i*setpoint_step, 1, 1, 6);  // equivalent to MATLAB eigen_x_des_traj_(idx:idx+N+1, 0:6);
      }
      // NODELET_INFO_STREAM("x_des_traj_N (w/ regulate), before update: " << x_des_traj_N);

      // regulation setpoints
      MatrixXd last_row = eigen_x_des_traj_.block(eigen_x_des_traj_.rows()-1, 1, 1, 6);
      x_des_traj_N.block(N_available, 0, N_rep, 6) = last_row.replicate(N_rep, 1);

      // NODELET_INFO_STREAM("setpoint end checks: " << setpoints_available << " " << N_rep << " " << N_available << " "
        // << " " << last_row.replicate(N_rep, 1));

      // NODELET_INFO_STREAM("x_des_traj_N (w/ regulate), after update: " << x_des_traj_N);
    }

    return x_des_traj_N;
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::prepare_mrpi(Eigen::MatrixXd w, Eigen::MatrixXd u_max, double dt, double mass, double Q_pos_anc, double Q_vel_anc, double R_anc){
    /* Call MRPI calc via ROS service to get K_dr and Au and bu. Use these for CasADi update.
    Inputs:
    w - [6x1] w for [rx ry rz vx vy vz]
    u_max [3x1]
    dt - scalar
    mass_
    Q_pos_anc
    Q_vel_anc
    R_anc

    Outputs:
    K_
    Au_
    bu_
    AZ_
    bZ_
    */
    using_fallback_mrpi_ = false;  // unset from first call to fallback

    // w, u_max, and dt to ROS msg format
    rattle_msgs::RattleSrvMRPI srv;  // contains .request and .response

    std_msgs::Float64MultiArray w_msg;
    tf::matrixEigenToMsg(w, w_msg);  // Eigen --> msg
    srv.request.w = w_msg;

    std_msgs::Float64MultiArray u_max_msg;
    tf::matrixEigenToMsg(u_max, u_max_msg);  // Eigen --> msg
    srv.request.u_max = u_max_msg;

    srv.request.dt = dt;
    srv.request.mass = mass;
    srv.request.Q_pos_anc = Q_pos_anc;
    srv.request.Q_vel_anc = Q_vel_anc;
    srv.request.R_anc = R_anc;

    // bare call to service
    if (ros::service::call("mrpi", srv))
    {
      NODELET_INFO_STREAM("mrpi service request sent...");
      rattle_msgs::RattleSrvMRPI::Response res = srv.response;

      // std::vector to Eigen
      // Stride is used for row-major: eigen defaults to column major but numpy uses row major!
      int i = res.K.layout.dim[0].size;
      int j = res.K.layout.dim[1].size;
      K_dr_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.K.data.data(), i, j);
      dm_K_dr_ = eigen2dm(K_dr_);

      // std::vector to Eigen
      i = res.Au.layout.dim[0].size;
      j = res.Au.layout.dim[1].size;
      Au_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.Au.data.data(), i, j);
      dm_Au_ = eigen2dm(Au_);

      // std::vector to Eigen
      i = res.bu.layout.dim[0].size;
      j = res.bu.layout.dim[1].size;
      bu_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.bu.data.data(), i, j);
      dm_bu_ = eigen2dm(bu_);

      i = res.AZ.layout.dim[0].size;
      j = res.AZ.layout.dim[1].size;
      AZ_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.AZ.data.data(), i, j);
      dm_AZ_ = eigen2dm(AZ_);

      i = res.bZ.layout.dim[0].size;
      j = res.bZ.layout.dim[1].size;
      bZ_ = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(res.bZ.data.data(), i, j);
      dm_bZ_ = eigen2dm(bZ_);

      NODELET_INFO_STREAM("...mrpi service call success!");
    }
    else {  // service call failed, use hard-coded values
      set_mRPI_fallback_values();
      NODELET_INFO_STREAM("...mrpi service unavailable!");
    }

    if ((bu_.array() < 0.0).any()) {  // use fallback if infeasible
      set_mRPI_fallback_values();
    }

    publish_mrpi();
    mrpi_finished_ = true;
  }
}  // end namespace casadi_nmpc

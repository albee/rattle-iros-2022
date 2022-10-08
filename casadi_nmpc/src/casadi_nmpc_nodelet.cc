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
  void CasadiNMPCNodelet::Initialize(ros::NodeHandle* nh) {
    /* This is called when the nodelet is loaded into the nodelet manager.
    */
    NODELET_INFO_STREAM("[CASADI_NMPC] Initialized.");

    ros::NodeHandle MTNH = getMTNodeHandle();  // multithread subscribers

    // If in the sim, we need the robot namespace for default topics
    ros::param::getCached("/rattle/ground", ground_);
    ros::param::getCached("/rattle/sim", sim_);
    // TODO: also check role and prefixes of Astrobee?

    // std::string name = ff_util::FreeFlyerNodelet::GetPlatform();

    // load in CasADi functions
    std::string DATA_PATH = ros::package::getPath("data")+"/";
    std::string TUBE_MPC_FILE = DATA_PATH + "input/casadi-functions/" + "tube_mpc_func_casadi.casadi";
    std::string MPC_FILE = DATA_PATH + "input/casadi-functions/" + "mpc_func_casadi.casadi";
    std::string DYNAMICS_FILE = DATA_PATH + "input/casadi-functions/" + "casadi_dynamics_mpc_dt.casadi";
    tube_mpc_func_serialized_ = casadi::Function::load(TUBE_MPC_FILE);
    mpc_func_serialized_ = casadi::Function::load(MPC_FILE);
    dynamics_func_serialized_ = casadi::Function::load(DYNAMICS_FILE);
    // mpc_func_casadi_ = casadi::external("mpc_func_casadi", CASADI_MPC_LIB);  // casadi needs to be able to find this .so
    // tube_mpc_func_casadi_ = casadi::external("tube_mpc_func_casadi", CASADI_ROBUST_TUBE_MPC_LIB);  // casadi needs to be able to find this .so

    // Get parameters from config files and from primary coordinator
    get_all_YAML_parameters();

    // get initial regulation parameters
    get_initial_regulation();

    // initialize the fallback mRPI if all else fails
    set_mRPI_fallback_values();
    // prepare_mrpi(w_bound_, u_mag_, MPC_dt_, params_model_.mass, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);

    // Non-parameter initializations
    x_des_traj_N_(N+1, 6);
    u_opt_.setZero(6, 1);
    w_bound_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    x_real_ << 10.9, -9.65, 4.9, 0.0, 0.0, 0.0;  // dummy data until estimator publishes
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0; // dummy data until estimator publishes

    // Init subscribers
    sub_param_est_ = MTNH.subscribe<param_est::Params>(TOPIC_PARAM_EST, 3, boost::bind(&CasadiNMPCNodelet::update_parameters_callback, this, _1)); // latest parameter estimates
    sub_ekf_ = nh->subscribe<ff_msgs::EkfState>(TOPIC_GNC_EKF, 5, boost::bind(&CasadiNMPCNodelet::ekf_callback, this, _1));  // incoming ekf
    sub_x_des_traj_ = nh->subscribe<std_msgs::Float64MultiArray>(TOPIC_RESWARM_TUBE_MPC_TRAJ, 5, boost::bind(&CasadiNMPCNodelet::x_des_traj_callback, this, _1));  // incoming full traj---used once
    sub_uc_bound_ = nh->subscribe<std_msgs::Float64MultiArray>(UC_BOUND_TOPIC, 5, boost::bind(&CasadiNMPCNodelet::w_bound_callback, this, _1));  // incoming uc_bound
    sub_status_ = nh->subscribe<rattle_msgs::RattleStatusPrimary>(TOPIC_RESWARM_STATUS, 5, boost::bind(&CasadiNMPCNodelet::status_callback, this, _1));
    sub_reg_setpoint_ = nh->subscribe<geometry_msgs::Pose>(TOPIC_RESWARM_TUBE_MPC_REG_SETPOINT, 5, boost::bind(&CasadiNMPCNodelet::regulation_setpoint_callback, this, _1));
    sub_rattle_instruct_= nh->subscribe<rattle_msgs::RattleTestInstruct>(TOPIC_RATTLE_TEST_INSTRUCT, 5, boost::bind(&CasadiNMPCNodelet::rattle_instruct_callback, this, _1));

    // Init pubs
    pub_ctl_ = nh->advertise<ff_msgs::FamCommand>(TOPIC_GNC_CTL_COMMAND, 5, true);  // outgoing FAM commands: /which_bee_/gnc/ctl/command
    pub_debug_ = nh->advertise<rattle_msgs::RattleCasadiDebug>(TOPIC_RESWARM_TUBE_MPC_DEBUG, 5);      // For timing info and MPC inputs calculated
    pub_casadi_status_ = nh->advertise<rattle_msgs::RattleCasadiStatus>(TOPIC_RESWARM_CASADI_STATUS, 5, true);  // Status
    pub_mrpi_ = nh->advertise<rattle_msgs::RattleMsgMRPI>(TOPIC_RESWARM_TUBE_MPC_MRPI, 5, true);  // MRPI

    thread_.reset(new std::thread(&CasadiNMPCNodelet::Run, this));  // hack to get timers to work
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::Run() {
    /* ROS spin loop. Runs (1) command timer at 1/control_dt_ and (2) output timer at commmand_rate_
    */
    // ros::Duration(1.0).sleep();
    // run_debug();

    ros::NodeHandle MTNH = getMTNodeHandle();  // multithread within nodelet

    /*
    (1) Command Timer (sends out u_opt_)
    sends out commands at command_rate_
    */
    ros::Timer timer_command_rate = MTNH.createTimer(ros::Duration(1/command_rate_),
      boost::bind(&CasadiNMPCNodelet::command_timer_callback, this, _1));  // send commands

    /*
    (2) MPC Request Timer
    request MPC updates at 1/control_dt_ (creates u_opt_ based on x_real)
    */
    ros::Timer timer_MPC_rate = MTNH.createTimer(ros::Duration(control_dt_),
      boost::bind(&CasadiNMPCNodelet::MPC_timer_callback, this, _1));  // send commands

    /*
    (3) z_poly_calc Update Timer
    request polytope updates at tube_update_dt_ period
    */
    ros::Timer timer_tube_update_rate = MTNH.createTimer(ros::Duration(tube_update_dt_),
      boost::bind(&CasadiNMPCNodelet::tube_update_timer_callback, this, _1));  // send commands

    ros::waitForShutdown();
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::run_debug() {
    NODELET_INFO_STREAM("debug...");
    Eigen::Matrix<double, 6, 1> w_bound = evaluate_w(8.0, 4.25);
    std::cout << w_bound << std::endl;
    ros::Duration(2.0).sleep();

    // these values are all set via YAML
    prepare_mrpi(w_bound, u_mag_, 0.2, 10.0, 5.0, 10.0, 2.0);
   
    // std::cout << "w_bound " << w_bound << "\nu_mag " << nodelet.u_mag_ << "\nMPC_dt " << nodelet.MPC_dt_ <<
    //               "\nmass " << nodelet.params_model_.mass  << "\nQ_pos " << nodelet.Q_pos_anc_factor_ <<
    //               "\nQ_vel " << nodelet.Q_vel_anc_factor_ << "\nR_anc " << nodelet.R_anc_factor_;
   
    std::cout << "K_ " << K_dr_ << "\nAu_ " << Au_ << "\nbu_ " << bu_ << "\nAZ_ " <<
              AZ_ << "\nbZ_ " << bZ_ << std::endl;
    // w_bound_ << 0.0331, 0.0260, 0.0537, 0.0069, 0.0055, 0.0073;  // usually w_bound is set by uc_bound...this one is large
    // x_real_ << 10.85, -9.65, 4.9, 0.0, 0.0, 0.0;

    // read_traj_from_file("input/MIT-ISS/");  // read it in ourselves

    // traj_idx_ = 0;
    // HAVE_TRAJ = true;

    // update_inertial_traj();

    // get_setpoints();

    // mpc call test
    // unit_test_mpc();

    // unit_test_pd();
  }

  /* ************************************************************************** */
  void CasadiNMPCNodelet::unit_test_mpc() {
    read_traj_from_file("input/MIT-ISS/");  // read it in ourselves

    Vector3d u_forces;
    Vector3d u_torques;

    Vector3d u0_mpc;
    Vector3d u0_dr;
    Matrix<double, 6, 1> x_nom = Matrix<double, 6, 1>::Zero();

    t_start_ = ros::Time::now();

    NODELET_INFO_STREAM(
       "\n******************\n"
    << "w_bound_: " << w_bound_
    << "\nu_mag_: " << u_mag_
    << "\nMPC_dt_: " << MPC_dt_
    << "\nmass_: " << params_model_.mass
    << "\nQ_pos_anc_factor: " << Q_pos_anc_factor_
    << "\nQ_vel_anc_factor: " << Q_vel_anc_factor_
    << "\nR_anc_factor: " << R_anc_factor_
    << "\nusing_fallback_mrpi: " << using_fallback_mrpi_
    << "\n******************");

    // NODELET_INFO_STREAM("u_mag_" << u_mag_);
    prepare_mrpi(w_bound_, u_mag_, MPC_dt_, params_model_.mass, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr, Au, bu, AZ, bZ ready

    NODELET_INFO_STREAM("calling tube MPC...");
    update_u_opt_("track_tube");

    /// Compute the tube MPC input
    std::tie(u_forces, u0_mpc, u0_dr, x_nom) = call_tube_mpc_func_casadi();
    u_torques = calc_torques_PD(false);
    u_opt_ << u_forces, u_torques;

    NODELET_INFO_STREAM(
       "\n******************\n"
    << "traj_idx_: " << traj_idx_
    << "\nr_des: " << eigen_x_des_traj_.block(traj_idx_, 1, 1, 3)
    << "\nr_real: " << x_real_complete_.segment(0, 3).transpose()
    << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
    << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
    << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
    << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
    << "\nu_forces: " << u_forces.transpose()
    << "\nu_torques: " << u_torques.transpose()
    << "\n******************");
  }


  /* ************************************************************************** */
  Vector3d CasadiNMPCNodelet::call_pd_and_print() {
    Vector3d u_torques = calc_torques_PD(false);
    NODELET_INFO_STREAM(
          "\n******************\n"
      << "traj_idx_: " << traj_idx_
      << "\nquat_des: " << eigen_x_des_traj_.block(traj_idx_, 7, 1, 4)
      << "\nquat_real: " << x_real_complete_.segment(3, 4).transpose()
      << "\nw_des: " << eigen_x_des_traj_.block(traj_idx_, 11, 1, 3)
      << "\nw_real: " << x_real_complete_.segment(10, 3).transpose()
      // << "\nu_forces: " << u_forces.transpose()
      << "\nu_torques: " << u_torques.transpose()
      << "\n******************");
      return u_torques;
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::unit_test_pd(){
    // x_real_complete_ = [x y z qx qy qz qw vx vy vz wx wyz wz]
    // eigen_x_des_traj_ = [[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd] ... ]
    Vector3d u_torques;
    traj_idx_ = 0;
    eigen_x_des_traj_(1,20);

    // (1) no error
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[0] != 0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    // (2) just x (positive)
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  -0.9589243, 0, 0, 0.2836622,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[0] <= 0.0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    // (3) just y (positive)
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0, -0.9589243, 0, 0.2836622,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[1] <= 0.0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    // (4) just z (positive)
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0, 0, -0.9589243, 0.2836622,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[2] <= 0.0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    // (5) just xd (positive)
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.5, 0.0, 0.0,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[0] <= 0.0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    // (6) just yd (positive)
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.5, 0.0,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[1] <= 0.0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    // (7) just zd (positive)
    x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    eigen_x_des_traj_ << 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.5,  0.0, 0.0, 0.0;
    u_torques = call_pd_and_print();
    if (u_torques[2] <= 0.0) {
      NODELET_INFO_STREAM("Unit test failed!");
    }

    unit_test_complete_ = true;
  }


  /* ************************************************************************** */
  void CasadiNMPCNodelet::run_unit_test() {
    read_traj_from_file("input/TEST7-GND/");  // read it in ourselves

    NODELET_INFO_STREAM("unit test...");
    x_real_ << 10.9, -9.65, 4.8, 0, 0, 0;  // we are here
    w_bound_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    double dt = MPC_dt_;
    traj_idx_ = 0;
    print_x_des_traj_(traj_idx_); // we want to be here

    // check successful read-in of reference traj
    // make sure to not include time!
    NODELET_INFO_STREAM("Traj read-in..." << eigen_x_des_traj_(2,3));
    if (abs(eigen_x_des_traj_(2,3) - 4.9) < 1E-6) {
      NODELET_INFO_STREAM("...okay!");
    }
    else {
      NODELET_INFO_STREAM("...failed!");
    }

    prepare_mrpi(w_bound_, u_mag_, MPC_dt_, params_model_.mass, Q_pos_anc_factor_, Q_vel_anc_factor_, R_anc_factor_);  // get K_dr and Au and bu ready
    NODELET_INFO_STREAM("\n" << w_bound_ << "\n" << u_mag_ << "\n" << dt);

    // check same values as MATLAB
    Vector3d u_forces;
    Vector3d u0_mpc;
    Vector3d u0_dr;
    Matrix<double, 6, 1> x_nom;
    u_forces = call_mpc_func_casadi(false);
    NODELET_INFO_STREAM("CasADi test..." << u_forces(1));
    if (abs(u_forces(2) - u_mag_[2]) < 0.001) {
      NODELET_INFO_STREAM("...okay!");
    }
    else {
      NODELET_INFO_STREAM("...failed!");
    }

    x_real_ << 10.9, -9.75, 4.9, 0, 0, 0;  // we are here
    u_forces = call_mpc_func_casadi(false);
    NODELET_INFO_STREAM("CasADi test..." << u_forces(1));
    if (abs(u_forces(1) - u_mag_[1]) < 0.001) {
      NODELET_INFO_STREAM("...okay!");
    }
    else {
      NODELET_INFO_STREAM("...failed!");
    }

    // Test Tube MPC
    /// (1) Compute the nominal MPC input
    std::tie(u_forces, u0_mpc, u0_dr, x_nom) = call_tube_mpc_func_casadi();

    /// (2) Compute the ancillary controller input and sum
    // u_forces = add_ancillary_input(x_nom, u_forces);
    ros::param::set("/rattle/tube_mpc/unit_test_complete", true);
  }
}  // end namespace casadi_nmpc

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(casadi_nmpc::CasadiNMPCNodelet, nodelet::Nodelet);

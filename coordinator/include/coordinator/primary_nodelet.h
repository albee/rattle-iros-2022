#pragma once
#include "coordinator/coordinator.tpp"
#include "traj_utils/traj_utils.h"

// status struct for primary Astrobee
struct primary_reswarm_status_struct {
  // LQRRRT
  bool lqrrrt_activated = false;
  bool lqrrrt_finished = false;
  bool traj_sent =false;

  // info_planner
  bool info_traj_send = false;
  bool info_traj_sent = false;

  // casadi_nmpc params
  int reswarm_gain_mode = 0;
  std::string control_mode = "inactive";

  bool uc_bound_finished = false;
  bool mrpi_finished = false;
  bool traj_finished = false;

  // DMPC Status
  int solver_status = -1;
  float cost_value = -1.0;
  float kkt_value = -1.0;
  float sol_time = -1.0;

  // rattle
  int rattle_use_params = 0;
  int rattle_weight_mode = 0;
  int rattle_initial_model_mode = 0;
  bool activate_rattle = false;

  // post-processing and telemetry helpers
  std::string description = "";
};


class PrimaryNodelet : public CoordinatorBase<reswarm_msgs::ReswarmStatusPrimary>, public ff_util::FreeFlyerNodelet {
 public:
  PrimaryNodelet(): ff_util::FreeFlyerNodelet(true) {}  // don't do anything ROS-related in the constructor! (call the Nodelet constructor)
  ~PrimaryNodelet() {};

 private:
  /*
  eigen_x_des_traj_ =
  [t x y z xd yd zd qw qx qy qz wx wy wz xdd ydd zdd wxd wyd wzd
  ...
  ]
  */
  enum WeightMode{no_weight, step_weight, exp_weight, pure_izz_weight, pure_mass_weight};
  enum InitialModelMode{ground_truth_iss, ground_truth_ground, incorrect_low, incorrect_high};  // initial system model to use

  primary_reswarm_status_struct primary_reswarm_status_;

  std::string UC_BOUND_TOPIC = "/reswarm/uc_bound/uc_bound";
  std::string RATTLE_TEST_INSTRUCT_TOPIC = "/rattle/test_instruct";
  std::string TOPIC_RESWARM_TUBE_MPC_TRAJ = "reswarm/tube_mpc/traj";
  std::string CONTROL_MODE_TOPIC = "reswarm/primary/control_mode";
  std::string TOPIC_RESWARM_TUBE_MPC_REG_SETPOINT = "reswarm/tube_mpc/reg_setpoint";

  ros::NodeHandle *nh_;
  ros::Publisher pub_x_des_traj_;
  ros::Publisher pub_uc_bound_;
  ros::Publisher pub_rattle_test_instruct_;
  ros::Publisher pub_reg_setpoint_;

  ros::Subscriber sub_uc_bound_status_;
  ros::Subscriber sub_casadi_status_;
  ros::Subscriber sub_planner_status_;
  ros::Subscriber sub_info_status_;
  ros::Subscriber sub_reswarm_status_;
  ros::Subscriber sub_rattle_status_;
  ros::Subscriber sub_dmpc_status_;
  ros::Subscriber sub_control_mode_;

  ros::Rate sleep_rate{10.0};
  float reg_time_;  // how long to regulate

  Eigen::Matrix<double, 7, 1> reg_setpoint_;  // handy for setting latest setpoint
  Eigen::MatrixXd eigen_x_des_traj_;
  Eigen::MatrixXd eigen_x_des_traj_body_;
  int Nf_ = 1;  // length of eigen_x_des_traj_;
  int ACTIVATE_DEFAULT_REGULATION_ = 0;  // whether regulation is allowed

  // Target position offsets
  Eigen::Vector3d targ_offset_;
  Eigen::Vector3d r_RI_;

  // Parameters
  bool ground_ = false;  // whether or not this is a ground test
  bool sim_ = false;
  std::string controller_ = "default";  // controller to send commands to
  std::string traj_filename_ = "";
  std::string flight_mode_check_;
  Eigen::Vector3d x0_;
  Eigen::Vector4d a0_;

  Eigen::Matrix<double, 7, 1> POINT_A_GRANITE;
  Eigen::Matrix<double, 7, 1> POINT_A_ISS;
  Eigen::Matrix<double, 7, 1> POINT_B_GRANITE;
  Eigen::Matrix<double, 7, 1> POINT_B_ISS;
  Eigen::Matrix<double, 7, 1> POINT_C_GRANITE;
  Eigen::Matrix<double, 7, 1> POINT_C_ISS;

  // Regulation thresholds
  float pos_reg_thresh_;
  float vel_reg_thresh_;
  float att_reg_thresh_;
  float omega_reg_thresh_;

  void get_reswarm_status_msg(reswarm_msgs::ReswarmStatusPrimary& msg) override;

  void Initialize(ros::NodeHandle* nh);
  void load_params();
  void Run(ros::NodeHandle *nh);

  // quick checkout
  void RunTest0(ros::NodeHandle *nh) override;  // quick checkout

  // Test list (overrides coordinator empty tests)
  void RunTest1(ros::NodeHandle *nh) override;
  void RunTest2(ros::NodeHandle *nh) override;
  void RunTest3(ros::NodeHandle *nh) override;

  // OOA and RATTLE cool functions
  void RunTest4(ros::NodeHandle *nh) override;  // LQR-RRT*/Traj Smoother checkout
  void RunTest5(ros::NodeHandle *nh) override;  // LQR-RRT*/Traj Smoother with PID control w/o coded obstacle
  void RunTest6(ros::NodeHandle *nh) override;  // LQR-RRT*/Traj Smoother with PID control w/ coded obstacle
  void RunTest7(ros::NodeHandle *nh) override;  // LQR-RRT*/Traj Smoother with standard MPC w/ coded obstacle
  void RunTest8(ros::NodeHandle *nh) override;  // LQR-RRT*/Traj Smoother with robust tube MPC w/ coded obstacle
  void RunTest9(ros::NodeHandle *nh) override;  // Non-info trajectory, B to C
  void RunTest10(ros::NodeHandle *nh) override;  // Mass info-gain, B to C
  void RunTest11(ros::NodeHandle *nh) override;  // Inertia info-gain, B to C
  void RunTest12(ros::NodeHandle *nh) override;  // Info gain for all 4 parameters (mass and inertias)
  void RunTest13(ros::NodeHandle *nh) override;  // Run Safe On-Orbit Assembly in conjunction with RATTLE
  void RunTest14(ros::NodeHandle *nh) override;  // tube MPC checkout test
  void RunTest15(ros::NodeHandle *nh) override;  // standard MPC checkout test
  void RunTest16(ros::NodeHandle *nh) override;  // RATTLE test: generic obstacle, run full pipeline (no est updates used)
  void RunTest17(ros::NodeHandle *nh) override;  // RATTLE test: "hard" obstacle, run full pipeline
  void RunTest18(ros::NodeHandle *nh) override;  // RATTLE test: run acado planner only?
  void RunTest19(ros::NodeHandle *nh) override;  // RATTLE test: run with high weight, then low weight
  void RunTest20(ros::NodeHandle *nh) override;  // RATTLE test: gather translation data for estimation checkout
  void RunTest21(ros::NodeHandle *nh) override;  // RATTLE test: mass excitation
  void RunTest22(ros::NodeHandle *nh) override;  // RATTLE test: izz excitation
  void RunTest77(ros::NodeHandle *nh) override;  // RATTLE full test (A to B)
  void RunTest78(ros::NodeHandle *nh) override;  // RATTLE replan test
 
  void RunDebug(ros::NodeHandle *nh) override;  // debug (test77)
  
  void check_regulate();
  void publish_dummy_uc_bound();

  void set_info_test_regulate_params();
  void process_rattle_test_number(int test_number);

  // status callbacks
  void uc_bound_status_callback(const reswarm_msgs::ReswarmUCBoundStatus::ConstPtr msg);
  void rattle_status_callback(const reswarm_msgs::ReswarmRattleStatus::ConstPtr msg);
  void casadi_status_callback(const reswarm_msgs::ReswarmCasadiStatus::ConstPtr msg);
  void planner_status_callback(const reswarm_msgs::ReswarmPlannerStatus::ConstPtr msg);
  void info_status_callback(const reswarm_msgs::ReswarmInfoStatus::ConstPtr msg);
  void control_mode_callback(const std_msgs::String::ConstPtr msg);  // externally update the control mode
  
  void pub_reg_setpoint(Eigen::MatrixXd reg_setpoint_);
  void gnc_ctl_setpoint_callback(const ros::TimerEvent& );
  void send_traj_to_controller(Eigen::MatrixXd eigen_x_des_traj);
  std::tuple<Eigen::MatrixXd, int> read_traj(std::string traj_filename);
  Eigen::Matrix3f q2dcm(const Eigen::Vector4f &q);
  // ff_msgs::ControlState PrimaryNodelet::create_ControlState_from_x_des(int t_idx);

  // DMPC Helper Methods
  void RunDMPC(ros::NodeHandle *nh);
  void LaunchDMPC();
  void KillDMPC();  // Callback
  void dmpc_status_cb(const reswarm_dmpc::DMPCTestStatusStamped::ConstPtr msg);
};

PLUGINLIB_EXPORT_CLASS(PrimaryNodelet, nodelet::Nodelet); // this is a nodelet!

#pragma once
#include "coordinator/coordinator.tpp"

// status struct for primary Astrobee
struct secondary_reswarm_status_struct {
  bool test_finished = false;
    // DMPC Status
  int solver_status = -1;
  float cost_value = -1.0;
  float kkt_value = -1.0;
  float sol_time = -1.0;
};


class SecondaryNodelet :  public CoordinatorBase<reswarm_msgs::ReswarmStatusSecondary>, public ff_util::FreeFlyerNodelet {
 public:
  SecondaryNodelet(): ff_util::FreeFlyerNodelet(true) {}  // don't do anything ROS-related in the constructor! (call the Nodelet constructor)
  ~SecondaryNodelet() {};

 private:
  ros::Subscriber sub_reswarm_status_;
  ros::Subscriber sub_dmpc_status_;
  secondary_reswarm_status_struct secondary_reswarm_status_;

  // Parameters
  std::string controller_ = "default";  // controller to send commands to

  std::string traj_filename_ = "";
  std::string flight_mode_check_;

  // Status parameters  
  bool ground_ = false;  // whether or not this is a ground test
  bool sim_ = false;
  bool reswarm_coord_ok_ = true;
  bool reswarm_regulate_finished_ = false;
  bool reswarm_motion_plan_finished_ = false;
  float reswarm_motion_plan_wait_time_ = 0.0;
  bool reswarm_uc_bound_finished_ = false;
  bool reswarm_mrpi_finished_ = false;
  bool reswarm_traj_finished_ = false;
  bool info_traj_sent_ = false;

  void get_reswarm_status_msg(reswarm_msgs::ReswarmStatusSecondary& msg) override;

  void Initialize(ros::NodeHandle* nh);
  void load_params();
  void Run(ros::NodeHandle *nh);

  // quick checkout
  void RunTest0(ros::NodeHandle *nh) override;

  // Test list (overrides coordinator empty tests)
  void RunTest1(ros::NodeHandle *nh) override;
  void RunTest2(ros::NodeHandle *nh) override;
  void RunTest3(ros::NodeHandle *nh) override;

  // DMPC Helper Methods
  void RunDMPC(ros::NodeHandle *nh);
  void LaunchDMPC();
  void KillDMPC();
  void dmpc_status_cb(const reswarm_dmpc::DMPCTestStatusStamped::ConstPtr msg);
};

PLUGINLIB_EXPORT_CLASS(SecondaryNodelet, nodelet::Nodelet); // this is a nodelet!

/* primary_nodelet.cc

The primary coordinator, which derives from CoorindatorBase and adds methods for DMPC, RATTLE, and assembly tests.
*/

#include "coordinator/secondary_nodelet.h"
#include "coordinator/secondary_dmpc_methods.hpp"

/* ************************************************************************** */
void SecondaryNodelet::Initialize(ros::NodeHandle* nh) {
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
  pub_status_ = nh->advertise<reswarm_msgs::ReswarmStatusSecondary>(TOPIC_RESWARM_STATUS, 5, true);
  
  // subscribers
  sub_flight_mode_= nh->subscribe<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 5,
    boost::bind(&SecondaryNodelet::flight_mode_callback, this, _1));  // flight mode setter
  sub_ekf_ = nh->subscribe<ff_msgs::EkfState>("gnc/ekf", 5,
    boost::bind(&SecondaryNodelet::ekf_callback, this, _1));
  sub_reswarm_test_number_ = nh->subscribe<reswarm_msgs::ReswarmTestNumber>(TOPIC_RESWARM_TEST_NUMBER, 5,
    boost::bind(&SecondaryNodelet::test_num_callback, this, _1));
  sub_dmpc_status_ = nh->subscribe<reswarm_dmpc::DMPCTestStatusStamped>("reswarm/dmpc_status", 5,
    boost::bind(&SecondaryNodelet::dmpc_status_cb, this, _1));

  // services
  serv_ctl_enable_ = nh->serviceClient<std_srvs::SetBool>(SERVICE_GNC_CTL_ENABLE);
  
  // Pass control to Run method (activate timers and spin)
  NODELET_INFO_STREAM("[SECONDARY COORD] Initialized. Passing control to Run method...");
  thread_.reset(new std::thread(&CoordinatorBase::Run, this, nh));
}

/* ************************************************************************** */
void SecondaryNodelet::get_reswarm_status_msg(reswarm_msgs::ReswarmStatusSecondary& msg){
  /**
   * @brief Fills the ReswarmStatus message with the state of the private
   * variables. Published by a coordinator Timer.
   * Inputs: base_reswarm_status_ and secondary_reswarm_status_
   * 
   */
  msg.stamp = ros::Time::now();
  msg.test_number = base_reswarm_status_.test_number;
  msg.default_control = base_reswarm_status_.default_control;
  msg.flight_mode = base_reswarm_status_.flight_mode;
  msg.test_finished = base_reswarm_status_.test_finished;
  msg.coord_ok = base_reswarm_status_.coord_ok;
  // DMPC status
  msg.test_finished = secondary_reswarm_status_.test_finished;
  msg.solver_status = secondary_reswarm_status_.solver_status;
  msg.cost_value = secondary_reswarm_status_.cost_value;
  msg.kkt_value = secondary_reswarm_status_.kkt_value;
  msg.sol_time = secondary_reswarm_status_.sol_time;
}

/* ************************************************************************** */
void SecondaryNodelet::load_params(){
  // Get sim and ground flags
  std::string sim_str, ground_str;
  ros::param::get("/reswarm/sim", sim_str);
  sim_ = !std::strcmp(sim_str.c_str(), "true"); // convert to bool
  ros::param::get("/reswarm/ground", ground_str);
  ground_ = !std::strcmp(ground_str.c_str(), "true");  // convert to bool
  
  NODELET_INFO_STREAM("[SECONDARY COORD] Parameters Loaded...");
}

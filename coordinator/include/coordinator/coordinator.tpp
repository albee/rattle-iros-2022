#pragma once

/*
# coordinator

High-level logic coordinating test operation.

Every test has a test#() function available in case it is needed by asap.py
*/

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include "eigen_conversions/eigen_msg.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>

// msg includes
#include <ff_msgs/ControlState.h>
#include <ff_msgs/FlightMode.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <msg_conversions/msg_conversions.h>
#include <ff_msgs/FamCommand.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Inertia.h>
#include <reswarm_msgs/ReswarmStatusPrimary.h>
#include <reswarm_msgs/ReswarmStatusSecondary.h>
#include <reswarm_msgs/ReswarmTestNumber.h>
#include <reswarm_msgs/ReswarmCasadiStatus.h>
#include <reswarm_msgs/ReswarmUCBoundStatus.h>
#include <reswarm_msgs/ReswarmInfoStatus.h>
#include <reswarm_msgs/ReswarmPlannerStatus.h>
#include <reswarm_dmpc/DMPCTestStatusStamped.h>
#include <reswarm_msgs/RattleInfoPlanInstruct.h>
#include <reswarm_msgs/RattleTestInstruct.h>
#include <reswarm_msgs/ReswarmRattleStatus.h>

// Actions
#include <ff_msgs/ControlAction.h>

// Service message
#include <std_srvs/SetBool.h>

// C++
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <chrono>
#include <string.h>


static std::string TOPIC_RESWARM_STATUS = "reswarm/status";
static std::string TOPIC_RESWARM_TEST_NUMBER = "reswarm/test_number";


// base status struct (key information)
struct BaseReswarmStatus {
  int test_number = -2;
  std::string flight_mode = "nominal";
  bool test_finished = false;

  bool coord_ok = true;
  bool regulate_finished = false;

  bool default_control = true;  // {true, false}: allow the default controller to run?
};


template <typename T>  // for PrimaryStatus or SecondaryStatus 
class CoordinatorBase {
 public:
  CoordinatorBase() {}; // don't do anything ROS-related in the constructor!
  ~CoordinatorBase() {};
  
  // Base main functions
  void Run(ros::NodeHandle *nh);
 protected:
  BaseReswarmStatus base_reswarm_status_;

  ros::NodeHandle MTNH;
  std::shared_ptr<std::thread> thread_;

  ros::Publisher pub_flight_mode_;
  ros::Publisher pub_status_;

  ros::Subscriber sub_flight_mode_;
  ros::Subscriber sub_ekf_;
  ros::Subscriber sub_reswarm_test_number_;

  ros::ServiceClient serv_ctl_enable_;

  ros::Timer status_timer_;
  ros::Timer ctl_disable_timer_;

  ff_msgs::FlightMode flight_mode_;
  ff_msgs::EkfState ekf_state_;

  // Parameters
  bool ground_ = false;  // whether or not this is a ground test
  bool sim_ = false;
  std::string flight_mode_name_;

  // Stored status parameters
  std::string stored_control_mode_ = "track";  // stored control_mode, set by parameter inputs
  int stored_gain_mode_ = 0;

  // Ekf state
  Eigen::Matrix<double, 16, 1> x_real_complete_;

  // Test number processing
  void process_test_number();
  void get_flight_mode();
  
  void publish_reswarm_status(const ros::TimerEvent&);  // templated for Primary or Secondary status
  virtual void get_reswarm_status_msg(reswarm_msgs::ReswarmStatusPrimary& msg) {};
  virtual void get_reswarm_status_msg(reswarm_msgs::ReswarmStatusSecondary& msg) {};

  void test_num_callback(const reswarm_msgs::ReswarmTestNumber::ConstPtr msg);
  void flight_mode_callback(const ff_msgs::FlightMode::ConstPtr msg);
  void ekf_callback(const ff_msgs::EkfState::ConstPtr msg);

  void debug();

  // Astrobee GNC interface
  void disable_default_ctl_callback(const ros::TimerEvent&);
  void disable_default_ctl();
  void enable_default_ctl();

  // define the reswarm_status callback for each robot
  // virtual void reswarm_status_cb(const reswarm_msgs::ReswarmStatus::ConstPtr msg);

  // Virtual test list: to be replaced on each derived coordinator
  virtual void RunTest0(ros::NodeHandle *nh) {};
  virtual void RunTest1(ros::NodeHandle *nh) {};
  virtual void RunTest2(ros::NodeHandle *nh) {};
  virtual void RunTest3(ros::NodeHandle *nh) {};
  virtual void RunTest4(ros::NodeHandle *nh) {};
  virtual void RunTest5(ros::NodeHandle *nh) {};
  virtual void RunTest6(ros::NodeHandle *nh) {};
  virtual void RunTest7(ros::NodeHandle *nh) {};
  virtual void RunTest8(ros::NodeHandle *nh) {};
  virtual void RunTest9(ros::NodeHandle *nh) {};
  virtual void RunTest10(ros::NodeHandle *nh) {};
  virtual void RunTest11(ros::NodeHandle *nh) {};
  virtual void RunTest12(ros::NodeHandle *nh) {};
  virtual void RunTest13(ros::NodeHandle *nh) {};
  virtual void RunTest14(ros::NodeHandle *nh) {};
  virtual void RunTest15(ros::NodeHandle *nh) {};
  virtual void RunTest16(ros::NodeHandle *nh) {};
  virtual void RunTest17(ros::NodeHandle *nh) {};
  virtual void RunTest18(ros::NodeHandle *nh) {};
  virtual void RunTest19(ros::NodeHandle *nh) {};
  virtual void RunTest20(ros::NodeHandle *nh) {};
  virtual void RunTest21(ros::NodeHandle *nh) {};
  virtual void RunTest22(ros::NodeHandle *nh) {};
  virtual void RunTest77(ros::NodeHandle *nh) {};
  virtual void RunTest78(ros::NodeHandle *nh) {};

  virtual void RunDebug(ros::NodeHandle *nh) {};
  // you can add more tests as desired in primary.h and secondary.h
};


/* ************************************************************************** */
// Coordinator template implementation
/* ************************************************************************** */

/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::Run(ros::NodeHandle *nh) {
  /**
   * @brief Interpret the `/test_number` topic and run tests. High-level test management
   * takes place here, and is delegated out to other nodes. 
   * Each test function is intended to be run just ONCE per test number received.
   * This is the place to add new test numbers!
   * 
   */
  x_real_complete_ << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0;

  // Status publisher in separate thread
  status_timer_ = MTNH.createTimer(ros::Duration(0.2),
    boost::bind(&CoordinatorBase::publish_reswarm_status, this, _1));  // send commands (5 Hz)

  // Controller disabler in separate thread
  // ctl_disable_timer_ = MTNH.createTimer(ros::Duration(5.0),
  //   boost::bind(&CoordinatorBase::disable_default_ctl_callback, this, _1));  // send disable commands (0.2 Hz)

  // Check updates at 10Hz
  ros::Rate sleep_rate(10.0);

  // (1) Start up and wait for test_number_
  while (ros::ok() && base_reswarm_status_.test_number == -2) {  // startup test number
    ros::spinOnce();
    sleep_rate.sleep();
  }

  // (2) Publish default flight mode so FAM will actually perform actuation
  get_flight_mode();
  pub_flight_mode_.publish(flight_mode_);  // TODO: should this be a more aggressive flight mode?
  ros::Duration(2.0).sleep();  // Pause so flight mode actually gets registered

  // (3) Execute test_number_ logic
  while (ros::ok()) { 
    if (!base_reswarm_status_.test_finished) {
      // Tests go below...
      if (base_reswarm_status_.test_number == 0) {
        RunTest0(nh);
      }
      if (base_reswarm_status_.test_number  == 1 ||
          (base_reswarm_status_.test_number >= 100 && base_reswarm_status_.test_number <= 199) ||
          (base_reswarm_status_.test_number >= 10000 && base_reswarm_status_.test_number <= 19999) ) {
        RunTest1(nh);
      }
      else if(base_reswarm_status_.test_number  == 2 ||
          (base_reswarm_status_.test_number >= 200 && base_reswarm_status_.test_number <= 299) ||
          (base_reswarm_status_.test_number >= 20000 && base_reswarm_status_.test_number <= 29999)) {
        RunTest2(nh);
      }
      else if(base_reswarm_status_.test_number  == 3 ||
          (base_reswarm_status_.test_number >= 300 && base_reswarm_status_.test_number <= 399) ||
          (base_reswarm_status_.test_number >= 30000 && base_reswarm_status_.test_number <= 39999)) {
        RunTest3(nh);
      }
      else if(base_reswarm_status_.test_number  == 4) {
        RunTest4(nh);
      }
      else if(base_reswarm_status_.test_number  == 5) {
        RunTest5(nh);
      }
      else if(base_reswarm_status_.test_number  == 6) {
        RunTest6(nh);
      }
      else if(base_reswarm_status_.test_number  == 7) {
        RunTest7(nh);
      }
      else if(base_reswarm_status_.test_number  == 8) {
        RunTest8(nh);
      }
      else if(base_reswarm_status_.test_number  == 9) {
        RunTest9(nh);
      }
      else if(base_reswarm_status_.test_number  == 10) {
        RunTest10(nh);
      }
      else if(base_reswarm_status_.test_number  == 11) {
        RunTest11(nh);
      }
      else if(base_reswarm_status_.test_number  == 12) {
        RunTest12(nh);
      }
      else if(base_reswarm_status_.test_number  == 13) {
        RunTest13(nh);
      }
      else if(base_reswarm_status_.test_number  == 14) {
        RunTest14(nh);
      }
      else if(base_reswarm_status_.test_number  == 15) {
        RunTest15(nh);
      }
      else if(base_reswarm_status_.test_number  == 16) {
        RunTest16(nh);
      }
      else if(base_reswarm_status_.test_number  == 17) {
        RunTest17(nh);
      }
      else if(base_reswarm_status_.test_number == 18) {
        RunTest18(nh);
      }
      else if(base_reswarm_status_.test_number == 19) {
        RunTest19(nh);
      }
      else if(base_reswarm_status_.test_number == 20) {
        RunTest20(nh);
      }
      else if(base_reswarm_status_.test_number == 21) {
        RunTest21(nh);
      }
      else if(base_reswarm_status_.test_number == 22) {
        RunTest22(nh);
      }
      else if(base_reswarm_status_.test_number == 77) {  // debug test
        RunDebug(nh);
      }
      else if(base_reswarm_status_.test_number == 78) {  // rattle replan test
        RunTest78(nh);
      }
      else if (base_reswarm_status_.test_number >= 77000 && base_reswarm_status_.test_number <= 77999) {
        RunTest77(nh);
      }
      base_reswarm_status_.test_finished = true;
    }
    ros::spinOnce();
    sleep_rate.sleep();
  }
}


/* ************************************************************************** */
template <typename T>
void CoordinatorBase<T>::get_flight_mode() {
  /* Get a nominal flight mode for desired test.
  */
  if (base_reswarm_status_.test_number != -1 and base_reswarm_status_.test_number != 0) {  // NOT shutdown test number or checkout test
    // create a nominal FlightMode
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, base_reswarm_status_.flight_mode)) {
      return;
    } 
  }
  else { // -1 shutdown test number
    // Shutdown Astrobee (turn off impellers)
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, "off")) {
      return;
    }  
  }
}


/* ************************************************************************** */
template <typename T>
void CoordinatorBase<T>::publish_reswarm_status(const ros::TimerEvent&) {
  /**
   * @brief Main coordinator of Base logic. Relies on get_reswarm_status_msg, defined in derived class.
   * Uses either Primary or Secondary status logic.
   * 
   */
  T msg;
  get_reswarm_status_msg(msg);
  pub_status_.publish(msg);
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::process_test_number() {
  /**
   * @brief Process test number logic for parameters
   * TODO: adapt this for reswarm parameter setting
   * 
   */

  if (base_reswarm_status_.test_number > 100) {
    std::string test_number_str = std::to_string(base_reswarm_status_.test_number);

    // Parameter settings xx(xxxxxx)
    // controller
    if (test_number_str[2] == '1') {  // standard MPC
      stored_control_mode_ = "track";
    }
    else if (test_number_str[2] == '2') {  // tube MPC
      stored_control_mode_ = "track_tube";
    }

    // gains
    if (test_number_str[3] == '1') {
      stored_gain_mode_ = 0;
    }
    else if (test_number_str[3] == '2') {
      stored_gain_mode_ = 1;
    }
    else if (test_number_str[3] == '3') {
      stored_gain_mode_ = 2;
    }
    else if (test_number_str[3] == '4') {
      stored_gain_mode_ = 3;
    }
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::test_num_callback(const reswarm_msgs::ReswarmTestNumber::ConstPtr msg) {
  /**
   * @brief Updates test numbers received from exec_asap
   * 
   */
  base_reswarm_status_.test_number = msg->test_number;
  if (base_reswarm_status_.test_number == -1) {
    // Re-enable default controller
    enable_default_ctl();

    // Set flight mode to off
    base_reswarm_status_.flight_mode = "off";
    if (!ff_util::FlightUtil::GetFlightMode(flight_mode_, base_reswarm_status_.flight_mode)) {
        return;
    }
    pub_flight_mode_.publish(flight_mode_);
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::flight_mode_callback(const ff_msgs::FlightMode::ConstPtr msg) {
  /**
   * @brief Callback for flight mode. And yes, still pancakes for Will (and Pedro).
   * 
   */
  flight_mode_name_ = msg->name;

  // kill ctl if it tries to turn on
  if (base_reswarm_status_.default_control == false){
    auto serv_start = std::chrono::high_resolution_clock::now();

    // Disable the default controller so custom controller can run
    std_srvs::SetBool srv;
    srv.request.data = false;
    serv_ctl_enable_.call(srv);

    auto serv_finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> serv_elapsed = serv_finish - serv_start;
    std::string response = srv.response.message;

    std::cout << "[COORDINATOR]: Controller disable service time: " << serv_elapsed.count() << " seconds."<< std::endl;
    std::cout << "[COORDINATOR]: Controller disable service result: " << response << std::endl;
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::ekf_callback(const ff_msgs::EkfState::ConstPtr msg) {
  /**
   * @brief The `gnc/ekf` subscriber callback. Called at 62.5 Hz.
   * Used to check if regulation is finished.
   * 
   */
  float qx = msg->pose.orientation.x;
  float qy = msg->pose.orientation.y;
  float qz = msg->pose.orientation.z;
  float qw = msg->pose.orientation.w;

  if (qx != 0 || qy != 0 || qz != 0 || qw != 0) {
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
    }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::debug(){
  /**
   * @brief debug function call to test compatibility with other Bases
   * 
   */

}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::disable_default_ctl_callback(const ros::TimerEvent&) {
  /**
   * @brief Switch default controller off, repeatedly.
   * @param base_reswarm_status.default_control is monitored for activation
   */
  if (base_reswarm_status_.default_control == false){
    auto serv_start = std::chrono::high_resolution_clock::now();

    // Disable the default controller so custom controller can run
    std_srvs::SetBool srv;
    srv.request.data = false;
    serv_ctl_enable_.call(srv);

    auto serv_finish = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> serv_elapsed = serv_finish - serv_start;
    std::string response = srv.response.message;

    std::cout << "[COORDINATOR]: Controller disable service time: " << serv_elapsed.count() << " seconds."<< std::endl;
    std::cout << "[COORDINATOR]: Controller disable service result: " << response << std::endl;
  }
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::disable_default_ctl() {
  /**
   * @brief Switch default controller off.
   * @param base_reswarm_status.default_control is monitored for activation
   */
  base_reswarm_status_.default_control = false;

  // Disable the default controller so custom controller can run
  std_srvs::SetBool srv;
  srv.request.data = false;
  serv_ctl_enable_.call(srv);
}


/* ************************************************************************** */
template<typename T>
void CoordinatorBase<T>::enable_default_ctl() {
  /**
   * @brief Switch default controller on.
   * 
   */
  ROS_DEBUG_STREAM("[COORDINATOR]: Enabling default controller...");

  // Disable the default controller so tube-MPC can run
  base_reswarm_status_.default_control = true;

  std_srvs::SetBool srv;
  srv.request.data = true;
  serv_ctl_enable_.call(srv);

  ROS_DEBUG_STREAM("[COORDINATOR]: Ctl enable service result: " << srv.response.message);
}
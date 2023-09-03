/*
rrt_nodelet.cc

This is a nodelet wrapper around an RRT generator.

Publishing to `rattle/rrt/params` will trigger a calculation.

Output is on `rattle/rrt/path/posearray` and `rattle/rrt/path/twistarray`

Keenan Albee, 2021
MIT Space Systems Lab
*/

#ifndef RRT_NODELET_H_
#define RRT_NODELET_H_

// Standard includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// Custom includes
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <string.h>

// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_common/ff_names.h>

// Msgs
#include <geometry_msgs/InertiaStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <msg_conversions/msg_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <rattle_rrt/RRTParams.h>
#include <rattle_rrt/TwistArray.h>
#include <rattle_rrt/ellipsoid.h>
#include <rattle_rrt/ellipsoidArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <param_est/Params.h>
#include <rattle_msgs/RattleTestInstruct.h>

// RRT includes
#include <rattle_rrt/types.h>
#include <rattle_rrt/tree.tpp>
#include <rbd/rigidBodyDynamics.h>
#include <rattle_rrt/params.h>

// std
#include <memory>
#include <vector>

// ROS
#include <ros/time.h>

using std::cout;
using std::shared_ptr;
using std::vector;
using std::make_shared;

namespace rrt {

class RRTNodelet : public ff_util::FreeFlyerNodelet {
 public:
  RRTNodelet() : ff_util::FreeFlyerNodelet(true) {}
  ~RRTNodelet() {}

 private:
  bool ground_{false};  // {true, false}
  bool use_params_{false};  // {true, false}: use estimated param update callback?
  enum InitialModelMode{ground_truth_iss, ground_truth_ground, incorrect_low, incorrect_high};  // initial system model to use
  int initial_model_mode_ = ground_truth_iss;

  // std::vector of [x, y, z, xd, yd, zd, qx, qy, qz, q, wx, wy, wz]
  // std::vector<rrt::Node<rrt::Vec13>> path_;  // vector of RRT solution nodes
  // std::vector<rrt::Node<rrt::Vec6>> path_ground_;  // vector of RRT solution nodes
  std::vector<rrt::Node<rrt::Vec6>> path_;  // vector of RRT solution nodes
  std::vector<rrt::Node<rrt::Vec4>> path_ground_;  // vector of RRT solution nodes
  double TIMESTEP;

  ros::Publisher pub_rrt_path_twistarray_;
  ros::Publisher pub_rrt_path_posearray_;
  ros::Publisher pub_rrt_params_;
  ros::Publisher pub_rrt_marker_;
  ros::Subscriber sub_rrt_params_;
  ros::Subscriber sub_obs_;
  ros::Subscriber sub_DOF_;
  ros::Subscriber sub_est_params_;
  ros::Subscriber sub_rattle_instruct_;  // RATTLE configuration options

  std::vector<rattle_rrt::ellipsoid> ellipsoids_;
  std::vector<Obstacle> obstacles_;

  // model parameters
  std::vector<Params> initial_mode_vec = {Params{9.58, 0.153, 0.143, 0.162}, Params{18.9715, 0.2517, 0.2517, 0.2517},
                                          Params{5.0, 0.075, 0.075, 0.075}, Params{20.0, 0.30, 0.30, 0.30}};
  Params params_model_ = initial_mode_vec[initial_model_mode_];  // mass Ixx Iyy Izz, use by system model
  Params params_est_{0.0, 0.0, 0.0, 0.0};  // mass Ixx Iyy Izz, considered for model updates
  double mass_lb_ = 7.0;  // kg
  double inertia_lb = 0.05;  // N-m
  double ymin_iss_ros_ = -9.8;  // ymin for ISS

  // for subscriber management
  std::shared_ptr<std::thread> thread_;

  void Initialize(ros::NodeHandle* nh);
  void Run();

  // TODO: would be beneficial to change to a rosservice
  void generate_rrt(double start_pos[7], double goal_pos[7], Params& model_params);
  std::tuple<std::shared_ptr<rrt::Tree<rrt::Vec4>>, shared_ptr<rrt::StateSpace4D>>
    setup_4d(double start_pos[7], double goal_pos[7], Params& model_params);  // ground
  std::tuple<std::shared_ptr<rrt::Tree<rrt::Vec6>>, shared_ptr<rrt::StateSpace6DISS>>
    setup_6d_iss(double start_pos[7], double goal_pos[7], Params& model_params);  // iss
  std::shared_ptr<rrt::Tree<rrt::Vec13>> setup_13d(double start_pos[7], double goal_pos[7]);  // not currently used

  void rrt_params_callback(const rattle_rrt::RRTParams::ConstPtr param_msg);
  void rrt_obs_callback(const rattle_rrt::ellipsoidArray::ConstPtr ellipsoids_msg);
  void update_parameters_callback(const param_est::Params::ConstPtr& msg);
  void rattle_instruct_callback(const rattle_msgs::RattleTestInstruct::ConstPtr& msg);

  template <typename T>
  void rrt_path_publish(const T &path, double start_pos[7]);

  void rrt_path_request();

  template <typename T>
  void pub_rrt_rviz_markers(std::vector<std::tuple<T, T>> rrt_edges);
};  // end class RRTNodelet
}  // end namespace rrt

// Declare the plugin
PLUGINLIB_EXPORT_CLASS(rrt::RRTNodelet, nodelet::Nodelet);

#endif  // RRT_NODELET_H_

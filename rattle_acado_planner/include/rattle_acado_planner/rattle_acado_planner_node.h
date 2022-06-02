#pragma once

#include "rattle_acado_planner/rattle_acado_planner.h"

#include <ros/ros.h>

// Action
#include <ff_msgs/DockAction.h>

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>

// Messages
#include <ff_msgs/EkfState.h>
#include <geometry_msgs/InertiaStamped.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/FlightMode.h>
#include <rattle_acado_planner/NMPCInstruct.h>
#include <rattle_rrt/TwistArray.h>
#include <rattle_rrt/WrenchArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <reswarm_msgs/RattleInfoPlanInstruct.h>
#include <reswarm_msgs/RattleTestInstruct.h>
#include <param_est/Params.h>
#include <std_msgs/Float64MultiArray.h>
#include <rattle_acado_planner/eigen_msg.h>


class RattlePlannerNode: RattlePlanner {
  public:
    RattlePlannerNode(ros::NodeHandle* nh);
   ~RattlePlannerNode() {};
  
    void pub_planner_output();

    void ekf_state_callback(const ff_msgs::EkfState::ConstPtr& msg);
    void info_plan_instruct_start_callback(const reswarm_msgs::RattleInfoPlanInstruct::ConstPtr& msg);
    void info_plan_instruct_callback(const reswarm_msgs::RattleInfoPlanInstruct::ConstPtr& msg);
    void parameters_callback(const geometry_msgs::InertiaStamped::ConstPtr& msg);
    void update_parameters_callback(const param_est::Params::ConstPtr& msg);
    void rattle_instruct_callback(const reswarm_msgs::RattleTestInstruct::ConstPtr& msg);

  private:
    ros::NodeHandle* nh;

    ros::Subscriber sub_current_pose_;
    ros::Subscriber sub_current_inertial_params_;
    ros::Subscriber sub_estimated_inertial_params_;
    ros::Subscriber sub_des_;
    ros::Subscriber sub_des_init;
    ros::Subscriber sub_rattle_instruct_;  // RATTLE configuration options

    ros::Publisher pub_local_path_twistarray_;
    ros::Publisher pub_local_path_posearray_;
    ros::Publisher pub_local_path_wrencharray_;

    ros::Publisher pub_weights_;
    ros::Publisher pub_psi_;
};

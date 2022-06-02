#pragma once
#include "coordinator/primary_nodelet.h"

/************************************************************************/
void PrimaryNodelet::RunTest16(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 0;
    msg.WEIGHT_MODE = no_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = ground_truth_ground;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = ground_truth_iss;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 2;  // {0:default, 1:"hard", 2:"fancy"}

    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest17(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 0;
    msg.WEIGHT_MODE = no_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 2;  // {0:default, 1:"hard", 2:"fancy"}
    
    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest18(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 1;
    msg.WEIGHT_MODE = no_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 2;  // {0:default, 1:"hard", 2:"fancy"}
    
    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest19(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 1;
    msg.WEIGHT_MODE = step_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 2;  // {0:default, 1:"hard", 2:"fancy"}
    
    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}

/************************************************************************/
void PrimaryNodelet::RunTest20(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 1;
    msg.WEIGHT_MODE = exp_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 2;  // {0:default, 1:"hard", 2:"fancy"}
    
    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest21(ros::NodeHandle *nh){
    /* mass only excitation
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "pure_mass_weight"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 0;
    msg.WEIGHT_MODE = no_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = ground_truth_ground;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = ground_truth_iss;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 1;  // {0:default, 1:"hard", 2:"fancy"}
    
    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest22(ros::NodeHandle *nh){
    /* izz only excitation
    */
    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "pure_izz_weight"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 0;
    msg.WEIGHT_MODE = no_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = ground_truth_ground;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = ground_truth_iss;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 1;  // {0:default, 1:"hard", 2:"fancy"}

    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest77(ros::NodeHandle *nh){
    /* Full RATTLE A to B test, with many options for test type.
    */
    process_rattle_test_number(base_reswarm_status_.test_number);  // update rattle params for test77(x)

    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = primary_reswarm_status_.rattle_use_params;
    msg.WEIGHT_MODE = primary_reswarm_status_.rattle_weight_mode;  // note: see enum definition in primary_nodelet.h
    msg.INITIAL_MODEL_MODE = primary_reswarm_status_.rattle_initial_model_mode;  // note: see enum definition in primary_nodelet.h
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 1;  // {0:default, 1:"hard", 2:"fancy"}

    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest78(ros::NodeHandle *nh){
    /* Full RATTLE A to B test, with a mandatory replanning
    */
    process_rattle_test_number(base_reswarm_status_.test_number);  // update rattle params for test77(x)

    primary_reswarm_status_.reswarm_gain_mode = 1;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;

    msg.test_num = "full_replan"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 1;
    msg.WEIGHT_MODE = step_weight;  // note: see enum definition in primary_nodelet.h
    msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 1;  // {0:default, 1:"hard", 2:"fancy"}

    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/* ************************************************************************** */
void PrimaryNodelet::control_mode_callback(const std_msgs::String::ConstPtr msg) {
    /* Update control_mode form an external node.
    NB: this is necessary! rattle_coordinator writes directly to control_mode, but it will be overwritten if not updated internally in coordinator.
    */
    primary_reswarm_status_.control_mode = msg->data;
}


void PrimaryNodelet::process_rattle_test_number(int test_number) {
  /**
   * @brief Process test number logic for parameters
   * 
   */

  if (test_number > 77000) {
    std::string test_number_str = std::to_string(test_number);

    // Parameter settings xx(xxx)
    // use_params
    if (test_number_str[2] == '0') {
      primary_reswarm_status_.rattle_use_params = 0;
    }
    else if (test_number_str[2] == '1') {
      primary_reswarm_status_.rattle_use_params = 1;
    }

    // weight
    if (test_number_str[3] == '0') {
      primary_reswarm_status_.rattle_weight_mode = 0;
    }
    else if (test_number_str[3] == '1') {
      primary_reswarm_status_.rattle_weight_mode = 1;
    }
    else if (test_number_str[3] == '2') {
      primary_reswarm_status_.rattle_weight_mode = 2;
    }
    else if (test_number_str[3] == '3') {
      primary_reswarm_status_.rattle_weight_mode = 3;
    }
    else if (test_number_str[3] == '4') {
      primary_reswarm_status_.rattle_weight_mode = 4;
    }
    else if (test_number_str[3] == '5') {
      primary_reswarm_status_.rattle_weight_mode = 5;
    }

    // ground truth
    if (test_number_str[4] == '0') {
      primary_reswarm_status_.rattle_initial_model_mode = 0;
    }
    else if (test_number_str[4] == '1') {
      primary_reswarm_status_.rattle_initial_model_mode = 1;
    }
    else if (test_number_str[4] == '2') {
      primary_reswarm_status_.rattle_initial_model_mode = 2;
    }
    else if (test_number_str[4] == '3') {
      primary_reswarm_status_.rattle_initial_model_mode = 3;
    }
  }
}

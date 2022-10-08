#pragma once
#include "coordinator/primary_nodelet.h"

/************************************************************************/
void PrimaryNodelet::RunTest0(ros::NodeHandle *nh){
    NODELET_INFO_STREAM("[PRIMARY_COORD]: Congratulations, you have passed quick checkout. " 
    "May your days be blessed with only warnings and no errors.");

    disable_default_ctl();

    ros::Duration(5.0).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
};


/************************************************************************/
void PrimaryNodelet::RunTest9(ros::NodeHandle *nh){
    primary_rattle_status_.rattle_gain_mode = 0;
    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    ros::Rate loop_rate(20.0);
    primary_rattle_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(1.0).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_rattle_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_rattle_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /rattle/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_rattle_status_.info_traj_send = false; // stop sending

    // regulate point C
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_C_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_C_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    ros::Duration(0.1).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest10(ros::NodeHandle *nh){
    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    primary_rattle_status_.rattle_gain_mode = 0;
    primary_rattle_status_.control_mode = "regulate";


    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    ros::Rate loop_rate(20.0);
    primary_rattle_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(1.0).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_rattle_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_rattle_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /rattle/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_rattle_status_.info_traj_send = false; // stop sending

    // regulate point C
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_C_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_C_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    ros::Duration(0.1).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest11(ros::NodeHandle *nh){
    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    primary_rattle_status_.rattle_gain_mode = 0;
    primary_rattle_status_.control_mode = "regulate";


    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    ros::Rate loop_rate(20.0);
    primary_rattle_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(1.0).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_rattle_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_rattle_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /rattle/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_rattle_status_.info_traj_send = false; // stop sending

    // regulate point C
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_C_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_C_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    ros::Duration(0.1).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest12(ros::NodeHandle *nh){
    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    primary_rattle_status_.rattle_gain_mode = 0;
    primary_rattle_status_.control_mode = "regulate";

    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // // regulate point B
    // bool B = true;
    // set_info_test_regulate_params(B);
    // primary_rattle_status_.control_mode = "regulate_and_get_setpoint";
    // ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    // primary_rattle_status_.control_mode = "regulate";
    
    ros::Rate loop_rate(20.0);
    primary_rattle_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_rattle_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_rattle_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /rattle/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_rattle_status_.info_traj_send = false; // stop sending

    // regulate point C
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_C_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_C_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    ros::Duration(0.1).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest14(ros::NodeHandle *nh){
    /* Tube MPC test
    */
    primary_rattle_status_.rattle_gain_mode = 0;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    std::string input_path = ros::package::getPath("data") + "/input/";
    std::string traj_path;
    if (ground_ == true){
        traj_path = input_path + "TEST14-GND/";  // data/input/traj/
    }
    else {
        traj_path = input_path + "MIT-ISS/";  // data/input/traj/
    }
    NODELET_DEBUG_STREAM(traj_path);

    Eigen::MatrixXd eigen_x_des_traj;
    std::tie(eigen_x_des_traj, std::ignore) = read_traj(traj_path);

    // trigger MPC to track, no delay
    publish_dummy_uc_bound();  // preset uc_bound calculation
    primary_rattle_status_.uc_bound_finished = true;
    primary_rattle_status_.control_mode = "track_tube";

    // wait for MPC to finish track
    while(primary_rattle_status_.traj_finished == false){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest15(ros::NodeHandle *nh){
    /* Standard MPC test
    */
    primary_rattle_status_.rattle_gain_mode = 0;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    std::string input_path = ros::package::getPath("data") + "/input/";
    std::string traj_path;
    if (ground_ == true){
        traj_path = input_path + "TEST15-GND/";  // data/input/traj/
    }
    else {
        traj_path = input_path + "MIT-ISS/";  // data/input/traj/
    }
    NODELET_DEBUG_STREAM(traj_path);

    Eigen::MatrixXd eigen_x_des_traj;
    std::tie(eigen_x_des_traj, std::ignore) = read_traj(traj_path);

    // trigger MPC to track, no delay
    primary_rattle_status_.control_mode = "track";

    // wait for MPC to finish track
    while(primary_rattle_status_.traj_finished == false){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}

/************************************************************************/
void PrimaryNodelet::RunTest16(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest17(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest18(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest19(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}

/************************************************************************/
void PrimaryNodelet::RunTest20(ros::NodeHandle *nh){
    /* RATTLE test: hand off control to RATTLE coordinator
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest21(ros::NodeHandle *nh){
    /* mass only excitation
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest22(ros::NodeHandle *nh){
    /* izz only excitation
    */
    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest77(ros::NodeHandle *nh){
    /* Full RATTLE A to B test, with many options for test type.
    */
    process_rattle_test_number(base_rattle_status_.test_number);  // update rattle params for test77(x)

    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = primary_rattle_status_.rattle_use_params;
    msg.WEIGHT_MODE = primary_rattle_status_.rattle_weight_mode;  // note: see enum definition in primary_nodelet.h
    msg.INITIAL_MODEL_MODE = primary_rattle_status_.rattle_initial_model_mode;  // note: see enum definition in primary_nodelet.h
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 1;  // {0:default, 1:"hard", 2:"fancy"}

    pub_rattle_test_instruct_.publish(msg);

    // wait for RATTLE to finish
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest78(ros::NodeHandle *nh){
    /* Full RATTLE A to B test, with a mandatory replanning
    */
    process_rattle_test_number(base_rattle_status_.test_number);  // update rattle params for test77(x)

    primary_rattle_status_.rattle_gain_mode = 1;
    primary_rattle_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // hand off to RATTLE
    rattle_msgs::RattleTestInstruct msg;

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
    // while(primary_rattle_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_rattle_status_.test_finished = true;
}

/* ************************************************************************** */
void PrimaryNodelet::check_regulate() {
    /* Spin until regulation is complete (within threshold or time limit)
    */
    NODELET_INFO_STREAM("[PRIMARY COORD]: Beginning regulation.");

    ros::Rate loop_rate(10.0);  // Hz
    auto regulate_start = std::chrono::high_resolution_clock::now();
    while (ros::ok() && !base_rattle_status_.regulate_finished) {
        auto regulate_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> regulate_elapsed = regulate_time - regulate_start;

        while (ros::ok() && std::chrono::duration<double>(regulate_elapsed).count() < reg_time_) {
            loop_rate.sleep();
            regulate_time = std::chrono::high_resolution_clock::now();
            regulate_elapsed = regulate_time - regulate_start;
        }
        base_rattle_status_.regulate_finished = true;
    } 

    NODELET_INFO_STREAM("[PRIMARY COORD]: Regulation time: " << ros::Time::now().toSec());
    NODELET_INFO_STREAM("[PRIMARY COORD]: Regulation complete.");
}


void PrimaryNodelet::publish_dummy_uc_bound() {
    NODELET_INFO_STREAM("[PRIMARY COORD]: Beginning UC bound calculation...");
    Eigen::MatrixXf eig_uc_bound_mat(4, 3);
    eig_uc_bound_mat << 0.02, 0.02, 0.02,
                        0.02, -0.02, -0.02,
                        0.002, 0.002, 0.002,
                        -0.002, -0.002, -0.002;  // reasonable values
    std_msgs::Float64MultiArray uc_bound_mat;
    tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);
    pub_uc_bound_.publish(uc_bound_mat);
}


/* ************************************************************************** */
void PrimaryNodelet::uc_bound_status_callback(const rattle_msgs::RattleUCBoundStatus::ConstPtr msg) {
    primary_rattle_status_.uc_bound_finished = msg->uc_bound_finished;
}


/* ************************************************************************** */
void PrimaryNodelet::rattle_status_callback(const rattle_msgs::RattleRattleStatus::ConstPtr msg) {
    primary_rattle_status_.traj_finished = msg->traj_finished;
}


/* ************************************************************************** */
void PrimaryNodelet::casadi_status_callback(const rattle_msgs::RattleCasadiStatus::ConstPtr msg) {
    base_rattle_status_.coord_ok = msg->coord_ok;
    primary_rattle_status_.mrpi_finished = msg->mrpi_finished;
    primary_rattle_status_.traj_finished = msg->traj_finished;
    // primary_rattle_status_.control_mode = msg->control_mode;
}


/* ************************************************************************** */
void PrimaryNodelet::planner_status_callback(const rattle_msgs::RattlePlannerStatus::ConstPtr msg) {
    primary_rattle_status_.lqrrrt_finished = msg->planner_finished;
    primary_rattle_status_.traj_sent = msg->sent_robustMPC;
}


/* ************************************************************************** */
void PrimaryNodelet::info_status_callback(const rattle_msgs::RattleInfoStatus::ConstPtr msg) {
    primary_rattle_status_.info_traj_sent = msg->info_traj_sent;
}


/* ************************************************************************** */
void PrimaryNodelet::control_mode_callback(const std_msgs::String::ConstPtr msg) {
    /* Update control_mode form an external node.
    NB: this is necessary! rattle_coordinator writes directly to control_mode, but it will be overwritten if not updated internally in coordinator.
    */
    primary_rattle_status_.control_mode = msg->data;
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
      primary_rattle_status_.rattle_use_params = 0;
    }
    else if (test_number_str[2] == '1') {
      primary_rattle_status_.rattle_use_params = 1;
    }

    // weight
    if (test_number_str[3] == '0') {
      primary_rattle_status_.rattle_weight_mode = 0;
    }
    else if (test_number_str[3] == '1') {
      primary_rattle_status_.rattle_weight_mode = 1;
    }
    else if (test_number_str[3] == '2') {
      primary_rattle_status_.rattle_weight_mode = 2;
    }
    else if (test_number_str[3] == '3') {
      primary_rattle_status_.rattle_weight_mode = 3;
    }
    else if (test_number_str[3] == '4') {
      primary_rattle_status_.rattle_weight_mode = 4;
    }
    else if (test_number_str[3] == '5') {
      primary_rattle_status_.rattle_weight_mode = 5;
    }

    // ground truth
    if (test_number_str[4] == '0') {
      primary_rattle_status_.rattle_initial_model_mode = 0;
    }
    else if (test_number_str[4] == '1') {
      primary_rattle_status_.rattle_initial_model_mode = 1;
    }
    else if (test_number_str[4] == '2') {
      primary_rattle_status_.rattle_initial_model_mode = 2;
    }
    else if (test_number_str[4] == '3') {
      primary_rattle_status_.rattle_initial_model_mode = 3;
    }
  }
}

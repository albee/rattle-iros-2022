#pragma once
#include "coordinator/primary_nodelet.h"


/************************************************************************/
void PrimaryNodelet::RunTest0(ros::NodeHandle *nh){
    NODELET_INFO_STREAM("[PRIMARY_COORD]: Congratulations, you have passed quick checkout. " 
    "May your days be blessed with only warnings and no errors.");

    disable_default_ctl();

    ros::Duration(5.0).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
};


/************************************************************************/
void PrimaryNodelet::RunTest4(ros::NodeHandle *nh){
    primary_reswarm_status_.lqrrrt_activated = true;
    enable_default_ctl();
    while(ros::ok() && !primary_reswarm_status_.lqrrrt_finished){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    primary_reswarm_status_.lqrrrt_activated = false;
    primary_reswarm_status_.control_mode = "inactive";
    //ros::param::getCached("/reswarm/traj_file", traj_filename_);

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest5(ros::NodeHandle *nh){
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();
    primary_reswarm_status_.lqrrrt_activated = true;

    while(ros::ok() && !primary_reswarm_status_.lqrrrt_finished){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    primary_reswarm_status_.lqrrrt_activated = false;
    primary_reswarm_status_.control_mode = "inactive";
    enable_default_ctl();
    //ros::param::getCached("/reswarm/traj_file", traj_filename_);

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest6(ros::NodeHandle *nh){
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();

    primary_reswarm_status_.lqrrrt_activated = true;
    enable_default_ctl();

    while(ros::ok() && !primary_reswarm_status_.lqrrrt_finished){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    primary_reswarm_status_.lqrrrt_activated = false;
    primary_reswarm_status_.control_mode = "inactive";
    enable_default_ctl();
    //ros::param::getCached("/reswarm/traj_file", traj_filename_);

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest7(ros::NodeHandle *nh){
    /* Test7, the traj smoother with standard tube MPC tracking test.
    (1) Astrobees start near point A (placed by crew).
    (2) Regulate precisely to point A; use standard MPC (if PD: planner_lqrrrt publishes to gnc/ctl/command).
    (3) Track to B using standard MPC.
    (4) Test complete.
    */

    // Default controller is disabled---we regulate and plan
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // LQR/RRT* is called with preset points
    primary_reswarm_status_.lqrrrt_activated = true;  // trigger the lqrrrt planner

    // wait until LQR/RRT* is done
    while(ros::ok() && !primary_reswarm_status_.traj_sent){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    // Below are used for tube MPC TODO: this needs to be calculated based on output of Monica's code
    // publish_dummy_uc_bound();
    // primary_reswarm_status_.reswarm_uc_bound_finished = true;
    primary_reswarm_status_.lqrrrt_activated = false;
    // trigger MPC to track, no delay
    primary_reswarm_status_.control_mode = "track";

    // wait for MPC to finish track
    while(ros::ok() && !primary_reswarm_status_.traj_finished){
        ros::spinOnce();
        sleep_rate.sleep();  
    };

    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    ros::Duration(0.1).sleep();

    primary_reswarm_status_.control_mode = "regulate";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest8(ros::NodeHandle *nh){
    /* Test8, the traj smoother with robust tube MPC tracking test.
    (1) Astrobees start near point A (placed by crew).
    (2) Regulate precisely to point A; use standard MPC (if PD: planner_lqrrrt publishes to gnc/ctl/command).
    (3) Track to B using robust MPC.
    (4) Test complete.
    */

    // Default controller is disabled---we regulate and plan
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // LQR/RRT* is called with preset points
    primary_reswarm_status_.lqrrrt_activated = true;  // trigger the lqrrrt planner

    // wait until LQR/RRT* is done
    while(ros::ok() && !primary_reswarm_status_.traj_sent){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    primary_reswarm_status_.lqrrrt_activated = false;
    // Below are used for tube MPC TODO: this needs to be calculated based on output of Monica's code
    publish_dummy_uc_bound();
    primary_reswarm_status_.uc_bound_finished = true;

    // trigger MPC to track, no delay
    primary_reswarm_status_.control_mode = "track_tube";

    // wait for MPC to finish track
        while(ros::ok() && !primary_reswarm_status_.traj_finished){
        ros::spinOnce();
        sleep_rate.sleep();  
    };

    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    ros::Duration(0.1).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest9(ros::NodeHandle *nh){
    primary_reswarm_status_.reswarm_gain_mode = 0;
    // regulate point B
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    primary_reswarm_status_.control_mode = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    ros::Rate loop_rate(20.0);
    primary_reswarm_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(1.0).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_reswarm_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_reswarm_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /reswarm/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_reswarm_status_.info_traj_send = false; // stop sending

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
    base_reswarm_status_.test_finished = true;
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
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";


    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    ros::Rate loop_rate(20.0);
    primary_reswarm_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(1.0).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_reswarm_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_reswarm_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /reswarm/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_reswarm_status_.info_traj_send = false; // stop sending

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
    base_reswarm_status_.test_finished = true;
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
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";


    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    ros::Rate loop_rate(20.0);
    primary_reswarm_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(1.0).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_reswarm_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_reswarm_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /reswarm/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_reswarm_status_.info_traj_send = false; // stop sending

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
    base_reswarm_status_.test_finished = true;
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
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";

    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

    // // regulate point B
    // bool B = true;
    // set_info_test_regulate_params(B);
    // primary_reswarm_status_.control_mode = "regulate_and_get_setpoint";
    // ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate/ekf settings before disabling default controller.
    // primary_reswarm_status_.control_mode = "regulate";
    
    ros::Rate loop_rate(20.0);
    primary_reswarm_status_.info_traj_send = true; // start sending info rich traj
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc receives x_des_traj
    primary_reswarm_status_.control_mode = "track";

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Sending out info traj");
    while(ros::ok() && !primary_reswarm_status_.info_traj_sent) {
     // keep looping till all of it has been sent (takes about a minute, 2Hz, 120 setpoints)
     // info_traj_sent is published by /reswarm/info_traj_status
        ros::spinOnce();
        loop_rate.sleep();
    }
    primary_reswarm_status_.info_traj_send = false; // stop sending

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
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest13(ros::NodeHandle *nh){
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
    primary_reswarm_status_.description = "regulate";
    ros::Duration(0.4).sleep(); // make sure casadi_nmpc gets the regulate settings before disabling default controller.
    
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: Disabling default controller...");
    disable_default_ctl();
    check_regulate();  // check regulation until satisfied

   // Plan from A to B
   // LQR/RRT* is called with preset points
    primary_reswarm_status_.lqrrrt_activated = true;  // trigger the lqrrrt planner
    primary_reswarm_status_.description = "lqrrrt_plan";
    // wait until LQR/RRT* is done
    while(ros::ok() && !primary_reswarm_status_.traj_sent){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    primary_reswarm_status_.lqrrrt_activated = false;
    // trigger MPC to track, no delay
    primary_reswarm_status_.control_mode = "track";
    primary_reswarm_status_.description = "lqrrrt_track";

    // wait for MPC to finish track
    while(ros::ok() && !primary_reswarm_status_.traj_finished){
        ros::spinOnce();
    };
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ... A to B complete!");

    // regulate point B
    if (ground_ == true){
        reg_setpoint_.col(0) << POINT_B_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_B_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    disable_default_ctl();
    primary_reswarm_status_.description = "regulate";
    ros::Duration(0.4).sleep();

    ros::Duration(15.0).sleep();

    // hand off to RATTLE
    reswarm_msgs::RattleTestInstruct msg;
    primary_reswarm_status_.traj_finished = false;
    primary_reswarm_status_.activate_rattle = true;  // trigger RATTLE (used by inv_fam)
    primary_reswarm_status_.description = "rattle";
    msg.test_num = "full"; // it's actually a string
    msg.USE_EKF_POSE = 1;
    msg.USE_PARAMS = 1;
    msg.OOA_TEST = 1;
    msg.WEIGHT_MODE = step_weight;  // note: see enum definition in primary_nodelet.h
    if (ground_) {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    else {
        msg.INITIAL_MODEL_MODE = incorrect_high;  // note: see enum definition in primary_nodelet.h
    }
    msg.ground = ground_;
    msg.USE_CSV = 0;
    msg.OBS_CONFIG = 3;  // {0:default, 1:"hard", 2:"fancy", 3:"none"}

    pub_rattle_test_instruct_.publish(msg);

    //wait for RATTLE to finish
    while(primary_reswarm_status_.traj_finished == false){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    primary_reswarm_status_.activate_rattle = false;
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ... B to C complete!");

    // izz maneuvers
    // primary_reswarm_status_.traj_finished = false;
    // primary_reswarm_status_.activate_rattle = true;
    // primary_reswarm_status_.description = "rattle_izz";
    // msg.test_num = "full"; // it's actually a string
    // msg.USE_EKF_POSE = 1;
    // msg.USE_PARAMS = 1;
    // msg.OOA_TEST = 2;  // POINT C
    // msg.WEIGHT_MODE = pure_izz_weight;  // note: see enum definition in primary_nodelet.h
    // if (ground_) {
    //     msg.INITIAL_MODEL_MODE = ground_truth_ground;  // note: see enum definition in primary_nodelet.h
    // }
    // else {
    //     msg.INITIAL_MODEL_MODE = ground_truth_iss;  // note: see enum definition in primary_nodelet.h
    // }
    // msg.ground = ground_;
    // msg.USE_CSV = 0;
    // msg.OBS_CONFIG = 3;  // {0:default, 1:"hard", 2:"fancy"}

    // pub_rattle_test_instruct_.publish(msg);
    // //wait for RATTLE to finish
    // while(primary_reswarm_status_.traj_finished == false){
    //     ros::spinOnce();
    //     sleep_rate.sleep();
    // };
    // primary_reswarm_status_.activate_rattle = false;
    // NODELET_DEBUG_STREAM("[PRIMARY COORD]: ... C maneuvers complete!");

    // regulate point C
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_C_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_C_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    primary_reswarm_status_.control_mode = "regulate";
    primary_reswarm_status_.description = "regulate";
    ros::Duration(0.4).sleep();

    ros::Duration(5.0).sleep();

    // Plan from C to A
    // LQR/RRT* is called with preset points
    primary_reswarm_status_.lqrrrt_finished = false;
    primary_reswarm_status_.lqrrrt_activated = true;  // trigger the lqrrrt planner
    primary_reswarm_status_.traj_sent = false;
    primary_reswarm_status_.description = "lqrrrt_plan";

    // wait until LQR/RRT* is done
    while(ros::ok() && !primary_reswarm_status_.traj_sent){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    primary_reswarm_status_.lqrrrt_activated = false;
    // Below are used for tube MPC TODO: this needs to be calculated based on output of Monica's code
    publish_dummy_uc_bound();
    primary_reswarm_status_.uc_bound_finished = true;

    // trigger MPC to track, no delay (use previous updates from RATTLE)
    primary_reswarm_status_.control_mode = "track_tube";
    primary_reswarm_status_.traj_finished = false;
    primary_reswarm_status_.description = "track";
    // wait for MPC to finish track
    while(ros::ok() && !primary_reswarm_status_.traj_finished){
        ros::spinOnce();
        sleep_rate.sleep();  
    };
    
    // regulate point A
    if (ground_ == true){ 
        reg_setpoint_.col(0) << POINT_A_GRANITE;
    }
    else {
        reg_setpoint_.col(0) << POINT_A_ISS;
    }
    pub_reg_setpoint(reg_setpoint_);
    // disable_default_ctl();
    ros::Duration(0.1).sleep();

    primary_reswarm_status_.description = "finished";
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest14(ros::NodeHandle *nh){
    /* Tube MPC test
    */
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
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
    primary_reswarm_status_.uc_bound_finished = true;
    primary_reswarm_status_.control_mode = "track_tube";

    // wait for MPC to finish track
    while(primary_reswarm_status_.traj_finished == false){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunTest15(ros::NodeHandle *nh){
    /* Standard MPC test
    */
    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
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
    primary_reswarm_status_.control_mode = "track";

    // wait for MPC to finish track
    while(primary_reswarm_status_.traj_finished == false){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
}


/************************************************************************/
void PrimaryNodelet::RunDebug(ros::NodeHandle *nh) {
    /* Debugging test (test77).
    */
    NODELET_INFO_STREAM("[PRIMARY_COORD]: Performing debug test.");

    primary_reswarm_status_.reswarm_gain_mode = 0;
    primary_reswarm_status_.control_mode = "regulate";
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

    Eigen::MatrixXd eigen_x_des_traj;
    std::tie(eigen_x_des_traj, std::ignore) = read_traj(traj_path);

    // trigger MPC to track, no delay
    publish_dummy_uc_bound();  // preset uc_bound calculation
    primary_reswarm_status_.uc_bound_finished = true;
    primary_reswarm_status_.control_mode = "track_tube";

    ros::Duration(4.0).sleep();
    send_traj_to_controller(eigen_x_des_traj);

    ros::Duration(4.0).sleep();
    send_traj_to_controller(eigen_x_des_traj);

    // wait for MPC to finish track
    while(primary_reswarm_status_.traj_finished == false){
        ros::spinOnce();
        sleep_rate.sleep();
    };
    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;

    // // sample uc_bound
    // Eigen::MatrixXf eig_uc_bound_mat(4, 3);
    // eig_uc_bound_mat << 0.005, 0.005, 0.005,
    //                     0.005, 0.005, 0.005,
    //                     0.0025, 0.0025, 0.0025,
    //                     -0.0025, -0.0025, 0.0025;  // +pos; -pos; +vel; -vel, per control_dt;

    // // mRPI trigger
    // std_msgs::Float64MultiArray uc_bound_mat;
    // tf::matrixEigenToMsg(eig_uc_bound_mat, uc_bound_mat);

    // ros::Rate loop_rate(10.0);

    // // pub_uc_bound_.publish(uc_bound_mat);
    // //
    // // // Calculate mRPI for tube MPC
    // // NODELET_INFO_STREAM("[COORD]: Beginning mRPI calculation.");
    // // while(ros::ok() && !mrpi_finished_) {
    // //   ros::spinOnce();
    // //   loop_rate.sleep();
    // // }
    // // NODELET_INFO_STREAM("[COORD]: mRPI calculation finished.");
    // //

    // ros::Duration(2.0).sleep();
}


/* ************************************************************************** */
void PrimaryNodelet::check_regulate() {
    /* Spin until regulation is complete (within threshold or time limit)
    */
    NODELET_INFO_STREAM("[PRIMARY COORD]: Beginning regulation.");

    ros::Rate loop_rate(10.0);  // Hz
    auto regulate_start = std::chrono::high_resolution_clock::now();
    while (ros::ok() && !base_reswarm_status_.regulate_finished) {
        auto regulate_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> regulate_elapsed = regulate_time - regulate_start;

        while (ros::ok() && std::chrono::duration<double>(regulate_elapsed).count() < reg_time_) {
            loop_rate.sleep();
            regulate_time = std::chrono::high_resolution_clock::now();
            regulate_elapsed = regulate_time - regulate_start;
        }
        base_reswarm_status_.regulate_finished = true;
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
void PrimaryNodelet::uc_bound_status_callback(const reswarm_msgs::ReswarmUCBoundStatus::ConstPtr msg) {
    primary_reswarm_status_.uc_bound_finished = msg->uc_bound_finished;
}


/* ************************************************************************** */
void PrimaryNodelet::rattle_status_callback(const reswarm_msgs::ReswarmRattleStatus::ConstPtr msg) {
    primary_reswarm_status_.traj_finished = msg->traj_finished;
}


/* ************************************************************************** */
void PrimaryNodelet::casadi_status_callback(const reswarm_msgs::ReswarmCasadiStatus::ConstPtr msg) {
    base_reswarm_status_.coord_ok = msg->coord_ok;
    primary_reswarm_status_.mrpi_finished = msg->mrpi_finished;
    primary_reswarm_status_.traj_finished = msg->traj_finished;
    // primary_reswarm_status_.control_mode = msg->control_mode;
}


/* ************************************************************************** */
void PrimaryNodelet::planner_status_callback(const reswarm_msgs::ReswarmPlannerStatus::ConstPtr msg) {
    primary_reswarm_status_.lqrrrt_finished = msg->planner_finished;
    primary_reswarm_status_.traj_sent = msg->sent_robustMPC;
}


/* ************************************************************************** */
void PrimaryNodelet::info_status_callback(const reswarm_msgs::ReswarmInfoStatus::ConstPtr msg) {
    primary_reswarm_status_.info_traj_sent = msg->info_traj_sent;
}


// TODO: decide if needed for reswarm
// /* ************************************************************************** */
// ff_msgs::ControlState PrimaryNodelet::create_ControlState_from_x_des(int t_idx){
//   /* Using DLR's x_des format, create a ControlState message
//       Input:
//       int t_idx: time index of eigen_x_des_traj_ to use
//   */
//     float t = eigen_x_des_traj_(t_idx, 0);  // data appears to be in inertial frame?
//     NODELET_DEBUG_STREAM(t);
//     float x = eigen_x_des_traj_(t_idx, 1);  // wrt frame @ t=0
//     float y = eigen_x_des_traj_(t_idx, 2);
//     float z = eigen_x_des_traj_(t_idx, 3);
//     float xd = eigen_x_des_traj_(t_idx, 4);
//     float yd = eigen_x_des_traj_(t_idx, 5);
//     float zd = eigen_x_des_traj_(t_idx, 6);

//     float qw = eigen_x_des_traj_(t_idx, 7);  // ang_vel in inertial frame
//     float qx = eigen_x_des_traj_(t_idx, 8);  // att in inertial frame
//     float qy = eigen_x_des_traj_(t_idx, 9);
//     float qz = eigen_x_des_traj_(t_idx, 10);
//     float wx = eigen_x_des_traj_(t_idx, 11);
//     float wy = eigen_x_des_traj_(t_idx, 12);
//     float wz = eigen_x_des_traj_(t_idx, 13);

//     float xdd = eigen_x_des_traj_(t_idx, 14);  // accel in inertial frame
//     float ydd = eigen_x_des_traj_(t_idx, 15);
//     float zdd = eigen_x_des_traj_(t_idx, 16);
//     float wxd = eigen_x_des_traj_(t_idx, 17);
//     float wyd = eigen_x_des_traj_(t_idx, 18);
//     float wzd = eigen_x_des_traj_(t_idx, 19);

//     // set up the trajectory publishing message
//     geometry_msgs::Point traj_pos;
//     geometry_msgs::Vector3 traj_vel;
//     geometry_msgs::Quaternion traj_quat;
//     geometry_msgs::Vector3 traj_omega;

//     // position
//     traj_pos.x = x;
//     traj_pos.y = y;
//     traj_pos.z = z;

//     // velocity
//     traj_vel.x = xd;
//     traj_vel.y = yd;
//     traj_vel.z = zd;

//     // attitude
//     traj_quat.x = qx;
//     traj_quat.y = qy;
//     traj_quat.z = qz;
//     traj_quat.w = qw;

//     // point to primary

//     // angular rate
//     traj_omega.x = wx;
//     traj_omega.y = wy;
//     traj_omega.z = wz;

//     // desired acceleration is 0
//     geometry_msgs::Vector3 traj_accel;
//     traj_accel.x = xdd;
//     traj_accel.y = ydd;
//     traj_accel.z = zdd;

//     // desired angular acceleration is 0 (will take a little while to get to smooth, constant w)
//     geometry_msgs::Vector3 traj_alpha;
//     traj_alpha.x = wxd;
//     traj_alpha.y = wyd;
//     traj_alpha.z = wzd;

//     if (ground_.compare("true") == 0) {  // zero out z-axis components
//       traj_pos.z = -0.7;
//       traj_vel.z = 0.0;

//       // zero quat and normalize
//       tf2::Quaternion q_gnd(0, 0, qz, qw);
//       q_gnd.normalize();
//       traj_quat.x = q_gnd[0];
//       traj_quat.y = q_gnd[1];
//       traj_quat.z = q_gnd[2];
//       traj_quat.w = q_gnd[3];

//       traj_omega.x = 0.0;
//       traj_omega.y = 0.0;

//       traj_accel.z = 0.0;
//       traj_alpha.x = 0.0;
//       traj_alpha.y = 0.0;
//     }

//     // Create the trajectory setpoint message
//     ff_msgs::ControlState new_state;
//     new_state.when = ros::Time::now();;
//     new_state.pose.position = traj_pos;
//     new_state.pose.orientation = traj_quat;
//     new_state.twist.linear = traj_vel;
//     new_state.twist.angular = traj_omega;
//     new_state.accel.linear = traj_accel;
//     new_state.accel.angular = traj_alpha;

//     return new_state;
// }


// TODO: decide if needed for reswarm
// /* ************************************************************************** */
// Eigen::Matrix3f PrimaryNodelet::q2dcm(const Eigen::Vector4f &q) {
//   Eigen::Matrix3f dcm;
//   dcm(0,0) = pow(q(3),2) + pow(q(0),2) - pow(q(1),2) - pow(q(2),2);
//   dcm(0,1) = 2*(q(0)*q(1) + q(3)*q(2));
//   dcm(0,2) = 2*(q(0)*q(2) - q(3)*q(1));
//   dcm(1,0) = 2*(q(0)*q(1) - q(3)*q(2));
//   dcm(1,1) = pow(q(3),2) - pow(q(0),2) + pow(q(1),2) - pow(q(2),2);
//   dcm(1,2) = 2*(q(1)*q(2) + q(3)*q(0));
//   dcm(2,0) = 2*(q(0)*q(2) + q(3)*q(1));
//   dcm(2,1) = 2*(q(1)*q(2) - q(3)*q(0));
//   dcm(2,2) = pow(q(3),2) - pow(q(0),2) - pow(q(1),2) + pow(q(2),2);
//   return dcm.transpose();
// }

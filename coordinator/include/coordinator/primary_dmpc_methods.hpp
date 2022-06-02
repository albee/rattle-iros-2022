#pragma once
#include "coordinator/primary_nodelet.h"
#include <typeinfo>

void PrimaryNodelet::RunTest1(ros::NodeHandle *nh){
    RunDMPC(nh);
    NODELET_INFO_STREAM("Test 1 finished cleanly!");
};

void PrimaryNodelet::RunTest2(ros::NodeHandle *nh){
    RunDMPC(nh);
    NODELET_INFO_STREAM("Test 2 finished cleanly!");
};

void PrimaryNodelet::RunTest3(ros::NodeHandle *nh){
    RunDMPC(nh);
    NODELET_INFO_STREAM("Test 3 finished cleanly!");
};

void PrimaryNodelet::RunDMPC(ros::NodeHandle *nh){
    load_params();
    LaunchDMPC();

    NODELET_INFO_STREAM("Create variables...");
    std::string start_service_str;
    ros::ServiceClient start_service;

    NODELET_INFO_STREAM("Preparing service call...");
    start_service_str = "/";
    if(sim_) start_service_str.append("queen/");
    start_service_str.append("start");
    
    NODELET_INFO_STREAM("Creating client...");
    start_service = nh->serviceClient<std_srvs::SetBool>(start_service_str.c_str());
    
    NODELET_INFO_STREAM("Waiting for service to be alive...");
    start_service.waitForExistence();
    
    // Start test
    std_srvs::SetBool start_srv_req;
    start_srv_req.request.data = true;
    NODELET_INFO_STREAM("Calling service: " << start_service_str);
    start_service.call(start_srv_req);
    if(!start_srv_req.response.success) ROS_ERROR_STREAM("Failed to start test!");

    // Wait for termination
    ros::Rate wait_for_test(1.0);
    base_reswarm_status_.test_finished = false;
    while(ros::ok() && base_reswarm_status_.test_finished != true) wait_for_test.sleep();

    // Kill nodes
    KillDMPC();
}

void PrimaryNodelet::LaunchDMPC(){
    int system_ret;
    std::string launch_command;

    // Parse test number
    launch_command = "roslaunch reswarm_dmpc primary_";
    if (base_reswarm_status_.test_number == 1 ||
          (base_reswarm_status_.test_number >= 100 && base_reswarm_status_.test_number <= 199) || 
          (base_reswarm_status_.test_number >= 10000 && base_reswarm_status_.test_number <= 19999)){
        launch_command.append("t1.launch ");
    }else if(base_reswarm_status_.test_number == 2 ||
          (base_reswarm_status_.test_number >= 200 && base_reswarm_status_.test_number <= 299) ||
          (base_reswarm_status_.test_number >= 20000 && base_reswarm_status_.test_number <= 29999)) {
        launch_command.append("t2.launch ");
    }else if (base_reswarm_status_.test_number == 3 ||
          (base_reswarm_status_.test_number >= 300 && base_reswarm_status_.test_number <= 399) ||
          (base_reswarm_status_.test_number >= 30000 && base_reswarm_status_.test_number <= 39999) ) {
        launch_command.append("t3.launch ");
    }else{
        NODELET_ERROR_STREAM("Wrong DMPC Test Number!");
        return;
    }

    // Add argument for weights configuration, main test number (1, 2, 3) is the best weights
    launch_command = launch_command.append("param:=");
    launch_command = launch_command.append(std::to_string(base_reswarm_status_.test_number));
    launch_command = launch_command.append(" ");

    // Check for HW/SIM and GND/ISS - defaults values: nspace:=/ , ground:=true
    if(sim_){
        launch_command = launch_command.append("nspace:=/queen ");
    }

    if(ground_){
        launch_command = launch_command.append("ground:=true ");
    }

    if(ground_ && !sim_){
        launch_command = launch_command.append("partner:=wannabee ");
    }

    // Non-blocking system call
    launch_command.append("&");
    
    NODELET_INFO_STREAM("Calling " << launch_command);
    system_ret = system(launch_command.c_str());
    if(system_ret != 0){
        NODELET_ERROR_STREAM("[PRIMARY/DMPC] Failed to Launch DMPC nodes.");
    }
    return;
};

void PrimaryNodelet::KillDMPC(){
    int system_ret;

    if(sim_){
        system_ret = system("rosnode kill /queen/primary_dmpc_iface /queen/primary_dmpc_ctl_node &");
    }else{
        system_ret = system("rosnode kill /primary_dmpc_iface /primary_dmpc_ctl_node &");
    }

    if(system_ret != 0){
        NODELET_ERROR_STREAM("[PRIMARY/DMPC] Failed to Kill DMPC nodes.");
    }
};

/* ************************************************************************** */
void PrimaryNodelet::dmpc_status_cb(const reswarm_dmpc::DMPCTestStatusStamped::ConstPtr msg){

    // Updated internal variables with received status
    base_reswarm_status_.test_finished = msg->test_finished;
    primary_reswarm_status_.solver_status = msg->solver_status;
    primary_reswarm_status_.cost_value = msg->cost_value;
    primary_reswarm_status_.kkt_value = msg->kkt_value;
    primary_reswarm_status_.sol_time = msg->sol_time;
}
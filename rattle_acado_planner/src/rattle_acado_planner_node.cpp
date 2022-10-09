/*
A ROS node for calling the ACADO trajectory optimization software and 
providing info-aware plans.

Monica Ekal and Keenan Albee, 2021.
Credit to ALG Prasad and Sambaran Ghoshal for ACADO integration details.
*/
#include "rattle_acado_planner/rattle_acado_planner_node.h"


RattlePlannerNode::RattlePlannerNode(ros::NodeHandle* nh) {
  this->nh = nh;

  if (!use_global_setpoints){
      sub_current_pose_ = nh->subscribe("gnc/ekf", 5, &RattlePlannerNode::ekf_state_callback, this);
  }

  // if using parameter updates, subscribe to /mob/inertia_est, which are the values coming in from the EKF-based sequential param
  sub_estimated_inertial_params_ = nh->subscribe("mob/inertia_est", 1, &RattlePlannerNode::update_parameters_callback, this);
  // sub_current_inertial_params_ = nh->subscribe("mob/inertia", 1, &RattlePlannerNode::parameters_callback, this);
  sub_des_ = nh->subscribe("rattle/local/info_plan_instruct", 5, &RattlePlannerNode::info_plan_instruct_callback, this);  // request a plan
  sub_des_init = nh->subscribe("rattle/local/info_plan_instruct_start", 5, &RattlePlannerNode::info_plan_instruct_start_callback, this);  // desired goal waypoint--important!
  sub_rattle_instruct_= nh->subscribe<rattle_msgs::RattleTestInstruct>("/rattle/test_instruct", 5, boost::bind(&RattlePlannerNode::rattle_instruct_callback, this, _1));

  // Publisher to send out latest plan
  pub_local_path_posearray_ = nh->advertise<geometry_msgs::PoseArray>("rattle/local/path/posearray", 5, true);  // outgoing FAM commands: /honey/gnc/ctl/command
  pub_local_path_twistarray_ = nh->advertise<rattle_rrt::TwistArray>("rattle/local/path/twistarray", 5, true);
  pub_local_path_wrencharray_ = nh->advertise<rattle_rrt::WrenchArray>("rattle/local/path/wrencharray", 5, true);
  pub_weights_ = nh->advertise<std_msgs::Float64MultiArray>("rattle/local/weights", 5, true);  // weightings in local planner
  pub_psi_ = nh->advertise<std_msgs::Float64MultiArray>("rattle/local/psi", 5, true);  // psi values

  ros::Duration(1.0).sleep();  // need to wait for parameter to be sent
  nh->param<std::string>("/rattle/ground", ground_, "false");
  nh->getParam("/rattle/planner/max_RTI_iter_", max_RTI_iter_);
  nh->getParam("/rattle/planner/Wx", Wx_);
  nh->getParam("/rattle/planner/Wu", Wu_);
  nh->getParam("/rattle/planner/WN", WN_);

  get_cost_matrix();  // force update
}


void RattlePlannerNode::pub_planner_output() {
  // Get the forces and torques
  rattle_rrt::WrenchArray wrencharray;
  for (int i = 0; i < N; ++i) {
    geometry_msgs::Wrench wrench;

    wrench.force.x = acadoVariables.u[0 + i*NU];
    wrench.force.y = acadoVariables.u[1 + i*NU];
    wrench.torque.z = acadoVariables.u[5 + i*NU];

    if (ground_.compare("true") == 0) {
        wrench.force.z = 0;   // acadoVariables.u[2];
        wrench.torque.x = 0;  // acadoVariables.u[3];
        wrench.torque.y = 0;  // acadoVariables.u[4];
    } else {
        wrench.force.z = acadoVariables.u[2 + i*NU];
        wrench.torque.x= acadoVariables.u[3 + i*NU];
        wrench.torque.y = acadoVariables.u[4 + i*NU];
    }
    wrencharray.WrenchArray.push_back(wrench);
  }

  // Get the poses and twists (and psi)
  geometry_msgs::PoseArray posearray;
  rattle_rrt::TwistArray twistarray;
  Eigen::MatrixXd psi_eig = Eigen::MatrixXd::Zero(N, NX);
  Eigen::MatrixXd weights_eig = Eigen::MatrixXd::Zero(1, NY);

  for (int i = 0; i < N; i++) {
    geometry_msgs::Pose pose;
    geometry_msgs::TwistStamped twist_stmp;

    pose.position.x = acadoVariables.x[0 + i*NX];
    pose.position.y = acadoVariables.x[1 + i*NX];

    pose.orientation.x = acadoVariables.x[6 + i*NX];
    pose.orientation.y = acadoVariables.x[7 + i*NX];
    pose.orientation.z = acadoVariables.x[8 + i*NX];
    pose.orientation.w = acadoVariables.x[9 + i*NX];  // return the actual quaternion

    twist_stmp.header.seq = i;
    twist_stmp.twist.linear.x = acadoVariables.x[3 + i*NX];
    twist_stmp.twist.linear.y = acadoVariables.x[4 + i*NX];
    twist_stmp.twist.angular.z = acadoVariables.x[12 + i*NX];

    // everything else is psi
    for (int j = 13; j < NX; j++) {
      // std::cout << j + i*NX << ": " << acadoVariables.x[j + i*NX] << std::endl;
      psi_eig(i, j - 13) = acadoVariables.x[j + i*NX];
    } 

    for (int i = 0; i < NY; i++) {
      for (int j = 0; j < NY; j++) {
        if( i==j ) {
          weights_eig(0, i) = acadoVariables.W[i * NY + j];
        }
      }
    }

    if (ground_.compare("true") == 0) {
        pose.position.z = 0.0;
        twist_stmp.twist.linear.z = 0.0;
        twist_stmp.twist.angular.x = 0.0;
        twist_stmp.twist.angular.y = 0.0;
    } else {
        pose.position.z = acadoVariables.x[2 + i*NX];
        twist_stmp.twist.linear.z = acadoVariables.x[5 + i*NX];
        twist_stmp.twist.angular.x = acadoVariables.x[10 + i*NX];
        twist_stmp.twist.angular.y = acadoVariables.x[11 + i*NX];
    }

    posearray.poses.push_back(pose);
    posearray.header.frame_id = "world";  // needed for rviz to display, might also be "map"
    twistarray.TwistArray.push_back(twist_stmp);
  }

  // compute FIM after-the-fact
  // acado_rhp_rattle::FI fim;
  // measurement model jacobian
  // Eigen::MatrixXd dh_dx(3, 9);  // full 3DoF states (rx, ry, vx, vy, qx, qy, qz, qw, wz) for other jacobians
  // dh_dx<< Eigen::MatrixXd::Zero(3, 9);
  // dh_dx(0, 2) = 1;
  // dh_dx(1, 3) = 1;
  // dh_dx(2, 8) = 1;
  // Eigen::MatrixXd Sigma(3, 3);  // measurement covariance, get this from the EKF, ~ 1e-7
  // Sigma << Eigen::MatrixXd::Identity(3, 3) * pow(10, -7);

  // Eigen::VectorXd FIM_diag(4);
  // Eigen::MatrixXd dx_dtheta(9, 4);
  // Eigen::MatrixXd FIM_local(4, 4);
  // Eigen::MatrixXd FIM_total(4, 4);
  // FIM_total << Eigen::MatrixXd::Zero(4, 4);

  // // Populate dx_dtheta
  // Eigen::VectorXd psi_k(n_psi);
  // for (int i = 0; i < N; i++) {
  //   for (int j = 0; j < n_psi; j++) {
  //     psi_k(j) = acadoVariables.x[13 + j + i * NX];
  //   }
  //   dx_dtheta.col(0) << psi_k[0], psi_k[1], psi_k[2], psi_k[3], Eigen::VectorXd::Zero(5);  // first col of dx_dtheta
  //   // zero vector is used because mass does not influence the reduced rotation states (qx, qy, qz, qw, wz), and the
  //   // corresponding jacobian elements are zero.
  //   dx_dtheta.col(1) << psi_k.segment(4, 9);   // 9 elements starting at position 4
  //   dx_dtheta.col(2) << psi_k.segment(13, 9);  // 9 elements starting at position 13
  //   dx_dtheta.col(3) << psi_k.segment(22, 9);  // 9 elements starting at position 22

  //   FIM_local = (dh_dx * dx_dtheta).transpose() * Sigma.inverse() * (dh_dx * dx_dtheta);
  //   // FIM over horizon = sum of FIM local
  //   FIM_total += FIM_local;
  // }

  // FIM_diag = FIM_total.diagonal();  // in case knowing the FIM over the horizon for each parameter becomes important
  
  /*
  fim.FI_trace = FIM_total.trace();
  fim.FI_diag = {FIM_diag(0), FIM_diag(1), FIM_diag(2),
                  FIM_diag(3)};  // find a better mapping from Eigenvector to array
  fim.header.stamp = ros::Time::now(); 
  pub_FIM_over_horizon_.publish(fim);
  */

  pub_local_path_posearray_.publish(posearray);
  pub_local_path_twistarray_.publish(twistarray);
  pub_local_path_wrencharray_.publish(wrencharray);

  std_msgs::Float64MultiArray psi_msg;
  tf::matrixEigenToMsg(psi_eig, psi_msg);  // Eigen --> msg
  pub_psi_.publish(psi_msg);

  std_msgs::Float64MultiArray weights_msg;
  tf::matrixEigenToMsg(weights_eig, weights_msg);  // Eigen --> msg
  pub_weights_.publish(weights_msg);
  
}


void RattlePlannerNode::parameters_callback(const geometry_msgs::InertiaStamped::ConstPtr& msg) {
  /* The `mob/inertia` subscriber callback
  TODO: Not in use! Initial parameters manually specified.
  */
  std::cout << "Mass parameters updated..." << std::endl;
  params_model_.mass = msg->inertia.m;
  params_model_.ixx = msg->inertia.ixx;
  params_model_.iyy = msg->inertia.iyy;
  params_model_.izz = msg->inertia.izz;
  params_model_.ixy = msg->inertia.ixy;
  params_model_.ixz = msg->inertia.ixz;
  params_model_.iyz = msg->inertia.iyz;
  params_model_.cx = msg->inertia.com.x;
  params_model_.cy = msg->inertia.com.y;
  params_model_.cz = msg->inertia.com.z;

  std::cout << params_model_ << std::endl;
}


void RattlePlannerNode::update_parameters_callback(const param_est::Params::ConstPtr& msg) {
  /* The `mob/inertia_est` subscriber callback
  */
  //std::cout << "Updating mass parameters using the latest estimates..." << std::endl;
  // keep updating the parameters as long as they are physically feasible
  if (use_params_) {
    params_est_.mass = msg->inertia.m;
    params_est_.izz = msg->inertia.izz;

    if (ground_.compare("true") == 0) {  // 3DOF
      if (params_est_.mass > mass_lb_) {
        params_model_.mass = params_est_.mass;
      }
      if (params_est_.izz > inertia_lb_) {
        params_model_.izz = params_est_.izz;
      }
    }
    else {  // 6DOF
      params_est_.ixx = msg->inertia.ixx;
      params_est_.iyy = msg->inertia.iyy;

      if (params_est_.mass > mass_lb_) {
        params_model_.mass = params_est_.mass;
      }
      if (params_est_.ixx > inertia_lb_) {
        params_model_.ixx = params_est_.ixx;
      }
      if (params_est_.iyy > inertia_lb_) {
        params_model_.iyy = params_est_.iyy;
      }
      if (params_est_.izz > inertia_lb_) {
        params_model_.izz = params_est_.ixx;
      }
    }
  }
}


void RattlePlannerNode::info_plan_instruct_start_callback(const rattle_msgs::RattleInfoPlanInstruct::ConstPtr& msg) {
  /* Set the start state x0
  */
 if (use_params_) {
  acado_startup();  // we need to reinitialize the solver for new OnlineData or the cost landscape is too different
 }

  acadoVariables.x0[0] = msg->x;
  acadoVariables.x0[1] = msg->y;
  acadoVariables.x0[2] = msg->z;
  acadoVariables.x0[3] = msg->vel_x;
  acadoVariables.x0[4] = msg->vel_y;
  acadoVariables.x0[5] = msg->vel_z;
  acadoVariables.x0[6] = msg->quat1;
  acadoVariables.x0[7] = msg->quat2;
  acadoVariables.x0[8] = msg->quat3;
  acadoVariables.x0[9] = msg->quat4;
  acadoVariables.x0[10] = msg->ang_vel_x;
  acadoVariables.x0[11] = msg->ang_vel_y;
  acadoVariables.x0[12] = msg->ang_vel_z;

  update_psi();
  ROS_INFO_STREAM("...local plan x0 initialized.");
}


/* ************************************************************************** */
void RattlePlannerNode::rattle_instruct_callback(const rattle_msgs::RattleTestInstruct::ConstPtr& msg)  {
    /* Configuration options for RATTLE.
    @msg USE_PARAMS: use paramter updates {0, 1}
    @msg INITIAL_MODEL_MODE: mode to use for calc, see enum
    */
    use_params_ = msg->USE_PARAMS;
    initial_model_mode_ = msg->INITIAL_MODEL_MODE;

    params_model_ = initial_mode_vec[initial_model_mode_];
}


void RattlePlannerNode::info_plan_instruct_callback(const rattle_msgs::RattleInfoPlanInstruct::ConstPtr& msg) {
  /* Get the desired position/orientation for ACADO to track, along with weighting info
  */
  ROS_INFO("...latest goal and info weights updated.");
  lin_des[0] = msg->x;
  lin_des[1] = msg->y;
  lin_des[2] = msg->z;
  lin_des[3] = msg->vel_x;
  lin_des[4] = msg->vel_y;
  lin_des[5] = msg->vel_z;
  orientation_des[0] = msg->quat1;
  orientation_des[1] = msg->quat2;
  orientation_des[2] = msg->quat3;
  orientation_des[3] = msg->quat4;
  orientation_des[4] = msg->ang_vel_x;
  orientation_des[5] = msg->ang_vel_y;
  orientation_des[6] = msg->ang_vel_z;
  STATUS = msg->status;
  INFO_WEIGHT_M = msg->info_weight_m;
  INFO_WEIGHT_IXX = msg->info_weight_Ixx;
  INFO_WEIGHT_IYY = msg->info_weight_Iyy;
  INFO_WEIGHT_IZZ = msg->info_weight_Izz;
  INFO_WEIGHT_IXY = msg->info_weight_Ixy;
  INFO_WEIGHT_IYZ = msg->info_weight_Iyz;
  INFO_WEIGHT_IXZ = msg->info_weight_Ixz;
  INFO_WEIGHT_CX = msg->info_weight_cx;
  INFO_WEIGHT_CY = msg->info_weight_cy;
  INFO_WEIGHT_CZ = msg->info_weight_cz;

  update_online_data(&params_model_);
  use_latest_info_gain();

  ROS_INFO_STREAM("Searching for local plan...");

  create_local_plan();  // update acadoVariables
  pub_planner_output();
}


void RattlePlannerNode::ekf_state_callback(const ff_msgs::EkfState::ConstPtr& msg) {
  /* The `gnc/ekf` subscriber callback
  Update ACADO with latest pose information
  */
  acadoVariables.x0[0] = msg->pose.position.x;
  acadoVariables.x0[1] = msg->pose.position.y;
  acadoVariables.x0[2] = msg->pose.position.z;
  acadoVariables.x0[3] = msg->velocity.x;
  acadoVariables.x0[4] = msg->velocity.y;
  acadoVariables.x0[5] = msg->velocity.z;
  acadoVariables.x0[6] = msg->pose.orientation.x;
  acadoVariables.x0[7] = msg->pose.orientation.y;
  acadoVariables.x0[8] = msg->pose.orientation.z;
  acadoVariables.x0[9] = msg->pose.orientation.w;
  acadoVariables.x0[10] = msg->omega.x;
  acadoVariables.x0[11] = msg->omega.y;
  acadoVariables.x0[12] = msg->omega.z;
  
  update_psi();
}


int main(int argc, char** argv) {
  /* Start node and start up ROS
  */
  // ROS_INFO_STREAM("N is " << ACADO_N << "\n" << "NY is " << ACADO_NY);

  ros::init(argc, argv, "rattle_acado_planner");
  ros::NodeHandle nh;

  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  if (!VERBOSE) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
    ros::console::notifyLoggerLevelsChanged();
  }
  RattlePlannerNode planner(&nh);

  ROS_INFO_STREAM("[RATTLE_PLANNER]: Initialized.");

  while (ros::ok()) {  // look for incoming requests!
    ros::spinOnce();
  }

  return 0;
}
/*
rrt_nodelet.cc

This is a nodelet wrapper around an RRT generator.

Publishing to `rattle/rrt/params` will trigger a calculation.

Output is on `rattle/rrt/path/posearray` and `rattle/rrt/path/twistarray`

*/

#include <rattle_rrt/rattle_rrt_nodelet.h>


namespace rrt {
 
 /* ************************************************************************** */
void RRTNodelet::Initialize(ros::NodeHandle* nh) {
  /* This is called when the nodelet is loaded into the nodelet manager
  */
  NODELET_INFO_STREAM("[RATTLE_RRT]: Initialized.");

  // publishers and subscribers
  sub_rrt_params_ = nh->subscribe<rattle_rrt::RRTParams>("rattle/rrt/params", 5, boost::bind(&RRTNodelet::rrt_params_callback, this, _1));  // incoming RRT parameters
  sub_est_params_ = nh->subscribe<param_est::Params>("mob/inertia_est", 5, boost::bind(&RRTNodelet::update_parameters_callback, this, _1));
  // sub_obs_ = nh->subscribe<rattle_rrt::ellipsoidArray>("rattle/obstacles", 5, boost::bind(&RRTNodelet::rrt_obs_callback, this, _1));  // incoming obstacle definitions
  sub_rattle_instruct_= nh->subscribe<reswarm_msgs::RattleTestInstruct>("/rattle/test_instruct", 5, boost::bind(&RRTNodelet::rattle_instruct_callback, this, _1));
  
  pub_rrt_path_posearray_ = nh->advertise<geometry_msgs::PoseArray>("rattle/rrt/path/posearray", 5, true);  // outgoing pose history
  pub_rrt_path_twistarray_ = nh->advertise<rattle_rrt::TwistArray>("rattle/rrt/path/twistarray", 5, true);  // outgoing twist history
  pub_rrt_params_ = nh->advertise<rattle_rrt::RRTParams>("rattle/rrt/params", 5, true);  // updated rrt parameters (call request)
  pub_rrt_marker_ = nh->advertise<visualization_msgs::Marker>("rattle/rrt/tree", 5, true);  // tree viz

  ros::Duration(1.0).sleep();  // need to wait for parameter to be sent
  ros::param::get("/rattle/rrt/ymin", ymin_iss_ros_);

  thread_.reset(new std::thread(&rrt::RRTNodelet::Run, this));
}


void RRTNodelet::Run() {
  /* ROS spin loop
  */
  while(ros::ok()) {
    ros::spinOnce();
  }
}


// TODO: would be beneficial to change to a rosservice
/* ************************************************************************** */
void RRTNodelet::generate_rrt(double start_pos[7], double goal_pos[7], Params& model_params) {
  /* Create an RRT based on the provided start and goal positions

  @param model_params: parameter struct for NE dynamics

  ground_: {false, true}, determines if 3DOF or 6DOF dynamics are used.
  */
  std::string RRT_OUTPUT_FILE = "latest_rrt_plan.csv";

  std::shared_ptr<rrt::Tree<rrt::Vec4>> rrt_ground;
  shared_ptr<rrt::StateSpace4D> state_space_ptr_4D; // state space in use
  
  std::shared_ptr<rrt::Tree<rrt::Vec6>> rrt_iss;
  shared_ptr<rrt::StateSpace6DISS> state_space_ptr_6D; // state space in use
  double t1;

  NODELET_INFO_STREAM("RRT version is ground: " << ground_);
  NODELET_INFO_STREAM("RRT model is:"  << params_model_);

  try {
    if (ground_ == false) {
      // setup
      std::tie(rrt_iss, state_space_ptr_6D) = setup_6d_iss(start_pos, goal_pos, params_model_);
      state_space_ptr_6D->initialize_bullet(obstacles_);  // obstacles are set on rrt_params_callback

      // run
      t1 = ros::Time::now().toSec();
      rrt_iss->build_RRT();
      path_ = rrt_iss->get_path_nodes();   // std::vector of nodes with state_ [x, y, z, vx, vy, vz]
      rrt_iss->write_path(RRT_OUTPUT_FILE);

      rrt_path_publish(path_, start_pos);
      pub_rrt_rviz_markers(rrt_iss->get_rrt_edges());  // get all the RRT edges
    }
    else if (ground_ == true) {
      // RBD params (for Astrobee)
      // setup
      std::tie(rrt_ground, state_space_ptr_4D) = setup_4d(start_pos, goal_pos, params_model_);
      state_space_ptr_4D->initialize_bullet(obstacles_);  // obstacles are set on rrt_params_callback

      // run
      t1 = ros::Time::now().toSec();
      rrt_ground->build_RRT();
      path_ground_ = rrt_ground->get_path_nodes();  // std::vector of nodes with state_ [x, y, vx, vy]
      rrt_ground->write_path(RRT_OUTPUT_FILE);

      rrt_path_publish(path_ground_, start_pos);
      pub_rrt_rviz_markers(rrt_ground->get_rrt_edges());  // get all the RRT edges
    }

    NODELET_INFO_STREAM("...RRT was built");
    NODELET_INFO_STREAM("Time " << ros::Time::now().toSec() - t1);  

  }
  catch (const std::exception &exc) {
    std::cerr << exc.what() << std::endl;
    std::cout << "[RRT_NODELET]: The RRT tried to run but instead we've ended up here :c" << std::endl;
  }
}


/* ************************************************************************** */
std::tuple<std::shared_ptr<rrt::Tree<rrt::Vec4>>, shared_ptr<rrt::StateSpace4D>>
RRTNodelet::setup_4d(double start_pos[7], double goal_pos[7], Params& params) {
  /* 2 DoF, planar dynamics
  */

  // Statespace params
  rrt::Vec4 min_bounds;
  double min_x = -0.8;
  double min_y = -0.8;
  double min_xd = -.05;
  double min_yd = -.05;
  min_bounds << min_x, min_y, min_xd, min_yd;

  rrt::Vec4 max_bounds;
  double max_x = 0.8;
  double max_y = 0.8;
  double max_xd = .05;
  double max_yd = .05;
  max_bounds << max_x, max_y, max_xd, max_yd;

  rrt::Vec4 start;
  start << start_pos[0], start_pos[1], 0, 0;

  rrt::Vec4 goal;
  goal << goal_pos[0], goal_pos[1], 0, 0;
  // params
  int max_iterations = 10000;
  bool is_ASC_enabled = false;
  bool remember_actions = true;
  double timestep = 2.0;
  double max_timestep = 2.0;
  double goal_bias = 0.05;
  double goal_max_dist = 5.0;

  // create the state space
  shared_ptr<rrt::StateSpace4D> state_space_ptr_4d =
    make_shared<rrt::StateSpace4D> (min_bounds, max_bounds, params.mass, params.ixx, params.iyy, params.izz);

  // now, make the tree
  shared_ptr<rrt::Tree<rrt::Vec4>> rrt_ground =
    make_shared<rrt::Tree<rrt::Vec4>>(state_space_ptr_4d);  // this is the rrt tree data structure, as a shared_ptr

  rrt_ground->start_state_ = start;
  rrt_ground->goal_state_ = goal;
  rrt_ground->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions, obstacles_);

  return std::make_tuple(rrt_ground, state_space_ptr_4d);
}


/* ************************************************************************** */
std::tuple<std::shared_ptr<rrt::Tree<rrt::Vec6>>, shared_ptr<rrt::StateSpace6DISS>>
RRTNodelet::setup_6d_iss(double start_pos[7], double goal_pos[7], Params& model_params) {
  /*
  6 DoF, translational dynamics (ISS)
  */

  // Statespace params
  rrt::Vec6 min_bounds;
  double min_x = 10.3;
  double min_y = ymin_iss_ros_;
  double min_z = 4.2;
  double min_xd = -.05;
  double min_yd = -.05;
  double min_zd = -.05;
  min_bounds << min_x, min_y, min_z, min_xd, min_yd, min_zd;

  rrt::Vec6 max_bounds;
  double max_x = 11.2;
  double max_y = -7.5;
  double max_z = 5.3;
  double max_xd = .05;
  double max_yd = .05;
  double max_zd = .05;
  max_bounds << max_x, max_y, max_z, max_xd, max_yd, max_zd;

  rrt::Vec6 start;
  start << start_pos[0], start_pos[1], start_pos[2],
            0, 0, 0;

  rrt::Vec6 goal;
  goal << goal_pos[0], goal_pos[1], goal_pos[2],
            0, 0, 0;
  // params
  int max_iterations = 10000;
  bool is_ASC_enabled = false;
  bool remember_actions = true;
  double timestep = 2.0;
  double max_timestep = 2.0;
  double goal_bias = 0.05;
  double goal_max_dist = 0.2;  // primarily position-weighted

  // create the state space
  shared_ptr<rrt::StateSpace6DISS> state_space_ptr_6d =
    make_shared<rrt::StateSpace6DISS> (min_bounds, max_bounds, model_params.mass, model_params.ixx, model_params.iyy, model_params.izz);

  // now, make the tree
  shared_ptr<rrt::Tree<rrt::Vec6>> rrt_iss =
    make_shared<rrt::Tree<rrt::Vec6>>(state_space_ptr_6d);  // this is the rrt tree data structure, as a shared_ptr

  rrt_iss->start_state_ = start;
  rrt_iss->goal_state_ = goal;
  rrt_iss->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions, obstacles_);

  return std::make_tuple(rrt_iss, state_space_ptr_6d);
}


/* ************************************************************************** */
// std::shared_ptr<rrt::Tree<rrt::Vec13>> RRTNodelet::setup_13d(double start_pos[7], double goal_pos[7]) {
//     /*
//     * 6 DoF, RBD
//     */

//     // RBD params (for Astrobee)
//     double mass = 9.58;  // kg
//     double ixx = 0.153;  // kg-m2
//     double iyy = 0.143;  // kg-m2
//     double izz = 0.162;  // kg-m2

//     // Statespace params
//     rrt::Vec9 min_bounds;
//     double min_x = -1.0;
//     double min_y = -1.0;
//     double min_z = -1.0;
//     double min_xd = -.1;
//     double min_yd = -.1;
//     double min_zd = -.1;
//     double min_phid = -1.0;
//     double min_thetad = -1.0;
//     double min_psid = -1.0;
//     min_bounds << min_x, min_y, min_z, min_xd, min_yd, min_zd, min_phid, min_thetad, min_psid;

//     rrt::Vec9 max_bounds;
//     double max_x = 1.0;
//     double max_y = 1.0;
//     double max_z = 1.0;
//     double max_xd = .1;
//     double max_yd = .1;
//     double max_zd = .1;
//     double max_phid = 1.0;
//     double max_thetad = 1.0;
//     double max_psid = 1.0;
//     max_bounds << max_x, max_y, max_z, max_xd, max_yd, max_zd, max_phid, max_thetad, max_psid;

//     // start state, goal state
//     rrt::Vec13 start, goal;
//     start << 0.0, 0.0, 0.0,         // x, y, z
//                 0.0, 0.0, 0.0,         // xd, yd, zd
//                 1.0, 0.0, 0.0, 0.0,    // qx, qy, qz, qth
//                 0.0, 0.0, 0.0;         // wx, wy, wz
//     goal << -0.5, -0.7, -0.7,
//             0.0, 0.0, 0.0,
//             // 1.0, 0.0, 0.0, 0.0,
//             // 0.577, 0.577, 0.577, 0.0,
//             0.0, 1.0, 0.0, 0.0,
//             0.0, 0.0, 0.0;

//     // params
//     int max_iterations = 5000;
//     bool is_ASC_enabled = false;
//     bool remember_actions = true;
//     double timestep = 2.0;
//     double max_timestep = 2.0;
//     double goal_bias = 0.05;
//     double goal_max_dist = 10.0;

//     // create the state space
//     shared_ptr<rrt::StateSpace13D> state_space_ptr =
//         std::make_shared<rrt::StateSpace13D> (min_bounds, max_bounds, mass, ixx, iyy, izz);

//     shared_ptr<rrt::Tree<rrt::Vec13>> rrt =
//         std::make_shared<rrt::Tree<rrt::Vec13>>(state_space_ptr);  // this is the rrt tree data structure, as a shared_ptr

//     rrt->start_state_  = start;
//     rrt->goal_state_ = goal;
//     rrt->set_params(max_iterations, is_ASC_enabled, timestep, max_timestep, goal_bias, goal_max_dist, remember_actions, obstacles_);

//     return rrt;
// }



/* ************************************************************************** */
void RRTNodelet::update_parameters_callback(const param_est::Params::ConstPtr& msg) {
  /* The `mob/inertia_est` subscriber callback.
  Keep updating the mass as long as it is feasible.
  */
  if (use_params_) {
    params_est_.mass = msg->inertia.m;
    if (params_est_.mass > mass_lb_ ) {
    params_model_.mass = params_est_.mass;
    }
  }
}


/* ************************************************************************** */
void RRTNodelet::rattle_instruct_callback(const reswarm_msgs::RattleTestInstruct::ConstPtr& msg)  {
    /* Configuration options for RATTLE.
    @msg USE_PARAMS: use paramter updates {0, 1}
    @msg INITIAL_MODEL_MODE: mode to use for calc, see enum
    */
    use_params_ = msg->USE_PARAMS;
    initial_model_mode_ = msg->INITIAL_MODEL_MODE;

    params_model_ = initial_mode_vec[initial_model_mode_];
}


/* ************************************************************************** */
void RRTNodelet::rrt_params_callback(const rattle_rrt::RRTParams::ConstPtr param_msg) {
  /*
  The `rrt_params` subscriber callback.

  Generates an RRT using the param_msg parameters.
  */
  NODELET_INFO_STREAM("RRT update requested");
  double start_pos[7];
  double goal_pos[7];

  ground_ = param_msg->ground;

  start_pos[0] = param_msg->start.position.x;
  start_pos[1] = param_msg->start.position.y;
  start_pos[2] = param_msg->start.position.z;
  start_pos[3] = param_msg->start.orientation.x;
  start_pos[4] = param_msg->start.orientation.y;
  start_pos[5] = param_msg->start.orientation.z;
  start_pos[6] = param_msg->start.orientation.w;

  goal_pos[0] = param_msg->goal.position.x;
  goal_pos[1] = param_msg->goal.position.y;
  goal_pos[2] = param_msg->goal.position.z;
  goal_pos[3] = param_msg->goal.orientation.x;
  goal_pos[4] = param_msg->goal.orientation.y;
  goal_pos[5] = param_msg->goal.orientation.z;
  goal_pos[6] = param_msg->goal.orientation.w;

  ellipsoids_ = param_msg->ellipsoids;

  // Load in obstacles---make sure start position is valid!
  obstacles_.clear();  // reset existing stored obstacles
  for(auto e : ellipsoids_) {
    // std::cout << "obz " << e << std::endl;
    obstacles_.push_back( Obstacle(e.x_semi, e.y_semi, e.z_semi, e.xyz.x, e.xyz.y, e.xyz.z));
  }

  generate_rrt(start_pos, goal_pos, params_model_);
}


/* ************************************************************************** */
// void RRTNodelet::rrt_obs_callback(const rattle_rrt::ellipsoidArray::ConstPtr ellipsoids_msg) {
//   /*
//   The `ellipsoids_msg` subscriber callback. For online updates of obstacles.
//   */
//   NODELET_INFO_STREAM("RRT obstacles received");

//   ellipsoids_ = ellipsoids_msg->ellipsoids;

//   // Load in obstacles---make sure start position is valid!
//   obstacles_.clear();  // reset existing stored obstacles
//   for(auto e : ellipsoids_) {
//     obstacles_.push_back( Obstacle(e.xyz.x, e.xyz.y, e.xyz.z, e.x_semi, e.y_semi, e.z_semi));
//   }
// }


/* ************************************************************************** */
template <typename T>
void RRTNodelet::rrt_path_publish(const T &path, double start_pos[7]) {
  /*
  Publishes the rrt path.

  @params: path of vector<rrt::Node<T>> RRT waypoints 
  @results: posearray and twistarray pubs on /rattle/rrt/path/...

  */
  geometry_msgs::PoseArray posearray;
  std::vector<geometry_msgs::TwistStamped> twistarray;

  int len_rrt = path.size();

  // build vector of poses and twists
  for (int i=0; i < len_rrt; i++) {
    geometry_msgs::Pose pose;
    geometry_msgs::TwistStamped twist_stmp;

    // path is std::vector of [x, y, z, vx, vy, vz]
    if (ground_ == false){
      pose.position.x = path_[i].state_[0];
      pose.position.y = path_[i].state_[1];
      pose.position.z = path_[i].state_[2];
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = -0.7071068;
      pose.orientation.w = 0.7071068;

      twist_stmp.header.seq = i;
      twist_stmp.twist.linear.x = path_[i].state_[3];
      twist_stmp.twist.linear.y = path_[i].state_[4];
      twist_stmp.twist.linear.z = path_[i].state_[5];
      twist_stmp.twist.angular.x = 0.0;
      twist_stmp.twist.angular.y = 0.0;
      twist_stmp.twist.angular.z = 0.0;
    }
    else if (ground_ == true){  // path is std::vector of [x y vx vy]
      pose.position.x = path_ground_[i].state_[0];
      pose.position.y = path_ground_[i].state_[1];
      pose.position.z = start_pos[2];  // get z
      pose.orientation.x = 0.0;
      pose.orientation.y = 0.0;
      pose.orientation.z = -0.7071068;
      pose.orientation.w = 0.7071068;

      twist_stmp.header.seq = i;
      twist_stmp.twist.linear.x = path_ground_[i].state_[2];
      twist_stmp.twist.linear.y = path_ground_[i].state_[3];
      twist_stmp.twist.linear.z = 0.0;
      twist_stmp.twist.angular.x = 0.0;
      twist_stmp.twist.angular.y = 0.0;
      twist_stmp.twist.angular.z = 0.0;
    }

    posearray.poses.push_back(pose);
    posearray.header.frame_id = "world";  // needed for rviz to display, might also be "map"
    twistarray.push_back(twist_stmp);
  }

  pub_rrt_path_posearray_.publish(posearray);
  pub_rrt_path_twistarray_.publish(twistarray);
}


/* ************************************************************************** */
template <typename T>
// typedef rrt::Vec6 T;
void RRTNodelet::pub_rrt_rviz_markers(std::vector<std::tuple<T, T>> rrt_edges) {
  /* Publish RViz markers for the RRT tree to visuaize in 3D.

  @params:
  rrt_edges: a std::vector of rrt edges([T, T])
  */
  visualization_msgs::Marker marker_array;  // a list of many lines
  marker_array.header.frame_id = "world";
  marker_array.header.stamp = ros::Time();
  marker_array.ns = "my_namespace";
  marker_array.id = 1;
  marker_array.type = visualization_msgs::Marker::LINE_LIST;
  marker_array.action = visualization_msgs::Marker::ADD;
  marker_array.pose.position.x = 0.0;
  marker_array.pose.position.y = 0.0;
  marker_array.pose.position.z = 0.0;
  marker_array.pose.orientation.x = 0.0;
  marker_array.pose.orientation.y = 0.0;
  marker_array.pose.orientation.z = 0.0;
  marker_array.pose.orientation.w = 1.0;
  marker_array.scale.x = 0.03;
  marker_array.color.a = 1.0; // Don't forget to set the alpha!
  marker_array.color.r = 1.0;
  marker_array.color.g = 1.0;
  marker_array.color.b = 0.0;

  for (size_t i = 0; i < rrt_edges.size(); i++) {
    std::tuple<T, T> edge = rrt_edges[i];
    T from = std::get<0>(edge);
    T to = std::get<1>(edge);

    // line in between
    geometry_msgs::Point p;
    p.x = from[0];
    p.y = from[1];
    if (std::is_same<T, rrt::Vec4>::value) { p.z = 0.0;}
    else { p.z = from[2]; }
    marker_array.points.push_back(p);

    p.x = to[0];
    p.y = to[1];
    if (std::is_same<T, rrt::Vec4>::value) { p.z = 0.0;}
    else { p.z = to[2]; }
    marker_array.points.push_back(p);

    // // marker1
    // visualization_msgs::Marker marker;
    // marker.header.frame_id = "world";
    // marker.header.stamp = ros::Time();
    // marker.ns = "my_namespace";
    // marker.id = i;
    // marker.type = visualization_msgs::Marker::SPHERE;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = from[0];
    // marker.pose.position.y = from[1];
    // if (std::is_same<T, rrt::Vec6>::value) {
    //   marker.pose.position.z = 0.0;
    // }
    // else {
    //   marker.pose.position.z = from[2];
    // }
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 0.05;
    // marker.scale.y = 0.05;
    // marker.scale.z = 0.05;
    // marker.color.a = 1.0; // Don't forget to set the alpha!
    // marker.color.r = 1.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
    // pub_rrt_marker_.publish(marker);

    // // marker2
    // marker.header.frame_id = "world";
    // marker.header.stamp = ros::Time();
    // marker.ns = "my_namespace";
    // marker.id = i + rrt_edges.size();
    // marker.type = visualization_msgs::Marker::SPHERE;
    // marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.position.x = to[0];
    // marker.pose.position.y = to[1];
    // if (std::is_same<T, rrt::Vec6>::value) {
    //   marker.pose.position.z = 0.0;
    // }
    // else {
    //   marker.pose.position.z = from[2];
    // }
    // marker.pose.orientation.x = 0.0;
    // marker.pose.orientation.y = 0.0;
    // marker.pose.orientation.z = 0.0;
    // marker.pose.orientation.w = 1.0;
    // marker.scale.x = 0.05;
    // marker.scale.y = 0.05;
    // marker.scale.z = 0.05;
    // marker.color.a = 1.0; // Don't forget to set the alpha!
    // marker.color.r = 1.0;
    // marker.color.g = 1.0;
    // marker.color.b = 0.0;
    // pub_rrt_marker_.publish(marker);
  }
  pub_rrt_marker_.publish(marker_array);
}


/* ************************************************************************** */
void RRTNodelet::rrt_path_request() {
  /*
  Requests an RRT path using params
  */
  rattle_rrt::RRTParams params_msg;

  params_msg.start.position.x = 0.0;
  params_msg.start.position.y = 0.0;
  params_msg.start.position.z = 0.0;
  params_msg.start.orientation.x = 1.0;
  params_msg.start.orientation.y = 0.0;
  params_msg.start.orientation.z = 0.0;
  params_msg.start.orientation.w = 0.0;

  params_msg.goal.position.x = -0.5;
  params_msg.goal.position.y = -0.5;
  params_msg.goal.position.z = -0.5;
  params_msg.goal.orientation.x = 1.0;
  params_msg.goal.orientation.y = 0.0;
  params_msg.goal.orientation.z = 0.0;
  params_msg.goal.orientation.w = 0.0;

  pub_rrt_params_.publish(params_msg);
}

}  // end namespace rrt

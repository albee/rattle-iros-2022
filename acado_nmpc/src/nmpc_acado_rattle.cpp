/*
A ROS node for calling the ACADO trajectory optimization software.
A LSQ cost function NMPC for the 6 DOF free-flying dynamics.

Copyright 2021 Monica Ekal, Keenan Albee

ACADO integration based on code by ALG Prasad and Sambaran Ghoshal
*/

#include <ros/ros.h>

// Action
#include <ff_msgs/DockAction.h>

// FSW includes
#include <ff_util/ff_names.h>
#include <ff_util/ff_flight.h>

// Messages
#include <ff_msgs/EkfState.h>
#include <geometry_msgs/InertiaStamped.h>
#include <inertia_estimator/params.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/FlightMode.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// ACADO
#include <acado_common.h>
#include <acado_auxiliary_functions.h>

// C++ STL inclues
#include <sstream>
#include <string>
#include <memory>
#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

// Get variables from Acado:
#define NX ACADO_NX
#define NOD ACADO_NOD  // number of online data variables
#define NU ACADO_NU  // number of inputs per timestep
#define NY ACADO_NY
#define NYN ACADO_NYN
#define N ACADO_N  // horizon length for controller
#define VERBOSE 1
#define GRANITE 0  // change to receive this parameter from the config file
/*
OnlineData format, [N x 29] (accels and vels in world frame):
  [mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3) u_des(6)]
u format [6 x 1] (body frame):
  [F_x F_y F_z T_x T_y T_z]
x format [13 x 1] (body frame):
  [r(3) v(3) q(4) w(3)]
 */

// Global vars
double online_data_vector[NOD];
double t1; double t;
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


double mass{};
double ixx{};
double iyy{};
double izz{};
double ixy{};
double iyz{};
double ixz{};
double cx{};
double cy{};
double cz{};

// The `mob/inertia` subscriber callback
void parameters_callback(const geometry_msgs::InertiaStamped::ConstPtr& msg) {
ROS_INFO("Mass parameters updated...");
mass = msg->inertia.m;
ixx = msg->inertia.ixx;
iyy = msg->inertia.iyy;
izz = msg->inertia.izz;
ixy = msg->inertia.ixy;
ixz = msg->inertia.ixz;
iyz = msg->inertia.iyz;
cx = msg->inertia.com.x;
cy = msg->inertia.com.y;
cz = msg->inertia.com.z;
}

/*#########################################
Node
#########################################*/
class NMPCAcado {
 public:
  NMPCAcado(ros::NodeHandle nh) { Initialize(nh); }
  ~NMPCAcado() {}

 private:
  ros::NodeHandle nh;

  ros::Subscriber sub_current_pose_;
  ros::Subscriber sub_current_inertial_params_;
  ros::Subscriber sub_path_ctl_twist_;
  ros::Subscriber sub_path_ctl_pose_;
  ros::Subscriber sub_path_ctl_wrench_;
  ros::Subscriber sub_flight_mode_;

  ros::Publisher pub_latest_cmd_;
  ros::Publisher pub_flight_mode_;

  ff_msgs::FlightMode flight_mode_;

  std::string CTL_ENABLED;
  double t_prev, time_elapsed;
  acado_timer t;

  // timing variables
  double DT_L_ = 1/10.0;  // local plan dt
  int N_L_ = 0;  // number of timesteps contained in the current local plan

  double DT_C_ = 1/20.0*10;  // controller dt
  int N_C_ = 0;  // number of timesteps, controller

  int HAS_LOCAL_PLAN_ = 0;
  int CONTROLLER_STATUS_ = 0;
  int USE_PARAM_UPDATES_ = 1;
  int iter_ = 0;

  // plan information
  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Wrench wrench_;

  /*
  Start up the info gain NMPC
  */

    void Initialize(ros::NodeHandle nh_main) {
    nh = nh_main;
    ROS_INFO_STREAM("nmpc_acado is running...");
    // Subscribers for state and parameter info
    sub_current_pose_ = nh.subscribe("gnc/ekf", 5, &NMPCAcado::ekf_state_callback, this);
    // get inertial property information from /mob/inertia
    sub_current_inertial_params_ = nh.subscribe("mob/inertia", 1, parameters_callback);
    sub_flight_mode_ = nh.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 5, &NMPCAcado::sub_flight_mode_callback, this);

    // Subscribers for local plan
    sub_path_ctl_pose_ = nh.subscribe("info_rich_traj/pose", 5, &NMPCAcado::path_ctl_pose_callback,
                                      this);  // desired trajectory!
    sub_path_ctl_twist_ = nh.subscribe("info_rich_traj/twist", 5, &NMPCAcado::path_ctl_twist_callback,
                                       this);  // desired trajectory!
    sub_path_ctl_wrench_ = nh.subscribe("info_rich_traj/wrench", 5, &NMPCAcado::path_ctl_wrench_callback,
                                        this);  // desired trajectory     !

    // Publisher to send out commands
    pub_latest_cmd_ = nh.advertise<ff_msgs::FamCommand>("gnc/ctl/command", 1, true);  // sent off to FAM

    pub_flight_mode_ = nh.advertise<ff_msgs::FlightMode>(TOPIC_MOBILITY_FLIGHT_MODE, 1,
                                                         true);  // note: `mob/flight_mode` is the topic of interest

    // Create nominal flight mode!
    std::string flight_mode_name_ = "difficult";  // FlightMode to enter
    ff_util::FlightUtil::GetFlightMode(flight_mode_, flight_mode_name_);
     ROS_INFO_STREAM("Still running");
    // Wait a bit, then update params. Not ready to run yet.
    ros::Duration(1.0).sleep();  // need to wait for parameter to be sent
    ROS_INFO_STREAM("Got param inits");
    // ROS_ASSERT(PARAM_INITS_.getType() == XmlRpc::XmlRpcValue::TypeArray);

    //    for (int32_t i = 0; i < PARAM_INITS_.size(); ++i) {
    //        ROS_ASSERT(PARAM_INITS[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    //        param_inits[i] = static_cast<double>(PARAM_INITS_[i]);
    //    }
    //    my_params.initialize_inertial_params(param_inits);
    //    ROS_INFO_STREAM(my_params.mass << " " << my_params.cx << " " << my_params.cy << " " << my_params.izz);
    acado_startup();  // set initial values for solver
    acado_tic(&t);

    ROS_INFO_STREAM("nmpc_acado is running...");
    print_state();

    // Publish the ctl DT_C_. Also spinOnce to update the local plan.
    // This should really be at 62.5 Hz TODO
    // Timer for incrementing local plan: make sure it does not start until local plan loaded. Uses period, NOT rate
    // should really do multithreading to get timers running independently
    ros::Rate ctl_pub_rate(1.0/DT_C_);
    ros::Timer local_plan_timer = nh.createTimer(ros::Duration(DT_L_), &NMPCAcado::local_plan_timer_callback, this);
    while (ros::ok()) {  // Publish plans at DT_C_, but update waypoints at rate DT_L_
      ros::spinOnce();

      nh.getParam("/rattle/CONTROLLER_STATUS", CONTROLLER_STATUS_);

      if (CONTROLLER_STATUS_ == 2) {  // only start FAM
          std::string flight_mode_name = "difficult";  // FlightMode to enter
          ff_util::FlightUtil::GetFlightMode(flight_mode_, flight_mode_name);
          pub_flight_mode_.publish(flight_mode_);  // Publish default flight mode
      } else if (CONTROLLER_STATUS_ == 1 && (iter_ > 0)) {  // want to make sure that the ref traj has been set once
        std::string flight_mode_name = "difficult";  // FlightMode to enter
        ff_util::FlightUtil::GetFlightMode(flight_mode_, flight_mode_name);
        pub_flight_mode_.publish(flight_mode_);  // Publish default flight mode
        nh.getParam("/rattle/USE_PARAM_UPDATES", USE_PARAM_UPDATES_);
        compute_and_send_F_T();
      } else if (CONTROLLER_STATUS_ == 0) {
        std::string flight_mode_name = "stopped";  // FlightMode to enter
        ff_util::FlightUtil::GetFlightMode(flight_mode_, flight_mode_name);
        pub_flight_mode_.publish(flight_mode_);  // Publish stopped flight mode
      }

      ctl_pub_rate.sleep();
    }
  }

  void debug() {
    /* Debug expected values
    */
    ROS_INFO_STREAM("Debug...");
    ROS_INFO_STREAM("Local plan length: " << N_L_ << " Controller horizon length: " << N);
    ROS_INFO_STREAM("Iteration: " << iter_);

    t1 = ros::Time::now().toSec();

    /*
    ACADO-dependent calls
    */

    time_elapsed = (double)acado_toc(&t);  // note down time elapsed to use in the exponential decay

    // Compute the NMPC feedback to reach the goal
    acado_preparationStep();
    acado_feedbackStep();  // cannot be too different from the first call!

    // Get the FIRST of the FAM forces and torques
    int i = 0;
    ff_msgs::FamCommand famcmd;
    famcmd.wrench.force.x = acadoVariables.u[0 + i*NU];
    famcmd.wrench.force.y = acadoVariables.u[1 + i*NU];
    famcmd.wrench.torque.z = acadoVariables.u[5 + i*NU];

    if (GRANITE == 1) {
      famcmd.wrench.force.z = 0;   // acadoVariables.u[2];
      famcmd.wrench.torque.x = 0;  // acadoVariables.u[3];
      famcmd.wrench.torque.y = 0;  // acadoVariables.u[4];
    } else {
      famcmd.wrench.force.z = acadoVariables.u[2 + i*NU];
      famcmd.wrench.torque.x= acadoVariables.u[3 + i*NU];
      famcmd.wrench.torque.y = acadoVariables.u[4 + i*NU];
    }

    // magic numbers---which is highest speed?
    famcmd.status = 3;
    famcmd.control_mode = 2;

    // Send out the most recent plan
    pub_latest_cmd_.publish(famcmd);

    /* shift the initialization---why is this needed?
    see here: http://acado.sourceforge.net/doc/html/db/daf/cgt_getting_started.html
    */
    // acado_shiftStates(2, 0, 0);
    // acado_shiftControls(0);

    /*
    ACADO-dependent calls
    */
    print_state();
    HAS_LOCAL_PLAN_ = 0;
    nh.setParam("/rattle/CONTROLLER_STATUS", 0);
    t_prev = t1;
  }

  void local_plan_timer_callback(const ros::TimerEvent& event) {
    /* Update the local plan iter_ indicator. Also update online_data
    At each step, Acado's .od and variables must be updated to reflect the latest reference.
    */
    if (HAS_LOCAL_PLAN_) {
      ROS_INFO_STREAM("Updating local plan setpoint...");
      update_ref_traj();  // update at DT_L_ rate!
      // print_state();
      iter_++;
      ROS_INFO_STREAM(iter_);
    }
  }

  void compute_and_send_F_T() {
    /* Compute control outputs
    */
    ROS_INFO_STREAM("Computing force and torque...");
    // ROS_INFO_STREAM("Local plan length: " << N_L_ << " Controller horizon length: " << N);
    ROS_INFO_STREAM("Iteration: " << iter_);

    acado_preparationStep();

    // Check if we overran the ref traj---will go to regulation
    if (iter_ >= N_L_ - 1) {
      ROS_ERROR_STREAM("Overrunning local plan...");
    }

    t1 = ros::Time::now().toSec();
    time_elapsed = (double)acado_toc(&t);  // note down time elapsed to use in the exponential decay

    // Compute the NMPC feedback to reach the goal
    acado_feedbackStep();

    /*Get the FIRST of the FAM forces and torques
    Forces and torques must be in the BODY frame!
    */
    int i = 0;
    ff_msgs::FamCommand famcmd;
    famcmd.header.stamp = ros::Time::now();
    famcmd.wrench.force.x = acadoVariables.u[0 + i*NU];
    famcmd.wrench.force.y = acadoVariables.u[1 + i*NU];
    famcmd.wrench.torque.z = acadoVariables.u[5 + i*NU];

    if (GRANITE == 1) {
      famcmd.wrench.force.z = 0;   // acadoVariables.u[2];
      famcmd.wrench.torque.x = 0;  // acadoVariables.u[3];
      famcmd.wrench.torque.y = 0;  // acadoVariables.u[4];
    } else {
      famcmd.wrench.force.z = acadoVariables.u[2 + i*NU];
      famcmd.wrench.torque.x= acadoVariables.u[3 + i*NU];
      famcmd.wrench.torque.y = acadoVariables.u[4 + i*NU];
    }

    // magic numbers---which is highest speed?
    famcmd.status = 3;
    famcmd.control_mode = 2;

    // Send out the most recent plan
    pub_latest_cmd_.publish(famcmd);

    if (VERBOSE) {
      print_state();
      // std::cout << "Time betwen cycles (secs, expected 0.016): " << t1-t_prev << std::endl;
      // printf("Real-Time Iteration %d:  KKT Tolerance = %.3e Objective = %.3e\n", iter_, acado_getKKT(),
      // acado_getObjective() ); printf("Params: %f %f %f %f\n\n",my_params.mass, my_params.ixx, my_params.iyy,
      // my_params.izz);
      printf("Params: %f %f %f %f\n\n", mass, cx, cy, izz);
    }

    acado_shiftStates(2, 0, 0);
    acado_shiftControls(0);

    t_prev = t1;
  }

/*#########################################
Pubs and subs
#########################################*/


  void path_ctl_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    /* Get the desired positions for ACADO to track.
    */
    ROS_INFO(" [nmpc_acado] Latest goal and status info updated...");
    pose_ = msg->pose;

    ROS_INFO_STREAM("Got latest local plan...");
    iter_ = 0;
    HAS_LOCAL_PLAN_ = 1;
//    N_L_ = pose_.size();
    ROS_INFO_STREAM("Plan is length " << N_L_);
  }

  void path_ctl_twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    /* Get the desired twists for ACADO to track.
    ALWAYS received together with pose_callback (or there will be problems)
    */
    twist_ = msg->twist;
  }

  void path_ctl_wrench_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    /* Get the desired wrenches for ACADO to track.
    ALWAYS received together with pose_callback (or there will be problems)
    */
    wrench_ = msg->wrench;
  }

  void ekf_state_callback(const ff_msgs::EkfState::ConstPtr& msg) {
    /* The `gnc/ekf` subscriber callback
    Update ACADO with latest pose information
    */
    // ROS_INFO("EKF estimate updated...");
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
  }

  void sub_flight_mode_callback(const ff_msgs::FlightMode::ConstPtr& msg) {
      ROS_INFO("[nmpc_acado] flight mode...");
    CTL_ENABLED = msg->control_enabled;
  }

/*#########################################
ACADO
#########################################*/
  void acado_startup() {
    /* Begin acado
    */
    acado_initializeSolver();  // Initialize the solver
    initialize_mpc();          // Initialize MPC-related ACADO variables
    acado_preparationStep();   // Prepare for solving
    getcostmatrix();           // Initialize cost function weights
    if ( VERBOSE ) {
      // acado_printHeader();
    }
  }

  void getcostmatrix() {
    /* ACADO updating and initialization
    */
    int i, j;
    for (i = 0; i < NY; i++) {
      for (j = 0; j < NY; j++) {
        if (i == j) {
          switch (j) {
            case 0:
              acadoVariables.W[i * NY + j] = 10;  // weight for error_pos x
              break;
            case 1:
              acadoVariables.W[i * NY + j] = 10;  // weight for error_pos y
              break;
            case 2:
              acadoVariables.W[i * NY + j] = 10;  // weight for error_pos z
              break;
            case 3:
              acadoVariables.W[i * NY + j] = 8.0;  // weight for error_vel x
              break;
            case 4:
              acadoVariables.W[i * NY + j] = 8.0;  // weight for error_vel y
              break;
            case 5:
              acadoVariables.W[i * NY + j] = 8.0;  // weight for error vel z
              break;
            case 6:
              acadoVariables.W[i * NY + j] = 0.05;  // weight for orientation error x
              break;
            case 7:
              acadoVariables.W[i * NY + j] = 0.05;  // weight for orientation error y
              break;
            case 8:
              acadoVariables.W[i * NY + j] = 0.05;  // weight for orientation error z
              break;
            default:
              acadoVariables.W[i * NY + j] = 0.2;
              break;
          }
        } else {
          acadoVariables.W[i * NY + j] = 0.0;
        }
      }
    }

    for (i = 0; i < NYN; i++) {
      for (j = 0; j < NYN; j++) {
        if (i == j) {
          acadoVariables.WN[i*NYN+j] = 25;
        } else {
          acadoVariables.WN[i * NYN + j] = 0;
        }
      }
    }
  }

  void update_online_data() {
    /* Update the parameter values
    */
    int i, j;

    // Online data vector: [mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3) u_des(6)]
    online_data_vector[0] = mass;
    online_data_vector[1] = ixx;
    online_data_vector[2] = iyy;
    online_data_vector[3] = izz;
    online_data_vector[4] = ixy;
    online_data_vector[5] = iyz;
    online_data_vector[6] = ixz;
    online_data_vector[7] = cx;
    online_data_vector[8] = cy;
    online_data_vector[9] = cz;

    // assign the repeating vector to acado variables od (online data)
    for (i = 0; i < N + 1; i++) {  // horizon
      for (j = 0; j <= 9; j++) {   // number of online data variables
        acadoVariables.od[i*NOD + j] = online_data_vector[j];
      }
    }
  }

  void update_ref_traj() {
    /*
    Update the trajectory reference to the next timestep.
    This is currently performing regulation!
    */
    int i, j;

    if (iter_ >= N_L_) {  // repeat the final point for regulation
      iter_ = N_L_ - 1;
    }

    // od format:
    // [mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3) u_des(6)]
    for (i = 0; i < N + 1; i++) {    // horizon
      for (j = 10; j <= NOD; j++) {  // number of online data variables
        if (j == 10) {
          acadoVariables.od[i*NOD + j] = pose_.position.x;
        } else if (j == 11) {
          acadoVariables.od[i*NOD + j] = pose_.position.y;
        } else if (j == 12) {
          acadoVariables.od[i*NOD + j] = pose_.position.z;
        } else if (j == 13) {
          acadoVariables.od[i*NOD + j] = twist_.linear.x;
        } else if (j == 14) {
          acadoVariables.od[i*NOD + j] = twist_.linear.y;
        } else if (j == 15) {
          acadoVariables.od[i*NOD + j] = twist_.linear.z;
        } else if (j == 16) {
          acadoVariables.od[i*NOD + j] = pose_.orientation.x;
        } else if (j == 17) {
          acadoVariables.od[i*NOD + j] = pose_.orientation.y;
        } else if (j == 18) {
          acadoVariables.od[i*NOD + j] = pose_.orientation.z;
        } else if (j == 19) {
          acadoVariables.od[i*NOD + j] = pose_.orientation.w;
        } else if (j == 20) {
          acadoVariables.od[i*NOD + j] = twist_.angular.x;
        } else if (j == 21) {
          acadoVariables.od[i*NOD + j] = twist_.angular.y;
        } else if (j == 22) {
          acadoVariables.od[i*NOD + j] = twist_.angular.z;
        }

        if (i != N) {  // u_des takes one less value
          if (j == 23) {
            acadoVariables.od[i*NOD + j] = wrench_.force.x;
          } else if (j == 24) {
            acadoVariables.od[i*NOD + j] = wrench_.force.y;
          } else if (j == 25) {
            acadoVariables.od[i*NOD + j] = wrench_.force.z;
          } else if (j == 26) {
            acadoVariables.od[i*NOD + j] = wrench_.torque.x;
          } else if (j == 27) {
            acadoVariables.od[i*NOD + j] = wrench_.torque.y;
          } else if (j == 28) {
            acadoVariables.od[i*NOD + j] = wrench_.torque.z;
          }
        }
      }
    }
  }

  void print_state() {
    /* Function to print the important values whenever needed.
    */
    ROS_INFO_STREAM("Current State: " << acadoVariables.x0[0] << " " << acadoVariables.x0[1] << " "
      << acadoVariables.x0[2] << " " << acadoVariables.x0[3] << " " << acadoVariables.x0[4] << " "
      << acadoVariables.x0[5] << " " << acadoVariables.x0[6] << " " << acadoVariables.x0[7] << " "
      << acadoVariables.x0[8] << " " << acadoVariables.x0[9] << " " << acadoVariables.x0[10] << " "
      << acadoVariables.x0[11] << " " << acadoVariables.x0[12]);

    ROS_INFO_STREAM("Desired State: " << acadoVariables.od[10] << " " << acadoVariables.od[11] << " "
          << acadoVariables.od[12] << " " << acadoVariables.od[13] << " " << acadoVariables.od[14] << " "
          << acadoVariables.od[15] << " " << acadoVariables.od[16] << " " << acadoVariables.od[17] << " "
          << acadoVariables.od[18] << " " << acadoVariables.od[19] << " " << acadoVariables.od[20] << " "
          << acadoVariables.od[21] << " " << acadoVariables.od[22]);

    ROS_INFO_STREAM("Current Input: " << acadoVariables.u[0] << " " << acadoVariables.u[1] << " "
                                      << acadoVariables.u[5]);
  }

  void initialize_mpc() {
    int i;
    /* Initialize the states and controls. */
    for (i = 0; i <  (N + 1)*NX; ++i)  acadoVariables.x[i] = 0.0;
    for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

    /* Initialize the measurements/reference. */
    for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0;
    for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = 0.0;

    /* Initialize the current state. */
    for (i = 0; i < NX; ++i) acadoVariables.x0[i] = 0.0;
    acadoVariables.x0[9] = 1.0;  // quaternion

    /* Initialize the online data values */
    for (i = 0; i < NOD * (N+1); ++i) acadoVariables.od[i] = 0.0;

    /* Initialize inertial params thought to be the correct values */
    for (i = 0; i < NOD * (N+1); i+=NOD) {
//      acadoVariables.od[i] = param_inits[0];
//      acadoVariables.od[i+1] = param_inits[1];
//      acadoVariables.od[i+2] = param_inits[2];
//      acadoVariables.od[i+3] = param_inits[3];
      acadoVariables.od[i + 19] = 1.0;  // quaternion
    }
  }
};


/*#########################################
Node
#########################################*/
/*
Make a repeated call to ACADO's traj opt
*/
int main(int argc, char** argv) {
  ros::init(argc, argv, "nmpc_acado_rattle");
  ROS_INFO_STREAM("nmpc_acado_rattle is running");
  if (VERBOSE) {
    acado_printDifferentialVariables();
    acado_printControlVariables();
  } else if (!VERBOSE) {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle nh("");
  NMPCAcado planner(nh);
  return 0;
}

'''
_rattle_coordinator_ros.py

ROS callbacks and publishers of rattle_coordinator.

Keenan Albee and Monica Ekal, 2021
MIT Space Systems Lab
'''
import rospy

# msgs
from ff_msgs.msg import EkfState
from ff_msgs.msg import FamCommand
from rattle_msgs.msg import RattleTestInstruct
from rattle_msgs.msg import RattleInfoPlanInstruct  # formerly NMPCInstruct
from rattle_rrt.msg import ellipsoid, ellipsoidArray, RRTParams, TwistArray, WrenchArray
from rattle_msgs.msg import RattleStatus

# std msgs
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseArray, Quaternion, Twist, Vector3, Pose, TwistStamped, Wrench
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np


class Mixin:
    #
    # Subs
    #
    def test_instruct_callback(self, msg):
        """ Execute a test with some configuration options.
        """
        # unpack RattleTestInstruct message
        test_num = msg.test_num
        self.USE_EKF_POSE = msg.USE_EKF_POSE
        self.USE_PARAMS = msg.USE_PARAMS
        self.WEIGHT_MODE = msg.WEIGHT_MODE
        self.INITIAL_MODEL_MODE = msg.INITIAL_MODEL_MODE
        self.ground_ = msg.ground
        self.OOA_TEST = msg.OOA_TEST
        
        self.USE_CSV = msg.USE_CSV
        self.obs_config_ = msg.OBS_CONFIG

        # load in desired obstacle set
        if self.ground_ is True:
            self.obstacles_ = self.obstacles_opts["ground"][self.obs_config_]
        else:
            self.obstacles_ = self.obstacles_opts["iss"][self.obs_config_]

        self.run_test(test_num)

    def ctls_callback(self, data):
        """ TODO
        """
        return 0
        # tmp_list = [ data.wrench.force.x, data.wrench.force.y, data.wrench.force.z,
        # 						   data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z, data.header.stamp.to_sec()]

    def ekf_state_callback(self, msg):
        """ Update latest estimated robot pose.

        x_est_ = [r(3) v(3) q(4) w(3)].T
        """
        r = np.array([[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]]).T
        v = np.array([[msg.velocity.x, msg.velocity.y, msg.velocity.z]]).T
        q = np.array([[msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]]).T
        w = np.array([[msg.omega.x, msg.omega.y, msg.omega.z]]).T
        self.x_est_ = np.vstack((r, v, q, w))
        self.GOT_EKF = 1
        # rospy.loginfo_throttle(2, "x_est_ updated! %s" %str(x_est_))

    def update_parameters_callback(self, msg):
        """
        The `mob/inertia_est` subscriber callback.
        Keep updating the covariances.

        @param msg.Cov 10-element list with [m, cx, cy, cz, ixx, iyy, izz, ixy, ixz, iyz]
        """
        sigma = np.asarray(msg.Cov)
        self.sigma_ = sigma[[0, 4, 5, 6]]
        # print("rattle cov: ", self.sigma_)

    def rrt_posearray_callback(self, msg):
        self.global_plan_["posearray"] = msg.poses
        self.GLOBAL_STATUS_ += 1

    def rrt_twistarray_callback(self, msg):
        self.global_plan_["twistarray"] = msg.TwistArray
        self.GLOBAL_STATUS_ += 1

    def local_posearray_callback(self, msg):
        self.local_plan_["posearray"] = msg
        self.LOCAL_STATUS_ += 1

    def local_twistarray_callback(self, msg):
        self.local_plan_["twistarray"] = msg
        self.LOCAL_STATUS_ += 1

    def local_wrencharray_callback(self, msg):
        self.local_plan_["wrencharray"] = msg
        self.LOCAL_STATUS_ += 1

    def publish_local_start_state(self, start_state):
        """ Update the start_state used for local planner.
        """
        msg = RattleInfoPlanInstruct()
        msg.x = start_state[0]
        msg.y = start_state[1]
        msg.z = start_state[2]
        msg.vel_x = start_state[3]
        msg.vel_y = start_state[4]
        msg.vel_z = start_state[5]
        msg.quat1 = start_state[6]
        msg.quat2 = start_state[7]
        msg.quat3 = start_state[8]
        msg.quat4 = start_state[9]
        msg.ang_vel_x = start_state[10]
        msg.ang_vel_y = start_state[11]
        msg.ang_vel_z = start_state[12]

        self.local_start_pub_.publish(msg)

    def publish_local_goal_instruct(self, goal, status, weightings):
        """ Send a message with updated desired position and weighting.
        @param goal: [1x13] state vec
        @param status: {0, 1, 2}: TODO! (unused?)
        @param weightings: [m Ixx Iyy Izz I_xy I_yz I_xz cx cy cz]
        """
        msg = RattleInfoPlanInstruct()
        msg.x = goal[0]
        msg.y = goal[1]
        msg.z = goal[2]
        msg.vel_x = goal[3]
        msg.vel_y = goal[4]
        msg.vel_z = goal[5]
        msg.quat1 = goal[6]
        msg.quat2 = goal[7]
        msg.quat3 = goal[8]
        msg.quat4 = goal[9]
        msg.ang_vel_x = goal[10]
        msg.ang_vel_y = goal[11]
        msg.ang_vel_z = goal[12]
        msg.status = status
        msg.info_weight_m = weightings[0]
        msg.info_weight_Ixx = weightings[1]
        msg.info_weight_Iyy = weightings[2]
        msg.info_weight_Izz = weightings[3]
        msg.info_weight_Ixy = weightings[4]
        msg.info_weight_Iyz = weightings[5]
        msg.info_weight_Ixz = weightings[6]
        msg.info_weight_cx = weightings[7]
        msg.info_weight_cy = weightings[8]
        msg.info_weight_cz = weightings[9]
        self.local_goal_pub_.publish(msg)

    def publish_RRTParams_msg(self, start, goal, obstacles, ground):
        """Send a message with start, goal, obstacles info. Basically acting like a service

        Inputs:
        start - [x y th xd yd thd] (3DOF)
        goal - [x y th xd yd thd] (3DOF)
        obstacles - [[a b c x y z]]
        DOF - {3 or 6}

        Outputs:
        rattle/rrt/params
        """
        # start/goal settings
        msg = RRTParams()

        msg.ground = ground

        msg.start.position.x = start[0]
        msg.start.position.y = start[1]
        msg.start.position.z = start[2]
        msg.start.orientation.x = start[3]
        msg.start.orientation.y = start[4]
        msg.start.orientation.z = start[5]
        msg.start.orientation.w = start[6]

        msg.goal.position.x = goal[0]
        msg.goal.position.y = goal[1]
        msg.goal.position.z = goal[2]
        msg.goal.orientation.x = goal[3]
        msg.goal.orientation.y = goal[4]
        msg.goal.orientation.z = goal[5]
        msg.goal.orientation.w = goal[6]

        # obstacles
        for each in obstacles:
            ellipsoid_msg = ellipsoid()
            ellipsoid_msg.x_semi = each[0]
            ellipsoid_msg.y_semi = each[1]
            ellipsoid_msg.z_semi = each[2]
            ellipsoid_msg.xyz.x = each[3]
            ellipsoid_msg.xyz.y = each[4]
            ellipsoid_msg.xyz.z = each[5]
            msg.ellipsoids.append(ellipsoid_msg)

        self.rrt_pub_.publish(msg)

    def publish_rattle_status(self, traj_finished):
        """
        Signal rattle completion.
        """
        msg = RattleStatus()

        msg.traj_finished = traj_finished
        print(msg)
        print('------------------')
        self.rattle_status_pub_.publish(msg)

    def publish_obs_msg(self, ellipsoids):
        """
        Send a message with updated position and weighting.
        """
        msg_array = ellipsoidArray()

        for each in ellipsoids:
            msg = ellipsoid()
            msg.xyz.x = each[0]
            msg.xyz.y = each[1]
            msg.xyz.z = each[2]
            msg.x_semi = each[3]
            msg.y_semi = each[4]
            msg.z_semi = each[5]
            msg_array.ellipsoids.append(msg)
        self.obs_pub_.publish(msg_array)

    def publish_local_plan_msg(self, plan_horizon):
        """
        Send pure sysid trajectory to the controller
        """
        pose_array = PoseArray()
        twist_array = []
        wrench_array = []
        for plan_i in plan_horizon:
            poses_i = Pose()
            poses_i.position.x = plan_i[0]
            poses_i.position.y = plan_i[1]
            poses_i.position.z = 0.0
            poses_i.orientation.x = plan_i[4]
            poses_i.orientation.y = plan_i[5]
            poses_i.orientation.z = plan_i[6]
            poses_i.orientation.w = plan_i[7]
            pose_array.poses.append(poses_i)
            pose_array.header.frame_id = "world"

            twists_i = TwistStamped()
            twists_i.header.stamp = rospy.Time.now()
            twists_i.twist.linear.x = plan_i[2]
            twists_i.twist.linear.y = plan_i[3]
            twists_i.twist.linear.z = 0.0
            twists_i.twist.angular.x = 0.0
            twists_i.twist.angular.y = 0.0
            twists_i.twist.angular.z = plan_i[8]
            twist_array.append(twists_i)

            wrenches_i = Wrench()
            wrenches_i.force.x = plan_i[9]
            wrenches_i.force.y = plan_i[10]
            wrenches_i.force.z = 0.0
            wrenches_i.torque.x = 0.0
            wrenches_i.torque.y = 0.0
            wrenches_i.torque.z = plan_i[11]
            wrench_array.append(wrenches_i)

        self.path_local_pose_pub_.publish(pose_array)
        self.path_local_twist_pub_.publish(twist_array)
        self.path_local_wrench_pub_.publish(wrench_array)

        self.path_ctl_pose_pub_.publish(pose_array)
        self.path_ctl_twist_pub_.publish(twist_array)
        self.path_ctl_wrench_pub_.publish(wrench_array)
        self.CONTROLLER_STATUS_ = 1
        self.EST_STATUS_ = self.CONTROLLER_STATUS_
        rospy.set_param("/rattle/CONTROLLER_STATUS", self.CONTROLLER_STATUS_)
        rospy.set_param("/rattle/EST_STATUS", self.EST_STATUS_)

    #
    # params
    #
    def initialize_fans(self):
        """ Tell acado to start the fans but don't move yet.
        """
        self.CONTROLLER_STATUS_ = 2
        rospy.set_param("/rattle/CONTROLLER_STATUS", self.CONTROLLER_STATUS_)
        rospy.loginfo("Starting FAM...")
        rospy.sleep(3.0)
        rospy.loginfo("...FAM ready.")

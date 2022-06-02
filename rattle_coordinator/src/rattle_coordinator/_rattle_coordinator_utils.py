'''
_rattle_coordinator_utils.py

Utility functions for RATTLE coordinator.

Keenan Albee and Monica Ekal, 2021
MIT Space Systems Lab
'''
import rospy

# std msgs
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point, PoseArray, Quaternion, Twist, Vector3, Pose, TwistStamped, Wrench
from visualization_msgs.msg import Marker, MarkerArray

# math
import numpy as np
from numpy import deg2rad
from tf.transformations import quaternion_from_euler

from functools import partial

import csv


class Mixin:
    def sanity_check_timing(self):
        """ Make sure that the global timing variables actually make sense.
        """
        assert(self.N_L_ >= self.N_C_)  # local plan is "harder" than control
        assert(self.DT_L_*self.N_L_ >= self.DT_C_*self.N_C_)  # local plan time horizon greater than control time horizon
        # rospy.loginfo("rattle_coordinator timing okay...")

    def global_plan_check(self):
        """ Make sure RRT is working as expected
        """
        obstacles = self.obstacles_  # [a b c x y z]
        ground = self.ground_

        if ground is True:  # ground
            start = self.POINT_A_GRANITE
            goal = self.POINT_B_GRANITE
        else:  # ISS
            start = self.POINT_A_ISS
            goal = self.POINT_B_ISS
        print(start)
        print(goal)

        rospy.loginfo("Start pose %s" % str(start))
        self.request_global_plan(start, goal, obstacles, ground)

    def planner_acado_check(self, VERSION):
        """Verify that planner_acado is called correctly.

        VERSION - switches init cond.
        """
        rospy.loginfo('Sending local plan request to acado...')
        if VERSION == 1:
            x = 0.5
            y = 0.5
            z = 0.5
            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0
            q = quaternion_from_euler(0, 0, deg2rad(-90))
            ang_vel_x = 0.0
            ang_vel_y = 0.0
            ang_vel_z = 0.0
        else:
            x = -0.5
            y = -0.5
            z = -0.5
            vel_x = 0.0
            vel_y = 0.0
            vel_z = 0.0
            q = quaternion_from_euler(0, 0, deg2rad(-90))
            ang_vel_x = 0.0
            ang_vel_y = 0.0
            ang_vel_z = 0.0

        status = 1
        info_weight_m = 0.0
        info_weight_Izz = 0.0
        info_weight_cx = 0.0
        info_weight_cy = 0.0
        self.publish_local_start_state((0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0))

        goal = (x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z)
        weightings = [info_weight_m, 0.0, 0.0, info_weight_Izz, 0.0, 0.0, 0.0, 0.0, info_weight_cx, info_weight_cy]
        self.publish_local_goal_instruct(goal, status, weightings)
        rospy.sleep(1.0)
        rospy.loginfo("F/T: %s %s %s", self.local_plan_["wrencharray"].WrenchArray[0].force.x, self.local_plan_["wrencharray"].WrenchArray[0].force.y,
                      self.local_plan_["wrencharray"].WrenchArray[0].torque.z)

    def mpc_check(self):
        """ Verify that info-aware planner and mpc are called correctly.
        The info-aware planner must be called first to provide a reference traj for the controller.
        """
        rospy.loginfo('Sending local plan request to mid-level planner...')
        x = 0.0
        y = -0.0
        z = 0.0
        vel_x = 0.0
        vel_y = 0.0
        vel_z = 0.0
        q = quaternion_from_euler(0, 0, deg2rad(-90))
        ang_vel_x = 0.0
        ang_vel_y = 0.0
        ang_vel_z = 0.0

        status = 1
        info_weight_m = 0.0
        info_weight_Izz = 0.0
        info_weight_cx = 0.0
        info_weight_cy = 7.0

        goal = (x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z)
        weightings = [info_weight_m, 0.0, 0.0, info_weight_Izz, 0.0, 0.0, 0.0, 0.0, info_weight_cx, info_weight_cy]
        self.publish_local_goal_instruct(goal, status, weightings)

        while self.LOCAL_STATUS_ < 2:  # wait for local plan to be computed---this should NOT take longer than DT_L_REPLAN_!
            pass

        rospy.loginfo('Local plan was computed...')

        # (2) Send out the local plan and have nmpc_acado regulate to it
        rospy.loginfo('Sending local plan to controller...')
        throwaway = 5
        self.send_local_plan_to_ctl(throwaway)

    def request_global_plan(self, start, goal, obstacles, ground):
        """
        Send out goal and obstacle information to RRT, get plan result.

        Inputs:
        start: start state [x y th xd yd thd]
        goal: goal state [-]
        obstacles: [[a b c x y z], ... ] of ellipsoids
        ground: 3 DOF or 6 DOF. Only 3 DOF supported for now. ground:= {true, false}

        Outputs:
        obstacles on: rattle/obstacles (optional)
        RRT params on: rattle/rrt/params
        """
        self.rviz_display_obs(obstacles)
        self.rviz_display_start_goal(start, goal)
        self.publish_RRTParams_msg(start, goal, obstacles, ground)
        self.GLOBAL_STATUS_ = 0
        rospy.loginfo("...global plan request made.")

    def globalplan2state(self, pose_des, twist_des):
        """Convert python global plan to state
        """
        x = pose_des.position.x
        y = pose_des.position.y
        z = pose_des.position.z
        vel_x = twist_des.linear.x
        vel_y = twist_des.linear.y
        vel_z = twist_des.linear.z
        q = [pose_des.orientation.x, pose_des.orientation.y, pose_des.orientation.z, pose_des.orientation.w]
        ang_vel_x = twist_des.angular.x
        ang_vel_y = twist_des.angular.y
        ang_vel_z = twist_des.angular.z

        return [x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z]

    def rviz_display_obs(self, obstacles):
        """ Display obstacles in rviz.

        @params:
        ground_: if True, will use 3D ellipsoids
        """
        # make rviz display obstacles!
        markerarray = MarkerArray()
        i = 0
        for obs in obstacles:
            marker_msg = Marker()
            marker_msg.pose.position.x = obs[3]
            marker_msg.pose.position.y = obs[4]

            marker_msg.pose.orientation.x = 0.0
            marker_msg.pose.orientation.y = 0.0
            marker_msg.pose.orientation.z = 0.0
            marker_msg.pose.orientation.w = 1.0
            marker_msg.scale.x = 2*obs[0]
            marker_msg.scale.y = 2*obs[1]

            marker_msg.color.a = 1.0
            marker_msg.color.r = 1.0

            marker_msg.header.frame_id = "world"  # "map"
            marker_msg.id = i

            # if self.ground_ is True:
            #     marker_msg.pose.position.z = 0.0
            #     marker_msg.scale.z = 1.0
            #     marker_msg.type = 3.0  # cylinders
            # else:
            marker_msg.pose.position.z = obs[5]
            marker_msg.scale.z = 2*obs[2]
            marker_msg.type = 2.0  # ellipsoids

            markerarray.markers.append(marker_msg)
            i += 1
        self.vis_pub_.publish(markerarray)

    def rviz_display_start_goal(self, start, goal):
        """ Display obstacles in rviz.

        @params:
        start: start xyz
        goal: goal xyz
        """
        # make rviz display obstacles!
        markerarray = MarkerArray()

        # start
        marker_msg = Marker()
        marker_msg.pose.position.x = start[0]
        marker_msg.pose.position.y = start[1]

        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2

        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0

        marker_msg.header.frame_id = "world"  # "map"
        marker_msg.id = 777
        marker_msg.type = 2.0  # sphere

        marker_msg.pose.position.z = start[2]

        markerarray.markers.append(marker_msg)

        # goal
        marker_msg = Marker()
        marker_msg.pose.position.x = goal[0]
        marker_msg.pose.position.y = goal[1]

        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.2
        marker_msg.scale.y = 0.2
        marker_msg.scale.z = 0.2

        marker_msg.color.a = 1.0
        marker_msg.color.g = 1.0

        marker_msg.header.frame_id = "world"  # "map"
        marker_msg.id = 778
        marker_msg.type = 2.0  # sphere

        marker_msg.pose.position.z = goal[2]

        markerarray.markers.append(marker_msg)
        self.vis_start_goal_pub_.publish(markerarray)

    def load_rrt_from_csv(self, path, filename, CREATE_MSG):
        """ Load an rrt solution from csv file.

        Inputs:
        path - without trailing '/'
        filename
        CREATE_MSG - publish and update global_plan_

        Outputs:
        PoseArray and TwistArray messages
        """

        plan = np.empty((0, 6), int)
        csv_name = path + "/" + filename
        with open(csv_name, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in reader:
                row = row[:-1]
                row = map(float, row)
                plan_i = np.asarray(row)
                plan = np.append(plan, [plan_i], axis=0)

        if CREATE_MSG:
            poses = []
            twistArray = []
            poseArray = PoseArray()
            for i in range(0, np.shape(plan)[0]):
                pose = Pose()
                pose.position.x = plan[i, 0]
                pose.position.y = plan[i, 1]
                pose.position.z = 0.0
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = plan[i, 2]

                twistStamped = TwistStamped()
                twistStamped.twist.linear.x = plan[i, 3]
                twistStamped.twist.linear.y = plan[i, 4]
                twistStamped.twist.linear.z = 0.0
                twistStamped.twist.angular.x = 0.0
                twistStamped.twist.angular.y = 0.0
                twistStamped.twist.angular.z = plan[i, 5]

                poses.append(pose)
                twistArray.append(twistStamped)
            poseArray.poses = poses
            poseArray.header.frame_id = "world"

            self.rrt_csv_pose_pub_.publish(poseArray)
            self.rrt_csv_twist_pub_.publish(twistArray)

            rospy.loginfo("...RRT loaded from CSV, msg published.")

        return plan

    def rrt_monte_carlo(self):
        """ A Monte Carlo test of the RRT speed.
        """
        N = 50
        times = np.zeros(N)

        for idx in range(N):
            # Create obstacle field
            i = 4
            j = 4
            np_obs = np.array(np.zeros((i*j, 6)))
            for m in range(0, i):
                for n in range(0, j):
                    np_obs[m*j + n, 0] = 0.1
                    np_obs[m*j + n, 1] = 0.1
                    np_obs[m*j + n, 2] = 0.1
                    np_obs[m*j + n, 3] = (-0.8 + (0.8*2/(i - 1))*m) + (np.random.rand() - 0.5)*2*0.1
                    np_obs[m*j + n, 4] = (-0.8 + (0.8*2/(j - 1))*n) + (np.random.rand() - 0.5)*2*0.1
                    np_obs[m*j + n, 5] = 0.0
            obstacles = np_obs.tolist()

            # Request an RRT
            t0 = rospy.get_time()
            start = [-0.5, 0.7, 0.0, 0.0, 0.0, 0.0, 1.0]
            goal = [0.5, -0.7, 0.0, 0.0, 0.0, 0.0, 1.0]
            DOF = 3
            ground = True
            rospy.loginfo("Start pose %s" % str(start))
            self.request_global_plan(start, goal, obstacles, ground)
            while self.GLOBAL_STATUS_ < 2:  # wait for global plan to be computed and received
                pass
            tf = rospy.get_time()
            self.GLOBAL_STATUS_ = 0

            times[idx] = (tf - t0)
            print(times)
        print(np.mean(times))
        print(np.std(times))

    def rviz_clear_all_markers(self):
        marker_array_msg = MarkerArray()
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(marker)
        self.vis_pub_.publish(marker_array_msg)

    def debug(self):
        """ Debug different components. Requires plotting.
        """
        rospy.loginfo('Debug...')

        if self.sim_ is True:
            import matplotlib.pyplot as plt
        else:
            rospy.loginfo('...on hardware, no plotting possible.')
            return
        # rospack = rospkg.RosPack()
        # path = rospack.get_path('data') + "/output"
        # filename = "blargh_3dof.csv"
        # load_rrt_from_csv(path, filename, 1)

        # print(global_plan_["twistarray"])
        # print(global_plan_["posearray"])

        len = 100

        w0_step = np.zeros((len, 4))
        w0_exp = np.zeros((len, 4))
        for i in range(0, len):
            w0_step[i, :] = self.get_weights_step(i, len, np.array([10, 10, 10, 10]))
            w0_exp[i, :] = self.get_weights_exponential(i, len, np.array([10, 10, 10, 10]))
        fig, ax = plt.subplots()
        ax.plot(range(0, len), w0_step[:, 0])
        ax.plot(range(0, len), w0_exp[:, 0])
        plt.show()

    def _numpy_to_multiarray(multiarray_type, np_array):
        multiarray = multiarray_type()
        multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
                                 np_array.shape[i],
                                 np_array.shape[i] * np_array.dtype.itemsize) for i in range(np_array.ndim)]
        multiarray.data = np_array.reshape([1, -1])[0].tolist()
        return multiarray

    np_to_multiarray_f64 = partial(_numpy_to_multiarray, Float64MultiArray)

    def estimation_debug_maneuvers(self):
        """
        Performs a series of translation maneuvers to gather data for the estimator
        Uses info-aware planner and mpc are called correctly.
        The info-aware planner must be called first to provide a reference traj for the controller.
        """
        # assuming that the primary initial position is 0,0.6 and quaternions are 0,0,0,1
        z = 0.0
        vel_x = 0.0
        vel_y = 0.0
        vel_z = 0.0
        q = [0, 0, 0, 1]
        ang_vel_x = 0.0
        ang_vel_y = 0.0
        ang_vel_z = 0.0

        status = 1
        info_weight_m = 0.0
        info_weight_Izz = 0.0
        info_weight_cx = 0.0
        info_weight_cy = 0.0

        x = [0.0, 0.5, 0.0]
        y = [-0.5, -0.5, 0.0]
        qz = [0, 0.7071, 0]
        qw = [1, 0.7071, 1]
        for i in range(3):
            self.LOCAL_STATUS_ = 0
            rospy.loginfo('Sending local plan request to mid-level planner...')

            goal = (x[i], y[i], z, vel_x, vel_y, vel_z, q[0], q[1], qz[i], qw[i], ang_vel_x, ang_vel_y, ang_vel_z)
            weightings = [info_weight_m, 0.0, 0.0, info_weight_Izz, 0.0, 0.0, 0.0, 0.0, info_weight_cx, info_weight_cy]
            self.publish_local_goal_instruct(goal, status, weightings)

            while self.LOCAL_STATUS_ < 2:  # wait for local plan to be computed
                pass

            rospy.loginfo('Local plan was computed...')

            # (2) Send out the local plan and have nmpc_acado regulate to it
            rospy.loginfo('Sending local plan to controller...')
            throwaway = 0
            self.send_local_plan_to_ctl(throwaway)

            rospy.sleep(30.0)

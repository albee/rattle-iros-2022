'''
_rattle_coordinator_core.py

Core coordinating code to manage RATTLE components.

Keenan Albee and Monica Ekal, 2021
MIT Space Systems Lab
'''
import rospy
import rospkg

# local plan
from geometry_msgs.msg import PoseArray
from rattle_rrt.msg import TwistArray, WrenchArray
from std_msgs.msg import Float64MultiArray

import csv
import numpy as np
import rospkg
from math import ceil, floor, exp
from numpy import deg2rad
from tf.transformations import quaternion_from_euler
import time


class Mixin:
    def run_test(self, test_num):
        """ Execute a test using configuration options.
        Triggered by `/rattle/test_instruct` callback.
        """
        rospy.loginfo("""
        ***********
        *[RATTLE_COORD]
        *Executing test: %s
        *USE_EKF_POSE: %d
        *USE_PARAMS: %d
        *WEIGHT_MODE: %s
        *INITIAL_MODEL_MODE: %s
        *ground_: %r
        *(good luck!)
        ***********""", test_num, self.USE_EKF_POSE, self.USE_PARAMS,
                      self.WeightMode[self.WEIGHT_MODE], self.InitialModelMode[self.INITIAL_MODEL_MODE], self.ground_)

        self.rviz_clear_all_markers()  # needed to get rid of old displays

        if test_num == "debug":
            self.debug()
        elif test_num == "global_plan_check":  # global planner
            self.global_plan_check()
        elif test_num == "planner_acado_check1":  # local planner
            self.planner_acado_check(1)
        elif test_num == "planner_acado_check2":
            self.planner_acado_check(2)
        elif test_num == "mpc_check":  # low-level controller
            self.mpc_check()
        elif test_num == "full":  # a full run of the RATTLE algo
            self.create_rrt_global_plan(self.USE_EKF_POSE, self.USE_CSV)
            self.run_global_plan(self.WEIGHT_MODE)
        elif test_num == "full_replan":  # a full run of the RATTLE algo
            self.create_rrt_global_plan(self.USE_EKF_POSE, self.USE_CSV)
            given_start = self.run_global_plan(self.WEIGHT_MODE, interrupt_mode=1)

            # oh no an astronaut
            if self.ground_:
                self.obstacles_ = self.obstacles_opts["ground"][4]
            else:
                self.obstacles_ = self.obstacles_opts["iss"][4]

            # replan!
            self.create_rrt_global_plan(0, self.USE_CSV, given_start)  # do NOT use EKF pose
            self.run_global_plan(0)  # no weighting
        elif test_num == "rrt_monte_carlo":
            self.rrt_monte_carlo()
        elif test_num == "pure_sys_id":
            self.pure_sys_id()
        elif test_num == "pure_izz_weight":
            self.pure_izz_weight()
        elif test_num == "pure_mass_weight":
            self.pure_mass_weight()
        elif test_num == "estimation_debug_maneuvers":
            self.estimation_debug_maneuvers()
        else:
            rospy.loginfo("...no tests match!")

    def create_rrt_global_plan(self, USE_EKF_POSE, USE_CSV, given_start=0):
        """ The main test script. Creates an RRT global plan, and runs the mid-level
        information-aware planner.

        Inputs:
        USE_EKF_POSE - specifies whether to use the estimated current state, {0, 1}
        USE_CSV - whether to load CSV from file, {0, 1}
        """
        obstacles = self.obstacles_
        ground = self.ground_
        RRT_FILENAME = self.RRT_FILENAME
        OOA_TEST = self.OOA_TEST

        """
        (1) Produce a global plan with request_global_plan. The RRT nodelet will publish
        the global plan on rattle/rrt/path/twistarray and rattle/rrt/path/posearray
        """
        if not USE_CSV:
            if USE_EKF_POSE:  # x_est_ = [r(3) v(3) q(4) w(3)].T
                while (self.GOT_EKF == 0):  # don't have EKF
                    pass
                rospy.sleep(0.7)  # wait for EKF...
                x_est_ = self.x_est_  # this freezes the x_est_ used!
                start = [x_est_[0, 0], x_est_[1, 0], x_est_[2, 0], x_est_[6, 0], x_est_[7, 0], x_est_[8, 0], x_est_[9, 0]]

            if self.ground_:
                # if this run is a part of the on-orbit assembly tests (test 13, then use
                # start and goal points accordingly)
                if OOA_TEST:
                    goal = self.POINT_C_GRANITE  # x = [x y z, quat]
                else:
                    goal = self.POINT_B_GRANITE  # x = [x y z, quat]
                if not USE_EKF_POSE:
                    if OOA_TEST == 1:
                        start = self.POINT_B_GRANITE  # x = [x y z, quat]
                    elif OOA_TEST == 2:
                        start = self.POINT_C_GRANITE  # x = [x y z, quat]
                    elif given_start:
                        start = given_start
                    else:
                        start = self.POINT_A_GRANITE  # x = [x y z, quat]
            else:
                if OOA_TEST:
                    goal = self.POINT_C_ISS  # x = [x y z, quat]
                else:
                    goal = self.POINT_B_ISS  # x = [x y z, quat]
                if not USE_EKF_POSE:
                    if OOA_TEST == 1:
                        start = self.POINT_B_ISS  # x = [x y z, quat]
                    elif OOA_TEST == 2:
                        start = self.POINT_C_ISS  # x = [x y z, quat]
                    elif given_start:
                        start = given_start
                    else:
                        start = self.POINT_A_ISS  # x = [x y z, quat]
            rospy.loginfo("start pose %s" % str(start))
            self.request_global_plan(start, goal, obstacles, ground)
        else:
            rospack = rospkg.RosPack()
            path = rospack.get_path('data') + "/output"
            filename = RRT_FILENAME
            self.load_rrt_from_csv(path, filename, 1)
            self.rviz_display_obs(obstacles)

        # self.initialize_fans()  # start up FAM now
        while self.GLOBAL_STATUS_ < 2:  # wait for global plan to be computed and received
            pass

    def run_global_plan(self, WEIGHT_MODE, throwaway_mode=0, interrupt_mode=0):
        """ Using the global plan, execute the plan to completion.

        @param WEIGHT_MODE: {"no_weight", "step_weight", "exp_weight", "izz", "mass"} as int, weighting scheme to use
        @param throwaway_mode: {0, 1} use throwaway 
        @param interrupt_mode: {0, 1} if interrupt, stop execution halfway through
        
        """

        """
        (1) Prepare waypoint timing
        """
        waypoint_idxs, waypoint_step, len_global_plan, tf = self.prep_global_waypoints()

        t_l_comp = 4.0  # typical time for local plan computation, [s]
        if throwaway_mode:
            throwaway = int(ceil(t_l_comp/self.DT_L_))  # number of points of local plan to drop (approx.)
        else:
            throwaway = 0

        """
        (2) Send out the global plan waypoints to the mid-level (local) planner
        on rattle/nmpc_planner/info_plan_instruct
        """
        rospy.loginfo('Executing global plan...')
        replan_rate = rospy.Rate(1.0/self.t_L_)
        iter = 0

        if interrupt_mode:
            waypoint_idxs = waypoint_idxs[0: int(floor(len(waypoint_idxs)/2))]

        # get the final setpoint of the global plan, as [x y z qx qy qz qw]
        start_pose = self.global_plan_["posearray"][waypoint_idxs[-1]]
        start_twist = self.global_plan_["twistarray"][waypoint_idxs[-1]].twist
        start_state = self.globalplan2state(start_pose, start_twist)
        given_start = start_state[0:7]

        for i in range(1, len(waypoint_idxs)):  # send out the global waypoints (NOT including start waypoint)
            idx_start = waypoint_idxs[i - 1]
            idx_goal = waypoint_idxs[i]

            self.LOCAL_STATUS_ = 0
            t = (iter)*self.t_L_  # the current time
            iter += 1

            rospy.loginfo("Current time: %s, final time: %s, global_plan iter: %s" % (str(t), str(tf), str(i)))

            # update start state for local planning
            start_pose = self.global_plan_["posearray"][idx_start]
            start_twist = self.global_plan_["twistarray"][idx_start].twist
            start_state = self.globalplan2state(start_pose, start_twist)
            self.publish_local_start_state(start_state)
            time.sleep(0.1)  # ensure start state received

            # update global goal state/info for local planning
            pose_des = self.global_plan_["posearray"][idx_goal]
            twist_des = self.global_plan_["twistarray"][idx_goal].twist

            state_des = self.globalplan2state(pose_des, twist_des)
            if (idx_goal == len_global_plan - 1):  # only do this for the actual final point
                state_des[6:10] = [0, 0, 0, 1]
                state_des[3] = 0.0
                state_des[4] = 0.0
                state_des[5] = 0.0
            status = 1

            """
            (2)(a) Get weighting scheme
            """
            weight_i = self.get_weighting(t, tf, self.sigma_floor_, self.sigma_, WEIGHT_MODE, self.ground_)
            if self.ground_ is True:
                info_weight_m = weight_i[0]
                info_weight_Ixx = 0.0
                info_weight_Iyy = 0.0
                info_weight_Izz = weight_i[1]
                info_weight_cx = weight_i[2]
                info_weight_cy = weight_i[3]
            else:
                info_weight_m = weight_i[0]
                info_weight_Ixx = weight_i[1]
                info_weight_Iyy = weight_i[2]
                info_weight_Izz = weight_i[3]
                info_weight_cx = 0.0
                info_weight_cy = 0.0

            # hand off goal to rattle_acado_planner
            weightings = [info_weight_m, info_weight_Ixx, info_weight_Iyy, info_weight_Izz, 0.0, 0.0, 0.0, 0.0, info_weight_cx, info_weight_cy]
            self.publish_local_goal_instruct(state_des, status, weightings)

            while self.LOCAL_STATUS_ < 3:  # wait for local plan to be computed---this should NOT take longer than DT_L_REPLAN_!
                pass

            """
            (3) Send out the local plan to the low-level controller on
            rattle/local/path_ctl/*_array.

            The local plan is used for tracking by the MPC.
            """

            self.send_local_plan_to_ctl(throwaway)

            if i != 1:  # for the first iteration, go on ahead to the next immediately
                replan_rate.sleep()

        # self.publish_rattle_status(True)
        rospy.loginfo('...maneuver complete!')
        return given_start

    def prep_global_waypoints(self, num_local_replans=None):
        """Prepare global waypoints to send to local planner.

        @params:
        [num_local_replans] (optional): number of times to force replanning
        """
        nominal_global_waypoints_per_local = int(floor(self.DT_L_*self.N_L_/self.DT_G_))  # number of global waypoints per local plan, nominal

        len_global_plan = len(self.global_plan_["posearray"])
        if num_local_replans is None:
            waypoint_step = int(ceil(self.t_L_ / self.DT_G_))
            tf = self.DT_G_ * len_global_plan
        else:
            waypoint_step = int(ceil(len_global_plan/(num_local_replans)))  # number of global waypoints to step per local plan (no local scaling)
            tf = num_local_replans*self.t_L_

        # if (waypoint_step > nominal_global_waypoints_per_local):
        #     waypoint_step = int(max_global_waypoints_per_seg/2)

        rospy.loginfo("""
              Timing info:
              DT_G_: %s
              t_L_: %s
              DT_L_: %s
              N_L_: %s
              waypoint_step: %s
              len_global_plan: %s
              """, self.DT_G_, self.t_L_, self.DT_L_, self.N_L_, waypoint_step, len_global_plan)

        # dt_l_replan_adj = self.t_L_/waypoint_step  # wall clock dt for local replanning
        waypoint_idxs = list(range(0, len_global_plan, waypoint_step))
        if (waypoint_idxs[-1] != len_global_plan):
            waypoint_idxs.append(len_global_plan - 1)  # add in the final point (extra replan)
        # print("idxs: ", waypoint_idxs)

        return waypoint_idxs, waypoint_step, len_global_plan, tf

    def send_local_plan_to_ctl(self, throwaway):
        """ Send out the local plan to the low-level controller on
        rattle/local/path_ctl/*_array

        Inputs:
        throwaway: number of timesteps to get rid of (due to computational lag)
        """
        # get rid of the first throwaway terms
        self.local_plan_["posearray"].poses = self.local_plan_["posearray"].poses[throwaway:]
        self.local_plan_["twistarray"].TwistArray = self.local_plan_["twistarray"].TwistArray[throwaway:]
        self.local_plan_["wrencharray"].WrenchArray = self.local_plan_["wrencharray"].WrenchArray[throwaway:]

        self.path_ctl_pose_pub_.publish(self.local_plan_["posearray"])
        self.path_ctl_twist_pub_.publish(self.local_plan_["twistarray"])
        self.path_ctl_wrench_pub_.publish(self.local_plan_["wrencharray"])
        self.CONTROLLER_STATUS_ = 1  # TODO are these used by controller?
        self.EST_STATUS_ = self.CONTROLLER_STATUS_
        rospy.set_param("/rattle/CONTROLLER_STATUS", self.CONTROLLER_STATUS_)
        rospy.set_param("/rattle/EST_STATUS", self.EST_STATUS_)

        # similar to send_traj_to_controller( ) function of primary_coordinator
        x_des_traj_msg = Float64MultiArray()
        x_des_traj_msg = self.ref_traj2float64multiarray(self.local_plan_)  # convert local plan to x_des_traj

        # publish on TOPIC_RATTLE_TUBE_MPC_TRAJ = `/rattle/tube_mpc/traj`

        self.path_ctl_x_des_traj_pub_.publish(x_des_traj_msg)
        time.sleep(0.3)  # make sure traj received
        self.control_mode_pub_.publish("track_tube")

    def ref_traj2float64multiarray(self, local_plan):
        """Convert the poseArray, twistArray, wrenchArray format to a Float64MultiArray.

        Float64MultiArray:
        """
        poseArray = local_plan["posearray"]
        twistArray = local_plan["twistarray"]
        wrenchArray = local_plan["wrencharray"]

        rows = len(poseArray.poses)
        cols = 20

        x_des_traj_msg = Float64MultiArray()
        x_des_traj_np = np.empty([rows, cols])

        # x_des_traj = [t, x, y, z, xd, yd, zd, qx, qy, qz, qw, wx, wy, wz, xdd, ydd, zdd, wxd, wyd, wzd; ...]
        for i in range(rows):
            t = self.DT_L_*i
            pose = poseArray.poses[i]
            twist = twistArray.TwistArray[i].twist  # TODO: make this match the std_msg format
            x_des_traj_np[i, 0] = t
            x_des_traj_np[i, 1] = pose.position.x
            x_des_traj_np[i, 2] = pose.position.y
            x_des_traj_np[i, 3] = pose.position.z
            x_des_traj_np[i, 4] = twist.linear.x
            x_des_traj_np[i, 5] = twist.linear.y
            x_des_traj_np[i, 6] = twist.linear.z
            x_des_traj_np[i, 7] = pose.orientation.x
            x_des_traj_np[i, 8] = pose.orientation.y
            x_des_traj_np[i, 9] = pose.orientation.z
            x_des_traj_np[i, 10] = pose.orientation.w
            x_des_traj_np[i, 11] = twist.angular.x
            x_des_traj_np[i, 12] = twist.angular.y
            x_des_traj_np[i, 13] = twist.angular.z
            x_des_traj_np[i, 14:20] = 0.0
            # print(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

        x_des_traj_msg = self.np_to_multiarray_f64(x_des_traj_np)
        # print(x_des_traj_msg)
        x_des_traj_msg.layout.dim[0].size = rows
        x_des_traj_msg.layout.dim[1].size = cols

        return x_des_traj_msg

    def get_weighting(self, t, tf, sigma_floor, sigma, WEIGHT_MODE, GROUND):
        """ Get the desired weighting scheme.

        @param WEIGHT_MODE: {"no_weight", "step_weight", "exp_weight", "izz", "mass"} as int
        @param ground: {0, 1} use 3DOF version
        @return weight_i - np.array((4)) [m Izz cx cy] ground / [m Ixx Iyy Izz] ISS
        """
        if WEIGHT_MODE == 0:
            weight_i = np.zeros(4)
        if WEIGHT_MODE == 1:  # step weighting
            if GROUND is True:
                weight_vec0 = np.array([30.0, 10.0, 3, 3])  # m, I_zz, cx, cy weightings
                weight_i = self.get_weights_step(t, tf, weight_vec0)
            else:
                weight_vec0 = np.array([50.0, 20.0, 20.0, 20.0])  # m, I_xx, I_yy, I_zz weightings
                weight_i = self.get_weights_step(t, tf, weight_vec0)
        if WEIGHT_MODE == 2:
            if GROUND is True:  # exp weighting
                weight_vec0 = np.array([30.0, 10.0, 3, 3])  # m, I_zz, cx, cy weightings
                weight_i = self.get_weights_exponential(t, tf, weight_vec0)
            else:
                weight_vec0 = np.array([50.0, 20.0, 20.0, 20.0])  # m, I_xx, I_yy, I_zz weightings
                weight_i = self.get_weights_exponential(t, tf, weight_vec0)
        if WEIGHT_MODE == 3:  # Izz info only
            if GROUND is True:
                weight_vec0 = np.array([0, 10.0, 0, 0])  # m, I_zz, cx, cy weightings
                weight_i = self.get_weights_step(t, tf, weight_vec0)
            else:
                weight_vec0 = np.array([0, 0.0, 0, 20.0])  # m, I_xx, I_yy, I_zz weightings
                weight_i = self.get_weights_step(t, tf, weight_vec0)
        if WEIGHT_MODE == 4:  # mass info only
            weight_vec0 = np.array([75, 0, 0, 0])  # m, I_zz, cx, cy weightings
            weight_i = self.get_weights_step(t, tf, weight_vec0)
        if WEIGHT_MODE == 5:  # covar-weighted info
            if GROUND is True:  # exp weighting
                weight_vec0 = np.array([30.0, 10.0, 3, 3])  # m, I_zz, cx, cy weightings
                weight_i = self.get_weights_covar(sigma_floor, sigma, weight_vec0)
            else:
                weight_vec0 = np.array([50.0, 20.0, 20.0, 20.0])  # m, I_xx, I_yy, I_zz weightings
                weight_i = self.get_weights_covar(sigma_floor, sigma, weight_vec0)

        # if t >= 2.0*tf/8.0:
        #     weight_i = np.zeros((4))  # no weighting after this point
        return weight_i

    def get_weights_step(self, t, tf, weight_vec0):
        """Get information weightings as a decaying step function

        Inputs:
        t - time of weighting
        tf - final time
        weight_vec0 - np.array([w_m w_Izz w_cx w_cy])

        Outputs:
        w0
        """
        w0 = weight_vec0
        print("t :", t)
        print("tf :", tf)

        if t >= tf/4.0 and t < 2.0*tf/4.0:
            w0 = w0*0.8
        elif t >= 2.0*tf/4.0 and t < 3.0*tf/4.0:
            w0 = w0*0.4
        elif t >= 3.0*tf/4.0:
            w0 = w0*0.0

        print("w0 :", w0)

        return w0

    def get_weights_covar(self, sigma_floor, sigma, weight_vec0):
        """Get information weightings as responsive covariance-weighted values.

        @param sigma_floor: sigma noise floor
        @param sigma: current parameter variances
        @param weight_vec0: initial gammas, np.array([w_m w_Izz w_cx w_cy])

        @return gamma: np.array(4) of weighting values
        """
        alpha = 2.2  # what fudge factor should we allow for noise floor?
        beta = 5  # how quickly should the exponential decay? (and what is the maximum drop?)
        gamma0 = weight_vec0
        gamma = gamma0

        print("Using covar weighting...")
        print("alpha :", alpha)
        print("beta :", beta)

        for idx, val in enumerate(weight_vec0):
            if sigma[idx] <= alpha*sigma_floor[idx]:  # zero out near noise floor
                gamma[idx] = 0.0
            else:
                gamma[idx] = val*exp(-beta*sigma_floor[idx]/sigma[idx])  # decrease as noise floor is approached

        print("gamma :", gamma)

        return gamma

    def get_weights_exponential(self, t, tf, weight_vec0):
        """Get decaying information weightings via decaying exponential.

        Inputs:
        t - time of weighting
        tf - final time
        weight_vec0 - np.array([w_m w_Izz w_cx w_cy])
        tau - time constant (time for ~1/2 decay)

        Outputs:
        w0
        """
        tau = tf/8
        t = float(t)
        tf = float(tf)

        c = np.exp(-t/tau)
        if np.exp(-t/tau) < 0.1:
            c = 0.0
        w0 = weight_vec0*c

        return w0

    def pure_sys_id(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('data') + "/output"
        filename = "SysID_plan.csv"
        csv_name = path + "/" + filename
        plan = np.empty((0, 12), int)  # 9 states( rx, ry, vx, vy, q, wz) and 3 actuation(fx, fy, tz)
        time = []
        with open(csv_name, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quotechar='|')
            for row in reader:
                row_ = row[1:]
                row_ = map(float, row_)
                plan_i = np.asarray(row_)
                plan = np.append(plan, [plan_i], axis=0)
                time = np.append(time, row[0])
        rospy.loginfo('Sending out trajectory plan...')
        local_plan_rate = rospy.Rate(1/(self.N_L*self.DT_L_))
        begin = 0
        for i in range(int(floor(plan.shape[0]/self.N_L_))):  # send out the waypoints
            rospy.loginfo('Iteration 1...')
            self.publish_local_plan_msg(plan[i*self.N_L_:(i + 1)*self.N_L_, :])
            # if (begin == 0):
            #    rospy.sleep(5)  # wait the robot to actually move
            #    begin = 1
            local_plan_rate.sleep()

    def pure_mass_weight(self):
        """Call local planner and command pure mass ID
        """
        n = 3  # how many informative local plans to run
        for i in range(n):
            if self.ground_:
                start_point = self.POINT_A_GRANITE
            else:
                start_point = self.POINT_A_ISS

            start_state = start_point[0:3] + [0.0, 0.0, 0.0] + start_point[3:7] + [0.0, 0.0, 0.0]
            self.publish_local_start_state(start_state)
            time.sleep(0.1)

            # hand off goal to rattle_acado_planner
            state_des = start_point[0:3] + [0.0, 0.0, 0.0] + start_point[3:7] + [0.0, 0.0, 0.0]
            status = 1
            info_weight_m = 100.0
            info_weight_Izz = 0.0
            info_weight_cx = 0.0
            info_weight_cy = 0.0
            # hand off goal to rattle_acado_planner
            weightings = [info_weight_m, 0.0, 0.0, info_weight_Izz, 0.0, 0.0, 0.0, 0.0, info_weight_cx, info_weight_cy]
            self.publish_local_goal_instruct(state_des, status, weightings)

            while self.LOCAL_STATUS_ < 3:  # wait for local plan to be computed---this should NOT take longer than DT_L_REPLAN_!
                pass

            self.send_local_plan_to_ctl(0)

            if i < n - 1:  # no sleep for the second to last plan
                time.sleep(self.DT_L_)

    def pure_izz_weight(self):
        """Call local planner and command pure I_zz ID
        """

        n = 3  # how many informative local plans to run
        for i in range(n):
            if self.ground_:
                start_point = self.POINT_A_GRANITE
            else:
                start_point = self.POINT_A_ISS
            start_state = start_point[0:3] + [0.0, 0.0, 0.0] + start_point[3:7] + [0.0, 0.0, 0.0]
            self.publish_local_start_state(start_state)
            time.sleep(0.1)

            # hand off goal to rattle_acado_planner
            state_des = start_point[0:3] + [0.0, 0.0, 0.0] + start_point[3:7] + [0.0, 0.0, 0.0]
            status = 1
            info_weight_m = 0.0
            info_weight_Izz = 30.0
            info_weight_cx = 0.0
            info_weight_cy = 0.0
            # hand off goal to rattle_acado_planner
            weightings = [info_weight_m, 0.0, 0.0, info_weight_Izz, 0.0, 0.0, 0.0, 0.0, info_weight_cx, info_weight_cy]
            self.publish_local_goal_instruct(state_des, status, weightings)

            while self.LOCAL_STATUS_ < 3:  # wait for local plan to be computed---this should NOT take longer than DT_L_REPLAN_!
                pass

            self.send_local_plan_to_ctl(0)

            if i < n - 1:  # no sleep for the second to last plan
                time.sleep(self.DT_L_)

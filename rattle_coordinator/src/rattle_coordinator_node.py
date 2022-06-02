#!/usr/bin/env python2
from rattle_coordinator import RattleCoordinator
import rospy

# msgs
from ff_msgs.msg import EkfState
from ff_msgs.msg import FamCommand
from reswarm_msgs.msg import RattleTestInstruct
from reswarm_msgs.msg import RattleInfoPlanInstruct  # formerly NMPCInstruct
from rattle_rrt.msg import ellipsoid, ellipsoidArray, RRTParams, TwistArray, WrenchArray
from reswarm_msgs.msg import ReswarmRattleStatus
from param_est.msg import Params

# std msgs
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Point, PoseArray, Quaternion, Twist, Vector3, Pose, TwistStamped, Wrench
from visualization_msgs.msg import Marker, MarkerArray

VERBOSE = 1


class RattleCoordinatorNode(RattleCoordinator):
    def __init__(self):
        """
        Create ROS params, subs, pubs.
        """
        rospy.loginfo('[RATTLE_COORD]: Initialized.')

        # ***Params***
        # set timing parameters
        rospy.set_param("/rattle/DT_C", self.DT_C_)
        rospy.set_param("/rattle/DT_G", self.DT_G_)
        rospy.set_param("/rattle/DT_L", self.DT_L_)
        rospy.set_param("/rattle/N_C", self.N_C_)
        rospy.set_param("/rattle/N_L", self.N_L_)
        rospy.set_param("/rattle/CONTROLLER_STATUS", self.CONTROLLER_STATUS_)
        rospy.set_param("/rattle/EST_STATUS", self.EST_STATUS_)

        # tracking points
        self.POINT_A_GRANITE = rospy.get_param("/reswarm/primary/point_a_granite")
        self.POINT_A_ISS = rospy.get_param("/reswarm/primary/point_a_iss")
        self.POINT_B_GRANITE = rospy.get_param("/reswarm/primary/point_b_granite")
        self.POINT_B_ISS = rospy.get_param("/reswarm/primary/point_b_iss")
        self.POINT_C_GRANITE = rospy.get_param("/reswarm/primary/point_c_granite")
        self.POINT_C_ISS = rospy.get_param("/reswarm/primary/point_c_iss")

        # ***Subs***
        # main test logic
        rospy.Subscriber("/rattle/test_instruct", RattleTestInstruct, self.test_instruct_callback)  # always global, triggers everything
        rospy.Subscriber("gnc/ekf", EkfState, self.ekf_state_callback)

        # subscribe to: high-level, mid-level, low-level statuses. TODO
        rospy.Subscriber("rattle/rrt_high_level/status", numpy_msg(FamCommand), self.ctls_callback)
        rospy.Subscriber("rattle/nmpc_acado_planner/status", numpy_msg(FamCommand), self.ctls_callback)
        rospy.Subscriber("rattle/nmpc_ctl/status", numpy_msg(FamCommand), self.ctls_callback)

        # global plan
        rospy.Subscriber("rattle/rrt/path/posearray", PoseArray, self.rrt_posearray_callback)
        rospy.Subscriber("rattle/rrt/path/twistarray", TwistArray, self.rrt_twistarray_callback)

        # local plan
        rospy.Subscriber("rattle/local/path/posearray", PoseArray, self.local_posearray_callback)
        rospy.Subscriber("rattle/local/path/twistarray", TwistArray, self.local_twistarray_callback)
        rospy.Subscriber("rattle/local/path/wrencharray", WrenchArray, self.local_wrencharray_callback)

        # estimator
        rospy.Subscriber("mob/inertia_est", Params, self.update_parameters_callback)

        # ***Pubs***
        # high-level global planner
        self.rrt_pub_ = rospy.Publisher("rattle/rrt/params", RRTParams, queue_size=1)
        self.obs_pub_ = rospy.Publisher("rattle/obstacles", ellipsoidArray, queue_size=1)
        self.vis_pub_ = rospy.Publisher("rattle/vis/obs", MarkerArray, queue_size=3)
        self.vis_start_goal_pub_ = rospy.Publisher("rattle/vis/start_goal", MarkerArray, queue_size=3)
        self.rrt_csv_pose_pub_ = rospy.Publisher("rattle/rrt/path/posearray", PoseArray, queue_size=1)
        self.rrt_csv_twist_pub_ = rospy.Publisher("rattle/rrt/path/twistarray", TwistArray, queue_size=1)

        # mid-level local planner
        self.local_goal_pub_ = rospy.Publisher("rattle/local/info_plan_instruct", RattleInfoPlanInstruct, queue_size=1)
        self.local_start_pub_ = rospy.Publisher("rattle/local/info_plan_instruct_start", RattleInfoPlanInstruct, queue_size=1)

        self.path_local_pose_pub_ = rospy.Publisher("rattle/local/path/posearray", PoseArray, queue_size=3)
        self.path_local_twist_pub_ = rospy.Publisher("rattle/local/path/twistarray", TwistArray, queue_size=3)
        self.path_local_wrench_pub_ = rospy.Publisher("rattle/local/path/wrencharray", WrenchArray, queue_size=3)

        # low-level controller
        self.path_ctl_pose_pub_ = rospy.Publisher("rattle/local/path_ctl/posearray", PoseArray, queue_size=3)
        self.path_ctl_twist_pub_ = rospy.Publisher("rattle/local/path_ctl/twistarray", TwistArray, queue_size=3)
        self.path_ctl_wrench_pub_ = rospy.Publisher("rattle/local/path_ctl/wrencharray", WrenchArray, queue_size=3)

        # reswarm
        self.rattle_status_pub_ = rospy.Publisher("reswarm/rattle/status", ReswarmRattleStatus, queue_size=1)
        self.path_ctl_x_des_traj_pub_ = rospy.Publisher("reswarm/tube_mpc/traj", Float64MultiArray, queue_size=3)  # for casadi_nmpc
        self.control_mode_pub_ = rospy.Publisher("reswarm/primary/control_mode", String, queue_size=3)  # for casadi_nmpc

        self.sanity_check_timing()


if __name__ == '__main__':
    if VERBOSE == 1:
        rospy.init_node('rattle_coordinator')
    else:
        rospy.init_node('rattle_coordinator', log_level=rospy.ERROR)
    node = RattleCoordinatorNode()
    while not rospy.is_shutdown():
        rospy.spin()

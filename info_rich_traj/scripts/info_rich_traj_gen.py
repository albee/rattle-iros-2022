#!/usr/bin/env python3
#from casadi import *
import csv
import rospy
import rospkg
import numpy as np
from functools import partial
from ff_msgs.msg import FlightMode, ControlState
from geometry_msgs.msg import PoseStamped, TwistStamped, Wrench
from reswarm_msgs.msg import ReswarmStatusPrimary
from reswarm_msgs.msg import ReswarmInfoStatus
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension


dt = 0.5

class InfoRichPlanner:
    def __init__(self, N, sim, Ground):
        self.N = N
        self.ind = 0
        self.ind_pause = 0
        self.ind_pause = 0
        self.test_num = -1
        self.info_traj_sent = False
        self.info_traj_send = False
        self.new_test = False
        self.p = [10, 0.1, 0.1, 0.1]
        if (sim == "true"):
            self.topic_prefix = "/queen/"
        else:
            self.topic_prefix = "/"
        if (Ground == "true"):
            self.env_prefix = "grd"
        else:
            self.env_prefix = "iss"

    def initializations_for_opt(self, x0, xN, xC, p, vec_weight):
        # Create scalar/matrix symbols
        self.v = MX.sym('vels', 3)
        self.x = MX.sym('pos', 3)
        self.w = MX.sym('omega', 3)
        self.q = MX.sym('quats', 4)
        self.force = MX.sym('force', 3)
        self.tau = MX.sym('tau', 3)
        self.mass = MX.sym('mass', 1)
        self.I_vec = MX.sym('I_vec', 3)

        # inertia tensor and its inverse
        self.I = blockcat([[self.I_vec[0], 0, 0], [0, self.I_vec[1], 0],
                           [0, 0, self.I_vec[2]]])
        self.inv_I = blockcat([[1 / self.I_vec[0], 0, 0], [0, 1 / self.I_vec[1], 0],
                               [0, 0, 1 / self.I_vec[2]]])

        self.intg_func()
        self.grad_func()
        self.opti_create(self.N, x0, xN, xC, p, vec_weight)  # create optimization problem

    def H_bar_T_w(self):
        # qdot = 0.5*H_bar_T_w*w
        out = vertcat(self.q[3] * self.w[0] - self.q[2] * self.w[1] + self.q[1] * self.w[2],
                      self.q[2] * self.w[0] + self.q[3] * self.w[1] - self.q[0] * self.w[2],
                      -self.q[1] * self.w[0] + self.q[0] * self.w[1] + self.q[3] * self.w[2],
                      -self.q[0] * self.w[0] - self.q[1] * self.w[1] - self.q[2] * self.w[2])
        return out

    def skew(self, vec):
        skew_mat = blockcat([[0, -vec[2], vec[1]],
                             [vec[2], 0, -vec[0]],
                             [-vec[1], vec[0], 0]])
        return skew_mat

    def rot_vec(self, q, vec):
        # Rotate a vector by a quaternion
        q_bar = q[0:3]
        q_scalar = q[3]
        skew_q_bar = skew(q_bar)
        rotated_vec = vec + 2 * skew_q_bar @ (skew_q_bar @ vec + q_scalar * vec)
        return rotated_vec

    def get_dyn(self):
        y = vertcat(self.v,
                    self.force / self.mass,
                    # convert acc to world frame - takes longer to solve to an acceptable level
                    0.5 * self.H_bar_T_w(),
                    -np.matmul(self.inv_I, np.cross(self.w, np.matmul(self.I, self.w))) + np.matmul(self.inv_I, self.tau))
        return y

    def get_accels(self):
        x = self.plan_full[0:13, :]
        u = self.plan_full[13:19, :]
        mass = self.p[0]
        I_vec = self.p[1:4]
        q = x[6:10, :]
        w = x[10:13, :]
        I = np.array([[I_vec[0], 0, 0], [0, I_vec[1], 0],
                      [0, 0, I_vec[2]]])
        inv_I = np.linalg.inv(I)
        self.accels = np.empty((6, self.N))
        i = 1
        for i in range(self.N):  
            self.accels[:, i] = np.hstack([np.array(u[0:3, i]).reshape(3) / mass,
                                           np.array(-np.matmul(inv_I, np.cross(w[:, i], np.matmul(I, w[:, i]))) +
                                           np.matmul(inv_I, u[3:6, i])).reshape(3)])
    def grad_func(self):
        # gradients for FIM calculation
        grad_y = jacobian(self.get_dyn(), vertcat(self.mass, self.I_vec))
        self.get_grad = Function('f', [self.force, self.tau, self.q, self.w,
                                       self.mass, self.I_vec], [grad_y])

    def intg_func(self):
        # Set up RK45 integrator
        ode = {'x': vertcat(self.x, self.v, self.q, self.w), 'p': vertcat(self.force, self.tau, self.mass, self.I_vec),
               'ode': self.get_dyn()}
        intg_options = {}
        # length of one integration step
        intg_options['tf'] = dt
        intg_options['simplify'] = True
        intg = integrator('intg', 'rk', ode, intg_options)
        result = intg(x0=vertcat(self.x, self.v, self.q, self.w),
                      p=vertcat(self.force, self.tau, self.mass, self.I_vec))
        next_states = result['xf']  # x0 for the next iteration is the result of the previous
        # # set up a function that does this mapping
        self.intg_ = Function('integrator_func',
                              [self.x, self.v, self.q, self.w, self.force, self.tau, self.mass, self.I_vec],
                              [next_states])  # states, controls to the next state

    def total_grad(self, states, p, N, vec_weights):
        # Calculate information gain over a series of states
        grad_sum = [0]
        # vec_weight = np.array([0, 0, 0, 0])
        W = np.identity(4) @ diag(vec_weights)  # weights for info gain based on parameter
        # print(W)
        for i in range(0, N):
            force = states[13:16, i]
            tau = states[16:19, i]
            quat = states[6:10, i]
            omega = states[10:13, i]
            grad = self.get_grad(force, tau, quat, omega, p[0], p[1:4])
            grad = grad.T @ grad
            grad_sum = grad_sum + grad
            return trace(W @ grad_sum)  # grad_sum[0] + grad_sum[1] + grad_sum[1] + grad_sum[1]

    def opti_create(self, N, x0, xN, xC, p, vec_weights):
        # Set up the optimization problem
        opti = casadi.Opti()
        x = opti.variable(19, N + 1)
        opti.minimize(-self.total_grad(x, p, N, vec_weights))  # total_grad(x, p, N)
        for k in range(N):
            opti.subject_to(
                x[0:13, k + 1] == self.intg_(x[0:3, k], x[3:6, k], x[6:10, k], x[10:13, k], x[13:16, k],
                                             x[16:19, k], p[0], p[1:4]))  # dynamic constraints
        opti.subject_to(x[0:6, 0] == x0[0:6])  # initial state constraint (pos, vels)
        opti.subject_to(x[6:10, 0] == x0[6:10])  # initial state constraint (quat)
        opti.subject_to(x[10:13, 0] == x0[10:13])  # initial state constraint (omega)
        opti.subject_to(x[0:3, N] == xN[0:3])  # final state constraint (pos)
        opti.subject_to(x[3:6, N] == xN[3:6])  # final state constraint (vels)
        opti.subject_to(x[6:10, N] == xN[6:10])  # final state constraint (quat)
        opti.subject_to(x[10:13, N] == xN[10:13])  # final state constraint (omega)

        opti.subject_to(opti.bounded(-0.01, diff(x[3, :])/dt, 0.01))  # input constraints - force
        opti.subject_to(opti.bounded(-0.01, diff(x[4, :]) / dt, 0.01))  # input constraints - force
        opti.subject_to(opti.bounded(-0.01, diff(x[5, :]) / dt, 0.01))  # input constraints - force
        opti.subject_to(opti.bounded(-0.09, diff(x[10, :])/dt, 0.09))  # input constraints - torques
        opti.subject_to(opti.bounded(-0.09, diff(x[11, :]) / dt, 0.09))  # input constraints - torques
        opti.subject_to(opti.bounded(-0.09, diff(x[12, :]) / dt, 0.09))  # input constraints - torques
        opti.subject_to(opti.bounded(-0.02, diff(x[3:6, :]) / dt, 0.02))  # input constraints - pos x
        # bounds
        opti.subject_to(opti.bounded(-0.25, x[13:16, :], 0.25))  # input constraints - force
        opti.subject_to(opti.bounded(-0.025, x[16:19, :], 0.025))  # input constraints - torques
        # based on experimental volume available in the JEM
        opti.subject_to(opti.bounded(xC[0, 0], x[0, :], xC[0, 1]))  # input constraints - pos x
        opti.subject_to(opti.bounded(xC[1, 0], x[1, :], xC[1, 1]))  # input constraints - pos y
        opti.subject_to(opti.bounded(xC[2, 0], x[2, :], xC[2, 1]))  # input constraints - pos z
        # linear velocity bounds
        opti.subject_to(opti.bounded(-0.15, x[3:6, :], 0.15))  # input constraints - vel lin
        opti.subject_to(opti.bounded(-0.1745, x[10:13, :], 0.1745))  # input constraints - vel angular

        p_opts = {"expand": True}
        s_opts = {"max_iter": 10000}
        opti.solver('ipopt', p_opts, s_opts)
        sol = opti.solve()
        self.traj = sol.value(x)

    def run(self):
        rospy.set_param('/info_traj_computed', 1)

        # save trajectory onto a .csv
        np.savetxt(self.path + "/" + self.file_name, self.traj, delimiter=", ", fmt="% s")


    def get_trajectory(self):
        # If nothing was computed, read from .csv
        
        plan = np.empty((self.N + 1,))
        csv_name = self.path + "/" + self.file_name
        with open(csv_name, 'rt') as csvfile:
            reader = csv.reader(csvfile, delimiter=',', quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                plan = np.vstack((plan, np.array(row)))
        self.plan_full = plan[1:, :]  # the first row is the empty array
        self.control_state_pub()

    def control_state_pub(self):
        # send out the next waypoint at 1/dt frequency
        #x_des_traj_np = np.empty([, 20])
        n = self.plan_full.shape[1]
        x_des_traj = np.empty([n, 20])
        x_des_traj_msg = Float64MultiArray()
        for i in range(n):
            plan_i = self.plan_full[:, i]
            x_des_traj[i, 0] = rospy.Time.now().to_sec() + i*dt # the plans correspond to dt of 0.5
            x_des_traj[i, 1:13] = plan_i[0:12] # positions, velocities, attitude in quaternions, ang vels.
            x_des_traj[i, 14:20] = 0.0 # the planned trajectory does not have controls, so set them to zero.

        x_des_traj_msg = self.np_to_multiarray_f64(x_des_traj)
        rospy.sleep(5.0) # allow it a few seconds to regulate
        print('Sending trajectory ...')
        self.path_ctl_x_des_traj_pub_.publish(x_des_traj_msg)
        self.info_traj_sent = True


    def info_state_pub(self):
        status = ReswarmInfoStatus()
        status.stamp = rospy.get_rostime()
        status.info_traj_sent = self.info_traj_sent
        self.pub_info_status.publish(status)


    def status_callback(self, data):
        self.test_num = data.test_number
        self.info_traj_send = data.info_traj_send

    def shutdown_func(self):
        self.info_traj_sent = True
        self.info_state_pub()
        self.timer.shutdown()

    def _numpy_to_multiarray(multiarray_type, np_array):
        multiarray = multiarray_type()
        multiarray.layout.dim = [MultiArrayDimension('dim%d' % i,
                                                     np_array.shape[i],
                                                     np_array.shape[i] * np_array.dtype.itemsize) for i in
                                 range(np_array.ndim)]
        multiarray.data = np_array.reshape([1, -1])[0].tolist()
        return multiarray

    np_to_multiarray_f64 = partial(_numpy_to_multiarray, Float64MultiArray)


def init_problem_params(test_num, Ground):
    """
        Test descriptions
        Test0: Regular trajectory from start to end point
        Test1: Info gain mass
        Test2: Info gain Ixx
        Test3: Info gain Iyy
        Test4: Info gain Izz
        Test5: All parameters
        Initialize start point, end point and weights for info gain according to the test
        """
    # Init

    if (Ground == "false"):
        # initial states and variables init [states(rx,ry,rz,vx,vy,vz,qx,qy,qz,qw,ox,oy,ow), controls(f,fy,fz,tx,ty,tz)]
        x0 = [10.5, -8, 4.4, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0]
        # vector of constraints [ul, ll] for pos x, y, z
        xC = np.array([[10.3, 11.5], [-9, -3.5], [4.3, 5.6]])
        if test_num == 6:
            # normal, non-info gain motion
            xN = [11.3, -6.6, 5.4, 0.0, 0.0, 0.0, 0.5785889, -0.3583631, 0.5889355, -0.4358505, 0.0, 0.0, 0.0]
            vec_weight = np.array([0, 0, 0, 0])
        if test_num == 7:
            # start point is the same as end point, but does it cause some translational movement?
            xN = x0[0:13]
            vec_weight = np.array([1, 0, 0, 0])
        if test_num == 8:
            # start point is the same as end point, but does it cause some rotation about x ?
            xN = x0[0:13]
            vec_weight = np.array([0, 1, 0, 0])
        if test_num == 9:
            # start point is the same as end point, but does it cause some rotation about y ?
            xN = x0[0:13]
            vec_weight = np.array([0, 0, 1, 0])
        if test_num == 10:
            # start point is the same as end point, but does it cause some rotation about z ?
            xN = x0[0:13]
            vec_weight = np.array([0, 0, 0, 1])
        if test_num == 11:
            # How different is this motion from the nominal, without info gain motion?
            xN = [11.3, -6.6, 5.4, 0.0, 0.0, 0.0, 0.5785889, -0.3583631, 0.5889355, -0.4358505, 0.0, 0.0, 0.0]
            vec_weight = np.array([1, 1, 1, 1])

    else:
        # ground testing
        # initial states and variables init [states(rx,ry,rz,vx,vy,vz,qx,qy,qz,qw,ox,oy,ow), controls(f,fy,fz,tx,ty,tz)]
        x0 = [-0.5, -0.5, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01]
        # vector of constraints [ul, ll] for pos x, y, z, experimental volume
        xC = np.array([[-0.6, 0.6], [-0.6, 0.6], [-0.6, 0.6]])
        if test_num == 6:
            # normal, non-info gain motion
            xN = [0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0, 0, -0.3894183, 0.921061, 0.0, 0.0, 0.0]
            vec_weight = np.array([0, 0, 0, 0])
        if test_num == 7:
            # start point is the same as end point, but does it cause some translational movement?
            xN = x0[0:13]
            vec_weight = np.array([1e5, 0, 0, 0])
        if test_num == 8:
            # no rotation about x and y on the granite table
            pass
        if test_num == 9:
            xN = x0[0:13]
            xN[0:3] = [0.5, 0.5, 0.0]
            vec_weight = np.array([0, 0, 0, 1])
        if test_num == 10:
            # start point is the same as end point, but does it cause some rotation about z ?
            xN = x0[0:13]
            vec_weight = np.array([0, 0, 0, 1])
        if test_num == 11:
            x0 = [-0.5, -0.5, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01]
            # How different is this motion from the nominal, without info gain motion?
            xN = [0.5, 0.5, 0.0, 0.0, 0.0, 0.0, 0, 0, -0.3894183, 0.921061, 0.0, 0.0, 0.0]
            vec_weight = np.array([10, 0, 0, 10])
    return x0, xN, xC, vec_weight


def main():
    # initialize node
    rospy.init_node('info_rich_traj_gen', log_level=rospy.ERROR)
    timer_set = False

    # print to console that the node is running
    rospy.loginfo('Started planner node')
    rospack = rospkg.RosPack()

    N = 120  # number of time steps/horizon. Currently 120 steps of dt sec each

    # read parameter, ground or ISS?
    sim = rospy.get_param('/reswarm/sim')
    Ground = rospy.get_param('/reswarm/ground')

    """
    Planner initializations
    """
    # Initialize planner class object
    plan_info_rich = InfoRichPlanner(N, sim, Ground)

    # publishers and subscribers
    # Subscribe to get the test number and coordinator commands from /reswarm/status
    rospy.Subscriber(plan_info_rich.topic_prefix + "reswarm/status", ReswarmStatusPrimary,
                     plan_info_rich.status_callback)
    # publishers -- will be needed for NMPC
    # plan_info_rich.traj_pub_pose = rospy.Publisher("info_rich_traj/pose", PoseStamped, queue_size=1)
    # plan_info_rich.traj_pub_twist = rospy.Publisher("info_rich_traj/twist", TwistStamped, queue_size=1)
    # plan_info_rich.traj_pub_wrench = rospy.Publisher("info_rich_traj/wrench", WrenchStamped, queue_size=1)
    plan_info_rich.pub_flight_mode = rospy.Publisher("mob/flight_mode", FlightMode, queue_size=1, latch=True)
    plan_info_rich.pub_info_status = rospy.Publisher(plan_info_rich.topic_prefix + "reswarm/info_traj_status", ReswarmInfoStatus, queue_size=1, latch=True)
    plan_info_rich.pub_control_state = rospy.Publisher(plan_info_rich.topic_prefix + "gnc/ctl/setpoint", ControlState, queue_size=1, latch=True)
    plan_info_rich.path_ctl_x_des_traj_pub_ = rospy.Publisher("reswarm/tube_mpc/traj", Float64MultiArray, queue_size=3)  # for casadi_nmpc

    while not rospy.is_shutdown():
        if(plan_info_rich.info_traj_send):
            if (plan_info_rich.info_traj_sent):
                # if trajectory has been already sent,
                pass
            else:
                if (9<=plan_info_rich.test_num<=12):
                    # load computed path
                    plan_info_rich.path = rospack.get_path('data') + "/input/info-trajectories"
                    plan_info_rich.file_name = "excitation_" + plan_info_rich.env_prefix + "_" + str(
                        plan_info_rich.test_num) + ".csv"
                    plan_info_rich.get_trajectory()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import pdb

import numpy as np
from autograd import jacobian
import rospy
from param_est.msg import Params
import param_est_config
import timeit

Quiet = 1
class Estimation():
    """
    NB: Here, states refer to the robot pose and are obtained from /gnc/ekf
    Parameters refer to the inertial parameters that estimated by this filter.
    Therefore, the update step compares the states propagated using the current parameter estimates
    and the observed states, to further refine the inertial estimates.
    The propagation step simply propagates the inertial parameters as they are,
    with consideration of some noise in Q, the process noise matrix.
    """
    def __init__(self, prop, use_vels):

        self.DoF = prop.DoF
        if self.DoF == 3:
            self.n_omega = 1
        else:
            self.n_omega = 3
        self.n_params = prop.dim # How many parameters are being estimated?
        if use_vels:
            self.n_meas = prop.DoF_accel # How many states are being measured?
        else:
            self.n_meas = self.DoF
        self.prop_obj = prop
        # track if and when the estimator was reset - a reset is performed if the estimator yields infeasible parameters
        self.reset_performed = False 
        self.n_est = self.n_params
        self.use_vels = use_vels
        self.initialize_param_est()



    def initialize_param_est(self):
        """
       Matrices needed by the estimator
       """
        # estimate covariance
        self.P = np.identity(self.n_est) * 10 ** -8

        if self.DoF == 3:
            if self.n_est > 2:
                self.P = param_est_config.P_3DOF
                self.Q = param_est_config.Q_3DOF
            else:
                self.P = param_est_config.P_3DOF_simplified
                self.Q = param_est_config.Q_3DOF_simplified
            if self.use_vels:
                self.R = param_est_config.R_3DOF
            else:
                self.R = param_est_config.R_no_vels_3DOF


        else:
            if self.n_est > 4:
                self.P = param_est_config.P_6DOF
                self.Q = param_est_config.Q_6DOF
            else:
                self.P = param_est_config.P_6DOF_simplified
                self.Q = param_est_config.Q_6DOF_simplified
            self.R = param_est_config.R_6DOF

        # process model Jacobian
        self.F = np.identity(self.n_est)
        # measurement model Jacobian
        self.H = np.zeros((self.n_meas, self.n_est))
        self.params_est = np.zeros((self.n_est))
        self.states_prop = np.zeros((self.n_meas))
        # The measurement vector consists of v, omega and v_dot
        self.states_meas = np.zeros((self.n_meas))
        self.params_init(self.prop_obj)
        self.init_time = rospy.get_time()

    def params_init(self, prop):
        if prop.DoF == 3 :
            self.get_jac = jacobian(prop.forward_dyn_3DoF)
            # if world = granite
            self.params_est = param_est_config.params_init_3DOF_simplified
            if self.n_est == 4:
                # if the model is not simplified
                self.params_est = param_est_config.params_init_3DOF
        else: # world = iss
            self.get_jac = jacobian(prop.forward_dyn_6DoF)
            self.params_est = param_est_config.params_init_6DOF_simplified
            if self.n_est == 10:
                # full dynamics
                self.params_est = param_est_config.params_init_6DOF


    def get_z_tilde(self, states_meas):
        vels_meas = states_meas[3:6]
        omega_meas = states_meas[10:13]
        accels_meas = states_meas[13:16]# accelerations in body frame
        q_meas = states_meas[6:10]
        q_meas[0:3] = -q_meas[0:3]  # perform negative rotation to convert to body frame.
        vels_body = self.rot_vec(q_meas, vels_meas)
        if self.n_meas == 3:
            z_tilde = np.hstack([omega_meas[2], accels_meas[0:2]])
        if self.n_meas == 5:
            z_tilde = np.hstack([vels_body[0:2], omega_meas[2], accels_meas[0:2] ])
        if self.n_meas == 6:
            z_tilde = np.hstack([omega_meas, accels_meas])
        if self.n_meas == 9:
            z_tilde = np.hstack([vels_body, omega_meas, accels_meas])
        return z_tilde

    def state_propagation(self, dt, control_ip):
        if self.prop_obj.DoF == 3:
            control_ip = np.hstack([control_ip[0:2], control_ip[5]])

        if self.n_meas == self.DoF: # No linear velocity measurements,
            dummy_states_vector = np.hstack([np.zeros(self.DoF- self.n_omega), self.states_prop[0:self.n_omega]]) # this is because the propagation function
            # expects vx, vy and omega_z to be the states. If we are not measuring velocities, our state prop is omega z, ax, ay
            ans_prop = self.prop_obj.euler_forward_state_propagation(self.params_est[0:self.n_params], dummy_states_vector, control_ip, dt)
            ans_jac = self.get_jac(self.params_est[0:self.n_params], dummy_states_vector, control_ip)

            H_temp = ans_jac[self.DoF - self.n_omega:self.DoF, :] * dt
            self.H[0:self.n_omega, :] = self.H[0:self.n_omega, :] + H_temp  # propagation for the omegas
            self.H[self.n_omega:, :] = ans_jac[0:self.n_meas - self.n_omega, :]     # jacobians corresponding to accelerations
            self.states_prop = ans_prop[self.DoF - self.n_omega:]
        else:
            ans_prop = self.prop_obj.euler_forward_state_propagation(self.params_est, self.states_prop[0:self.DoF],
                                                                     control_ip, dt)
            ans_jac = self.get_jac(self.params_est, self.states_prop[0:self.DoF], control_ip)
            H_temp = ans_jac[0:self.DoF, :] * dt  # for the linear vels and omegas
            self.H[0:self.DoF, :] = self.H[0:self.DoF, :] + H_temp
            self.H[self.DoF:, :] = ans_jac[0:self.n_meas - self.DoF, :]
            self.states_prop = ans_prop

    def prediction_step(self):
        self.params_est = self.params_est
        self.P = np.matmul(self.F, np.dot(self.P, self.F.transpose())) + self.Q

    def update_step(self, dt, state_meas):
        z_tilde = self.get_z_tilde(state_meas)
        self.prediction_step()
        y = z_tilde - self.states_prop

        S = np.dot(self.H, np.dot(self.P, self.H.transpose())) + self.R #H_kPk_kminus1H_k^T + R
        K = np.dot(self.P, np.dot(self.H.transpose(), np.linalg.inv(S)))  #K_k = P_k_kminus1 H^T_kS_k^-1
        self.params_est = self.params_est + np.matmul(K, y) #x_k = x_k_kminus1 + K_ky_k
        self.P = np.dot((np.identity(self.n_est) - np.dot(K, self.H)), self.P)#P_k = (I - K_kH_k)P_k
        # use states to be propagated (pre measurement states at time k-1) as those obtained by the EKF.
        # The measurement model compares the states obtained by propagating
        self.states_prop = z_tilde # initialize self.states_prop every time.
        # some debugging
        if Quiet==0:
            print(self.params_est)
            print(self.P)
        self.H = np.zeros((self.n_meas, self.n_est))#reset the measurement jacobian after every update step
        self.publish_inertia_params()
        self.params_feasibility_check()


    def skew(self, vec):
        """
        convert from vector to skew matrix
        """
        skew_mat =  np.array([[0, -vec[2],  vec[1]],
                            [vec[2],   0, -vec[0]],
                            [-vec[1],  vec[0],  0]])
        return skew_mat

    def params_feasibility_check(self):
        """
        This function tests the feasibility of the inertial parameters - positive mass, positive definite inertia
        If the estimates are not feasible, the estimator is reset
        """

        if self.DoF == 3:
            mass_est = self.params_est[0]
            izz_est = self.params_est[-1]
            if (mass_est > 0 and izz_est > 0):
                pass
            else:
                self.reset_parameter_estimator()
        else:
            mass_est = self.params_est[0]
            ixx_est = self.params_est[1]
            iyy_est = self.params_est[2]
            izz_est = self.params_est[3]
            if (mass_est > 0 and izz_est > 0 and ixx_est > 0 and iyy_est > 0):
                a = ixx_est + iyy_est > izz_est
                b = ixx_est + izz_est > iyy_est
                c = iyy_est + izz_est > ixx_est
                if a and b and c:
                    pass
                else:
                    self.reset_parameter_estimator()
            else:
                self.reset_parameter_estimator()

    def reset_parameter_estimator(self):
        time_now = rospy.get_time()
        if time_now - self.init_time >= 15:
            print("Reseting param_est")
            self.initialize_param_est()
            self.reset_performed = True



    def rot_vec(self, q, vec):
        """
        Rotate a vector by a quaternion

        :q: desired rotation expressed as a quaternion
        :vec: vector to be rotated
        :returns: rotated_vec
        """
        q_bar = q[0:3]
        q_scalar = q[3]
        skew_q_bar = self.skew(q_bar)
        rotated_vec = vec + 2*np.matmul(skew_q_bar,(np.matmul(skew_q_bar,vec) + q_scalar*vec))
        return rotated_vec

    def publish_inertia_params(self):
        """
        Populate the rosmsg to publish the most recent parameter estimates
        """

        msg = Params()
        msg.header.stamp = rospy.Time.now()
        # order of inertial parameters in the message: m, cx, cy, cz, ixx, iyy, izz, ixy, ixz, iyz
        msg.inertia.m = self.params_est[0]
        msg.Cov[0] = self.P[0,0]
        if self.DoF == 3:
             if self.n_params>2:
                 msg.inertia.com.x = self.params_est[1]
                 msg.Cov[1] = self.P[1, 1]
                 msg.inertia.com.y = self.params_est[2]
                 msg.Cov[2] = self.P[2, 2]
                 msg.inertia.izz = self.params_est[3]
                 msg.Cov[6] = self.P[3, 3]


             else:
                 msg.inertia.izz = self.params_est[1]
                 msg.Cov[6] = self.P[1, 1]

        else:
            if self.n_params > 4:
                for i in range(1,10):
                    msg.Cov[i] = self.P[i, i]
                msg.inertia.com.x = self.params_est[1]
                msg.inertia.com.y = self.params_est[2]
                msg.inertia.com.z = self.params_est[3]
                msg.inertia.ixx = self.params_est[4]
                msg.inertia.iyy = self.params_est[5]
                msg.inertia.izz = self.params_est[6]
                msg.inertia.ixy = self.params_est[7]
                msg.inertia.ixz = self.params_est[8]
                msg.inertia.iyz = self.params_est[9]
            else:
                msg.inertia.ixx = self.params_est[1]
                msg.Cov[4] = self.P[1, 1]
                msg.inertia.iyy = self.params_est[2]
                msg.Cov[5] = self.P[2, 2]
                msg.inertia.izz = self.params_est[3]
                msg.Cov[6] = self.P[3, 3]
        msg.reset_performed = self.reset_performed
        if self.reset_performed == True:
            self.reset_performed = False
        self.inertial_params_pub.publish(msg)

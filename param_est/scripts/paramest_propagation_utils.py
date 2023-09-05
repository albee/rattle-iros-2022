#!/usr/bin/env python3
from __future__ import division
import autograd.numpy as np
from autograd import jacobian
import rospy



class Propagation():
    """
    The member functions of this class perform forward state propagation using rigid body dynamics. The following simplifications of the dynamics are available:
    1. 3DoF dynamics considerint two inertial parameters (mass, and inertia about the Z axis, izz. The center of mass offsets cx and cy are considered to be zero)
    2. 3DoF dynamics considering all four parameters, i.e, mass, cx, cy, izz
    3. 6DoF dynamics neglecting center of mass offsets, i.e, mass and the principal moments of inertia - mass, ixx, iyy and izz
    4. 6DoF dynamic considering all ten inertial parameters, i.e, mass, cx, cy, cz, ixx, iyy, izz, ixy, izy, ixz


    Either of RK4 or Euler discretization can be used.
    """
    def __init__(self, DoF, simplified):

        self.DoF = DoF
        self.accel = 1 # Set to one if the states also include accelerations
        self.simplified = simplified # set to one if CoM offsets are to be ignored


        if self.DoF == 3:
            self.func_name = self.forward_dyn_3DoF
            if self.simplified:
                self.dim = 2 # mass, izz
                self.params_string = "mass, izz"
            else:
                self.dim = 4 # mass, cx, cy, izz
                self.params_string = "mass, cx, cy, izz"
            if self.accel:
                self.DoF_accel = self.DoF + 2 # lin accel x and lin accel y
            else:
                self.DoF_accel = self.DoF
        else:
            self.func_name = self.forward_dyn_6DoF
            if self.simplified:
                self.dim = 4 # mass, ixx, iyy, izz
                self.params_string = "mass, ixx, iyy, izz"
            else:
                self.dim = 10 # mass, cx, cy, cz, ixx, iyy, ixx, ixy, ixz, iyz
                self.params_string = "mass, cx, cy, cz, ixx, iyy, ixx, ixy, ixz, iyz"
            if self.accel:
                self.DoF_accel = self.DoF + 3 # lin accel x y and z
            else:
                self.DoF_accel = self.DoF
        if self.accel:
            meas_string = 'velocities and linear acceleration'
        else:
            meas_string = "velocity"
        rospy.loginfo('[param est]' + str(DoF) + 'DoF dynamics selected, with ' + str(meas_string) + ' measurements')
        rospy.loginfo('[param est] Parameters to be estimated: ' + str(self.params_string))


    # integrators
    def RK4_forward_state_propagation(self, params, x, u, dt):
        
        """
        An RK4 integrator implementation

        :x: states at time k
        :dt: integration time step
        :u: control inputs (forces and torques)
        :params: inertial parameters
        :return: integrated result
        """
        f1 = dt*self.func_name(params, x, u)
        f2 = dt*self.func_name(params, x + .5*f1, u)
        f3 = dt*self.func_name(params, x + .5*f2, u)
        f4 = dt*self.func_name(params, x + f3, u)
        self. x_kplus1 = (x + (1 / 6*(f1 + 2*f2 + 2*f3 + f4)))
        if self.accel:
            self.x_kplus1 = np.hstack([self.x_kplus1, self.func_name(params, x, u)[0:self.DoF_accel-self.DoF]])
        return self.x_kplus1

    def euler_forward_state_propagation(self, params, x, u, dt):

        """
        A Euler integrator implementation

        :x: states at time k
        :dt: integration time step
        :u: control inputs (forces and torques)
        :params: inertial parameters
        :return: integrated result
        """
        f1 = self.func_name(params, x, u)
        self.x_kplus1 = x + dt*f1 # velocities
        if self.accel:
            self.x_kplus1 = np.hstack([self.x_kplus1, f1[0:self.DoF_accel-self.DoF]])
        return self.x_kplus1


    # dynamics implementation
    def forward_dyn_3DoF(self, params, x, u):  
    # Planar Astrobee dynamics
        # inertial params
        if self.simplified:
            mass = params[0]
            cx = 0.0
            cy = 0.0
            izz = params[-1]
        else:
            mass = params[0]
            cx = params[1]
            cy = params[2]
            izz = params[-1]

        # states
        omega_z = x[2]

        # spatial inertia matrix
        M = np.array([[mass, 0, -mass*cy],
             [0, mass, mass*cx],
             [-mass*cy, mass*cx, izz + mass*(cx*cx + cy*cy)]])
        inv_M = np.linalg.inv(M)

        C = np.array([-omega_z*omega_z*cx*mass, -omega_z*omega_z*cy*mass, 0])
        ans = np.matmul(inv_M, u - C)
        return ans


    def forward_dyn_6DoF(self, params, x, u):
    # 6DoF dynamics
        # inertial params
        if self.simplified:
            mass = params[0]
            cx = 0.0
            cy = 0.0
            cz = 0.0
            ixx = params[1]
            iyy = params[2]
            izz = params[-1]
            ixz = 0.0
            iyz = 0.0
            ixy = 0.0
        else:
            mass = params[0]
            cx = params[1]
            cy = params[2]
            cz = params[3]
            ixx = params[4]
            iyy = params[5]
            izz = params[6]
            ixy = params[7]
            ixz = params[8]
            iyz = params[9]

        omega = np.array(x[3:6])


        I_cm = np.array([[ixx, ixy, ixz],[ixy, iyy, iyz],[ixz, iyz, izz]])
        c = np.array([cx, cy, cz])

        # spatial inertia matrix
        M = np.vstack([np.hstack([np.identity(3)*mass, -mass*self.skew(c)]),
             np.hstack([mass*self.skew(c), I_cm - mass*np.matmul(self.skew(c), self.skew(c))])])
        I_intermediate = I_cm - np.dot(mass, np.matmul(self.skew(c), self.skew(c)))
        inv_M = np.linalg.inv(M)

        C = np.hstack([np.dot(mass,np.matmul(np.matmul(self.skew(omega), self.skew(omega)), c)),
                      np.matmul(self.skew(omega), np.matmul(I_intermediate, omega))])

        ans = np.matmul(inv_M, u - C)
        return ans


    def skew(self, vec):
        skew_mat = np.array([[0, -vec[2], vec[1]],
                             [vec[2], 0, -vec[0]],
                             [-vec[1], vec[0], 0]])
        return skew_mat



# TODO: delete after final checks
# some pre-requisite, sanity check values.
# # # some hypothetical values
dt = 0.1
params = np.array([18.9715, 0.0, 0.0, 0.2517])
#params = np.array([18.9715, 0.2517])
controls = np.array([1.0, 1.0, 0.1])
states = np.array([1.0, 1.0, 1.0])
# #
# params = np.array([18.9715, 0.2517, 0.2517, 0.2517])
# controls = np.array([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])
# states = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
#
# params = np.array([18.9715, 0.0, 0.0, 0.0, 0.2517, 0.2517, 0.2517, 0.0, 0.0, 0.0])
# testing hypothetical values
# DoF = 3
# simplified = 0 # check Jacobian for the non simplified case
# lin_accel = 1
# prop = Propagation(DoF, simplified)
# get_jac = jacobian(prop.forward_dyn_3DoF)
# print(prop)
# print(get_jac)
# # start = timeit.default_timer()
# ans = get_jac(params, states, controls)
# print(ans)
# stop = timeit.default_timer()
# print('jacobian Time2: ', stop - start)

#!/usr/bin/env python3
import numpy as np

"""
Estimate covariances and initializations:
3DOF_simplified: Estimate mass and izz
3DOF: Estimate mass, cx, cy, izz
6DOF_simplified: Estimate mass, ixx, iyy, izz
6DOF: Estimate mass, cx, cy, cz, ixx, ixy, ixz, iyy, iyz, izz
"""
# for simulation
vec_cov_3DOF_simplified = np.array([20, 1])
P_3DOF_simplified = np.diag(vec_cov_3DOF_simplified)
params_init_3DOF_simplified = np.array([10, 0.1])


vec_cov_3DOF = np.array([20, 10**-7, 10**-7, 0.005])
P_3DOF = np.diag(vec_cov_3DOF)
params_init_3DOF = np.array([10, 0.001, 0.001, 0.1])


vec_cov_6DOF_simplified = np.array([15, 0.005, 0.005, 0.005])
P_6DOF_simplified = np.diag(vec_cov_6DOF_simplified)
params_init_6DOF_simplified = init_3DOF = np.array([15, 0.1, 0.1, 0.1])


vec_cov_6DOF = np.array([10, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005])
P_6DOF = np.diag(vec_cov_6DOF)
params_init_6DOF = np.array([10, 0.001, 0.001, 0.001, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0])


# for hardware
vec_cov_3DOF_simplified_hw = np.array([10, 1])
P_3DOF_simplified_hw = np.diag(vec_cov_3DOF_simplified_hw)

vec_cov_6DOF_simplified_hw = np.array([20, 5, 5, 5])
P_6DOF_simplified_hw = np.diag(vec_cov_6DOF_simplified_hw)


"""
Measurement noise covariances
R_3DOF: measure vx, vy, ox, accx, accy
R_no_vels_3DOF: measure ox, accx, accy
R_6DOF: measure vx, vy, vz, ox, oy, oz, accx, accy, accz
R_no_vels_6DOF:  measure ox, oy, oz, accx, accy, accz
"""

vec_R_3DOF = np.array([0.5 * 10 ** -5, 0.5 * 10 ** -5, 0.5 * 10 ** -7, 0.5 * 10 ** -5, 0.5 * 10 ** -5])
R_3DOF = np.diag(vec_R_3DOF)

vec_R_no_vels_3DOF= np.array([0.25*10 ** -7, 0.25*10 ** -5, 0.25*10 ** -5])
R_no_vels_3DOF= np.diag(vec_R_no_vels_3DOF)

vec_R_6DOF = np.array([0.5*10**-5, 0.5*10**-5, 0.5*10**-5, 0.25*10**-3, 0.25*10**-3, 0.25*10**-3, 0.5*10**-5, 0.5*10**-5, 0.5*10**-5])
R_6DOF = np.diag(vec_R_6DOF)

vec_R_no_vels_6DOF = np.array([0.25*10**-3, 0.25*10**-3, 0.25*10**-3, 0.5*10**-5, 0.5*10**-5, 0.5*10**-5])
R_no_vels_6DOF = np.diag(vec_R_no_vels_6DOF)

# for hardware:
vec_R_3DOF_hw = np.array([0.25*10**-3, 0.25*10**-5, 0.25*10**-3, 0.5*10**-3, 0.5*10**-5])
R_3DOF_hw = np.diag(vec_R_3DOF_hw)

vec_R_6DOF_hw = np.array([0.25*10**-3, 0.25*10**-3, 0.25*10**-3, 0.75*10**-2, 0.75*10**-3, 0.75*10**-3, 0.25*10**-3, 0.25*10**-3, 0.25*10**-3])
R_6DOF_hw = np.diag(vec_R_6DOF_hw)


"""
Process model covariance
"""
Q_3DOF_simplified = np.identity(2)*10**-8
Q_3DOF = np.identity(4)*10**-8

Q_6DOF_simplified = np.identity(4)*10**-8
Q_6DOF = np.identity(10)*10**-8

vec_Q_3DOF_simplified_hw = np.array([10**-4, 0.5*10**-2])
Q_3DOF_simplified_hw = np.diag(vec_Q_3DOF_simplified_hw)
vec_Q_6DOF_simplified_hw = np.array([2*10**-5, 5*10**-7, 5*10**-7, 5*10**-7])
Q_6DOF_simplified_hw = np.diag(vec_Q_6DOF_simplified_hw)


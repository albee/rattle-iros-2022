"""
_rattle_config.py

RATTLE configuration paramters (for non-ROS testing).
TODO: convert to YAML in longterm.

Keenan Albee and Monica Ekal, 2021
MIT Space Systems Lab
"""

import numpy as np

# -------------------------------------------
# Local plan
# -------------------------------------------
"""
ACHTUNG
Verify that these values match acado, casadi, and rrt!
"""
DT_G_ = 2.0  # dt between global waypoints fed to the local planner, [s]---the time between local planner replans

local_scale_ = 1.0  # [0, inf], scaling applied to DT_G_ to slow down the dynamics
DT_L_ = 0.3  # dt within the local planner used by Acado, [s]
N_L_ = 40  # local planner horizon length
t_L_ = DT_L_ * N_L_ * local_scale_  # time between local plan recomputations

DT_C_ = 1.0  # dt within the controller model, [s] (NB: controller assumes 0.2 [s] for ref traj!)
N_C_ = 5  # controller horizon length
"""
ACHTUNG
Verify that these values match acado, casadi, and rrt!
"""

# -------------------------------------------
# Weighting settings
# -------------------------------------------
throwaway_mode_ = 1  # {0, 1}: 1 for throwaway mode, which uses gnc/ekf for the initial pose of the local plan
RRT_FILENAME = "latest_rrt_plan.csv"

WeightMode = {0: 'no_weight', 1: 'step_weight', 2: 'exp_weight', 3: 'izz', 4: 'mass', 5: 'cov_weight'}
InitialModelMode = {0: 'ground_truth_iss', 1: 'ground_truth_ground', 2: 'incorrect_low', 3: 'incorrect_high'}

sigma_floor = np.array([0.03, 0.0001, 0.0001, 0.0001])  # [m, Ixx, Iyy, Izz]
sigma = np.array([10000.0, 10000.0, 10000.0, 10000.0])  # init to very large values, should be updated by callbacks

# -------------------------------------------
# Obstacle placements
# -------------------------------------------
# Ground
# default
obs_ground = []
obs = np.array([[0.15, 0.15, 5.0, 0.3, 0.3, -0.0],  # [a b c x y z]
                [0.2, 0.2, 5.0, -0.25, -0.1, -0.0],
                [0.2, 0.05, 5.0, 0.5, 0.5, -0.0]])
obs_ground.append(obs)

# astronaut room w/ payload
obs = np.array([[0.7, 0.1, 5.0, -0.3, 0.0, -0.0],  # room  # [a b c x y z]
                [0.2, 0.2, 5.0, -0.75, -0.3, -0.0],
                [0.2, 0.2, 5.0, 0.75, -0.75, -0.0],
                [0.2, 0.2, 5.0, -0.05, -0.8, -0.0]])
obs_ground.append(obs)

# standard "hard" RRT test, granite
obs = np.array([[0.3, 0.3, 5.0, -0.6, 0.2, -0.0],
                [0.05, 0.8, 5.0, 0.5, -0.35, -0.0]])
obs_ground.append(obs)

# none!
obs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, -0.0]])
obs_ground.append(obs)

# astronaut room w/ payload + new obstacle [4]
obs = np.array([[0.7, 0.1, 5.0, -0.3, 0.0, -0.0],  # room  # [a b c x y z]
                [0.2, 0.2, 5.0, -0.75, -0.3, -0.0],
                [0.2, 0.2, 5.0, 0.75, -0.75, -0.0],
                [0.2, 0.2, 5.0, -0.05, -0.8, -0.0],
                [0.1, 0.1, 5.0, 0.4, -0.45, -0.0]])
obs_ground.append(obs)


# ISS
JEM_R_W = np.array([10.9, -6.65, 4.9])  # reference wrt world

obs_iss = []
# standard RRT test, ISS
obs = np.array([[0.2, 0.2, 0.65, 10.8, -9.0, 4.9],  # [a b c x y z]
                [0.2, 0.2, 0.65, 10.4, -8.5, 4.3],
                [0.2, 0.2, 0.2, 11.1, -8.0, 5.0]])
obs_iss.append(obs)

# standard "hard" RRT test
obs = np.array([[0.5, 0.05, 2.0, 10.3, -8.5, 4.9],
                [0.5, 0.05, 2.0, 11.2, -9.0, 4.9]])
obs_iss.append(obs)

# fancy (MIT)
obs = np.array([[0.15, 0.15, 0.75, 10.2, -9.1, 4.8],  # M [a b c x y z]
                [0.2, 0.2, 0.3, 10.5, -9.1, 4.7],
                [0.15, 0.15, 0.75, 10.8, -9.1, 4.8],
                [0.2, 0.2, 0.3, 11.1, -9.1, 4.7],
                [0.15, 0.15, 0.75, 11.4, -9.1, 4.8],
                [0.75, 0.15, 0.15, 10.8, -8.6, 4.1],  # I
                [0.15, 0.15, 0.75, 10.8, -8.6, 4.7],
                [0.75, 0.15, 0.15, 10.8, -8.6, 5.3],
                [0.75, 0.15, 0.15, 10.8, -8.1, 4.7],  # T
                [0.15, 0.15, 0.75, 10.8, -8.1, 5.3]])
obs_iss.append(obs)

# none!
obs = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
obs_iss.append(obs)

# standard "hard" RRT test + new obstacle [4]
obs = np.array([[0.5, 0.05, 2.0, 10.3, -8.5, 4.9],
                [0.5, 0.05, 2.0, 11.2, -9.0, 4.9],
                [0.2, 0.2, 0.6, 11.4, -8.3, 4.7]])
obs_iss.append(obs)


# for obs in obs_iss:
#     obs[:, 3:6] += JEM_R_W  # offset positions

obstacles_opts = {'ground': obs_ground, 'iss': obs_iss}

# # long way 'round
# obstacles = [[0.05, 0.5, 0.1, -0.3, -0.5, 0.0],  # [a b c x y z]
#              [0.4, 0.05, 0.3, -0.1, 0.0, 0.0],
#              [0.05, 0.2, 0.2, 0.2, 0.2, 0.0]]

# # astronaut room
# obstacles = [[0.1, 0.5, 0.3, -0.4, -0.5, 0.0],  # [a b c x y z]
#             [0.45, 0.1, 0.3, 0.05, 0.0, 0.0],  # room
#             [0.2, 0.2, 0.3, 0.35, -0.3, 0.0],
#             [0.2, 0.2, 0.3, 0.85, -0.85, 0.0],
#             [0.2, 0.2, 0.3, -0.05, -0.8, 0.0]]

# big one
# obstacles = [[0.6, 0.6, 0.1, -0.0, -0.0, 0.0]]  # [a b c x y z]

# a bunch
# i = 4
# j = 4
# np_obs = np.array(np.zeros((i*j, 6)))
# for m in range(0,i):
#     for n in range(0,j):
#         np_obs[m*j+n, 0] = 0.1
#         np_obs[m*j+n, 1] = 0.1
#         np_obs[m*j+n, 2] = 0.1
#         np_obs[m*j+n, 3] = (-0.8 + (0.8*2/(i-1))*m) + (np.random.rand() - 0.5)*2*0.1
#         np_obs[m*j+n, 4] = (-0.8 + (0.8*2/(j-1))*n) + (np.random.rand() - 0.5)*2*0.1
#         np_obs[m*j+n, 5] = 0.0
# obstacles = np_obs.tolist()

# hallway
# obstacles = [[0.1, 0.8, 0.1, -0.2, 0.3, 0.0],  # [a b c x y z]
#           [0.1, 0.8, 0.1, 0.3, -0.35, 0.0]]

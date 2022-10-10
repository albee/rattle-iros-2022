'''
rattle_coordinator.py

RattleCoordinator class definition and ROS initialization.

TODO: ROS implementation should be moved outside of the coordinator core functions.

Keenan Albee and Monica Ekal, 2021
MIT Space Systems Lab
'''
from __future__ import division
import rospy
import rospkg

# math
from numpy import deg2rad
from tf.transformations import quaternion_from_euler
from math import ceil, floor

# misc
from time import sleep
import csv
import numpy as np

# class components (Mixins)
from . import _rattle_coordinator_core
from . import _rattle_coordinator_utils
from . import _rattle_coordinator_ros

# configuration options
from . import _rattle_config


# N.B.: we are using multiple inheritance to divide up the monolithic coordinator class.
class RattleCoordinator(_rattle_coordinator_utils.Mixin, _rattle_coordinator_ros.Mixin, _rattle_coordinator_core.Mixin):
    # Class configuration options
    DT_G_ = _rattle_config.DT_G_
    t_L_ = _rattle_config.t_L_
    DT_L_ = _rattle_config.DT_L_
    N_L_ = _rattle_config.N_L_
    DT_C_ = _rattle_config.DT_C_
    N_C_ = _rattle_config.N_C_

    VERBOSE = 1
    USE_EKF_POSE = 1
    USE_CSV = 0
    USE_PARAMS = 0  # use parameter updates? [0, 1]
    WEIGHT_MODE = 0  # weighting mode, see dict
    INITIAL_MODEL_MODE = 0  # initial model type, see dict
    RRT_FILENAME = _rattle_config.RRT_FILENAME

    WeightMode = _rattle_config.WeightMode
    InitialModelMode = _rattle_config.InitialModelMode

    sigma_floor_ = _rattle_config.sigma_floor
    sigma_ = _rattle_config.sigma

    obstacles_opts = _rattle_config.obstacles_opts
    obstacles_ = obstacles_opts["iss"][0]  # default until set

    # Class variables
    global_plan_ = {"posearray": 0,
                    "twistarray": 0}  # the global plan
    local_plan_ = {"posearray": 0,
                   "twistarray": 0,
                   "wrencharray": 0}   # the local plan

    ground_ = False  # {False, True}
    sim_ = False  # {False, True}  TODO: get this from rattle/status

    GLOBAL_STATUS_ = 0  # {0=not ready, increments by 1 per callback. 2=ready}
    LOCAL_STATUS_ = 0  # {0=not ready, 1=ready}
    CONTROLLER_STATUS_ = 0  # {0=off, 1=active}
    EST_STATUS_ = 0  # {0=off, 1=active}, active est when needed (perhaps for certain informative segments)
    GOT_EKF = 0  # {0=not ready, 1=ready}

    # RATTLE tracking points [x y z qx qy qz qw], set via primary_coord_params.yaml
    POINT_A_GRANITE = []
    POINT_A_ISS = []
    POINT_B_GRANITE = []
    POINT_B_ISS = []
    POINT_C_GRANITE = []
    POINT_C_ISS = []

    local_init_ = False  # use gnc/ekf initialization?

    x_est_ = np.zeros((13, 1))  # should be updated from estimator, initialize to zeros
    x_est_[0:2, 0] = [-0.5, 0.7]

    def __init__(self):
        """
        Unused.
        """
        pass

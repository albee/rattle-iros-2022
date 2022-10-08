#!/usr/bin/env python
"""
# asap_primary.py

Python interface to set options for primary Astrobee.

Keenan Albee and Charles Oestreich, 2021
MIT Space Systems Laboratory
"""

import time
import rospy
import rospkg
import math
import argparse
from std_msgs.msg import String

rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data") + "/"


def primary_execute_test(bee_topic_prefix, test_number=-1, ground='false', sim='false'):
    """
    Run a primary test.
    """
    # Set baseline parameters
    rospy.set_param('/rattle/ground', ground)  # options are: ['false', 'true']
    rospy.set_param('/rattle/sim', sim)  # options are: ['false', 'true']

    if (ground == "true"):
        r_RI = [0.0, 0.0, 0.0]  # the test volume reference frame (TVR) wrt INERTIAL frame
        r_CR = [0.0, 0.6, -0.7]  # primary position wrt TVR frame
        q_CR = [0.0, 0.0, -0.7071068, 0.7071068]  # quaternion [qx qy qz qw wrt TVR frame]
        r_TR = [0.0, -0.5, -0.7]  # Target position [x y z] wrt TVR frame   TODO: can drop for rattle
        set_params_IC(r_RI, r_CR, q_CR, r_TR)
    else:
        # r_RI_ISS = [10.9, -6.65, 4.9]  # the test volume reference frame (TVR) wrt INERTIAL frame
        r_RI_ISS = [10.8, -9.75, 4.8]  # rattle Point A (TVR)
        r_CR = [0.0, 0.0, 0.0]  # primary position wrt TVR frame
        q_CR = [0, 0, -0.7071068, 0.7071068]  # quaternion [qx qy qz qw wrt TVR frame]
        r_TR = [0.0, 0.0, 0.0]  # Target position [x y z] wrt TVR frame  TODO: can drop for rattle
        set_params_IC(r_RI_ISS, r_CR, q_CR, r_TR)


def set_params_IC(r_RI, r_CR, q_CR, r_TR):
    # these params are ONLY used for Roberto's old format TODO: drop for rattle?
    rospy.set_param('/rattle/x_TI', r_RI[0] + r_TR[0])  # x Target wrt Inertial, INERTIAL
    rospy.set_param('/rattle/y_TI', r_RI[1] + r_TR[1])  # y
    rospy.set_param('/rattle/z_TI', r_RI[2] + r_TR[2])  # z

    rospy.set_param('/rattle/x_CT', r_CR[0] - r_TR[0])  # primary wrt Target fixed frame
    rospy.set_param('/rattle/y_CT', r_CR[1] - r_TR[1])
    rospy.set_param('/rattle/z_CT', r_CR[2] - r_TR[2])

    # these params are what matter for actual testing, wrt INERTIAL frame
    rospy.set_param('/rattle/primary/x_start', r_RI[0] + r_CR[0])  # x
    rospy.set_param('/rattle/primary/y_start', r_RI[1] + r_CR[1])  # y
    rospy.set_param('/rattle/primary/z_start', r_RI[2] + r_CR[2])  # z
    rospy.set_param('/rattle/primary/qx_start', q_CR[0])  # x
    rospy.set_param('/rattle/primary/qy_start', q_CR[1])  # y
    rospy.set_param('/rattle/primary/qz_start', q_CR[2])  # z
    rospy.set_param('/rattle/primary/qw_start', q_CR[3])  # w

    rospy.set_param('/rattle/primary/targ_offset_x', r_RI[0] + r_TR[0])  # x
    rospy.set_param('/rattle/primary/targ_offset_y', r_RI[1] + r_TR[1])  # y
    rospy.set_param('/rattle/primary/targ_offset_z', r_RI[2] + r_TR[2])  # z

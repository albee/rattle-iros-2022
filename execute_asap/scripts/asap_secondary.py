#!/usr/bin/env python
"""
# asap_secondary.py

Python interface to set options for secondary Astrobee.

Keenan Albee and Charles Oestreich, 2021
MIT Space Systems Laboratory
"""

import time
import rospy
import rospkg
import math

rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data") + "/"


def secondary_execute_test(bee_topic_prefix, test_number=-1, ground='false', sim='false'):
    """
    Run a secondary test.
    """
    rospy.set_param('/reswarm/ground', ground)  # options are: ['false', 'true']
    rospy.set_param('/reswarm/sim', sim)  # options are: ['false', 'true']

    # Set initial position
    if (ground == "true"):
        r_RI = [0.0, 0.0, 0.0]  # the test volume reference frame (TVR) wrt INERTIAL frame
        r_TR = [0.0, -0.5, -0.7]  # secondary position [x y z] wrt TVR frame
        q_TR = [0.0, 0.0, 0.0, 1.0]  # quaternion [qx qy qz qw wrt TVR frame]

        set_params_IC(r_RI, r_TR, q_TR)
    else:
        r_RI_ISS = [10.9, -6.65, 4.9]  # the test volume reference frame (TVR) wrt INERTIAL frame

        r_TR = [0.0, 0.0, 0.0]  # secondary position [x y z] wrt TVR frame
        q_TR = [0.0, 0.0, 0.7071, -0.7071]  # quaternion [qx qy qz qw wrt TVR frame]
        set_params_IC(r_RI_ISS, r_TR, q_TR)


def set_params_IC(r_RI, r_TR, q_TR):
    rospy.set_param('/reswarm/secondary/x_start', r_RI[0] + r_TR[0])
    rospy.set_param('/reswarm/secondary/y_start', r_RI[1] + r_TR[1])
    rospy.set_param('/reswarm/secondary/z_start', r_RI[2] + r_TR[2])
    rospy.set_param('/reswarm/secondary/qx_start', q_TR[0])  # x
    rospy.set_param('/reswarm/secondary/qy_start', q_TR[1])  # y
    rospy.set_param('/reswarm/secondary/qz_start', q_TR[2])  # z
    rospy.set_param('/reswarm/secondary/qw_start', q_TR[3])  # w

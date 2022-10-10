#!/usr/bin/env python3
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
    # deprecated for RATTLE public release.

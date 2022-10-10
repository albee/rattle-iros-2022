#!/usr/bin/env python3

from threading import Thread
import sys
import argparse
import rospy
from std_msgs.msg import String

import subprocess
import shlex
from signal import signal, SIGINT, pause
from sys import exit
import time

# ROS
import rospy
from rattle_msgs.msg import RattleTestNumber

if (__name__ == "__main__"):
    """ Publish on the topics GDS will use to activate nodes.

    -g, --ground: Indicates ground flag.
    -s, --sim: Indicates sim flag.
    -t, --test: Also publishes the test number (simulation only). TODO: do topic prefixing
    [test_num]: The test number to use.
    """
    parser = argparse.ArgumentParser(description='Run a desired test session.',
                                     usage='%(prog)s [options]')
    parser.add_argument('-g', '--ground', action='store_true', help='-g to indicate ground. Defaults to ISS.')
    parser.add_argument('-s', '--sim', action='store_true', help='-s to indicate simulation. Defaults to hardware.')
    parser.add_argument('-t', '--test', action='store_true', help='-t to indicate ROS pub test_num. Defaults to off.')
    parser.add_argument('test_num', metavar='TEST', type=String, nargs=1, help='A test number to run.')
    args = parser.parse_args()

    if args.sim is True:
        SIM = "sim"
    else:
        SIM = "hardware"
    if args.ground is True:
        GROUND = "true"
    else:
        GROUND = "false"
    TEST_NUMBER = int(args.test_num[0].data)
    print('GDS params publishing. Press Ctrl+C to stop publishing.')

    rospy.init_node('pub_gds_topics')
    test_number_topic_name = "/rattle/test_number"
    pub_test_number = rospy.Publisher(test_number_topic_name, RattleTestNumber, queue_size=10)

    rospy.set_param("/asap/gds_test_num", TEST_NUMBER)
    rospy.set_param("/asap/gds_ground", GROUND)
    rospy.set_param("/asap/gds_sim", SIM)
    rospy.set_param("/asap/gds_role", "robot_name")

    time.sleep(1.0)

    if args.test is True:
        msg = RattleTestNumber()
        msg.stamp = rospy.get_rostime()
        msg.test_number = TEST_NUMBER
        msg.role = 'primary'
        pub_test_number.publish(msg)

    print('GDS parameters set.')

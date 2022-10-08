#!/usr/bin/env python2
import rospy
import argparse
import sys

from time import sleep
from numpy import deg2rad
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from rattle_msgs.msg import RattleTestInstruct

# For rosbag and CTL-C
import rospkg
import rosparam
import subprocess
import shlex
from signal import signal, SIGINT, pause
from sys import exit

active_bag = None
ROSBAG_NAME = "latest_bag"
ROSBAGS = "gnc/ctl/command rattle/nmpc_planner/info_plan_instruct /gnc/ekf /hw/pmc/command /mob/inertia_est /mob/inertia_est_latched " \
          "rattle/local/path_ctl/posearray rattle/local/path_ctl/twistarray /rattle/local/fim_over_horizon /rattle/nmpc/nominal_pose /rattle/nmpc/nominal_twist " \
          "rattle/rrt/path/posearray rattle/rrt/path/twistarray rattle/vis"  # don't forget space after EOL!

rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path('data')


def rosbag_start(active_bag):
    """Top one is for hardware, bottom is for local machine
    Directory must exist for rosbag to start!
    """
    # command = "rosbag record -O /data/info-weight/02-12-20/"+ROSBAG_NAME+".bag gnc/ctl/command /gnc/ctl/nmpc_instruct /gnc/ekf /hw/pmc/command /mob/estimator"
    command = "rosbag record -O " + DATA_PATH + "/output/bags/" + ROSBAG_NAME + ".bag " + ROSBAGS
    command = shlex.split(command)
    active_bag = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)  # we do this so SIGINT is caught by the parent
    print('Starting rosbag...')


def rosbag_stop(active_bag):
    active_bag.send_signal(subprocess.signal.SIGINT)


def handler(signal_received, frame):
    print('\nSIGINT or CTRL-C detected...')
    try:
        rosbag_stop()
        print('...done, rosbags killed.')
    except Exception as ex:
        print('...done.')
    sys.exit(1)


if __name__ == '__main__':
    """ Gets args, parses, and sends msg to rattle_coordinator (primary_coordinator not required).
    """

    signal(SIGINT, handler)

    try:
        parser = argparse.ArgumentParser(description='Run a desired test session.')
        parser.add_argument('test_num', metavar='TEST', type=String, nargs=1,
                            help='A test number to run.')
        parser.add_argument('-c', '--csv', action='store_true', help='-c to indicate use csv.')
        parser.add_argument('-e', '--noekf', action='store_true', help='-e to indicate NO EKF.')
        parser.add_argument('-r', '--record', action='store_true', help='-r to indicate record a bag.')
        parser.add_argument('-g', '--ground', action='store_true', help='-g to run ground version.')
        parser.add_argument('-nu', '--noupdates', action='store_true', help='-nu to indicate NO PARAMETER UPDATES.')
        # updated parameters obtained from the est will be used by default.
        args = parser.parse_args()
        test_num = args.test_num[0].data  # the test number (string) to run

        if args.csv is True:
            USE_CSV = 1
        else:
            USE_CSV = 0
        if args.noekf is True:
            USE_EKF_POSE = 0
        else:
            USE_EKF_POSE = 1
        if args.noupdates is True:
            USE_PARAMS = 0
        else:
            USE_PARAMS = 1
        if args.ground is True:
            ground = True
        else:
            ground = False
        rospy.init_node('run_test', anonymous=True)
        if args.record is True:
            rosbag_start()  # subscribing takes a moment, but usually not long
            sleep(1.7)

        rattle_instruct_pub = rospy.Publisher('rattle/test_instruct', RattleTestInstruct, queue_size=10)
        sleep(0.3)  # need to wait or the node won't start up in time. 0.3 [s] is good.

        msg = RattleTestInstruct()
        msg.test_num = test_num
        msg.USE_EKF_POSE = USE_EKF_POSE
        msg.USE_CSV = USE_CSV
        msg.USE_PARAMS = USE_PARAMS
        msg.WEIGHT_MODE = 1
        msg.INITIAL_MODEL_MODE = 0
        msg.ground = ground
        msg.OBS_CONFIG = 2
        rattle_instruct_pub.publish(msg)
        print('Ready, TestInstruct sent.')

        while not rospy.is_shutdown():
            rospy.spin()

    except rospy.ROSInterruptException:
        pass

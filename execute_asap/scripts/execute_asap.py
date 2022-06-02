#!/usr/bin/env python

"""
This is the main entrypoint to running test scripts. Fill out the ASAP class and begin calling
scripts using execute_test().

Run using: pub_gds_topics.py.

Keenan Albee and Charles Oestreich, 2021
MIT Space Systems Laboratory
"""

# Misc
import argparse
from threading import Thread
import time
import os
import datetime

# msgs
from std_msgs.msg import String
from std_srvs.srv import SetBool
from reswarm_msgs.msg import ReswarmStatusPrimary, ReswarmStatusSecondary
from reswarm_msgs.msg import ReswarmTestNumber
from ff_msgs.msg import SignalState

# For rosbag and CTL-C
import subprocess
import shlex
from signal import signal, SIGINT, pause
from sys import exit
import yaml
from rosbag.bag import Bag

# For data dir determination. No more filling in paths!
import rospy
import rospkg
import sys

# for launching and killing nodelets
import roslaunch

# Supporting ASAP modules
import asap_primary  # primary Astrobee config
import asap_secondary  # secondary Astrobee config
import asap_config  # global configuration params


class ASAP:
    # see asap_config.py for configuration parameters
    DATA_PATH = asap_config.DATA_PATH
    BAG_PATH = ""  # uses Ames convention if hardware
    BAG_PATH_SIM = asap_config.BAG_PATH_SIM
    ASAP_SECONDARY_LAUNCH_PATH = asap_config.ASAP_SECONDARY_LAUNCH_PATH
    ASAP_PRIMARY_LAUNCH_PATH = asap_config.ASAP_PRIMARY_LAUNCH_PATH
    TOPICS_SIM_PRIMARY_EXTRA = asap_config.TOPICS_SIM_PRIMARY_EXTRA
    TOPICS_SIM_PRIMARY = asap_config.TOPICS_SIM_PRIMARY
    TOPICS_SIM_SECONDARY = asap_config.TOPICS_SIM_SECONDARY
    TOPICS_HARDWARE_PRIMARY_EXTRA = asap_config.TOPICS_HARDWARE_PRIMARY_EXTRA
    TOPICS_HARDWARE_PRIMARY = asap_config.TOPICS_HARDWARE_PRIMARY
    TOPICS_HARDWARE_SECONDARY = asap_config.TOPICS_HARDWARE_SECONDARY
    ROSBAG_NAME = asap_config.ROSBAG_NAME
    NODE_LIST_SIM_SECONDARY = asap_config.NODE_LIST_SIM_SECONDARY
    NODE_LIST_HARDWARE_SECONDARY = asap_config.NODE_LIST_HARDWARE_SECONDARY
    NODE_LIST_SIM_PRIMARY = asap_config.NODE_LIST_SIM_PRIMARY
    NODE_LIST_HARDWARE_PRIMARY = asap_config.NODE_LIST_HARDWARE_PRIMARY

    def __init__(self, bee_roles, bee_topic_prefixes):
        """
        bee_roles : ['primary', 'secondary', 'tertiary']
        bee_topic_prefixes : ['/names of astrobee/', ...]

        my_role : {'primary', 'secondary', 'tertiary'}. Uses /robot_name on hardware to set.
        ground: environment, {'true', 'false'} (as a string)
        sim: {'true', 'false'} (as a string)
        """
        # Data for both Astrobeees
        self.bee_roles = bee_roles
        self.bee_topic_prefixes = bee_topic_prefixes

        # Data specific to each Astrobee
        self.ground = 'false'
        self.sim = 'false'
        self.test_num = -1
        self.my_role = 'primary'
        self.test_started = False
        self.gds_role = 'robot_name'
        self.flight_mode = "off"
        self.default_control = "true"
        self.llp_ip = "10.42.0.4"
        self.bag_proc = []  # process for bag recording
        self.bag_filename = ''
        self.gds_roam_bagger = "enabled"  # {enabled or disabled} is the roam bagger enabled?
        self.bag_robot_name = "bumble"
        self.gds_telem = ['.'] * 17  # telemetry to send to GDS, TODO: update start size if modifying or APK will error!

    def start_nodelets(self):
        """ Start up all nodelets. Works for /robot_prefix/ or / namespaces.
        Uses a system `roslaunch` call to start launch files for the LLP and MLP.
        """
        if self.my_role == 'primary':
            if self.sim == "true":
                primary_launch_command = "roslaunch " + self.ASAP_PRIMARY_LAUNCH_PATH + " sim:=" + self.sim + " ground:=" + self.ground + " ns:=" + self.bee_topic_prefixes[0]
            else:
                primary_launch_command = "roslaunch " + self.ASAP_PRIMARY_LAUNCH_PATH + " sim:=" + self.sim + " ground:=" + self.ground + " ns:=/"\
                    + " llp:=" + self.llp_ip
            command = primary_launch_command
        elif self.my_role == 'secondary':
            if self.sim == "true":
                secondary_launch_command = "roslaunch " + self.ASAP_SECONDARY_LAUNCH_PATH + " sim:=" + self.sim + " ground:=" + self.ground + " ns:=/" + self.bee_topic_prefixes[1]
            else:
                secondary_launch_command = "roslaunch " + self.ASAP_SECONDARY_LAUNCH_PATH + " sim:=" + self.sim + " ground:=" + self.ground + " ns:=/"\
                    + " llp:=" + self.llp_ip
            command = secondary_launch_command
        else:
            raise NameError('Invalid role.')

        launch_proc = subprocess.Popen(command, shell=True)

    def stop_nodelets(self):
        """ Stop all nodelets. Works for /robot_prefix/ or / namespaces.
        """
        commands = []

        if self.my_role == 'primary':
            if self.sim == "true":
                prefix = self.bee_topic_prefixes[0]
                node_kill_list = self.NODE_LIST_SIM_PRIMARY
            else:
                prefix = "/"
                node_kill_list = self.NODE_LIST_HARDWARE_PRIMARY

        elif self.my_role == 'secondary':
            if self.sim == "true":
                prefix = self.bee_topic_prefixes[1]
                node_kill_list = self.NODE_LIST_SIM_SECONDARY
            else:
                prefix = "/"
                node_kill_list = self.NODE_LIST_HARDWARE_SECONDARY

        # Use ROS to kill processes
        processes = []
        for node in node_kill_list:
            command = "rosnode kill " + prefix + node
            p = subprocess.Popen(command, shell=True)
            processes.append(p)
        time.sleep(5)

        for p in processes:
            p.kill()

    def rosbag_start(self, test_number):
        """ Start rosbag recording.
        """
        self.BAG_PATH = self.get_bag_path()  # uses Ames convention if hardware

        epoch_str = datetime.datetime.now().strftime("%Y%m%d_%H%M")
        self.ROSBAG_NAME = epoch_str + "_test" + str(test_number)
        # epoch_str = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        # ROSBAG_NAME = "test" + str(test_number) + "_" + epoch_str

        # Starts a bag for either local machine or hardware.
        command = ""

        # Choose specific topic types
        if self.sim == "true":  # sim
            if (self.my_role == 'secondary'):
                command = "rosbag record --split --size=95 -O " + self.BAG_PATH + self.ROSBAG_NAME + "_secondary.bag " + self.TOPICS_SIM_SECONDARY + " __name:=roam_bagger"
            else:
                if (test_number == 9999):  # TODO: add magic number to asap_config.py
                    command = "rosbag record --split --size=95 -O " + self.BAG_PATH + self.ROSBAG_NAME + "_primary_extra.bag " + self.TOPICS_SIM_PRIMARY_EXTRA + " __name:=roam_bagger"
                else:
                    command = "rosbag record --split --size=95 -O " + self.BAG_PATH + self.ROSBAG_NAME + "_primary.bag " + self.TOPICS_SIM_PRIMARY + " __name:=roam_bagger"
        else:  # hardware
            if (self.my_role == 'secondary'):
                command = "rosbag record --split --size=95 -O " + self.BAG_PATH + self.ROSBAG_NAME + "_secondary.bag " + self.TOPICS_HARDWARE_SECONDARY + " __name:=roam_bagger"
            else:
                if (test_number == 9999):  # TODO: add magic number to asap_config.py
                    command = "rosbag record --split --size=95 -O " + self.BAG_PATH + self.ROSBAG_NAME + "_primary_extra.bag " + self.TOPICS_HARDWARE_PRIMARY_EXTRA + " __name:=roam_bagger"
                else:
                    command = "rosbag record --split --size=95 -O " + self.BAG_PATH + self.ROSBAG_NAME + "_primary.bag " + self.TOPICS_HARDWARE_PRIMARY + " __name:=roam_bagger"

        command = shlex.split(command)
        p = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)  # so SIGINT is caught by the parent

    def rosbag_stop(self):
        """ Kill rosbag on ctl-c or -1.
        """
        # self.bag_proc.send_signal(subprocess.signal.SIGINT)
        print('\n[EXECUTE_ASAP]: SIGINT or CTRL-C detected, checking for rosbags...')
        try:
            subprocess.check_call(['rosnode', 'kill', 'roam_bagger'])
            print("[EXECUTE_ASAP]: ...rosbag killed.")
        except Exception:
            print("[EXECUTE_ASAP]: ...no rosbag started.")

    def handler(self, signal_received, frame):
        """ Catch ctl-c.
        """
        self.rosbag_stop()
        sys.exit(0)

    def get_bag_path(self):
        """ Gives the rosbag data path.
        """
        epoch_str = datetime.datetime.now().strftime("%Y-%m-%d")

        if self.sim == "false":  # hardware
            bag_path = "/data/bags/" + epoch_str + "/" + self.bag_robot_name + "/delayed/"
        else:  # sim
            # bag_path = self.DATA_PATH + "/output/rosbags/data/bags/" + epoch_str + "/" + self.bag_robot_name + "/delayed/"
            bag_path = self.BAG_PATH_SIM
        return bag_path

    def run_test(self):
        """ Runs a test, either for 'primary' or 'secondary'.
        """
        self.test_started = True
        test_number = self.test_num

        sim = self.sim
        ground = self.ground

        # print("[EXECUTE_ASAP]: Beginning test " + str(test_number) + "...")
        # print("[EXECUTE_ASAP]: sim:= " + sim)
        # print("[EXECUTE_ASAP]: ground:= " + ground)

        rospy.loginfo("""
        *******************
        *[EXECUTE_ASAP]
        *test_num:= %s
        *sim:= %s
        *ground:= %s
        *(good luck!)
        *******************""", str(test_number), sim, ground)

        # Begin rosbagging
        if test_number != -1 and self.gds_roam_bagger == "enabled":
            self.rosbag_start(test_number)

        # Start payload nodes/nodelets
        if self.my_role == 'primary':
            # Run primary_asap for test parameters
            asap_primary.primary_execute_test(self.bee_topic_prefixes[0], test_number, ground, sim)
        elif self.my_role == 'secondary':
            # Run secondary_asap for test parameters
            asap_secondary.secondary_execute_test(self.bee_topic_prefixes[1], test_number, ground, sim)

        # Ensure params are set before nodelets start running
        time.sleep(2)

        print("[EXECUTE_ASAP]: Launching nodelets.")
        self.start_nodelets()

        # coordinators take it from here
        print("[EXECUTE_ASAP]: Test passed to coordinator.")

    def stop_test(self):
        """ Stop the test.
        """
        self.test_started = False
        print('[EXECUTE_ASAP]: Test canceled.')

        # Stop ROS bag recording
        print('[EXECUTE_ASAP]: Killing ROS bag.')
        self.rosbag_stop()

        # Wait to make sure flight mode is off and controller is default before
        # killing RESWARM nodelets.
        t0 = rospy.get_time()
        while self.default_control != "true" or self.flight_mode != "off":
            print('[EXECUTE_ASAP]: Waiting for coordinator to re-enable default control and set flight mode to off.')
            print(self.flight_mode)
            print(self.default_control)
            time.sleep(1)
            if (rospy.get_time() - t0 > 8.0):
                print('[EXECUTE_ASAP]: Timeout on default_control...')
                break

        # Kill RESWARM nodelets
        print('[EXECUTE_ASAP]: Killing nodelets.')
        self.stop_nodelets()
        print('[EXECUTE_ASAP]: Nodelets killed.')

        # Turn off any signal signal_light_state
        msg = SignalState.STOP_ALL_LIGHTS

    def bee_name_callback(self, name_data):
        """ Only used for hardware. Reads in role from /robot_name.
        * Roles are HARDCODED, capitalization matters.
        * Bypassed if self.gds_role is not robot_name.
        * Also sets bag robot name (must be lowercase).
        """
        # for bagger
        if name_data.data == "Honey":
            self.bag_robot_name = "honey"
        elif name_data.data == "Bsharp":
            self.bag_robot_name = "bsharp"
        elif name_data.data == "Wannabee":
            self.bag_robot_name = "wannabee"
        elif name_data.data == "Bumble":
            self.bag_robot_name = "bumble"
        elif name_data.data == "Queen":
            self.bag_robot_name = "queen"

        # for role
        if self.gds_role == "robot_name":  # if role has not been set by GDS
            if name_data.data == "Honey" or name_data.data == "Queen" or name_data.data == "Bsharp":
                self.my_role = 'primary'
            else:
                self.my_role = 'secondary'

    def status_callback(self, status_msg):
        """ Subscribe to status published by C++ coordinator.
        Used to check to make sure flight mode is set to off and control is
        default before killing C++ nodelets. Also updates info for GDS (via param).
        """
        if status_msg.default_control:
            self.default_control = 'true'
        else:
            self.default_control = 'false'

        self.flight_mode = status_msg.flight_mode

        if self.my_role == "primary":
            self.gds_telem = [
                str(status_msg.test_finished),
                str(status_msg.coord_ok),
                str(status_msg.control_mode),
                str(status_msg.regulate_finished),
                str(status_msg.uc_bound_activated),
                str(status_msg.uc_bound_finished),
                str(status_msg.mrpi_finished),
                str(status_msg.traj_sent),
                str(status_msg.traj_finished),
                str(status_msg.gain_mode),
                str(status_msg.lqrrrt_activated),
                str(status_msg.lqrrrt_finished),
                str(status_msg.info_traj_send),
                ".",
                ".",
                ".",
                "."]
        elif self.my_role == "secondary":
            self.gds_telem = [
                str(status_msg.test_finished),
                str(status_msg.coord_ok),
                ".",
                ".",
                ".",
                ".",
                ".",
                ".",
                ".",
                ".",
                ".",
                ".",
                ".",
                str(status_msg.solver_status),
                str(status_msg.cost_value),
                str(status_msg.kkt_value),
                str(status_msg.sol_time)]

    def update_gds_telemetry(self, global_gds_param_count):
        """ Set params to send GDS telemetry (5x slower).
        TODO: Update telemetry vector in GDS!
        """
        rospy.set_param("/asap/gds_telem",
                        [str(global_gds_param_count), str(ASAP_main.test_num), str(ASAP_main.flight_mode)] + self.gds_telem)

    def test_num_okay(self):
        """ Only allow valid test numbers.
        TODO: verify for reswarm
        """
        test_num = self.test_num
        str_test_num = str(test_num)
        OKAY = False
        print("TESTING OK: ", test_num)

        # unit tests (1-15) and the debug test, 77, and rattle's special test
        if (test_num >= 0) and (test_num <= 22) or test_num == 77 or test_num == 78:
            OKAY = True

        #  RATTLE's parameter tests
        if len(str_test_num) == 5:  # check digit-by-digit
            if (int(str_test_num[0]) == 7) and (int(str_test_num[1]) == 7) and \
               (int(str_test_num[2]) >= 0) and (int(str_test_num[2]) <= 1) and \
               (int(str_test_num[3]) >= 0) and (int(str_test_num[3]) <= 5) and \
               (int(str_test_num[4]) >= 0) and (int(str_test_num[4]) <= 3):
                OKAY = True

        # Pedro's parameter tests
        if (test_num >= 100 and test_num <= 199) or \
           (test_num >= 10000 and test_num <= 19999) or \
           (test_num >= 200 and test_num <= 299) or \
           (test_num >= 20000 and test_num <= 29999) or \
           (test_num >= 300 and test_num <= 399) or \
           (test_num >= 30000 and test_num <= 39999):
            OKAY = True

        # stop test
        if test_num == -1:
            OKAY = True
        if not OKAY:
            print("[execute_asap]: Invalid test number")
        return OKAY

    def stop_signal_lights(self):
        """ Stop the signal lights on call.
        """
        try:
            state = SignalState.STOP_ALL_LIGHTS
            msg = SignalState()
            msg.state = state
            pub_signal.publish(msg)
        except Exception:
            print("Unable to stop signal lights.")

    def publish_test_num(self, test_num, my_role):
        """ Publish a test number for coordinator.
        """
        msg = ReswarmTestNumber()
        msg.stamp = rospy.get_rostime()
        msg.test_number = test_num
        msg.role = my_role
        pub_test_number.publish(msg)

    def update_status_sub(self):
        """ Subscriber for RESWARM status message (published by coordinators).
        Make sure it is called AFTER bee_name obtained.
        """
        if self.my_role == 'primary':
            rospy.Subscriber(status_msg_name, ReswarmStatusPrimary, self.status_callback)
        elif self.my_role == 'secondary':
            rospy.Subscriber(status_msg_name, ReswarmStatusSecondary, self.status_callback)
        else:
            raise ValueError("Unknown Role")

    def ros_loop(self):
        """ The main ROS loop for:
        1) checking test number from GDS
        2) publishing test number
        3) checking subscribers for robot name/status message
        """
        ASAP_RATE = 0.5
        STATUS_SUB_SET = False
        sleep_rate = rospy.Rate(ASAP_RATE)
        param_set_count = 0
        global_gds_param_count = 0

        while not rospy.is_shutdown():
            # 1) check GDS params (from Astrobee Android)
            gds_test_num = rospy.get_param("/asap/gds_test_num")  # {int}
            gds_ground = rospy.get_param("/asap/gds_ground")  # {"false", "true"}
            gds_sim = rospy.get_param("/asap/gds_sim")  # {"sim", "hardware"}
            gds_role = rospy.get_param("/asap/gds_role")  # {"robot_name", "primary", "secondary", etc.}; overrides role-setting from /robot_name if set!
            gds_roam_bagger = rospy.get_param("/asap/gds_roam_bagger")  # {"false", "true"}

            # set ASAP based on GDS info
            self.test_num = gds_test_num
            self.ground = gds_ground

            if gds_sim == "sim":
                self.sim = 'true'
            else:
                self.sim = 'false'

            self.gds_role = gds_role
            if gds_role != "robot_name" and self.sim == "false":  # if on hardware and not using robot_name role
                self.my_role = gds_role  # note: you only want to update this on hardware, otherwise it's set during sim launch

            self.roam_bagger = gds_roam_bagger

            # 2) publish test_number and start all nodelets (including coordinator)
            self.publish_test_num(self.test_num, self.my_role)

            # Update GDS ROS params (ground telemetry)
            param_set_count += 1
            if (param_set_count > 5):
                param_set_count = 0
                global_gds_param_count += 1
                self.update_gds_telemetry(global_gds_param_count)

            # Start test if not -1. Otherwise, wait.
            if (self.test_num is not -1 and self.test_started is False):
                if self.test_num_okay():  # sanity check the test num before sending
                    self.run_test()
            if (self.test_num == -1 and self.test_started is True):
                self.stop_test()
                self.stop_signal_lights()

            # 3) keep spinning
            if (STATUS_SUB_SET is False):
                self.update_status_sub()
                STATUS_SUB_SET = True

            sleep_rate.sleep()


if __name__ == "__main__":
    """ Run on node startup. Waits for GDS parameters to be set before proceeding.
    """
    # Initialize node
    rospy.init_node('execute_asap', anonymous=True)

    # Set up the main testing interface.
    ASAP_main = ASAP(bee_roles=['primary', 'secondary', 'tertiary'],  # TODO: add tertiary interface
                     bee_topic_prefixes=['/queen/', '/bumble/', '/honey/'])

    signal(SIGINT, ASAP_main.handler)  # for ctl-c handling

    # Below is a special comparison that only works if an LLP.launch call with the correct number of arguments (2) is used.
    # The hardware launch process will always use "/"
    # Simulation launches will always use "/robot_prefix 0"
    myargv = rospy.myargv(argv=sys.argv)

    try:  # simulation
        arg_robot_name = myargv[1]  # robot_prefix, SIMULATION ONLY!
        if arg_robot_name == "honey":
            ASAP_main.my_role = 'primary'
            test_number_msg_name = "/honey/reswarm/test_number"
            status_msg_name = "/honey/reswarm/status"
            signal_msg_name = "/honey/signals"
        elif arg_robot_name == "bumble":
            ASAP_main.my_role = 'secondary'
            test_number_msg_name = "/bumble/reswarm/test_number"
            status_msg_name = "/bumble/reswarm/status"
            signal_msg_name = "/bumble/signals"
        elif arg_robot_name == "queen":
            ASAP_main.my_role = 'primary'
            test_number_msg_name = "/queen/reswarm/test_number"
            status_msg_name = "/queen/reswarm/status"
            signal_msg_name = "/queen/signals"
        elif arg_robot_name == "/":
            ASAP_main.my_role = 'primary'
            test_number_msg_name = "/reswarm/test_number"
            status_msg_name = "/reswarm/status"
            signal_msg_name = "/signals"
        print("[EXECUTE_ASAP]: Using namespace -- " + arg_robot_name)
    except Exception:  # hardware
        ASAP_main.my_role = 'primary'
        test_number_msg_name = "/reswarm/test_number"
        status_msg_name = "/reswarm/status"
        signal_msg_name = "/signals"
        print("[EXECUTE_ASAP]: Using hardware namespace -- /.")

    # ROS initialization
    # initialize GDS params (need to change astrobee android!)
    rospy.set_param("/asap/gds_ground", "false")  # 'true' or 'false'
    rospy.set_param("/asap/gds_sim", "hardware")  # 'hardware' or 'sim'
    rospy.set_param("/asap/gds_test_num", -1)
    rospy.set_param("/asap/gds_role", "robot_name")  # default to using robot_name
    rospy.set_param("/asap/gds_roam_bagger", "enabled")  # "enabled" or "disabled"

    # subscriber for robot name topic
    rospy.Subscriber("/robot_name", String, ASAP_main.bee_name_callback)

    # publisher for test_number
    pub_test_number = rospy.Publisher(test_number_msg_name, ReswarmTestNumber, queue_size=10)
    # signal lights
    pub_signal = rospy.Publisher(signal_msg_name, SignalState, queue_size=1, latch=True)

    time.sleep(1.0)  # wait for subs to come in
    print("[EXECUTE_ASAP]: Initialized.")
    ASAP_main.ros_loop()

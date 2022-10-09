#!/usr/bin/env python3
from __future__ import division

import rospy
import paramest_propagation_utils
import paramest_estimation_utils
import numpy as np
from inv_fam.msg import  InverseFAM
from ff_msgs.msg import EkfState
from rospy.numpy_msg import numpy_msg
from param_est.msg import Params
from reswarm_msgs.msg import ReswarmStatusPrimary


class Callbacks():

    def __init__(self, rosparams):
        self.process = 0
        self.init_fam_time = 0
        self.batch_dim = 30
        self.states_init = False
        self.ekf_stack_full = 0
        self.fam_stack_full = 0
        self.ekf_ctr = 0
        self.fam_ctr = 0
        self.ekf_data = np.zeros((self.batch_dim, 17))
        self.fam_data = np.zeros((self.batch_dim, 7))
        self.ekf_log = np.zeros((self.batch_dim, 17))
        self.fam_log = np.zeros((self.batch_dim, 7))
        self.combined_prev = np.zeros((2*self.batch_dim, 17))
        self.combined = np.zeros((2*self.batch_dim, 17))
        self.ip_and_meas_list = []
        self.ts_prev = 0 # previous time stamp
        self.ts_curr = 0 # current time stamp
        self.ctl_prev = np.zeros((6,))
        self.info_traj_send = False
        self.test_num = -1
        self.reg_delay = 0 # delay before starting the estimation loop (FAM spin-up + regulation maneuver)

        sim = rosparams[0]
        if (sim == "true"):
            self.topic_prefix = "/queen/"
        else:
            self.topic_prefix = "/"

        ground = rosparams[1]
        if (ground == "true"):
            self.DoF = 3
        else:
            self.DoF = 6

        self.reg_delay = rosparams[2]
        est = paramest_estimation_utils

    def status_callback(self, data):
        self.test_num = data.test_number
        self.info_traj_send = data.info_traj_send
        self.activate_rattle = data.activate_rattle

    def ekf_callback(self, data, est):
        #print("ekf calbak" + str(data.header.stamp.secs + 10**-9 * data.header.stamp.nsecs))
        if self.ekf_ctr == self.batch_dim:
            self.ekf_ctr = 0
            self.ekf_stack_full = 1
            self.ekf_log = self.ekf_data
            self.check_callback(est)
            self.ekf_data = np.zeros((self.batch_dim, 17)) # set the array to zero
        self.ekf_data[self.ekf_ctr, 0] = data.header.stamp.secs + 10**-9 * data.header.stamp.nsecs
        self.ekf_data[self.ekf_ctr, 1:4] = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        self.ekf_data[self.ekf_ctr, 4:7] = [data.velocity.x, data.velocity.y, data.velocity.z]
        self.ekf_data[self.ekf_ctr, 7:11] = [data.pose.orientation.x,
                                   data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.ekf_data[self.ekf_ctr, 11:14] =  [data.omega.x, data.omega.y, data.omega.z]
        self.ekf_data[self.ekf_ctr, 14:17] = [data.accel.x, data.accel.y, data.accel.z]
        self.ekf_ctr+=1

    def fam_callback(self, data, est):
        #print("fam calbak" + str(data.header.stamp.secs + 10**-9 * data.header.stamp.nsecs))
        t = data.header.stamp.secs + 10**-9 * data.header.stamp.nsecs
        if self.init_fam_time == 0:
            self.init_fam_time = t
            print(" [PARAM_EST] FAM activation at : " + str(self.init_fam_time))
            print(" [PARAM_EST] Updates will begin at: " + str(self.init_fam_time + self.reg_delay))

        if t >= self.init_fam_time + self.reg_delay: # only start recording fam data after delay has elapsed
            if self.fam_ctr == self.batch_dim:
                self.fam_ctr = 0
                self.fam_stack_full = 1
                self.fam_log = self.fam_data
                self.check_callback(est)
                self.fam_data = np.zeros((self.batch_dim, 7))
            self.fam_data[self.fam_ctr, 0] = t
            self.fam_data[self.fam_ctr, 1:4] = [data.force.x, data.force.y, data.force.z]
            self.fam_data[self.fam_ctr, 4:7] = [data.torque.x, data.torque.y, data.torque.z]
            self.fam_ctr += 1

    def check_callback(self, est):
        #print("check callback")
        # print(self.fam_ctr)
        # print(self.ekf_ctr)
        if (self.ekf_stack_full & self.fam_stack_full):
            # check the time stamps
            self.ekf_stack_full = 0
            self.fam_stack_full = 0
            combined_unsorted = np.vstack([np.pad(self.fam_log,((0,0),(0,10)),'constant'), self.ekf_log])
            # check if they have been ordered
            self.combined_sort = combined_unsorted[combined_unsorted[:, 0].argsort()]
            if self.combined_sort[0, 0]>=self.combined_prev[-1, 0]:
                self.combined = self.combined_prev # use the previous one
                self.combined_prev = self.combined_sort # current becomes previous for the next round
            else:
                combined_unsorted = np.vstack([self.combined_prev, self.combined_sort])
                self.combined_prev_and_current = combined_unsorted[combined_unsorted[:, 0].argsort()]
                self.combined = self.combined_prev_and_current[0:2*self.batch_dim,:] # use the previous one
                self.combined_prev = self.combined_prev_and_current[2*self.batch_dim:,:] # current becomes previous for the next round
            self.ip_and_meas_list.append(self.combined.tolist())
            #print(len(self.ip_and_meas_list))
            self.process = 1

    def run_ekf(self, est):

        if (len(self.ip_and_meas_list)>0):
            data_batch = np.array(self.ip_and_meas_list[0])
            self.ip_and_meas_list = self.ip_and_meas_list[1:]
            """
            print("run ekf" + str(len(self.ip_and_meas_list)))
            print(data_batch[:,0])
            print("new_batch")
            """
            if (self.test_num == 13 and self.activate_rattle == False):
                pass
            else:
                for n in range(0, self.batch_dim*2, 10):
                    row = data_batch[n, :]
                    self.ts_curr = row[0]
                    dt = self.ts_curr - self.ts_prev
                    if self.states_init == True:
                        if all(row[7:]==0): # if the data corresponds to fam

                            # propagate using the previous control input over this time step
                            if dt>0:
                                est.state_propagation(dt, self.ctl_prev)
                            self.ctl_prev = np.array((row[1:7])).reshape((6,))

                        else: # the data corresponds to the localization ekf
                            # propagate using the previous control input over this time step
                            if dt>0:
                                est.state_propagation(dt, self.ctl_prev)
                            # then perform the update when comparing to the measured states
                            states_meas = np.array((row[1:])).reshape((16,))
                            est.update_step(dt, states_meas)

                    else:
                        # If the states are not initialized, used the first measurement to initialize them - typically the
                        # very first time step
                        if any(row[7:] != 0):
                            states_meas = np.array((row[1:])).reshape((16,))
                            # use the measured states to initialize the linear and angular velocity
                            est.states_prop = est.get_z_tilde(states_meas)
                            self.states_init = True
                    self.ts_prev = self.ts_curr


def process_data():
    # initialize node
    rospy.init_node('param_estimate')
    # read parameter, simulation or hardware
    sim = rospy.get_param('/reswarm/sim')

    # parameter, ground or ISS
    ground = rospy.get_param('/reswarm/ground')
    # regulation delay parameter
    reg_delay = rospy.get_param('/reswarm/primary/reg_time')

    # for some simulation rosbag testing.
    # sim = "false"
    # ground = "true"


    # arbitrary value to ensure that the data are processed as soon as a batch is complete.
    loop_rate = rospy.Rate(50)


    call = Callbacks([sim, ground, reg_delay])


    # initialize the propagation object.
    # simplified dynamics - without com offsets for 3 DoF
    # and without CoM offsets and products of inertia for 6 DoF
    # use_vels_meas - use acceleration measurements in the EKF?
    use_vels_meas = 1
    if ground == "true":
        if sim == "true":
            simplified_dynamics = 0
        else:
            simplified_dynamics = 1
    else:
        simplified_dynamics = 1
    prop = paramest_propagation_utils.Propagation(call.DoF, simplified_dynamics)

    est = paramest_estimation_utils.Estimation(prop, use_vels_meas, sim)

    est.inertial_params_pub = rospy.Publisher(call.topic_prefix + "mob/inertia_est", Params, queue_size=1)

    # subscribe to the localization ekf and the inv_fam node to get the applied forces and torques.
    rospy.Subscriber(call.topic_prefix + "gnc/ekf", numpy_msg(EkfState), call.ekf_callback, est)
    rospy.Subscriber(call.topic_prefix + "inv_fam/appliedFandT", numpy_msg(InverseFAM), call.fam_callback, est)
    # Subscribe to get the test number and coordinator commands from /reswarm/status
    rospy.Subscriber(call.topic_prefix + "reswarm/status", ReswarmStatusPrimary,
                     call.status_callback)


    while not rospy.is_shutdown():

        if call.process == 1:
            call.run_ekf(est)
        loop_rate.sleep()

if __name__ == '__main__':
    process_data()


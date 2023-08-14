#!/usr/bin/env python
"""
# inv_fam_implementation.py

Inverse fam implementation for the Astrobee Robot software (convert nozzle positions to applied/post-saturation forces and torques).
Supporting constants are derived from the Astrobee Simulink model from release 0.15.1 

Monica Ekal, 2021.
Institute for Systems and Robotics, Instituto Superior Tecnico.

"""
from __future__ import division
import pdb
import rospy
import numpy as np
import FAM_constants
import FAM_functions
import message_filters
from ff_msgs.msg import FamCommand
from ff_hw_msgs.msg import PmcCommand
from rospy.numpy_msg import numpy_msg
from inv_fam.msg import ForwardFAM
from inv_fam.msg import InverseFAM
from geometry_msgs.msg import InertiaStamped

# The diagnostics mode computes the forward mixer, i.e, it recreates Astrobee FAM functions (F&T to nozzle positions) for debugging
DIAGNOSTICS = 0
class Mixer:

    def __init__(self, prefix):
        self.topic_prefix = prefix

    def update_mixer(self):
        # initialization of mixer parameters
        nozzle_moment_arm = FAM_constants.P_nozzle_B_B - self.CoM
        thrust2torque_B = - np.cross(nozzle_moment_arm, FAM_constants.nozzle_orientations)
        thrust2force_B = -FAM_constants.nozzle_orientations
        self.Matrix_mixer = np.linalg.pinv(np.hstack([thrust2force_B, thrust2torque_B])).T
        self.inv_matrix_mixer = np.hstack([thrust2force_B, thrust2torque_B]).T

    def get_FandT_from_nozzle_angles(self, angles):
        # commanded nozzle opening
        S_i = FAM_constants.nozzle_height - np.cos(angles) * FAM_constants.nozzle_flap_length
        # commanded area per nozzle
        A_i = S_i * 2 * FAM_constants.nozzle_widths
        # prep for look-up
        T_norm_by_delta_P  = np.empty((12))
        T_N = np.empty((12))
        for i in range(12):
            T_norm_by_delta_P[i] = 2*A_i[i]*FAM_constants.Cd[i]
        sum_T_norm_by_delta_P_1_6 = sum(T_norm_by_delta_P[0:5])
        sum_T_norm_by_delta_P_7_12 = sum(T_norm_by_delta_P[6:11])
        # use look-up to find deltaP (one for each PM, nozzles 1 to 6 and 7 to 12)
        delta_P1 = np.interp(sum_T_norm_by_delta_P_1_6, FAM_constants.thrust_modified_div_by_delta_P,
                             FAM_constants.Delta_P)
        delta_P2 = np.interp(sum_T_norm_by_delta_P_7_12, FAM_constants.thrust_modified_div_by_delta_P,
                             FAM_constants.Delta_P)

        # find thrust per nozzle using deltaP
        for i in range(6):
            T_N[i] = 2*A_i[i]*FAM_constants.Cd[i]**2*delta_P1

        for i in range(6,12):
            T_N[i] = 2*A_i[i]*FAM_constants.Cd[i]**2*delta_P2

        # inverse mixing to find FandT from thrust per nozzle
        FandT = np.matmul(self.inv_matrix_mixer, T_N)
        return FandT


def calc_FandT():

    rospy.init_node('inv_fam')

    # specify topic prefix according to the Astrobee(s) in use
    topic_prefix = "/queen/"
    mixer_obj = Mixer(topic_prefix)

    # get the center of mass offset for FAM
    inertia_sub = rospy.Subscriber(mixer_obj.topic_prefix + "mob/inertia", numpy_msg(InertiaStamped), inertiaCallback,
                                   mixer_obj)
    # a delay for allowing the COM to be updated from /mob/inertia
    rospy.sleep(5.)

    
    # subscribe to receive the commanded nozzle angles
    pmc_sub = rospy.Subscriber(mixer_obj.topic_prefix + "hw/pmc/command", numpy_msg(PmcCommand), Pmccallback,
                               [pmc_pub, mixer_obj])
    # publish the estimated post-saturation wrenches 
    pmc_pub = rospy.Publisher(mixer_obj.topic_prefix + "inv_fam/appliedFandT", InverseFAM, queue_size=1)

    if DIAGNOSTICS:
    # The diagnostics mode computes the forward mixer, i.e, it recreates Astrobee FAM (F&T to nozzle positions)
    # as a sanity check for the FAM calculation, and thus the FAM constants.py
        forward_mixer_obj = FAM_functions.Forward_Mixer(mixer_obj.Matrix_mixer)
        nozzle_pub = rospy.Publisher(mixer_obj.topic_prefix + "fam/commandedNozzles", ForwardFAM, queue_size=1)
        fam_sub = rospy.Subscriber(mixer_obj.topic_prefix + "gnc/ctl/command", numpy_msg(FamCommand), FandTcallback,
                                   [nozzle_pub, forward_mixer_obj])

    rate = rospy.Rate(62.5)
    while not rospy.is_shutdown():
        rate.sleep()


def inertiaCallback(data, arg):
    mixer_obj = arg
    mixer_obj.CoM = np.array([data.inertia.com.x, data.inertia.com.y, data.inertia.com.z])
    mixer_obj.update_mixer()

def Pmccallback(data, arg):
    pmc_pub = arg[0]
    mixer_obj = arg[1]

    Nozzle_pos_1 = data.goals[0].nozzle_positions
    Nozzle_pos_2 = data.goals[1].nozzle_positions
    nozzle_positions_all = np.empty(12)
    for i in range (6):
        nozzle_positions_all[i] = ord(Nozzle_pos_1[i])
        nozzle_positions_all[i+6] = ord(Nozzle_pos_2[i])
    angles = nozzle_positions_all*FAM_constants.step_in_rad
    angles += FAM_constants.nozzle_min_angle
    # perform the computation to get F&T from the nozzle commands
    FandT = mixer_obj.get_FandT_from_nozzle_angles(angles)
    pub_applied_FandT(pmc_pub, data.header.stamp, nozzle_positions_all, FandT)


def pub_applied_FandT(pmc_pub, stamp, nozzle_positions_all, FandT):
    # publishes computed FandT from nozzle positions (inverse fam) and the nozzle openings
    # from /hw/pmc/command
    msg = InverseFAM()
    msg.header.stamp = stamp
    msg.header.frame_id = 'body'
    msg.force.x = FandT[0]
    msg.force.y = FandT[1]
    msg.force.z = FandT[2]
    msg.torque.x = FandT[3]
    msg.torque.y = FandT[4]
    msg.torque.z = FandT[5]
    # nozzle positions, for debugging, same as /hw/pmc/command
    msg.nozzle_openings = nozzle_positions_all
    pmc_pub.publish(msg)



"""
Diagnostic/forward fam related subs and pubs, only active if DIAGNOSTICS = 1
"""
def FandTcallback(data, arg):
    # calculates FandT from /gnc/ctl/command and the supposed nozzle openings (forward fam)
    # for sanity check
    FandT_pub = arg[0]
    forward_mixer_obj = arg[1]
    forces_body = np.array([data.wrench.force.x, data.wrench.force.y, data.wrench.force.z])
    torque_z = np.array([data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z])
    FandT = np.hstack([forces_body, torque_z])
    nozzle_pos = forward_mixer_obj.replicate_FAM(FandT)
    pub_commanded_nozzles(FandT_pub, data.header.stamp, nozzle_pos, FandT)


def pub_commanded_nozzles(FandT_pub, stamp, nozzle_positions_all, FandT):
    # publishes FandT from /gnc/ctl/command and the supposed nozzle openings (forward fam)
    # for sanity check

    msg = ForwardFAM()
    msg.header.stamp = stamp
    msg.header.frame_id = 'body'
    # FandT from /gnc/ctl/command
    msg.force.x = FandT[0]
    msg.force.y = FandT[1]
    msg.force.z = FandT[2]
    msg.torque.x = FandT[3]
    msg.torque.y = FandT[4]
    msg.torque.z = FandT[5]
    # computed nozzle positions
    msg.nozzle_openings = nozzle_positions_all
    FandT_pub.publish(msg)




if __name__ == '__main__':
    calc_FandT()

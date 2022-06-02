#!/usr/bin/env python

'''
Plotting utilities to visualize data from a bag file.

Monica Ekal, Keenan Albee Mar 2021
'''

import rosbag
import rospkg
import time
import string
import numpy as np
import math
from matplotlib import pyplot as plt

# set up LaTeX
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "sans-serif",
    "font.sans-serif": ["Helvetica"],
    'font.size': 40})
## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
    'font.size': 30,
    'axes.labelsize': 28,
    'xtick.labelsize': 20,
    'ytick.labelsize': 20
})

class BagReader:
    """ BagReader reads in topics from bag files and provides plotting templates.

    plot(): main function that is called on startup
    read_topics(): reads in all bag topics of interest
    """
    rospack = rospkg.RosPack()
    PATH = "/media/albee/Shared/keenan/data/granite-lab-testing-04-22-21/bsharp/"
    # BAG = "test5_2021-04-22-23-11-36_chaser_0.bag"
    BAG = "test9_2021-04-22-17-43-28_chaser_0.bag"
    # bag = rosbag.Bag(PATH +'/bags/ground-data-03-12-21/bsharp/test9_2021-03-12-202641_chaser_w_haz.bag')
    bag = rosbag.Bag(PATH + BAG)

    # bag = rosbag.Bag('../output/bags/blargh_bag.bag')

    # topics = ['/gnc/ekf', '/td/traj_gen_dlr/mpdebug_numbers']
    topics = ['/td/tube_mpc/debug']
    # topics = ['/td/traj_gen_dlr/mpdebug_numbers']
    # topics = ['/td/uc_bound/uc_bound']

    def __init__(self):
        self.plot()

    def plot(self):
        topic = self.topics[0]

        topic_contents = self.read_topic(self.bag, topic)
        msgs = self.get_msgs(topic_contents)

        self.plot_comp_time(msgs)
        # print(msgs)

        # specify desired plotting below!

        # self.plot_global_plan()
        # self.plot_ekf()
        # self.plot_F_and_T()
        # self.plot_params()

        # color = "#940110"
        # highlight = "#f5a2aa"
        # self.plot_params_all_bags(self.bags, color, highlight)
        #
        # color = "#0961e6"
        # highlight = "#aac6f0"
        # self.plot_params_all_bags(self.bags2, color, highlight)
        # self.plot_params_both_bags(self.bags, self.bags2)

        plt.show()

    def plot_comp_time(self, msgs):
        """ Print the computational time.
        """
        comp_time = []
        for msg in msgs:
            comp_time.append(msg.casadi_comp_time.data)
            print(msg.casadi_comp_time.data)
        comp_time_np = np.asarray(comp_time)
        mean_comp_time = np.mean(comp_time_np)

        plt.figure()

        plt.plot(comp_time_np)
        plt.hlines(0.2, 0, len(msgs), color='r')
        plt.hlines(mean_comp_time, 0, len(msgs), color='b')

        plt.legend(['Computation Time'], loc="best")
        plt.xlabel('Sample [-]')
        plt.ylabel('Computation Time [s]')
        plt.title('Computation Time')

    def read_topic(self, bag, topic):
        """ Reads topics from bag.
        topics: list of str topics
        bag: bagfile object

        Output:
        topic_contents, topics from a bag
        """
        # for topic gnc/ekf
        topic_contents = bag.read_messages(topics=topic)
        return topic_contents

    def get_msgs(self, topic_conents):
        """ Return a list of messages from topic_contents.

        Output:
        msgs, list of messages
        """
        msgs = []
        for topic, msg, t in topic_conents:  # note: this is a triple-nested for loop
            msgs.append(msg)
        return msgs

    def read_topics_sample(self, bag):
        # for topic gnc/ekf
        topicContents = bag.read_messages(topics=['/gnc/ekf'])
        pos = []
        orientation = []
        velocity = []
        time_ekf = []
        for topic, msg, t in topicContents:
        	pos.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        	orientation.append([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        	velocity.append([msg.velocity.x, msg.velocity.y, msg.velocity.z])
        	time_ekf.append(t.to_sec())
        positions_tracked = np.array(pos)
        orientations_tracked = np.array(orientation)
        velocity_tracked = np.array(velocity)

        self.positions_tracked = positions_tracked
        self.orientations_tracked = orientations_tracked
        self.velocity_tracked = velocity_tracked
        self.time_ekf = time_ekf

        # for topic /rattle/local/
        topicContents = bag.read_messages(topics=['rattle/local/path_ctl/posearray'])
        pos_l = []
        orientation_l = []
        for topic, msg, t in topicContents:
        	for msg_poses in msg.poses:
        		pos_l.append([msg_poses.position.x, msg_poses.position.y, msg_poses.position.z])
        		orientation_l.append([msg_poses.orientation.x, msg_poses.orientation.y, msg_poses.orientation.z, msg_poses.orientation.w])
        positions_local = np.array(pos_l)
        orientations_local = np.array(orientation_l)

        self.positions_local = positions_local
        self.orientations_local = orientations_local

        # for topic /gnc/ctl/nmpc_instruct
        pos_d = []
        orientation_d = []

        poses = []
        twists = []
        topicContents = bag.read_messages(topics=['rattle/rrt/path/posearray'])
        for topic, msg, t in topicContents:
            poses = msg.poses
        topicContents = bag.read_messages(topics=['rattle/rrt/path/twistarray'])
        for topic, msg, t in topicContents:
            twists = msg.TwistArray

        for each in poses:
            pos_d.append([each.position.x, each.position.y, each.position.z])
            orientation_d.append([each.orientation.x, each.orientation.y, each.orientation.z, each.orientation.w])
        positions_global = np.array(pos_d)
        orientations_global = np.array(orientation_d)

        self.positions_global = positions_global
        self.orientations_global = orientations_global

        # for topic /gnc/ctl/command
        topicContents = bag.read_messages(topics = ['gnc/ctl/command'])
        fx = []
        fy = []
        tz = []
        time_command = []
        for topic, msg, t in topicContents:
        	fx.append([msg.wrench.force.x])
        	fy.append([msg.wrench.force.y])
        	tz.append([msg.wrench.torque.z])
        	time_command.append(t.to_sec())

        self.time_command = time_command
        self.fx = fx
        self.fy = fy
        self.tz = tz

        # for topic /mob/inertia_est
        topicContents = bag.read_messages(topics=['/mob/inertia_est'])
        pose_inertia_est = []
        twist_inertia_est = []
        param_estimates = []
        covariance = []
        time_inertia_est = []
        for topic, msg, t in topicContents:
            #pose_inertia_est.append([msg.x, msg.y, msg.eul_z])
            #twist_inertia_est.append([msg.vel_x, msg.vel_y, msg.ang_vel_z])
            param_estimates.append([msg.mass, msg.cx, msg.cy, msg.izz])
            covariance.append([msg.Cov[0], msg.Cov[1], msg.Cov[2], msg.Cov[3]])
            time_inertia_est.append(t.to_sec())
        pose_est = np.array(pose_inertia_est)
        twist_est = np.array(twist_inertia_est)
        param_est = np.array(param_estimates)
        est_cov = np.array(covariance)
        time_inertia_est = np.array(time_inertia_est)

        self.pose_est = pose_est
        self.twist_est = twist_est
        self.param_est = param_est
        self.est_cov = est_cov

        t0 = time_inertia_est[0]
        time_inertia_est =  time_inertia_est - t0
        self.time_inertia_est = time_inertia_est

    def plot_params_all_bags(self, bags, color, highlight):
        # plot params for multiple bags
        fig = plt.figure()

        for bag in bags:
            self.read_topics(bag)
            fig = self.plot_params(color, highlight, fig)

    def plot_params_both_bags(self, bags1, bags2):
        # plot params for multiple bags
        fig = plt.figure()

        color = "#940110"
        highlight = "#f5a2aa"
        for bag in bags1:
            self.read_topics(bag)
            fig = self.plot_params(color, highlight, fig, 0)

        color = "#940110"
        highlight = "#f5a2aa"
        for bag in bags2:
            self.read_topics(bag)
            fig = self.plot_params(color, highlight, fig, 4)

    def plot_global_plan(self):
        # global plan plot
        positions_global = self.positions_global
        positions_local = self.positions_local
        positions_tracked = self.positions_tracked

        plt.figure()

        plt.plot(positions_global[:,0], positions_global[:,1], 'g-o')
        plt.plot(positions_local[:,0], positions_local[:,1],'ro', markersize = 3.5)
        plt.plot(positions_tracked[:,0], positions_tracked[:,1],'k:',Linewidth = 2)
        plt.plot(positions_tracked[-1,0], positions_tracked[-1,1],'go:')
        plt.gca().invert_yaxis()

        plt.legend(['Global Plan', 'Local Plan', 'Tracked Path'], loc="best")
        plt.xlabel('X-Position [m]')
        plt.ylabel('Y-Position [m]')
        plt.title('Top-Down View of Global and Local Plans')

    def plot_ekf(self):
        # plot ekf response

        time_ekf = self.time_ekf
        positions_tracked = self.positions_tracked
        orientations_tracked = self.orientations_tracked

        plt.figure()
        for i in range(1, 4):
        	plt.subplot(2, 2, i)
        	# plot obtained positions
        	plt.plot(time_ekf, positions_tracked[:, i - 1])
        plt.suptitle("Tracked trajectory, positions")

        plt.figure()
        for i in range(1, 5):
        	plt.subplot(2, 2, i)
        	# plot obtained orientations
        	plt.plot(time_ekf, orientations_tracked[:, i - 1])
        plt.suptitle("Tracked trajectory, orientation")

    def plot_F_and_T(self):
        time_command = self.time_command
        fx = self.fx
        fy = self.fy
        tz = self.tz

        #plot_FandT:
        fig = plt.figure()
        plt.subplot(2,2,1)
        plt.plot(time_command,fx)
        plt.title('commanded fx')
        plt.subplot(2,2,2)

        plt.plot(time_command,fy)
        plt.title('comanded fy')
        plt.subplot(2,2,3)

        plt.plot(time_command,tz)
        plt.title('commanded  tz')
        fig.suptitle('FandT')

    def plot_params(self, color, highlight, *args):
        # plot parameters with 1-sigma bound
        time_inertia_est = self.time_inertia_est
        param_est = self.param_est
        est_cov = self.est_cov

        cur_fig = []
        if len(args) != 0:
            cur_fig = args[0]
            plt.figure(cur_fig.number)
        else:
            plt.figure()

        # dual plot
        if len(args) == 2:
            iter = args[1]  # {0 or 4}

            for i in range(1, 5):  # [m, cx, cy, I_zz]
                ax = plt.subplot(2, 4, i+iter)
                # plot obtained parameter estimates
                plt.plot(time_inertia_est, param_est[:, i - 1], color=color)
                plt.hlines(self.param_ground_truth[i-1], time_inertia_est[0], time_inertia_est[-1])
                ax.set_xlabel("Time [s]")
                # ax.legend(['1', '2', '3', '4', '5'])
                if i==1:
                    ax.set_ylabel("$\hat{m}\ $ [kg]")
                elif i==2:
                    ax.set_ylabel("$c_x\ $ [m]")
                    ax.set_ylim(-0.2, 0.2)
                elif i==3:
                    ax.set_ylabel("$c_y\ $ [m]")
                    ax.set_ylim(-0.2, 0.2)
                elif i==4:
                    ax.set_ylabel("$I_{zz}\ $ [kg-m$^2$]")
                    ax.set_ylim(0.0, 1.0)

                # get covar
                err_plus = param_est[:, i - 1] + np.sqrt(est_cov[:, i - 1])
                err_minus = param_est[:, i - 1] - np.sqrt(est_cov[:, i - 1])
                plt.fill_between(time_inertia_est, err_minus, err_plus, color=highlight)
            plt.suptitle("Parameter Estimates")

        # normal plot
        else:
            for i in range(1, 5):  # [m, cx, cy, I_zz]
                ax = plt.subplot(2, 2, i)
                # plot obtained parameter estimates
                plt.plot(time_inertia_est, param_est[:, i - 1], color=color)
                plt.hlines(self.param_ground_truth[i-1], time_inertia_est[0], time_inertia_est[-1])
                ax.set_xlabel("Time [s]")
                # ax.legend(['1', '2', '3', '4', '5'])
                if i==1:
                    ax.set_ylabel("$\hat{m}\ $ [kg]")
                    ax.set_ylim(15, 33)
                elif i==2:
                    ax.set_ylabel("$\hat{c}_x\ $ [m]")
                    ax.set_ylim(-0.2, 0.2)
                elif i==3:
                    ax.set_ylabel("$\hat{c}_y\ $ [m]")
                    ax.set_ylim(-0.2, 0.2)
                elif i==4:
                    ax.set_ylabel("$\hat{I}_{zz}\ $ [kg-m$^2$]")
                    ax.set_ylim(0.0, 1.0)

                # get covar
                err_plus = param_est[:, i - 1] + np.sqrt(est_cov[:, i - 1])
                err_minus = param_est[:, i - 1] - np.sqrt(est_cov[:, i - 1])
                plt.fill_between(time_inertia_est, err_minus, err_plus, color=highlight)

        return cur_fig

        # plt.figure()
        # for i in range(1, 5):
        #     plt.subplot(2, 2, i)
        #     # plot obtained parameter estimates
        #     plt.plot(time_inertia_est, est_cov[:, i - 1])
        # plt.suptitle("Covariance")

    """
        # Plotting for diagnostics, ignore for now.
        # plot ekf response vs param_est estimates for debugging.
        plt.figure()
        for i in range(1, 3):
        	plt.subplot(2, 2, i)
        	# plot obtained positions
        	plt.plot(time_ekf, positions_tracked[:, i - 1])
            plt.plot(time_inertia_est, pose_est[:, i - 1])
            plt.legend(["localization ekf", "param_est_est"])
        plt.suptitle("Debugging - Tracked trajectory and est traj, positions")

        # plot ekf response vs param_est estimates for debugging.
        plt.figure()
        for i in range(1, 3):
            plt.subplot(2, 2, i)
            # plot obtained positions
            plt.plot(time_ekf, velocity_tracked[:, i - 1])
            plt.plot(time_inertia_est, twist_est[:, i - 1])
            plt.legend(["localization ekf", "param_est_est"])
        plt.suptitle("Debugging - Tracked trajectory and est traj, velocities")
        plt.show()

    """

if __name__ == '__main__':
    reader = BagReader()

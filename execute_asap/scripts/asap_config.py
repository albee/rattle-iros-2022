#!/usr/bin/env python

"""
asap_config.py

Global variables for sim and hardware.

Keenan Albee and Charles Oestreich, 2021
MIT Space Systems Laboratory
"""
import rospy
import rospkg
from asap_helper import create_namespaced_topics


# Globals for sim and hardware.
rospack = rospkg.RosPack()
DATA_PATH = rospack.get_path("data")

# -------------------------------------------------------------------------------------------------------------
# directory for rosbags to go to in simulation (hardware uses Ames convention, located in /data/bags/...)
BAG_PATH_SIM = DATA_PATH + "/output/rosbags/"  # must end in /, dir must exist!

# rosbag name
ROSBAG_NAME = "reswarm_standard_rosbag"  # this gets overwritten to use Ames' standard naming convention!


# -------------------------------------------------------------------------------------------------------------
# Node launch files for ASAP nodes
ASAP_PRIMARY_LAUNCH_PATH = rospack.get_path("execute_asap") + "/launch/asap_primary.launch"
ASAP_SECONDARY_LAUNCH_PATH = rospack.get_path("execute_asap") + "/launch/asap_secondary.launch"


# -------------------------------------------------------------------------------------------------------------
# Node list for shutdown for ASAP nodes
NODE_LIST_SIM_PRIMARY = ["primary_coordinator", "casadi_nmpc", "traj_smoother", "uc_bound", "z_poly_calc",
                         "inv_fam", "info_rich_traj", "param_est", "primary_dmpc_iface", "primary_dmpc_ctl_node", "rattle_coordinator",
                         "rattle_rrt", "rattle_acado_planner", "primary_repeater"]
NODE_LIST_HARDWARE_PRIMARY = ["primary_coordinator", "casadi_nmpc", "traj_smoother", "uc_bound", "z_poly_calc",
                              "inv_fam", "info_rich_traj", "param_est", "primary_dmpc_iface", "primary_dmpc_ctl_node", "rattle_coordinator",
                              "rattle_rrt", "rattle_acado_planner", "primary_repeater"]
NODE_LIST_SIM_SECONDARY = ["secondary_dmpc_iface", "secondary_dmpc_ctl_node", "secondary_coordinator", "secondary_repeater"]
NODE_LIST_HARDWARE_SECONDARY = ["secondary_dmpc_iface", "secondary_dmpc_ctl_node", "secondary_coordinator", "secondary_repeater"]


# -------------------------------------------------------------------------------------------------------------
# Topics to record TODO: sim namespacing should link up with role from execute_asap

# DMPC topics
DMPC_PRIMARY = ["gnc/ekf", "bumble/gnc/ekf", "loc/pose", "loc/twist", "gnc/ctl/command",
                "gs/data", "bumble/gs/data", "mob/flight_mode", "reswarm/acado_status",
                "reswarm/dmpc_status"]
DMPC_SECONDARY = ["gnc/ekf", "queen/gnc/ekf", "loc/pose", "loc/twist", "gnc/ctl/command",
                  "gs/data", "queen/gs/data", "mob/flight_mode", "reswarm/acado_status",
                  "reswarm/dmpc_status"]
DMPC_HW_PRIMARY_TOPICS = create_namespaced_topics(DMPC_PRIMARY, sim=False)
DMPC_HW_SECONDARY_TOPICS = create_namespaced_topics(DMPC_SECONDARY, sim=False)
DMPC_SIM_PRIMARY_TOPICS = create_namespaced_topics(DMPC_PRIMARY, sim=True, bee_name="queen")
DMPC_SIM_SECONDARY_TOPICS = create_namespaced_topics(DMPC_SECONDARY, sim=True, bee_name="bumble")

# Localization topics
LOC_TOPICS = ["loc/ar/features", "loc/ml/features", "loc/of/features", "graph_loc/state", "sparse_mapping/pose"]
LOC_HW_TOPICS = create_namespaced_topics(LOC_TOPICS, sim=False)

# RATTLE topics
RATTLE_PRIMARY = ["rattle/test_instruct", "rattle/rrt_high_level/status", "rattle/nmpc_acado_planner/status",
                  "rattle/nmpc_ctl/status", "rattle/rrt/path/posearray", "rattle/rrt/path/twistarray", "rattle/local/path/posearray",
                  "rattle/local/path/twistarray", "rattle/local/path/wrencharray", "rattle/rrt/params", "rattle/obstacles",
                  "rattle/rrt/path/posearray", "rattle/rrt/path/twistarray", "rattle/local/info_plan_instruct", "rattle/local/path/posearray",
                  "rattle/local/path/twistarray", "rattle/local/path/wrencharray", "rattle/local/path_ctl/posearray", "rattle/local/path_ctl/twistarray",
                  "rattle/local/path_ctl/wrencharray", "rattle/local/info_plan_instruct_start", "rattle/local/psi", "rattle/local/weights"]
RATTLE_HW_PRIMARY_TOPICS = create_namespaced_topics(RATTLE_PRIMARY, sim=False)
RATTLE_SIM_PRIMARY_TOPICS = create_namespaced_topics(RATTLE_PRIMARY, sim=True, bee_name="queen")

# FSW topics
FSW_TOPICS = ["gnc/ctl/command", "gnc/ekf", "gnc/ctl/setpoint", "hw/pmc/command", "hw/imu", "mob/flight_mode", "mob/inertia"]
FSW_HW_TOPICS = create_namespaced_topics(FSW_TOPICS, sim=False)
FSW_SIM_TOPICS = create_namespaced_topics(FSW_TOPICS, sim=True, bee_name="queen")

# Tube MPC topics
TUBE_MPC = ["reswarm/uc_bound/uc_bound", "reswarm/tube_mpc/traj", "reswarm/tube_mpc/debug", "reswarm/tube_mpc/mrpi"]
TUBE_MPC_HW_TOPICS = create_namespaced_topics(TUBE_MPC, sim=False)
TUBE_MPC_SIM_TOPICS = create_namespaced_topics(TUBE_MPC, sim=True, bee_name="queen")

# Misc ReSWARM topics
RESWARM_MISC = ["reswarm/lqrrrt/traj", "reswarm/planner_lqrrrt/status", "inv_fam/appliedFandT", "mob/inertia_est", "reswarm/status"]
RESWARM_MISC_HW_TOPICS = create_namespaced_topics(RESWARM_MISC, sim=False)
RESWARM_MISC_SIM_TOPICS = create_namespaced_topics(RESWARM_MISC, sim=True, bee_name="queen")

# Sim only topics (ground truth)
SIM_ONLY = ["loc/truth/pose", "loc/truth/twist"]
SIM_ONLY_TOPICS = create_namespaced_topics(SIM_ONLY, sim=True, bee_name="queen")

# -------------------------------------------------------------------------------------------------------------
# Topics for recording: TOPICS_$SIM_$ROBOT_ROLE
# Note: oddball topics that NEVER use namespacing must be explicitly listed in sim
TOPICS_HARDWARE_PRIMARY = FSW_HW_TOPICS + LOC_HW_TOPICS + DMPC_HW_PRIMARY_TOPICS + RATTLE_HW_PRIMARY_TOPICS + TUBE_MPC_HW_TOPICS + RESWARM_MISC_HW_TOPICS

TOPICS_SIM_PRIMARY = " /reswarm/tube_mpc/mrpi /reswarm/planner_lqrrrt/status /rattle/test_instruct" \
                     + SIM_ONLY_TOPICS + FSW_SIM_TOPICS + DMPC_SIM_PRIMARY_TOPICS + RATTLE_SIM_PRIMARY_TOPICS + TUBE_MPC_SIM_TOPICS + RESWARM_MISC_SIM_TOPICS

TOPICS_HARDWARE_SECONDARY = FSW_HW_TOPICS + LOC_HW_TOPICS + DMPC_HW_SECONDARY_TOPICS + RESWARM_MISC_HW_TOPICS

TOPICS_SIM_SECONDARY = SIM_ONLY_TOPICS + FSW_SIM_TOPICS + DMPC_SIM_SECONDARY_TOPICS + RESWARM_MISC_SIM_TOPICS

# print(TOPICS_HARDWARE_PRIMARY)
# print(TOPICS_SIM_PRIMARY)
# print(TOPICS_HARDWARE_SECONDARY)
# print(TOPICS_SIM_SECONDARY)

# unused for reswarm
TOPICS_HARDWARE_PRIMARY_EXTRA = ""
TOPICS_SIM_PRIMARY_EXTRA = ""

if __name__ == "__main__":
    print(TOPICS_HARDWARE_PRIMARY)

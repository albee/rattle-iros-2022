#!/bin/bash

# Astrobee FSW topics
rostopic echo -b $1 -p /gnc/ekf > $2/ekf.csv
rostopic echo -b $1 -p /gnc/ctl/command > $2/ctl_command.csv
rostopic echo -b $1 -p /hw/imu > $2/imu.csv
rostopic echo -b $1 -p /hw/pmc/command > $2/pmc_command.csv

# TD status topic
rostopic echo -b $1 -p /td/status > $2/td_status.csv

# SLAM topics
rostopic echo -b $1 -p /td/mit_slam/chaser_pose > $2/mit_slam_chaser_pose.csv
rostopic echo -b $1 -p /td/mit_slam/chaser_twist > $2/mit_slam_chaser_twist.csv
rostopic echo -b $1 -p /td/mit_slam/target_pose > $2/mit_slam_target_pose.csv
rostopic echo -b $1 -p /td/mit_slam/target_twist > $2/mit_slam_target_twist.csv
rostopic echo -b $1 -p /td/mit_slam/delta_pose > $2/mit_slam_delta_pose.csv
rostopic echo -b $1 -p /td/mit_slam/loop_delta_pose > $2/mit_slam_loop_delta_pose.csv
rostopic echo -b $1 -p /td/mit_slam/pose_GC > $2/mit_slam_pose_GC.csv
rostopic echo -b $1 -p /td/mit_slam/pose_GT > $2/mit_slam_pose_GT.csv
rostopic echo -b $1 -p /td/mit_slam/t_WT > $2/mit_slam_t_WT.csv
rostopic echo -b $1 -p /td/mit_slam/target_centroid > $2/mit_slam_centroid.csv
rostopic echo -b $1 -p /td/mit_slam/timing_info > $2/mit_slam_timing_info.csv

# Motion planner topics
rostopic echo -b $1 -p /td/traj_gen_dlr/chaser_traj_pose0 > $2/motion_plan_chaser_pose0.csv
rostopic echo -b $1 -p /td/traj_gen_dlr/chaser_traj_twist0 > $2/motion_plan_chaser_twist0.csv
rostopic echo -b $1 -p /td/traj_gen_dlr/target_traj_pose0 > $2/motion_plan_target_pose0.csv
rostopic echo -b $1 -p /td/traj_gen_dlr/target_traj_twist0 > $2/motion_plan_target_twist0.csv
rostopic echo -b $1 -p /td/traj_gen_dlr/mpdebug_path > $2/motion_plan_debug_path.csv
rostopic echo -b $1 -p /td/traj_gen_dlr/mpdebug_numbers > $2/motion_plan_debug_numbers.csv
rostopic echo -b $1 -p /td/traj_gen_dlr/mpdebug_violations > $2/motion_plan_debug_violations.csv

# Tube-MPC topics
rostopic echo -b $1 -p /td/tube_mpc/debug > $2/mpc_debug.csv
rostopic echo -b $1 -p /td/tube_mpc/mrpi > $2/mpc_mrpi.csv
rostopic echo -b $1 -p /td/tube_mpc/traj > $2/mpc_traj.csv
rostopic echo -b $1 -p /td/uc_bound/uc_bound > $2/mpc_uc_bound.csv



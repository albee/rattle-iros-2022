%{
# main_read_bags.m

Read rosbag data for Astrobee topics.
Uses MATLAB's rosbag interface.

## Usage

main_read_bags.m:
(1) Select bag settings. (Each set of bags has different topic naming.)
(2) Read in desired data. get_data_xxx.m files specify which bags to use.

main_plot_bags.m:
(3) (Optional) Select trajectory type to use (ROAM). Plots updated or
non-updated trajectory.
(4) Perform plotting. Select desired plotting types.

Keenan Albee, 2021.
%}
clear;

%% -----------------------------------------------------------------
% (1) Select bag settings.
%-------------------------------------------------------------------
% Read settings
bag_set_name = 'granite_lab_testing_10_21_21';
%{
{sim, ...
granite_lab_testing_05_27_21, ...
granite_lab_testing_10_21_21
iss_05_03_21, iss_07_07_21, iss_08_19_21} 
%}
bag_type = 'reswarm';  % {reswarm, roam}
robot_role = 'primary';  % {'primary', 'chaser', 'secondary', 'target'};
robot_name = 'queen';
%{
only needed if bag_set_name = 'sim';
 {queen, bumble, bsharp, honey, wannabee, etc.}
%}


ROAM_PATH = '~/workspaces/astrobee-ws-td/freeflyer-shared-td';
addpath(genpath('../../../develop/data/scripts'));  % add tumble plotting
addpath(genpath(strcat(ROAM_PATH, '/develop/data/scripts')));  % add tumble plotting
make_plots_purdy();

%% -----------------------------------------------------------------
% (2) Read in desired data. Select chaser/primary or target/secondary.
%-------------------------------------------------------------------
%% get topic prefix
if bag_set_name == "sim"
  topic_prefix = "/" + robot_name;
else
  topic_prefix = "";
end

% get correct data
if bag_set_name == "sim"
  [ROSBAG_PATH, ROSBAG_NAME] = get_data_sim(robot_role);  %{chaser, target, primary , secondary}
elseif bag_set_name == "granite_lab_testing_05_27_21"
  [ROSBAG_PATH, ROSBAG_NAME] = get_data_granite_lab_testing_05_27_21(robot_role);  %{chaser, target}
elseif bag_set_name == "iss_05_03_21"
  [ROSBAG_PATH, ROSBAG_NAME] = get_data_iss_05_03_21(robot_role);  %{chaser, target}
elseif bag_set_name == "iss_07_07_21"
  [ROSBAG_PATH, ROSBAG_NAME] = get_data_iss_07_07_21(robot_role);  %{primary, secondary}
elseif bag_set_name == "iss_08_19_21"
  [ROSBAG_PATH, ROSBAG_NAME] = get_data_iss_08_19_21(robot_role);  %{primary, secondary}
elseif bag_set_name == "granite_lab_testing_10_21_21"
  [ROSBAG_PATH, ROSBAG_NAME] = get_data_granite_lab_testing_10_21_21(robot_role);  %{primary, secondary}
end


%% Read bag data (for either target/secondary or chaser/primary)
pbd = read_bags(ROSBAG_PATH + ROSBAG_NAME, topic_prefix, bag_type);  % prepped bag data (pbd)

if bag_type == "reswarm"
  track = pbd.track;
  traj = pbd.traj;
  ekf_msgs = pbd.ekf_msgs;
  mpc_msgs = pbd.mpc_msgs;
  ctl_msgs = pbd.ctl_msgs;
  t_traj_start = pbd.t_traj_start;
  t_delay = pbd.t_delay;
  bag = pbd.bag;
  bag_info = pbd.bag_info;
  mrpi_msgs = pbd.mrpi_msgs;
  uc_msgs = pbd.uc_msgs;
  fam_msgs = pbd.fam_msgs;
  rrt_traj_msgs = pbd.rrt_traj_msgs;
  
  traj_to_plot = traj;
  rattle_msgs_struct = pbd.rattle_msgs_struct
  
elseif bag_type == "roam"
  track = pbd.track;
  traj = pbd.traj;
  ekf_msgs = pbd.ekf_msgs;
  mpc_msgs = mpc_msgs.track;
  ctl_msgs = pbd.ctl_msgs;
  t_traj_start = pbd.t_traj_start;
  t_delay = pbd.t_delay;
  bag = pbd.bag;
  bag_info = pbd.bag_info;
  dlr_msgs = pbd.dlr_msgs;
  traj_body = pbd.traj_body;
  traj_updated_msgs = pbd.traj_updated_msgs;
  q_targ_0_hist = pbd.q_targ_0_hist;
  mrpi_msgs = pbd.mrpi_msgs;
  q_targ_slam_hist = pbd.q_targ_slam_hist;
  t_slam_start = pbd.t_slam_start;
  uc_msgs = pbd.uc_msgs;
  fam_msgs = pbd.fam_msgs;
  loc_features = pbd.loc_features;
   
  traj_to_plot = traj;
end
  

function make_plots_purdy()
  set(0,'DefaultAxesFontName', 'CMU Serif')
  set(0, 'DefaultLineLineWidth', 3);
  set(groot,'defaulttextinterpreter','latex');  
  set(groot, 'defaultAxesTickLabelInterpreter','latex');  
  set(groot, 'defaultLegendInterpreter','latex');
  set(0,'defaultAxesFontSize', 32)
end
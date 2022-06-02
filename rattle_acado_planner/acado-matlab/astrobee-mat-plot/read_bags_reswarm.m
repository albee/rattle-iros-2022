%{
read_bags_reswarm.m

Read rosbag data for Astrobee GNC and custom topics.
Uses MATLAB's rosbag interface.

rosbag_path: full path of the bag, including its name, e.g., ~/home/data/my_bag.bag
topic_prefix: topic prefix used if simulation data, e.g., '/honey'

pbd : bag data struct.

Keenan Albee, 2021.
%}

function [pbd] = read_bags_reswarm(rosbag_path, topic_prefix)

     if ~exist('topic_prefix','var')
          topic_prefix = "";
     end

    %Read topics
    % read in rosbag
    [bag, bag_info] = read_bag_info(rosbag_path);

    % access arbitrary bag info
    disp("Topics are:");
    {bag_info.Topics.Topic}

    %---------------------------------------------------------------------
    %% Astrobee FSW
    topic = strcat(topic_prefix, '/gnc/ekf');
    ekf_msgs = get_bag_msgs(bag, topic);
    ekf_msgs = correct_ekf_msgs(ekf_msgs);
    
    topic = strcat(topic_prefix, '/gnc/ctl/command');
    ctl_msgs = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/hw/pmc/command');
    fam_msgs = get_bag_msgs(bag, topic);
    
    %---------------------------------------------------------------------
    %% Localization
    topic = strcat(topic_prefix, '/loc/ml/features');
    loc_msgs = get_bag_msgs(bag, topic);
    
    %---------------------------------------------------------------------
    %% Standard ReSWARM topics
    topic = strcat(topic_prefix, '/reswarm/status');
    reswarm_status_list = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/reswarm/tube_mpc/debug');
    mpc_msgs = get_bag_msgs(bag, topic);  % msgs as a matlab struct
    
    topic = '/reswarm/tube_mpc/mrpi';
    mrpi_msgs = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/reswarm/tube_mpc/traj');
    traj = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/reswarm/lqrrrt/traj');
    rrt_traj_msgs = get_bag_msgs(bag, topic);

    topic = strcat('/reswarm/uc_bound/uc_bound');
    uc_msgs = get_bag_msgs(bag, topic);
    
    %%
%     test1 = get_bag_msgs(bag, 'rattle/rrt/params')
%     test1{1}

    %% RATTLE topics
    rattle_topics = {
      '/rattle/test_instruct' '/rattle/rrt_high_level/status' '/rattle/nmpc_acado_planner/status' ...
      '/rattle/nmpc_ctl/status' '/rattle/rrt/path/posearray' '/rattle/rrt/path/twistarray' ...
      '/rattle/local/path/posearray' '/rattle/local/path/twistarray' '/rattle/local/path/wrencharray' ...
      '/rattle/rrt/params' '/rattle/obstacles' '/rattle/rrt/path/posearray' ...
      '/rattle/rrt/path/twistarray' '/rattle/nmpc_planner/info_plan_instruct' '/rattle/local/path/posearray' ...
      '/rattle/local/path/twistarray' '/rattle/local/path/wrencharray' '/rattle/local/path_ctl/posearray' ...
      '/rattle/local/path_ctl/twistarray' '/rattle/local/path_ctl/wrencharray'};
    
    rattle_msgs_struct = {};  % convert to struct with fieldnames of e.g., rattle_msgs_struct.rattle_test_instruct
    for topic = rattle_topics
      topic_field = regexprep(topic, '^\/', '');
      topic_field = regexprep(topic_field, '/', '_');
      topic = strcat(topic_prefix, topic);
      rattle_msgs_struct.(topic_field{1}) = get_bag_msgs(bag, topic);
    end
    
    % get reference trajectory (traj)
    try
        [rows, cols] = traj{1}.Layout.Dim.Size;
        traj = reshape(traj{1}.Data, cols, rows)';  % [n x 20], t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
    catch
        traj = [];
    end

    % get tracked state from gnc/ekf
    track = []; % x y z xd yd zd qx qy qz qw wx wy wz
    for i = 1:1:length(ekf_msgs)
        track(i, :) = ekf2state(ekf_msgs{i});
    end
    
    % identify important epoch times/elapsed times
    t_traj_start = get_traj_start_time(reswarm_status_list);  % start of trajectory (epoch time)
    t_begin = stamp2time(ekf_msgs{1}.Header.Stamp);  % start of entire test (epoch time)
    t_delay = t_traj_start - t_begin;  % delay time before traj began

    % pack the bag data
    pbd = { };
    pbd.track = track;
    pbd.traj = traj;
    pbd.ekf_msgs = ekf_msgs;
    pbd.mpc_msgs = mpc_msgs;
    pbd.ctl_msgs = ctl_msgs;
    pbd.t_traj_start = t_traj_start;
    pbd.t_delay = t_delay;
    pbd.bag = bag;
    pbd.bag_info = bag_info;
    pbd.mrpi_msgs = mrpi_msgs;
    pbd.uc_msgs = uc_msgs;
    pbd.fam_msgs = fam_msgs;
    pbd.rrt_traj_msgs = rrt_traj_msgs;
    pbd.rattle_msgs_struct = rattle_msgs_struct;
end

function epoch_time = get_traj_start_time(reswarm_status_list)
    %{
    Get epoch time of when trajectory tracking started
    %}
    epoch_time = 0.0;
    for i = 1:1:length(reswarm_status_list)
        reswarm_status = reswarm_status_list{i};
        if strcmp(reswarm_status.ControlMode, 'track') || strcmp(reswarm_status.ControlMode, 'track_tube')
            epoch_time = stamp2time(reswarm_status.Stamp);
            break;
        end
    end
end

function [bag, bag_info] = read_bag_info(rosbag_path)
    bag = rosbag(rosbag_path);
    bag_info = rosbag('info', rosbag_path);
end

function [bag_topic_msgs] = get_bag_msgs(bag, topic)
    topic_struct = select(bag, 'Topic', topic);
    bag_topic_msgs = readMessages(topic_struct, 'DataFormat', 'struct');
end

function [state] = ekf2state(ekf_msg)
    %{ 
    Convert an ekf/msg to a standard reswarm state vector
    Outputs:
    x y z xd yd zd qx qy qz qw wx wy wz
    %}
    state = [ekf_msg.Pose.Position.X, ekf_msg.Pose.Position.Y, ekf_msg.Pose.Position.Z, ...
             ekf_msg.Velocity.X, ekf_msg.Velocity.Y, ekf_msg.Velocity.Z, ...
             ekf_msg.Pose.Orientation.X, ekf_msg.Pose.Orientation.Y, ekf_msg.Pose.Orientation.Z, ekf_msg.Pose.Orientation.W, ...
             ekf_msg.Omega.X, ekf_msg.Omega.Y, ekf_msg.Omega.Z];
end

function [input] = wrench2input(wrench)
    input = [wrench.Force.X, wrench.Force.Y, wrench.Force.Z, ...
     wrench.Torque.X, wrench.Torque.Y, wrench.Torque.Z];
end

function quat = pose2quat(pose)
    quat = [pose.Orientation.X; pose.Orientation.Y; 
        pose.Orientation.Z; pose.Orientation.W];
end

function time = stamp2time(stamp)
    %{
    Inputs:
    stamp - ROS stamp
    start - start time in seconds
    %}
    s = stamp.Sec;
    ns = stamp.Nsec;
    time = double(s) + double(ns)/1E9;
end

function ekf_msgs = correct_ekf_msgs(ekf_msgs)
    %{
    Fix ekf_msgs to get rid of weird quaternion flips
    %}

    % get all data in a matrix
    track = []; % x y z xd yd zd qx qy qz qw wx wy wz
    for i = 1:1:length(ekf_msgs)
        track(i, :) = ekf2state(ekf_msgs{i});
    end
    track(:, 7:10) = quat_flipper(track(:, 7:10));
    
    % now update ekf_msgs
    for i = 1:1:length(ekf_msgs)
        ekf_msgs{i}.Pose.Orientation.X = track(i, 7);
        ekf_msgs{i}.Pose.Orientation.Y = track(i, 8);
        ekf_msgs{i}.Pose.Orientation.Z = track(i, 9);
        ekf_msgs{i}.Pose.Orientation.W = track(i, 10);
    end
end

function q_astrobee = quat_flipper(q_astrobee)
    %{
    q_astrobee is an (X by 4) quaternion array, [qx qy qz qw; ...]. 

    Signs are adjusted to match Astrobee convention.
    %}
    for i = 2:length(q_astrobee(:,1))
        if (norm(q_astrobee(i,:) - q_astrobee(i-1,:)) > norm(-q_astrobee(i,:) - q_astrobee(i-1,:)))
            q_astrobee(i,:) = -q_astrobee(i,:);
        end
    end
    
    % also flip ALL quaternions for reswarm
    q_astrobee = -q_astrobee;
end

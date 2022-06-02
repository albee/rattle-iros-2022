%{
Read rosbag data for Astrobee GNC and custom topics.
Uses MATLAB's rosbag interface.

Keenan Albee, 2021.
%}

function [track, traj, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_delay, bag, bag_info, dlr_msgs, traj_body, traj_updated_msgs, ...
    q_targ_0_hist, mrpi_msgs, q_targ_slam_hist, t_slam_start, uc_msgs, fam_msgs, loc_features] = read_bags_roam(rosbag_path, topic_prefix)

     if ~exist('topic_prefix','var')
          topic_prefix = "";
     end

    addpath(genpath('../../../develop/data/scripts'));  % add tumble plotting

    %% Read topics
    % read in rosbag
    [bag, bag_info] = read_bag_info(rosbag_path);

    % access arbitrary bag info
    disp("Topics are:");
    {bag_info.Topics.Topic}

    % read in topic
    topic = strcat(topic_prefix, '/td/tube_mpc/debug');
    msgs = get_bag_msgs(bag, topic);  % msgs as a matlab struct

    topic = strcat(topic_prefix, '/gnc/ekf');
    ekf_msgs = get_bag_msgs(bag, topic);
    ekf_msgs = correct_ekf_msgs(ekf_msgs);

    topic = strcat(topic_prefix, '/gnc/ctl/command');
    ctl_msgs = get_bag_msgs(bag, topic);
    
%     topic = strcat(topic_prefix, '/gnc/ctl/setpoint');
%     targ = get_bag_msgs(bag, topic)
    
    topic = strcat(topic_prefix, '/hw/pmc/command');
    fam_msgs = get_bag_msgs(bag, topic);

    topic = strcat('/td/uc_bound/uc_bound');
    uc_msgs = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/loc/of/features');
    loc_features = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/td/tube_mpc/debug');
    mpc_msgs = get_bag_msgs(bag, topic);

    topic = strcat(topic_prefix, '/td/status');
    td_status_list = get_bag_msgs(bag, topic);
%     size(td_status_list)
%     td_status_list{100}

    topic = strcat(topic_prefix, '/td/tube_mpc/debug');
    mpc_msgs = get_bag_msgs(bag, topic);
    
    topic = '/td/mit_slam/target_pose';
    q_targ_slam_msgs = get_bag_msgs(bag, topic);
    q_targ_slam_hist = [];
    for i = 1:1:length(q_targ_slam_msgs)
       q_targ_slam_hist(i, :) = pose2quat(q_targ_slam_msgs{i}.Pose)';
    end
    
    topic = '/td/tube_mpc/mrpi';
    mrpi_msgs = get_bag_msgs(bag, topic);
    
    topic = strcat(topic_prefix, '/td/tube_mpc/traj');
    traj = get_bag_msgs(bag, topic);

    try
        topic = '/td/tube_mpc/traj_body';
        traj_body = get_bag_msgs(bag, topic);
        [rows, cols] = traj_body{1}.TrajBody.Layout.Dim.Size;
        % [qx; qy; qz; qw] ... matrix
        q_targ_0_hist = [[traj_body{1}.QTarg0Hist.X]; [traj_body{1}.QTarg0Hist.Y]; [traj_body{1}.QTarg0Hist.Z]; [traj_body{1}.QTarg0Hist.W]]';
        traj_body = reshape(traj_body{1}.TrajBody.Data, cols, rows)';  % [n x 20], t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
    catch
        traj_body = [];
        q_targ_0_hist = [];
    end
    
    try
        topic = strcat(topic_prefix, '/td/tube_mpc/traj_updated');
        traj_updated_msgs = get_bag_msgs(bag, topic); %list of [n x 20], t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
    catch
        traj_updated_msgs = [];
    end
        
    try
        dlr_msgs = get_dlr_msgs(bag, topic_prefix);  % list of topics
    catch
        dlr_msgs = [];
    end
    
    try
        [rows, cols] = traj{1}.Layout.Dim.Size;
        traj = reshape(traj{1}.Data, cols, rows)';  % [n x 20], t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
    catch
        traj = [];
    end

    % identify start time of the trajectory
    t_traj_start = get_traj_start_time(td_status_list);
    t_begin = stamp2time(ekf_msgs{1}.Header.Stamp);
    t_delay = t_traj_start - t_begin;  % delay time before traj began
    t_slam_start = get_slam_start_time(td_status_list);

    % get gnc/ekf state
    track = []; % x y z xd yd zd qx qy qz qw wx wy wz
    for i = 1:1:length(ekf_msgs)
        track(i, :) = ekf2state(ekf_msgs{i});
    end
    
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
    pbd.dlr_msgs = dlr_msgs;
    pbd.traj_body = traj_body;
    pbd.traj_updated_msgs = traj_updated_msgs;
    pbd.q_targ_0_hist = q_targ_0_hist;
    pbd.mrpi_msgs = mrpi_msgs;
    pbd.q_targ_slam_hist = q_targ_slam_hist;
    pbd.t_slam_start = t_slam_start;
    pbd.uc_msgs = uc_msgs;
    pbd.fam_msgs = fam_msgs;
    pbd.loc_features = loc_features;
end

function epoch_time = get_slam_start_time(td_status_list)
    epoch_time = 0.0;
    for i = 1:1:length(td_status_list)
        td_status = td_status_list{i};
        if td_status.SlamActivate == true
            epoch_time = stamp2time(td_status.Stamp);
            break;
        end
    end
end

function dlr_msgs = get_dlr_msgs(bag, topic_prefix)
    dlr_msgs = {};
    dlr_topics = {
    strcat(topic_prefix, '/td/traj_gen_dlr/chaser_traj_pose0'),
    strcat(topic_prefix, '/td/traj_gen_dlr/chaser_traj_twist0'),
    strcat(topic_prefix, '/td/traj_gen_dlr/mpdebug_numbers'),
    strcat(topic_prefix, '/td/traj_gen_dlr/mpdebug_path'),
    strcat(topic_prefix, '/td/traj_gen_dlr/mpdebug_violations'),
    strcat(topic_prefix, '/td/traj_gen_dlr/target_traj_pose0'),
    strcat(topic_prefix, '/td/traj_gen_dlr/target_traj_twist0')};

    for i = 1:1:length(dlr_topics)
        msgs = get_bag_msgs(bag, dlr_topics{i});
        dlr_msgs{end+1} = msgs{1};
    end
end

function epoch_time = get_traj_start_time(td_status_list)
    %{
    Get epoch time of when trajectory tracking started
    %}
    epoch_time = 0.0;
    for i = 1:1:length(td_status_list)
        td_status = td_status_list{i};
        if strcmp(td_status.TdControlMode, 'track') || strcmp(td_status.TdControlMode, 'track_tube')
            epoch_time = stamp2time(td_status.Stamp);
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
    Convert an ekf/msg to a standard TD state vector
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

function target_est_att = quat_flipper(target_est_att)
    %{
    target_est_att is an (X by 4) quaternion array, [qx qy qz qw; ...]
    %}
    for i = 2:length(target_est_att(:,1))
        if (norm(target_est_att(i,:) - target_est_att(i-1,:)) > norm(-target_est_att(i,:) - target_est_att(i-1,:)))
            target_est_att(i,:) = -target_est_att(i,:);
        end
    end
end

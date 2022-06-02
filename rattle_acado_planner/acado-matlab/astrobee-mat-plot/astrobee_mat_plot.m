%{
# main_plot_bags.m

Plots rosbag data for Astrobee topics.
Select plotting options below, reqiures prior read-in
of data using main_read_bags.m.

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

%% -----------------------------------------------------------------
% (4) Perform plotting.
%-------------------------------------------------------------------
%% Plot Bryce lqrrrt
x_lqr_traj = plot_lqr_rrt(rrt_traj_msgs);

%% Number of features
plot_features(loc_features, t_traj_start);
% plot_fam(t_feats, feats)

%% Plot gnc/ekf history(3D)
state = plot_gnc_ekf(ekf_msgs);

%% Plot ref traj history (3D)
r0_mat = traj(:, 2:4);
R0_mat = traj(:, 8:11);
des_traj = r0_mat;
anim_tumble(r0_mat, R0_mat, des_traj)

%% Plot MPC timing
[avg_t, sig_t] = plot_mpc_timing(mpc_msgs)

%% Plot ekf timing
plot_ekf_timing(ekf_msgs)

%% Plot pos, vel, and 3d traj multi-plot (only for MP runs)
plot_mp_multi(traj_to_plot)

%% Plot track and trajectory
plot_track_and_traj(track, traj_to_plot)

%% Plot tracks
% figure;
% sgtitle('Standard vs. Robust MPC Tracking', 'FontSize', 40);
    
% % standard
subplot(1,2,1);
hold on;
plot(track(1600:end, 2), track(1600:end, 3), 'color', [0, 0.4470, 0.7410]);

title('Standard MPC Position')
xlabel('y [m]');
ylabel('z [m]');
set(gca,'ydir','reverse')
grid on;
axis equal;

% robust
% subplot(1,2,2);
% hold on;
% plot(track(1800:end, 2), track(1800:end, 3), 'color', [0, 0.4470, 0.7410]);
% 
% title('Robust MPC Position')
% xlabel('y [m]');
% ylabel('z [m]');
% set(gca,'ydir','reverse')
% grid on;
% axis equal;

%% Plot FAM
plot_fam(fam_msgs, t_traj_start)

%% Plot MPC (translational ctl history)
plot_mpc(track, traj_to_plot, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_delay)

%% Plot tube MPC (translational ctl history)
plot_tube_mpc(track, traj_to_plot, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_delay)

%% Plot z0 (tube MPC runs only)
plot_z0_from_msgs(mrpi_msgs, mpc_msgs, ekf_msgs, track, traj_to_plot,  t_traj_start)

%% Plot attitude control history, with quats
plot_att_controller(track, traj_to_plot, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_delay, 'quat');

%% Plot attitude control history, with Euler angles
t_traj_end = t_traj_start + 40.0;
plot_att_controller(track, traj_to_plot, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_traj_end, t_delay, 'euler');

%% Plot actual commanded inputs
plot_gnc_ctl_command(ctl_msgs, mpc_msgs, t_delay, t_traj_start)

%% Compute RMSE
x_des = ctl_state2loc_state(traj')';
t_x_des = traj(:, 1);

x_real = track;
t_x_real = get_t_from_epoch(ekf_msgs, t_traj_start);

t_start = 10.0;
t_end = 135.0;
[RMS, vec_RMS] = calc_RMS_error(t_start, t_end, t_x_des, t_x_real, x_des, x_real, false, true, 'translation');  % sync times, translation states only
RMS

%% TODO: compute normal to trajectory error


function plot_z0_from_msgs(mrpi_msgs, mpc_msgs, ekf_msgs, track, traj,  t_traj_start)
    % AZ and bZ
    AZ= array2mat(mrpi_msgs{1}.AZ)';
    bZ = array2mat(mrpi_msgs{1}.BZ)';

    % X_hist
    X_hist = track(:, 1:6);  % ~62.5 Hz, starts @ t_begin
    epoch_times = [];
    for i = 1:1:length(ekf_msgs)
        epoch_times(i) = stamp2time(ekf_msgs{i}.Header.Stamp);
    end
    [~, idx] = min(abs(t_traj_start - epoch_times));
    t_X_hist = epoch_times(idx:end) - t_traj_start;
    X_hist = X_hist(idx:end, :);  % X_hist trimmed for start time
        
    % x0_mpc_hist
    x0_mpc_hist = [];
    epoch_times = [];
    for i = 1:1:length(mpc_msgs)
        x0_mpc_hist(i, :) = array2mat(mpc_msgs{i}.XNom)';
        epoch_times(i) = stamp2time(mpc_msgs{i}.Header.Stamp);
    end
    [~, idx] = min(abs(t_traj_start - epoch_times));
    t_x0_hist = epoch_times(idx:end) - t_traj_start;
    num_t = length(t_x0_hist);  % length of history
    x0_mpc_hist = x0_mpc_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time

    X_des_hist = traj(:, 2:7);
    
    % correct for ISS coordinates to view
    X_hist_zeroed = X_hist;
    X_hist_zeroed(:, 1:3) = X_hist_zeroed(:, 1:3) - [10.9 -8.15 4.9];
    
    x0_mpc_hist_zeroed = x0_mpc_hist;
    x0_mpc_hist_zeroed(:, 1:3) = x0_mpc_hist_zeroed(:, 1:3) - [10.9 -8.15 4.9];

    X_des_hist_zeroed = X_des_hist;
    X_des_hist_zeroed(:, 1:3) = X_des_hist_zeroed(:, 1:3) - [10.9 -8.15 4.9];
    
    plot_z0(AZ, bZ, X_hist_zeroed', x0_mpc_hist_zeroed', X_des_hist_zeroed')
end


function plot_tube_mpc(track, traj, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_delay)
    % X_hist
    X_hist = track(:, 1:6);  % ~62.5 Hz, starts @ t_begin
    epoch_times = [];
    for i = 1:1:length(ekf_msgs)
        epoch_times(i) = stamp2time(ekf_msgs{i}.Header.Stamp);
    end
    [~, idx] = min(abs(t_traj_start - epoch_times));
    t_X_hist = epoch_times(idx:end) - t_traj_start;
    X_hist = X_hist(idx:end, :);  % X_hist trimmed for start time

    % U_hist
    U_hist = [];  % ~5 Hz, starts @ t_regulate
    U_dr_hist = [];
    U_mpc_hist = [];
    epoch_times = [];
    for i = 1:1:length(mpc_msgs)
        U_hist(i, :) = wrench2input(mpc_msgs{i}.Wrench);
        U_dr_hist(i, :) = [mpc_msgs{i}.U0Dr.X; mpc_msgs{i}.U0Dr.Y; mpc_msgs{i}.U0Dr.Z];
        U_mpc_hist(i, :) = [mpc_msgs{i}.U0Mpc.X; mpc_msgs{i}.U0Mpc.Y; mpc_msgs{i}.U0Mpc.Z];
        epoch_times(i) = stamp2time(mpc_msgs{i}.Header.Stamp);
    end
    [~, idx] = min(abs(t_traj_start - epoch_times));
    t_U_hist = epoch_times(idx:end) - t_traj_start;
    num_t = length(t_U_hist);  % lenght of history
    U_hist = U_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time
    U_dr_hist = U_dr_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time
    U_mpc_hist = U_mpc_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time
    
    % X_des_hist
    t_X_des_hist = traj(:, 1);  % 5 Hz, starts @ t_traj_start
    X_des_hist = traj(:, 2:7);

    % U_ctl
    U_ctl = [];  % 62.5 Hz, starts @ t_regulate
    epoch_times = [];
    for i = 1:1:length(ctl_msgs)
        U_ctl(i, :) = wrench2input(ctl_msgs{i}.Wrench);
        epoch_times(i) = stamp2time(ctl_msgs{i}.Header.Stamp);
    end
    % we have NO timing data from TS1 bags for famcommand---assume 62.5 Hz
    idx = floor(t_delay*62.5);
    U_ctl = U_ctl(idx:end, :);
    t_U_ctl = (0:1:size(U_ctl, 1)-1)/62.5;
    
    % plot_mpc_and_ctl_command(t_U_hist, t_U_ctl, U_hist', U_ctl')

    X_hist_zeroed = X_hist;
    X_hist_zeroed(:, 1:3) = X_hist_zeroed(:, 1:3) - [10.9 -8.15 4.9];

    X_des_hist_zeroed = X_des_hist;
    X_des_hist_zeroed(:, 1:3) = X_des_hist_zeroed(:, 1:3) - [10.9 -8.15 4.9];


    plot_tube_mpc_multi_start(t_X_hist, t_U_hist, t_U_hist, t_U_hist, t_X_des_hist, ...
        X_hist_zeroed', U_hist', U_dr_hist', U_mpc_hist', X_des_hist_zeroed');
end


function plot_mpc(track, traj, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_delay)
    % X_hist
    X_hist = track(:, 1:6);  % ~62.5 Hz, starts @ t_begin
    [t_X_hist, idx] = get_t_from_epoch(ekf_msgs, t_traj_start);
    X_hist = X_hist(idx:end, :);  % X_hist trimmed for start time

    % U_hist
    U_hist = [];  % ~5 Hz, starts @ t_regulate
    epoch_times = [];
    for i = 1:1:length(mpc_msgs)
        U_hist(i, :) = wrench2input(mpc_msgs{i}.Wrench);
    end
    [t_U_hist, idx] = get_t_from_epoch(mpc_msgs, t_traj_start);
    num_t = length(t_U_hist);  % lenght of history
    U_hist = U_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time

    % X_des_hist
    epoch_times = [];
    for i = 1:1:length(ekf_msgs)
        epoch_times(i) = stamp2time(ekf_msgs{i}.Header.Stamp);
    end
    t_X_des_hist = traj(:, 1);  % 5 Hz, starts @ t_traj_start
    X_des_hist = traj(:, 2:7);
    
    t_final = epoch_times(end) - t_traj_start;
    [~, idx] = min(abs(t_final - t_X_des_hist));  % get idx of final t_X_des_hist
    t_X_des_hist = t_X_des_hist(1:idx);
    X_des_hist = X_des_hist(1:idx, :);

    % U_ctl
    U_ctl = [];  % 62.5 Hz, starts @ t_regulate
    epoch_times = [];
    for i = 1:1:length(ctl_msgs)
        U_ctl(i, :) = wrench2input(ctl_msgs{i}.Wrench);
        epoch_times(i) = stamp2time(ctl_msgs{i}.Header.Stamp);
    end
    % we have NO timing data from TS1 bags for famcommand---assume 62.5 Hz
    idx = floor(t_delay*62.5);
    U_ctl = U_ctl(idx:end, :);
    t_U_ctl = (0:1:size(U_ctl, 1)-1)/62.5;
    
    % plot_mpc_and_ctl_command(t_U_hist, t_U_ctl, U_hist', U_ctl')

    X_hist_zeroed = X_hist;
    X_hist_zeroed(:, 1:3) = X_hist_zeroed(:, 1:3) - [10.9 -9.65 4.9];

    X_des_hist_zeroed = X_des_hist;
    X_des_hist_zeroed(:, 1:3) = X_des_hist_zeroed(:, 1:3) - [10.9 -9.65 4.9];

    plot_mpc_multi_start(t_X_hist, t_U_hist, t_X_des_hist, X_hist_zeroed', U_hist', X_des_hist_zeroed')
    
%     figure;
%     hold on;
%     plot(U_ctl(:, 1));
%     plot(U_ctl(:, 2));
%     plot(U_ctl(:, 3));
end


function X_hist = plot_att_controller(track, traj, ekf_msgs, mpc_msgs, ctl_msgs, t_traj_start, t_traj_end, t_delay, format)
    %{
    Plot attitude controller performance. Show computed inputs, desired
    trajectory, and gnc/ekf reference trajectory.
    Inputs:
    track, gnc/ekf with real history in Ames convention
        % [n x 13], x y z xd yd zd qx qy qz qw wx wy wz
    traj, td/tube_mpc/traj with desired trajectory
        [n x 20], t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
        Target is [n x 14] in same order, but only angular data
    mpc_msgs, /td/tube_mpc/debug with pd controller inputs
    %}

    % X_hist
    X_hist = track(:, 7:13);  % ~62.5 Hz, starts @ t_begin
    [t_X_hist, idx, idxf] = get_t_from_epoch(ekf_msgs, t_traj_start, t_traj_end);
    X_hist = X_hist(idx:idxf, :);  % X_hist trimmed for start time
    
    % U_hist
    U_hist = [];  % ~5 Hz, starts @ t_regulate
    for i = 1:1:length(mpc_msgs)
        U_hist(i, :) = wrench2input(mpc_msgs{i}.Wrench);
    end
    [t_U_hist, idx] = get_t_from_epoch(mpc_msgs, t_traj_start);
    num_t = length(t_U_hist);  % lenght of history
    U_hist = U_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time

    % X_des_hist
%     t_X_des_hist = traj(:, 1);  
    t_X_des_hist = 0:0.2:0.2*(size(traj, 1)-1);  % 5 Hz, starts @ t_traj_startf
    X_des_hist = traj(:, 8:14);  % [qx qy qz qw wx wy wz]
    
    if format == "quat"
      plot_att_multi_start(t_X_hist, t_U_hist, t_X_des_hist, X_hist', U_hist', X_des_hist', "quat");
    elseif format == "euler"
      plot_att_multi_start(t_X_hist, t_U_hist, t_X_des_hist, X_hist', U_hist', X_des_hist', "euler");
    else
      disp("Invalid format.");
    end
end


function [avg_comp_time, std_comp_time] = plot_mpc_timing(mpc_msgs)
    comp_time = [];
    for i = 1:1:length(mpc_msgs)
        comp_time(i) = mpc_msgs{i}.CasadiCompTime.Data;
    end
    plot(comp_time)
    avg_comp_time = mean(comp_time)
    std_comp_time = std(comp_time)
    yline(avg_comp_time);
    title('MPC Comp Time')
    ylabel('[s]')
    xlabel('[-]')
end


function mat = array2mat(array_msg)
    data = array_msg.Data;
    [rows, cols] = array_msg.Layout.Dim.Size;
    mat = reshape(data, cols, rows);
end
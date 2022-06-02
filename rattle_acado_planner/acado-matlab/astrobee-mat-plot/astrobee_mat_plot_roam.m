%{
# main_plot_bags_roam.m

Plots rosbag data for Astrobee topics.
Select plotting options below, reqiures prior read-in
of data using main_read_bags.m. Contains special ROAM-only
functions.

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
% (3) (Optional) Select trajectory type to use (ROAM).
%-------------------------------------------------------------------
%% Use online-updated ref trajectory
[online_traj, traj_updated_vec] = get_online_traj(mpc_msgs, traj_updated_msgs);
traj_to_plot = online_traj;

%% DLR stuff (only for MP runs)
mpMIT_args = dlr_msgs{3}.Data(3:30);
TRAJ_PATH_ = dlr_msgs{4}.Data;

%% Plot track, traj and traj_body
traj_updated = plot_traj_and_traj_body(track, traj_to_plot, traj_body, traj_updated_msgs, mpc_msgs);

%% Plot quats of Target
plot_quats_targ(t_traj_start, t_slam_start, q_targ_0_hist, q_targ_slam_hist)

%% Animate traj_updated (3D)
animate_traj_updated(traj_updated);


function animate_traj_updated(traj_updated)
    %{
    Animate the updated trajectory
    %}
    anim_traj(traj_updated);
end

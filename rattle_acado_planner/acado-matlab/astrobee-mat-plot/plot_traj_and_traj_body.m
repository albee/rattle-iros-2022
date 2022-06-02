%{
Plot track and des traj. Includes online-updated trajectory (ROAM).
%}
function traj_updated = plot_traj_and_traj_body(track, traj, traj_body, traj_updated_vec, mpc_msgs)
	ISS_xyz = [10.9, -8.15, 4.9];
    
  figure;
  view(45, 45);
  axis equal;
  hold on;

  plot3(track(:, 1) - ISS_xyz(1), track(:, 2) - ISS_xyz(2), track(:, 3) - ISS_xyz(3), 'color', 'black');

  scatter3(traj(1, 2) - ISS_xyz(1), traj(1, 3) - ISS_xyz(2), traj(1, 4) - ISS_xyz(3), 40, 'green', 'filled')
  plot3(traj(:, 2) - ISS_xyz(1), traj(:, 3) - ISS_xyz(2), traj(:, 4) - ISS_xyz(3), 'color', 'green');
  scatter3(traj_body(1, 2), traj_body(1, 3) - 1.5, traj_body(1, 4), 40, 'red', 'filled')
  plot3(traj_body(:, 2), traj_body(:, 3) - 1.5, traj_body(:, 4), 'color', 'red');

  % find when tracking begins
  track_start_idx = get_track_start_idx(mpc_msgs);

  [rows, cols] = traj_updated_vec{track_start_idx}.Layout.Dim.Size;
  traj_updated = reshape(traj_updated_vec{track_start_idx}.Data, cols, rows)';
  scatter3(traj_updated(1, 2) - ISS_xyz(1), traj_updated(1, 3) - ISS_xyz(2), traj_updated(1, 4) - ISS_xyz(3), 40, 'blue', 'filled')
  plot3(traj_updated(:, 2) - ISS_xyz(1), traj_updated(:, 3) - ISS_xyz(2), traj_updated(:, 4) - ISS_xyz(3), 'color', 'blue', 'LineWidth', 1);
end
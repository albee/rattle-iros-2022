function [online_traj, traj_updated_vec] = get_online_traj(mpc_msgs, traj_updated_msgs)
  %{
  Get the online updated trajectory actually used for tracking (when online
  updates are enabled)

  Outputs:
  online_traj: The online-updated trajectory.
  traj_updated_vec: Array of trajectory updated online.
  %}
  track_start_idx = get_track_start_idx(mpc_msgs);
  online_traj = [];
  traj_updated_vec = {};
  
  [rows, cols] = traj_updated_msgs{track_start_idx}.Layout.Dim.Size;
  traj_len = rows  % get the length of a full trajectory
  
  out_idx = 1;
  
  for i = track_start_idx:1:track_start_idx+traj_len-1
    [rows, cols] = traj_updated_msgs{i}.Layout.Dim.Size;
    traj_updated = reshape(traj_updated_msgs{i}.Data, cols, rows)';
    online_traj(out_idx, :) = traj_updated(out_idx, :);  % row equivalent to where we are on the trajectory
    traj_updated_vec{out_idx} = traj_updated;
    out_idx = out_idx + 1;
  end
end
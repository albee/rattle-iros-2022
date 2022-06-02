function [t_hist, idx, idxf] = get_t_from_epoch(msgs_w_stamp, t_traj_start, t_traj_end)
  %{
  Get t_hist, the vector of times from ROS-formatted timestamps.
  msgs_w_stamp: msgs that have a Header.Stamp
  t_traj_start: epoch time at which to begin
  t_traj_end: epoch time at which to stop

  t_hist: history of times, adjusted for start time
  idx: index of start time
  %}
  if exist('t_traj_end','var')
    epoch_times = [];
    for i = 1:1:length(msgs_w_stamp)
        epoch_times(i) = stamp2time(msgs_w_stamp{i}.Header.Stamp);
    end
    [~, idx] = min(abs(t_traj_start - epoch_times));
    [~, idxf] = min(abs(t_traj_end - epoch_times));
    t_hist = epoch_times(idx:idxf) - t_traj_start;
  else
    epoch_times = [];
    for i = 1:1:length(msgs_w_stamp)
        epoch_times(i) = stamp2time(msgs_w_stamp{i}.Header.Stamp);
    end
    [~, idx] = min(abs(t_traj_start - epoch_times));
    t_hist = epoch_times(idx:end) - t_traj_start;
    idxf = -1;
  end
end
function plot_gnc_ctl_command(ctl_msgs, mpc_msgs, t_delay, t_traj_start)
  % U_ctl
  U_ctl = [];  % 62.5 Hz, starts @ t_regulate
  epoch_times = [];
  for i = 1:1:length(ctl_msgs)
      U_ctl(i, :) = wrench2input(ctl_msgs{i}.Wrench);
      epoch_times(i) = stamp2time(ctl_msgs{i}.Header.Stamp);
  end
  % we have NO timing data from TS1 bags for famcommand---assume 62.5 Hz
%   idx = floor(t_delay*62.5);
%   U_ctl = U_ctl(idx:end, :);
%   U_ctl
%   t_U_ctl = (0:1:size(U_ctl, 1)-1)/62.5;
  t_U_ctl = epoch_times - t_traj_start;
  size(t_U_ctl)
  size(U_ctl)
  figure;
  hold on;
  plot(t_U_ctl, U_ctl(:, 1), 'r');
  plot(t_U_ctl, U_ctl(:, 2), 'g');
  plot(t_U_ctl, U_ctl(:, 3), 'b');
  
  
  % U_hist
  U_hist = [];  % ~5 Hz, starts @ t_regulate
  epoch_times = [];
  for i = 1:1:length(mpc_msgs)
      U_hist(i, :) = wrench2input(mpc_msgs{i}.Wrench);
      epoch_times(i) = stamp2time(mpc_msgs{i}.Header.Stamp);
  end
  [~, idx] = min(abs(t_traj_start - epoch_times));
  t_U_hist = epoch_times(idx:end) - t_traj_start;
  num_t = length(t_U_hist);  % lenght of history
  U_hist = U_hist(idx:idx+num_t-1, :);  % U_hist trimmed for start time
  
  plot(t_U_hist, U_hist(:, 1), 'r:');
  plot(t_U_hist, U_hist(:, 2), 'g:');
  plot(t_U_hist, U_hist(:, 3), 'b:');
  
  
  figure;
  hold on;
  title('Torques')
  plot(t_U_ctl, U_ctl(:, 4), 'r');
  plot(t_U_ctl, U_ctl(:, 5), 'g');
  plot(t_U_ctl, U_ctl(:, 6), 'b');
  plot(t_U_hist, U_hist(:, 4), 'r:');
  plot(t_U_hist, U_hist(:, 5), 'g:');
  plot(t_U_hist, U_hist(:, 6), 'b:');
end
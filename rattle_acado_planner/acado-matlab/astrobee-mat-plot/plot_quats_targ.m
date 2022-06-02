%{
Plot target quaternions (ROAM).
%}
function plot_quats_targ(t_traj_start, t_slam_start, q_targ_0_hist, q_targ_slam_hist)
  %{ Plot quaternions of the target (history and SLAM)
  %}
  
  % the q_targ poses go out after slam_activate is true
  % the motion plan starts when track control_mode is activated
  figure;
  hold on;
  t_extra = t_traj_start - t_slam_start;
  idx = floor(t_extra/0.2);
  q_targ_0_hist(1, :)
  plot(q_targ_0_hist(:, 1), 'black');
  plot(q_targ_0_hist(:, 4), 'black');
  plot(q_targ_slam_hist(idx:end, 1), 'red');
  plot(q_targ_slam_hist(idx:end, 4), 'red');
end
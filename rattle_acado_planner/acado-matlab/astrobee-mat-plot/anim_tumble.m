%{
 Takes in a matrix of r0 and a matrix of q and animates. Time varies with the
 row.

Input
r0_mat - matrix of [x, y, z] position
R0_mat - matrix of [qx, qy, qz, qw] quaternions of orientation. Scalar LAST, represents B wrt I
des_traj - in INERTIAL frame [x, y, z]. Plots separately.
varargin - frames, indices to downselect plotting to.
%}
function anim_tumble(r0_mat, R0_mat, des_traj, varargin)
  fig = figure('units','normalized','outerposition',[0 0 1 1]);
  grid on
  view(115, 45);
  hold on
  axis equal;
  start_pos = r0_mat(1, :);
  axis([-1, 1, -1, 1, -1, 1]*0.6 + [start_pos(1), start_pos(1), start_pos(2), start_pos(2), start_pos(3), start_pos(3)]);
  
  x = des_traj(:,1);
  y = des_traj(:,2);
  z = des_traj(:,3);
  plot3(x, y, z, 'black', 'Linewidth', 3);
  pause(1.0)
  
  if length(varargin) == 1
    frames = varargin{1};
    anim_FK3(r0_mat, R0_mat, fig, frames);
  else
    anim_FK3(r0_mat, R0_mat, fig);
  end
end
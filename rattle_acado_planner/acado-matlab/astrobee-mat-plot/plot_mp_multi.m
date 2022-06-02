function plot_mp_multi(traj)
  %{
  Plot nominal motion plan, in position, velocity, and 3D

  traj=
  [t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
  ...
  ]
  %}
  % Positions
  close all;
  figure
  hold on;
  set(0,'defaultAxesFontSize', 24)
  
  XYZ_0 = traj(1, 2:4);
  
  % position plots
  subplot(2,2,1);
  hold on;
  title('Chaser Nominal Positions');
  plot(traj(:,1), traj(:, 2)  - XYZ_0(1), 'color', 'red');
  plot(traj(:,1), traj(:, 3)  - XYZ_0(2), 'color', 'green');
  plot(traj(:,1), traj(:, 4) - XYZ_0(3), 'color', 'blue');
  legend('$x$', '$y$', '$z$');
  ylabel('Position $[m]$')
  xlabel('Time $[s]$')

  
  %velocity plots
  subplot(2,2,2);
  hold on;
  title('Chaser Nominal Velocities');
  plot(traj(:,1), traj(:, 5), 'color', 'red');
  plot(traj(:,1), traj(:, 6), 'color', 'green');
  plot(traj(:,1), traj(:, 7), 'color', 'blue');
  legend('$\dot{x}$', '$\dot{y}$', '$\dot{z}$');
  ylabel('Velocity $[\frac{m}{s}]$')
  xlabel('Time $[s]$')
  
  %subplot
  subplot(2,2,[3,4]);
  hold on;
  view(290, 35);
  axis equal;
  title('3D Chaser Nominal Motion Plan');
  plot3(traj(:, 2) - XYZ_0(1), traj(:, 3) - XYZ_0(2), traj(:, 4) - XYZ_0(3), 'color', 'black');  % substract start pos
  h1 = scatter3(traj(1, 2) - XYZ_0(1), traj(1, 3) - XYZ_0(2), traj(1, 4) - XYZ_0(3), 40, 'green', 'filled');
  h2 = scatter3(traj(end, 2) - XYZ_0(1), traj(end, 3) - XYZ_0(2), traj(end, 4) - XYZ_0(3), 40, 'red', 'filled');
  legend([h1, h2], 'Start Pos.', 'Final Pos.');
  xlabel('Position, x $[m]$')
  ylabel('Position, y $[m]$')
  zlabel('Position, z $[m]$')
end
function plot_mpc_multi_start(t_X_hist, t_U_hist, t_X_des_hist, X_hist, U_hist, X_des_hist)
  %{
  Plot MPC history, showing inputs and states.
  t_*_hist = time vector of length equal to corresponding X or U vector.
  X_hist = [x y z xd yd zd] (note time is by column)
  U_hist = [u1 u2 u3]
  X_des_hist = [x y z xd yd zd], desired
  %}
  % Positions
  close all;
  figure
  hold on;
  sgtitle('Translational MPC Inputs and Localization Estimated State (Position)', 'FontSize', 40)

  
  % x
  subplot(2,2,1);
  xlim([0, t_X_des_hist(end)]);
  hold on
  
  plot(t_X_hist, X_hist(1,:), 'r');
  plot(t_X_des_hist, X_des_hist(1,:), ':r');
  
  xlabel('t [s]');
  title('x');
  ylabel('position [m]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(1,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$r_x$', '$r_{x, des}$', '$F_x$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
  
  % y
  subplot(2,2,2);
  xlim([0, t_X_des_hist(end)]);
  hold on
  
  plot(t_X_hist, X_hist(2,:), 'g');
  plot(t_X_des_hist, X_des_hist(2,:), ':g');
  
  xlabel('t [s]');
  title('y');
  ylabel('position [m]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(2,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$r_y$','$r_{y, des}$','$F_y$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
  
  % z
  subplot(2,2,3);
  xlim([0, t_X_des_hist(end)]);
  hold on
  
  plot(t_X_hist, X_hist(3,:), 'b');
  plot(t_X_des_hist, X_des_hist(3,:), ':b');
  
  xlabel('t [s]');
  title('z');
  ylabel('position [m]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(3,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$r_z$','$r_{z, des}$','$F_z$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
  
  % Velocities
  figure
  hold on
  sgtitle('Translational MPC Inputs and Localization Estimated State (Vel.)', 'FontSize', 40)
  
  %x
  subplot(2,2,1);
  xlim([0, t_X_des_hist(end)]);
  hold on
  plot(t_X_hist, X_hist(4,:), 'r');  
  plot(t_X_des_hist, X_des_hist(4,:), ':r');
    
  xlabel('t [s]');
  title('$\dot{x}$');
  ylabel('velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(1,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{r}_x$','$\dot{r}_{x, des}$','$F_x$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
      
  %y
  subplot(2,2,2);
  xlim([0, t_X_des_hist(end)]);
  hold on
  plot(t_X_hist, X_hist(5,:), 'g');
  plot(t_X_des_hist, X_des_hist(5,:), ':g');
      
  xlabel('t [s]');
  title('$\dot{y}$');
  ylabel('velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(2,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{r}_y$','$\dot{r}_{y, des}$','$F_y$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
       
  %z
  subplot(2,2,3);
  xlim([0, t_X_des_hist(end)]);
  hold on
  plot(t_X_hist, X_hist(6,:), 'b');
  plot(t_X_des_hist, X_des_hist(6,:), ':b');
 
  xlabel('t [s]');
  title('$\dot{z}$');
  ylabel('velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(3,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{r}_z$','$\dot{r}_{z, des}$','$F_z$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
    
  %{
  % actual
  plot(t_X_hist, X_hist(1,:), 'r');
  plot(t_X_hist, X_hist(2,:), 'g');
  plot(t_X_hist, X_hist(3,:), 'b');
  
  % desired
  plot(t_X_des_hist, X_des_hist(1,:), ':r');
  plot(t_X_des_hist, X_des_hist(2,:), ':g');
  plot(t_X_des_hist, X_des_hist(3,:), ':b');
  
  xlabel('t [s]');
  title('Trajectory-Following With Linearly-Constrained MPC, 3DOF Double Integrator');
  ylabel('position [m], velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(1,:), '-r*', 'LineWidth', 2)
  stairs(t_U_hist, U_hist(2,:), '-g*', 'LineWidth', 2)
  stairs(t_U_hist, U_hist(3,:), '-b*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('x1','x2','x3','$x1_{des}$','$x2_{des}$','$x3_{des}$','u1','u2','u3');
  
  % Velocities
  figure
  hold on
  % actual
  plot(t_X_hist, X_hist(4,:), 'r');
  plot(t_X_hist, X_hist(5,:), 'g');
  plot(t_X_hist, X_hist(6,:), 'b');
  
  % desired
  plot(t_X_des_hist, X_des_hist(4,:), ':r');
  plot(t_X_des_hist, X_des_hist(5,:), ':g');
  plot(t_X_des_hist, X_des_hist(6,:), ':b');
  
  xlabel('t [s]');
  title('Trajectory-Following With Linearly-Constrained MPC, 3DOF Double Integrator');
  ylabel('position [m], velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_U_hist, U_hist(1,:), '-r*', 'LineWidth', 2)
  stairs(t_U_hist, U_hist(2,:), '-g*', 'LineWidth', 2)
  stairs(t_U_hist, U_hist(3,:), '-b*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{x1}$', '$\dot{x2}$', '$\dot{x3}$', '$\dot{x1}_{des}$','$\dot{x2}_{des}$','$\dot{x3}_{des}$','u1','u2','u3');
  %}
end
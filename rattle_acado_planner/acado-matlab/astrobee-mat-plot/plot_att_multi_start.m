function plot_att_multi_start(t_X_hist, t_U_hist, t_X_des_hist, X_hist, U_hist, X_des_hist, format)
  %{
  Plot attitude control history, showing inputs and states.
  t_*_hist = time vector of length equal to corresponding X or U vector.
  X_hist = [qx qy qz qw wx wy wz] (note time is by column)
  U_hist = [u1 u2 u3]
  X_des_hist = [qx qy qz qw wx wy wz] (note time is by column), desired
  format = {quat, euler}
  %}
  % Orientations
%   close all;
  figure
  sgtitle('Rotational Control Inputs and Localization Estimated State (Att.)', 'FontSize', 40)
  
  if format == "quat"
    % qx
    subplot(2,2,1);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist(1,:), 'r');  % actual
    plot(t_X_des_hist, X_des_hist(1,:), ':r');  % desired  
    xlabel('t [s]');
    title('$q_x$');
    ylabel('orientation [-]');

    yyaxis right
    stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    set(gca,'FontSize',24);
    legend('$qx$', '$qx_{des}$', '$\tau_x$','$\tau_y$','$\tau_z$');
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';

    % qy
    subplot(2,2,2);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist(2,:), 'g');
    plot(t_X_des_hist, X_des_hist(2,:), ':g');

    xlabel('t [s]');
    title('$q_y$');
    ylabel('orientation [-]');

    yyaxis right
    stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    set(gca,'FontSize',24);
    legend('$qy$', '$qy_{des}$', '$\tau_x$','$\tau_y$','$\tau_z$');

    % qz
    subplot(2,2,3);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist(3,:), 'b');
    plot(t_X_des_hist, X_des_hist(3,:), ':b');

    xlabel('t [s]');
    title('$q_z$');
    ylabel('orientation [-]');

    yyaxis right
    stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    set(gca,'FontSize',24);
    legend('$qz$', '$qz_{des}$', '$\tau_x$','$\tau_y$','$\tau_z$');

    % qw
    subplot(2,2,4);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist(4,:), 'black');
    plot(t_X_des_hist, X_des_hist(4,:), ':black');

    xlabel('t [s]');
    title('$q_w$');
    ylabel('orientation [deg]');

    yyaxis right
    stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
    stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    set(gca,'FontSize',24);
    legend('$qw$', '$qw_{des}$', '$\tau_x$','$\tau_y$','$\tau_z$');
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    
  elseif format == "euler"  % uses XYZ body-fixed
    % produce [thx thy thz; ...] matrix
    X_des_hist_eul = quat2eul(X_des_hist(1:4, :)', 'ZYX')'*180/pi;  % is this intrinsic or extrinsic??
    X_hist_eul = quat2eul(X_hist(1:4, :)', 'ZYX')'*180/pi;  % is this intrinsic or extrinsic??
    
    % theta_x
    subplot(2,2,1);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist_eul(1,:), 'r');  % actual
    plot(t_X_des_hist, X_des_hist_eul(1,:), ':r');  % desired  
    xlabel('t [s]');
    title('$\theta_x$');
    ylabel('orientation [deg]');
    ylim([-180, 180]);

    yyaxis right
%     stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
%     stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
%     stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    set(gca,'FontSize',24);
    legend('$\theta_x$', '$\theta_{x,des}$', '$\tau_x$','$\tau_y$','$\tau_z$');
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';

    % theta_y
    subplot(2,2,2);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist_eul(2,:), 'g');
    plot(t_X_des_hist, X_des_hist_eul(2,:), ':g');

    xlabel('t [s]');
    title('$\theta_y$');
    ylabel('orientation [deg]');
    ylim([-180, 180]);

    yyaxis right
%     stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
%     stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
%     stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    set(gca,'FontSize',24);
    legend('$\theta_y$', '$\theta_{y,des}$', '$\tau_x$','$\tau_y$','$\tau_z$');

    % theta_z
    subplot(2,2,3);
    xlim([0, t_X_hist(end)]);
    hold on
    plot(t_X_hist, X_hist_eul(3,:), 'b');
    plot(t_X_des_hist, X_des_hist_eul(3,:), ':b');

    xlabel('t [s]');
    title('$\theta_z$');
    ylabel('orientation [deg]');
    ylim([-180, 180]);

    yyaxis right
%     stairs(t_U_hist, U_hist(4,:), '-r*', 'LineWidth', 2)
%     stairs(t_U_hist, U_hist(5,:), '-g*', 'LineWidth', 2)
%     stairs(t_U_hist, U_hist(6,:), '-b*', 'LineWidth', 2)
    ylabel('input [N-m]');
    ylim([-0.15, 0.15]);
    ax = gca;
    ax.YAxis(1).Color = 'k';
    ax.YAxis(2).Color = 'k';
    set(gca,'FontSize',24);
    legend('$\theta_z$', '$\theta_{z,des}$', '$\tau_x$','$\tau_y$','$\tau_z$');
  end

  % Angular Velocities
  figure
  sgtitle('Rotational Control Inputs and Localization Estimated State (Ang. Vel.)', 'FontSize', 40)

  subplot(2,2,1);
  xlim([0, t_X_hist(end)]);
  hold on;
  plot(t_X_hist, X_hist(5,:), 'r');  % actual
  plot(t_X_des_hist, X_des_hist(5,:), ':r');  % desired

  xlabel('t [s]');
  title('$\omega_x$');
  ylabel('velocity [rad/s]');
  yyaxis right
  ylabel('input [N-m]');
  stairs(t_U_hist, U_hist(4,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{\omega_x}$', '$\dot{\omega_x}_{des}$', '$\tau_x$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';

  subplot(2,2,2);
  xlim([0, t_X_hist(end)]);
  hold on;
  plot(t_X_hist, X_hist(6,:), 'g');
  plot(t_X_des_hist, X_des_hist(6,:), ':g');

  xlabel('t [s]');
  title('$\omega_y$');
  ylabel('velocity [rad/s]');
  yyaxis right
  ylabel('input [N-m]');
  stairs(t_U_hist, U_hist(5,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{\omega_y}$', '$\dot{\omega_y}_{des}$','$\tau_y$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';

  subplot(2,2,3);
  xlim([0, t_X_hist(end)]);
  hold on;
  plot(t_X_hist, X_hist(7,:), 'b');
  plot(t_X_des_hist, X_des_hist(7,:), ':b');

  xlabel('t [s]');
  title('$\omega_z$');
  ylabel('velocity [rad/s]');
  yyaxis right
  ylabel('input [N-m]');
  stairs(t_U_hist, U_hist(6,:), '-black*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{\omega_z}$', '$\dot{\omega_z}_{des}$','$\tau_z$');
  ax = gca;
  ax.YAxis(1).Color = 'k';
  ax.YAxis(2).Color = 'k';
end
function plot_fam(fam_msgs, t_traj_start)
  %{
  Plot the FAM commands directly issued to each thruster.

  t_fam: [1xn] of times
  fam_cmds: [12 x n] of nozzle commands, 1-255
  %}
  fam_cmds = [];
  t_fam = [];
  for i = 1:1:length(fam_msgs)
     t_fam(i) = stamp2time(fam_msgs{i}.Header.Stamp) - t_traj_start;
     fam_cmds(:, i) = reshape([fam_msgs{i}.Goals.NozzlePositions], [12,1]);
  end

  % Positions
  figure
  hold on;
  
  % x
  subplot(2,2,1);
  xlim([0, t_fam(end)]);
  hold on
  stairs(t_fam, fam_cmds(1,:), '-black*', 'LineWidth', 2)  % all x nozzles
  stairs(t_fam, fam_cmds(8,:), '-black*', 'LineWidth', 2)  % all x nozzles
  stairs(t_fam, fam_cmds(7,:), '-black*', 'LineWidth', 2)  % all x nozzles
  stairs(t_fam, fam_cmds(9,:), '-black*', 'LineWidth', 2)  % all x nozzles
  set(gca,'FontSize',24);
  title('x');
  xlabel('t [s]');
  ylabel('nozzle PWM [-]');
  
  % y
  subplot(2,2,2);
  xlim([0, t_fam(end)]);
  hold on
  stairs(t_fam, fam_cmds(3,:), '-black*', 'LineWidth', 2)  % all y nozzles
  stairs(t_fam, fam_cmds(4,:), '-black*', 'LineWidth', 2)  % all y nozzles
  stairs(t_fam, fam_cmds(9,:), '-black*', 'LineWidth', 2)  % all y nozzles
  stairs(t_fam, fam_cmds(10,:), '-black*', 'LineWidth', 2)  % all y nozzles
  set(gca,'FontSize',24);
  title('y');
  xlabel('t [s]');
  ylabel('nozzle PWM [-]');
  
  % z
  subplot(2,2,3);
  xlim([0, t_fam(end)]);
  hold on
  stairs(t_fam, fam_cmds(5,:), '-black*', 'LineWidth', 2)  % all z nozzles
  stairs(t_fam, fam_cmds(6,:), '-black*', 'LineWidth', 2)  % all z nozzles
  stairs(t_fam, fam_cmds(11,:), '-black*', 'LineWidth', 2)  % all z nozzles
  stairs(t_fam, fam_cmds(12,:), '-black*', 'LineWidth', 2)  % all z nozzles
  set(gca,'FontSize',24);
  title('x');
  xlabel('t [s]');
  ylabel('nozzle PWM [-]');
end

%{
Unit tests for `casadi_3dof_double_integrator` and 
`casadi_3dof_double_integrator_z0`

Necessary setup, using terminal:
(1): `roscore`
(2): `rosrun z_poly_calc z_poly_calc.py`
%}

%% User vars
close all;
clear all;

UNIT_TESTS = 0;
ROSBAG_TOOLS_PATH = '~/repos/thesis-data-analysis/mat-plot/astrobee-mat-plot';

%% ROS networking
rosshutdown
rosinit
% create_and_add_ros_msgs();  % Only required if generating new msg and srv

%% Import CasADi and Python interface
make_plots_purdy();

import casadi.*

addpath(genpath(ROSBAG_TOOLS_PATH));  % plotting tools for Astrobee rosbag data
addpath(genpath('../../data/scripts'));  % trajectory generation scripts

% Only required if using MATLAB's Python
% interface. We will use ROS instead.
% load_python_z_poly_calc()  

%% Set timing parameters, get MPC
T = 5.0; % Time horizon, [s]
N = 5; % Number of control intervals over time horizon, [-]
DT = T/N;  % delta-t, [s]
control_dt = 0.2;
time_factor = round(DT/control_dt);

% ISS-TS1 values
% T = 2.0; % Time horizon, [s]
% N = 10; % Number of control intervals over time horizon, [-]
% DT = T/N;  % delta-t, [s]

% Get CasADi functions
[mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt] = casadi_3dof_double_integrator_mpc(T, N, control_dt);
[tube_mpc_func_casadi, ~] = casadi_3dof_double_integrator_tube_mpc(T, N);

%% Get reference trajectory
DES_TRAJ = 'SINUSOID';  % {'SINUSOID', 'ZERO'}
TF_TRAJ = 10*T;
USE_HARDWARE_TRAJ = 1;  % {0 or 1}
ROSBAG_PATH = "~/repos/thesis-data-analysis/data/roam-bags/iss-test1-05-03-21/chaser/bags/";
ROSBAG_NAME = "411_test_2021-05-03-17-16-25_chaser.bag";

ISS_XYZ = [10.9; -9.65; 4.9];
x0_ISS = [ISS_XYZ; 0; 0; 0];

x0 = [0.1; 0.1; 0.0; 0; 0; 0];
% x0 = [10.85; -9.65; 4.9; 0; 0; 0.0];

[x_des_traj, traj_dt, num_timesteps] = get_trajectory(x0_ISS(1:3), control_dt, DT, T, TF_TRAJ, DES_TRAJ, USE_HARDWARE_TRAJ, ROSBAG_PATH, ROSBAG_NAME);

%% Set up mRPI, input constraints, weights, and gains
% u_mag = [0.4; 0.4; 0.4];
u_mag = [0.15; 0.15; 0.15];
m = 9.5;  % mass (ground units), [kg]
USE_TS1 = 0;  % use ROAM-TS1 values?

[gains_mpc, gains_dr] = get_gains(USE_TS1);

[noise, noise_mpc_dt] = get_noise(time_factor);

% Utight and Z_poly constraints for tube MPC
% DT = 0.2
DT = 1.0
[K_dr, Autight, butight, AZ_actual, bZ_actual] = calc_mRPI_ROS(u_mag, m, DT, noise, gains_dr);
[AZ, bZ] = get_casadi_polytope(AZ_actual, bZ_actual);
% plot_lines(AZ_actual, bZ_actual)

%% Run a simulation
USE_TUBE_MPC = 1;
sim_hist = run_dynamics(x0, u_mag, m, Autight, butight, AZ, bZ, K_dr, gains_mpc, gains_dr, noise, x_des_traj, DT, N, traj_dt, control_dt, num_timesteps, ...
    mpc_func_casadi, tube_mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt, USE_TUBE_MPC);

%% Run a simulation
USE_TUBE_MPC = 0;
sim_hist = run_dynamics(x0, u_mag, m, Autight, butight, AZ, bZ, K_dr, gains_mpc, gains_dr, noise, x_des_traj, DT, N, traj_dt, control_dt, num_timesteps, ...
    mpc_func_casadi, tube_mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt, USE_TUBE_MPC);

%% Plot
% plot_mpc(sim_hist.t_hist, sim_hist.X_hist, sim_hist.U_hist, sim_hist.X_des_hist)
plot_mpc_multi_start(sim_hist.t_hist, sim_hist.t_hist(2:end), sim_hist.t_hist, sim_hist.X_hist, sim_hist.U_hist, sim_hist.X_des_hist)

%% Plot tube MPC
plot_tube_mpc_multi_start(sim_hist.t_hist, sim_hist.t_hist(2:end), sim_hist.t_hist(2:end), sim_hist.t_hist(2:end), sim_hist.t_hist,...
    sim_hist.X_hist, sim_hist.U_hist, sim_hist.U_dr_hist, sim_hist.U_mpc_hist, sim_hist.X_des_hist)
% plot_z0(AZ, bZ, sim_hist.X_hist, sim_hist.x0_mpc_hist, sim_hist.X_des_hist)

%% Unit tests
run_casadi_unit_tests()

%%

function [AZ, bZ] = get_casadi_polytope(AZ_actual, bZ_actual)
    %{
    Convery polytope approximation to 100-length one expected by CasADi
    %}
    AZ = zeros(100, 6);
    bZ = zeros(100, 1);
    len_AZ = size(AZ_actual,1);
    AZ(1:len_AZ, :) = AZ_actual;
    bZ(1:len_AZ, :) = bZ_actual;
end

function [x_des_traj, traj_dt, num_timesteps] = get_trajectory(x0_pos, control_dt, DT, T, TF_TRAJ, DES_TRAJ, USE_HARDWARE_TRAJ, ROSBAG_PATH, ROSBAG_NAME)
    %{
    Create a trajectory, or load one from hardware
    %}
    if USE_HARDWARE_TRAJ  % get hardware trajectory
        x_des_traj = get_traj_from_bag(ROSBAG_PATH, ROSBAG_NAME);
        num_timesteps = ((control_dt*length(x_des_traj)) - T)/control_dt;
    else  % create own trajectory
        x_des_traj = create_des_traj(control_dt, TF_TRAJ, DES_TRAJ, [x0_pos], [0; 0; 0; 1]);
        num_timesteps = (TF_TRAJ-T)/control_dt;

    end
    traj_dt = x_des_traj(2, 1) - x_des_traj(1, 1);
    x_des_traj = x_des_traj(:,2:7);  % clip off extra
    x_des_traj(:, 1:3) = x_des_traj(:, 1:3) - x0_pos';  % reference to 0
end

function [traj] = get_traj_from_bag(ROSBAG_PATH, ROSBAG_NAME)
    %{
    traj := [n x 20], t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
    %}   
    rosbag_path = ROSBAG_PATH + ROSBAG_NAME;
    pbd = read_bags(rosbag_path, '', "roam", "chaser");
    
    %}  
    track = pbd.track;
    traj = pbd.traj;
    ekf_msgs = pbd.ekf_msgs;
    mpc_msgs = pbd.mpc_msgs;
    ctl_msgs = pbd.ctl_msgs;
    t_traj_start = pbd.t_traj_start;
    t_delay = pbd.t_delay;
    bag = pbd.bag;
    bag_info = pbd.bag_info;
    dlr_msgs = pbd.dlr_msgs;
end

function [K_dr, Autight, butight, AZ, bZ] = calc_mRPI_python(mass, DT, noise, gains_dr)
  %{
  Call the Python implementation of the mRPI calculation.
  W: a 6-vector noise struct
  gains_dr: a struct of ancillary controller gains
  %}
  z_poly = py.z_poly_calc.ZPoly();
  
  % get discrete A and B matrices 
  W = [noise.r1, noise.r2, noise.r3, noise.v1, noise.v2, noise.v3]';
  W = mat2np(W);  
  U_max = mat2np([0.4, 0.4, 0.4]');  % input, [N]
  dt = double(DT);
  mass = double(mass);
  
  Q_LQR = [gains_dr.Q1, gains_dr.Q2, gains_dr.Q3, gains_dr.Q4, gains_dr.Q5, gains_dr.Q6];
  R_LQR = [gains_dr.R1, gains_dr.R2, gains_dr.R3];
  Q_LQR = py.numpy.diag(Q_LQR);
  R_LQR = py.numpy.diag(R_LQR);
  
  output = z_poly.calc_MRPI_and_K_dr(W, dt, U_max, mass, Q_LQR, R_LQR);
  
  K_dr = np2mat(output{1});
  Autight = np2mat(output{2});
  butight = np2mat(output{3});
  AZ = np2mat(output{4});
  bZ = np2mat(output{5});
  return 
end

function plot_mpc(t_hist, X_hist, U_hist, X_des_hist)
  %{
  Plot MPC history, showing inputs and states.
  %}
  % Positions
  close all;
  figure
  hold on
  % actual
  plot(t_hist, X_hist(1,:), 'r');
  plot(t_hist, X_hist(2,:), 'g');
  plot(t_hist, X_hist(3,:), 'b');
  
  % desired
  plot(t_hist, X_des_hist(1,:), ':r');
  plot(t_hist, X_des_hist(2,:), ':g');
  plot(t_hist, X_des_hist(3,:), ':b');
  
  xlabel('t [s]');
  title('Sinusoidal Trajectory-Following With Linearly-Constrained MPC, 3DOF Double Integrator');
  ylabel('position [m], velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_hist(2:end), [U_hist(1,:)], '-r*', 'LineWidth', 2)
  stairs(t_hist(2:end), [U_hist(2,:)], '-g*', 'LineWidth', 2)
  stairs(t_hist(2:end), [U_hist(3,:)], '-b*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('x1','x2','x3','$x1_{des}$','$x2_{des}$','$x3_{des}$','u1','u2','u3');
  
  % Velocities
  figure
  hold on
  % actual
  plot(t_hist, X_hist(4,:), 'r');
  plot(t_hist, X_hist(5,:), 'g');
  plot(t_hist, X_hist(6,:), 'b');
  
  % desired
  plot(t_hist, X_des_hist(4,:), ':r');
  plot(t_hist, X_des_hist(5,:), ':g');
  plot(t_hist, X_des_hist(6,:), ':b');
  
  xlabel('t [s]');
  title('Sinusoidal Trajectory-Following With Linearly-Constrained MPC, 3DOF Double Integrator');
  ylabel('position [m], velocity [m/s]');
  yyaxis right
  ylabel('input [N]');
  stairs(t_hist(2:end), [U_hist(1,:)], '-r*', 'LineWidth', 2)
  stairs(t_hist(2:end), [U_hist(2,:)], '-g*', 'LineWidth', 2)
  stairs(t_hist(2:end), [U_hist(3,:)], '-b*', 'LineWidth', 2)
  set(gca,'FontSize',24);
  legend('$\dot{x1}$', '$\dot{x2}$', '$\dot{x3}$', '$\dot{x1}_{des}$','$\dot{x2}_{des}$','$\dot{x3}_{des}$','u1','u2','u3');
end

function make_plots_purdy()
  set(0,'DefaultAxesFontName', 'CMU Serif')
  set(0, 'DefaultLineLineWidth', 3);
  set(groot,'defaulttextinterpreter','latex');  
  set(groot, 'defaultAxesTickLabelInterpreter','latex');  
  set(groot, 'defaultLegendInterpreter','latex');
  set(0,'defaultAxesFontSize', 32)
end

function npary = mat2np(mat)
  % convert matlab matrix to python (Numpy) ndarray 
  sh = int64(fliplr(size(mat)));
  mat2 = reshape(mat, 1, numel(mat));  % [1, n] vector
  npary = py.numpy.array(mat2);
  npary = npary.reshape(sh).transpose();  % python ndarray
end

function mat = np2mat(npary)
  % convert python (Numpy) ndarray to matlab
  sh = double(py.array.array('d',npary.shape));
  npary2 = double(py.array.array('d',py.numpy.nditer(npary)));
  mat = reshape(npary2,fliplr(sh))';  % matlab 2d array 
end

function plot_lines(A, b)
  p1 = Polyhedron('A', A(:, [1,4]), 'b', b);
  p2 = Polyhedron('A', A(:, [2,5]), 'b', b);
  p3 = Polyhedron('A', A(:, [3,6]), 'b', b);
  figure;
  hold on;
  p1.plot('color', 'r');
  p2.plot('color', 'g');
  p3.plot('color', 'b');
end

function passed = run_casadi_unit_tests()
    %{
    Run some check on the CasADi dynamics
    %}
    passed = 1;
    
    % Test integrator
    disp('integrator...')
    % 10 seconds, 1/10 m/s^2, should be +5 m
    u_mag = [5; 5; 5];
    Q1 = eye(N+1)*100;
    Q2 = eye(N+1)*100;
    Q3 = eye(N+1)*100;
    Q4 = eye(N+1)*10;
    Q5 = eye(N+1)*10;
    Q6 = eye(N+1)*10;
    R1 = eye(N)*1;
    R2 = eye(N)*1;
    R3 = eye(N)*1;

    % u_N = ones(3,1).*sin(linspace(1,7,N));
    x0 = [1; 2; 3; 0; 0; 0];
    u_N = ones(3,N);
    X_hist(:,1) = x0;
    x = x0;
    
    for i=1:N
      u = u_N(:,i);
      x = full(integrator_func_casadi(x, u, m));
      X_hist(:,i+1) = x;
    end
    assert( sum(x - [1+5; 2+5; 3+5; 0+1; 0+1; 0+1]) < eps*3);

    % figure
    % tgrid = linspace(0,T,N+1);
    % plot(tgrid,full(X_log));
    % legend('x1','x2', 'x3', 'x1d', 'x2d', 'x3d');
    % xlabel('t [s]');
    % sum(x - [1+5; 2+5; 3+5; 0; 0; 0])
    disp('...okay!')

    %% Test MPC at various state values
    disp('mpc...')
    x_test = [1; 1; 1; 0; 0; 0];
    x_des_traj = create_des_traj(DT, T, 'ZERO', [0; 0; 0], [0; 0; 0; 1]);  % {'ZERO', 'SINUSOID'}
    x_des_traj = x_des_traj(:,2:7);
       
    u_test = full(mpc_func_casadi(x_test, x_des_traj, u_mag, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, m));
    assert( sum(abs(u_test - [-4.6259, -4.6259, -4.6259]')) < .0001);

    % x_test = [1; 1; 1; 0; 0; 0];
    % mpc_func_casadi(x_test)
    % 
    % x_test = [1; 1; 1; 0; 0; 0];
    % mpc_func_casadi(x_test)
    disp('...okay!')
end

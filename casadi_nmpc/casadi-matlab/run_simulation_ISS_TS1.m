
%{
Unit tests for `casadi_3dof_double_integrator` and 
`casadi_3dof_double_integrator_z0`
%}
import casadi.*

close all;
clear all;
clear classes;  % needed to reload Python changes

% Ensure module is on PYTHONPATH, force reload. Needs >=Python3.5!
PYTHON_MODULE_PATH = '/home/albee/workspaces/astrobee-ws-td/freeflyer-shared-td/develop/casadi_nmpc/scripts';
P = py.sys.path;
if count(P, PYTHON_MODULE_PATH) == 0
    insert(P,int32(0), PYTHON_MODULE_PATH);
end
z_poly_calc = py.importlib.import_module('z_poly_calc');
py.importlib.reload(z_poly_calc);
%%
addpath(genpath('../../data/scripts'));

make_plots_purdy();

T = 2.0; % Time horizon, [s]
N = 10; % Number of control intervals over time horizon, [-]
DT = T/N;  % delta-t, [s]

control_dt_ = 0.2;

GEN_CODE = 0;
UNIT_TESTS = 0;

[mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt] = casadi_3dof_double_integrator_mpc(GEN_CODE, T, N, control_dt);
[tube_mpc_func_casadi, ~] = casadi_3dof_double_integrator_tube_mpc(GEN_CODE, T, N);

%% Test trajectory tracking

% simulation options
CONTROL = 'TUBE_MPC';  % {'TUBE_MPC, or 'MPC'}
DES_TRAJ = 'SINUSOID';  % {'SINUSOID', 'ZERO'}
DIMS = '3D';  % ['1D', '3D', or Limon]
TF_TRAJ = 10*T;
% ISS_XYZ = [10.9; -9.65; 4.9];
% x0 = [ISS_XYZ; 0; 0; 0];
x0 = [0.1; 0.1; 0.0; 0; 0; 0];
% x0 = [10.85; -9.65; 4.9; 0; 0; 0.0];

% x_des_traj = create_des_traj(DT, TF_TRAJ, DES_TRAJ, [ISS_XYZ], [0; 0; 0; 1]);
x_des_traj = create_des_traj(control_dt, TF_TRAJ, DES_TRAJ, [0; 0; 0], [0; 0; 0; 1]);
x_des_traj = x_des_traj(:,2:7);  % clip off extra

u_mag = [0.4; 0.4; 0.4];

% nominal constraints
Au = [eye(3); -eye(3)];
bu = [u_mag; u_mag];  % box constraint, sign is correct

m = 8.0;  % mass (ground units), [kg]

% weights
Q1 = 50;
Q2 = 50;
Q3 = 50;
Q4 = 5;
Q5 = 5;
Q6 = 5;
R1 = 11;
R2 = 11;
R3 = 11;
QN1 = 100;
QN2 = 100;
QN3 = 100;
QN4 = 100;
QN5 = 100;
QN6 = 100;

Q_LQR = [5, 5, 5, 50, 50, 50];
R_LQR = [0.1, 0.1, 0.1];

% Utight and Z_poly constraints for tube MPC
[K_dr, Autight, butight, AZ_actual, bZ_actual] = calc_mRPI_python(m, DT, Q_LQR, R_LQR);

AZ = zeros(100, 6);
bZ = zeros(100, 1);
len_AZ = size(AZ_actual,1);
AZ(1:len_AZ, :) = AZ_actual;
bZ(1:len_AZ, :) = bZ_actual;

% plot_lines(AZ_actual, bZ_actual)

x_test = x0;

X_hist = x0;
U_hist = [];
U_mpc_hist = [];
x0_mpc_hist = [];
t_hist = [0];
X_des_hist = x_des_traj(1:end - ceil(T/control_dt), :)';
computation_hist = [];

% for tube MPC
u_mpc = 0;
x0_mpc = 0;

for i=1:1:(TF_TRAJ-T)/control_dt  % 10*T*N timesteps---leave extra T horizon for MPC
  x_des_traj_N = x_des_traj(i:i+N, :);
  tic;
  
  % Standard MPC
%   u = full(mpc_func_casadi(x_test, x_des_traj_N, u_mag, m, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6));  % solve control for i-th instant over DT time

  % Tube MPC
  [dm_u0_opt, dm_u0_mpc, dm_u0_dr, dm_x0_mpc] = tube_mpc_func_casadi(x_test, x_des_traj_N, u_mag, m, Autight, butight, AZ, bZ, K_dr, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6);
  u = full(dm_u0_opt); u_mpc = full(dm_u0_mpc); x0_mpc = full(dm_x0_mpc);

  % History recording
  computation_hist(i) = toc;
  t_hist(end+1) = i*control_dt;
  U_hist(:,i) = u;
  X_hist(:,end+1) = x_test;
  
  U_mpc_hist(:,i) = u_mpc;
  x0_mpc_hist(:,end+1) = x0_mpc;
  abs(x_test - x0_mpc);

  % simulate system
%   x_test = full(integrator_func_casadi(x_test, u, m))+ unifrnd(-0.002, 0.002, 6, 1);  % with noise
  r_noise = 0.001;
  v_noise = 0.0001;
  noise = [unifrnd(-r_noise, r_noise, 3, 1); unifrnd(-v_noise, v_noise, 3, 1)];
  x_test = full(casadi_dynamics_control_dt(x_test, u, m)) + noise;  % no noise
end

fprintf("mean computation time was %f seconds \n", mean(computation_hist));
disp('...units tests complete.');

%% Plot
plot_mpc(t_hist, X_hist, U_hist, X_des_hist)
% plot_z0(AZ, bZ, X_hist, x0_mpc_hist, X_des_hist)

%%
if UNIT_TESTS == 1
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

function plot_z0(AZ, bZ, X_hist, x0_mpc_hist, X_des_hist)
  %{
  Plot the initial state constraint for a single axis (2D phase plane plot).
  %}
  close all;
  figure;
  hold on;
  
  mRPI = Polyhedron('A', AZ(:, [1,4]), 'b', bZ);
  rx = X_hist(1, :);
  rdx = X_hist(4, :);
  
  r0x_mpc = x0_mpc_hist(1, :);
  r0dx_mpc = x0_mpc_hist(4, :);
  
  rx_des = X_des_hist(1, :);
  rdx_des = X_des_hist(4, :);
  
  % Plot all polytopes
  for idx=1:1:size(x0_mpc_hist, 2)
    plot_polytope_offset(mRPI, x0_mpc_hist([1,4], idx)');
  end
  
  ax1 = plot(rx, rdx, 'Color',  'red');  % real state
  ax2 = plot(rx_des, rdx_des, ':', 'Color',  'black');  % desired state
  ax3 = plot(r0x_mpc, r0dx_mpc, 'Color', 'green');  % mpc state
  legend([ax1, ax2, ax3], {'$\mathbf{x}_{real}$', '$\mathbf{x}_{des}$', '$\mathbf{x}_{mpc}$'});
    
  axis([-0.5, 0.5, -0.1, 0.1]);
  title('Phase Plot of Single-Axis Tube MPC Tracking');
  ylabel('$\dot{r}$ [m/s]');
  xlabel('$r$ [m]');
end

function plot_polytope_offset(polytope, offset)
  %{
  Plot 2D polytope with offset
  %}
  verts = polytope.V + offset;
  verts_idx = convhull(verts);
  verts = verts(verts_idx, :);
  patch(verts(:, 1), verts(:, 2), [0,0,0]+0.8)
end

function [K_dr, Autight, butight, AZ, bZ] = calc_mRPI_python(mass, DT, Q_LQR, R_LQR)
  %{
  Call the Python implementation of the mRPI calculation.
  %}
  z_poly = py.z_poly_calc.ZPoly();
  
  % get discrete A and B matrices 
%   [0.014867931604385376, 0.005751967430114746, 0.0, -0.014867931604385376, -0.005751967430114746, -0.0, 0.0019519033376127481, 0.0007975660264492035, 0.0, -0.0019519033376127481, -0.0007975660264492035, -0.0]
  W = mat2np([0.0331, 0.0260, 0.0537, 0.0069, 0.0055, 0.0073]');  
  U_max = mat2np([0.4, 0.4, 0.4]');  % input, [N]
  dt = double(DT);
  mass = double(mass);
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



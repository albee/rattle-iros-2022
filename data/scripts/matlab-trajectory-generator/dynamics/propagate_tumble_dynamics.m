%{
# propagate_tumble_dynamics.m

A script to run the tumble dynamics and plot.

Uses casadi_6dof_newton_euler_dynamics.

GEN_CODE: {0, 1} 1 generates C code of CasADi dynamics.
DATA_DIR: directory with prerecorded .mats of input and time history.
PLOT_DIR: directory with plotting tools.

Keenan Albee, 2021
MIT Space Systems Lab
%}
clear all;
close all;

GEN_CODE = 0;
DATA_DIR = "../data";
PLOT_DIR = "../plotting";

addpath(genpath(DATA_DIR));
addpath(PLOT_DIR);

load('rbd_info_gain_traj1.mat');  % specify the .mat you want here. `results` is the struct with the important info

global u_hist
global t_hist

u_hist = results.input;  % [F_x F_y F_z T_x T_y T_z], time goes by row
t_hist = results.time;

%% Create the dynamics model (specify everything explicitly)
u_func = @u_func_tumble;  % reference to function that supplies input history
newton_euler_dynamics = casadi_6dof_newton_euler_dynamics(GEN_CODE);

% Initial conditions
dt = 0.01;  % [s]
tf = t_hist(end);  % [s]
% tf = 5.0;  % [s]

% mass
m = 10.0;

% moments of inertia (principal coordinates)
I_xx = 0.1;
I_yy = 0.1;
I_zz = 0.1;

% inertia ratios
p_x = (I_yy - I_zz)/I_xx;
p_y = (I_zz - I_xx)/I_yy;
p_z = (I_xx - I_yy)/I_zz;

p = [m; I_xx; I_yy; I_zz];  % parameters

% position
r_x0 = 0.0;
r_y0 = 0.0;
r_z0 = 0.0;
r_i = [r_x0; r_y0; r_z0];

% velocity
v_x0 = 0.0;
v_y0 = 0.0;
v_z0 = 0.0;
v_i = [v_x0; v_y0; v_z0];

% quaternion
q_x0 = 0.0;
q_y0 = 0.0;
q_z0 = 0.0;
q_w0 = 1.0;
q_i = [q_x0; q_y0; q_z0; q_w0];

% angular velocity (body frame)
w_x0 = 0.0;%3.0;
w_y0 = 0.0;%0.01;
w_z0 = 0.0;%0.0;
w_i = [w_x0; w_y0; w_z0];
norm(w_i);

x0 = [r_i; v_i; q_i; w_i];

%% Propagate
[t_hist_ne, x_hist_ne, u_hist_ne] = run_dynamics(dt, tf, x0, p, newton_euler_dynamics, u_func);

%% Plotting
r0_mat = x_hist_ne(:, 1:3);
R0_mat = x_hist_ne(:, 7:10);

plot_dt = 0.1;
frames = get_frames(t_hist_ne, plot_dt);
anim_tumble(r0_mat, R0_mat, r0_mat, frames);  % (desired trajectory, quaternion history, desired trajectory)
%%

function [t_hist, x_hist, u_hist] = run_dynamics(dt, tf, x0, p, dynamics_func, u_func)
  %{
  Runs the dynamics using dt to propagate and tf final time. Also plots.
  
  Inputs:
  dt - timestep
  tf - final time
  x0 - start state
  p - parameters
  dynamics_func - dynamics
  %}
  x_i = x0;  
  t_hist = [];
  x_hist = [];  % [timestep, n]
  u_hist = [];  % [timestep, m]
  
  for t = 0:dt:tf
    x_hist(end+1, :) = x_i';
    u_i = u_func(t);
    u_hist(end+1, :) = u_i';
    t_hist(end+1,:) = t;
%     u_i = [0.005; 0.005; 0.005; 0.05; 0; 0];  % [Fx; Fy; Fz; Tx; Ty; Tz], [N], forces in inertial frame, torques in body frame
    x_i = full(dynamics_func(x_i, u_i, p, dt));  % full is required to convert from casadi.DM    
  end
end

function frames = get_frames(t_hist, plot_dt)
  %{
  Downselect the frames used for plotting

  Inputs:
  t_hist - entire time history
  plot_dt - dt to use for the plotting
  %}
  tvec = t_hist;
  tf = tvec(end);
  t = 0:plot_dt:tf;
  cntr = 1;
  frames = [];
  for i = 1:1:size(tvec,1)
      if tvec(i) > t(cntr)
          cntr = cntr+1;
          frames = [frames; i];
      end
  end
end

function check_rotation()
  %% Check angular velocity magnitude is unchanged
  % moments of inertia
  I_xx = 2.0;
  I_yy = 2.0;
  I_zz = 1.0;

  % inertia ratios
  p_x = (I_yy - I_zz)/I_xx;
  p_y = (I_zz - I_xx)/I_yy;
  p_z = (I_xx - I_yy)/I_zz;

  % quaternion
  q_x0 = 0.0;
  q_y0 = 0.0;
  q_z0 = 0.0;
  q_w0 = 1.0;
  q_i = [q_x0; q_y0; q_z0; q_w0];

  % angular velocity
  w_x0 = 1.0;
  w_y0 = 1.0;
  w_z0 = 1.0;
  w_i = [w_x0; w_y0; w_z0];
  w1 = full(norm([w_x0, w_y0, w_z0]));

  x_i = [q_i; w_i];

  t_next = 1.0;

  x_next = newton_euler_dynamics(x_i, [0; 0; 0], [p_x; p_y; p_z], t_next);
  w2 = full(norm(x_next(5:7)));

  assert((w1 - w2) < 0.001, 'Angular velocity is not conserved!')
end
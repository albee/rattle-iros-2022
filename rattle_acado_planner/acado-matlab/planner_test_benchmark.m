%{
planner_test.m

Testing information-weighted mid-level planner using ACADO 
The dynamics used are those of a 6 DoF rigid body with info gain enabled
for 3 dof parameters(mass, comx, comy, izz)

The state vector is X = [r(3)  v(3) q(4) w(3) psi(33)], see generate_info_gain_planner.m.
Note that the augmented state psi has 6 + 9*3, or 33 elements,
discounting all the zeros that we dont want to waste time computing.

Inputs:
Initialization parameters, see below.

Outputs:
MATLAB workspace variables for plotting

** The gamma shifter function that updates variable gamma is copied over from
ASTRA, but has not been implemented in this script **
%}
%% Initialization
close all;
clear all;

addpath('../');
addpath('NMPC_export');
addpath('SIM_export');
addpath('RHP_export');
addpath('RHP_6DOF_export');
addpath('astrobee-mat-plot');  % I manually added it here too
addpath('~/repos/thesis-data-analysis/mat-plot/astrobee-mat-plot');

ISS = 1;  %{0, 1}: 1 indicates 6 DOF dynamics
USE_FIM = 1;  %{0, 1}: 1 indicates use version with FIM

Ts = 0.3;  % length of time step
N = 40;  % horizon length

if ISS == 0
    m_est = 18.95; I_xx_est = 0.2517; I_yy_est = 0.2517; I_zz_est = 0.2517;
else
    m_est = 9.85; I_xx_est = 0.15; I_yy_est = 0.14; I_zz_est = 0.16;
end

th_hat = [m_est I_xx_est I_yy_est I_zz_est];

fprintf('%s\n',...
        'Running planner with parameters:',...
        strcat({'ISS: '}, string(ISS)),...
        strcat({'USE_FIM: '}, string(USE_FIM)));


%% Planner setup variables, |x| = 13
% x_sim = [0.5;0.5;0.5; 0;0;0; 0.2227177;0.0445435;0.4454354;0.8660254; 0;0;0.0]';  % |x| = 13 
% x_sim = [-1.5;-1.5;-2; 0.6030691; 0.3481821; 0.5509785; 0.4598907; 0;0;0; 0;0;0.0]';
% x_sim = [0.1; 0.1;0.0; 0;0;0; 0;0;0;1; 0.0;0.0;0.0]';  % state of the simulated system (x0)
% x_sim = [0 0.6 0 0 0 0 0 0 0 1 0 0 0];
x_sim = [10.7964   -9.7510    4.8011   -0.0047    0.0047    0.0036   -0.0001   -0.0000   -0.7071    0.7071   0.0 0.0 0.0];

% some desired states for checking separate translation and rotation performance
% note: 3 DOF must have axis of rotation parallel to z-axis!
% x_des = [0.0, 0.6, 0.0,  0, 0, 0,  0.0, 0.0, 0.0, 1.0,  0, 0, 0]; % no state error
% x_des = [0.106869 0.371323 0 0.0153218 -0.0459653 0 0 0 0.707, 0.707 0 0 0];

% x_des = [0.0, 0.3, 0.0,  0, 0, 0,  0.0, 0.707, 0, 0.707,  0, 0, 0];

% x_des = [1 -2 0.0 0 0 0 0 0 0 1 0 0 0]; % only pos errors
%x_des = [0.0 0.0 0.0 0 0 0 0 0 0.7071 0.7071 0 0 0]; % only rot errors
%x_des = [1 -2 0.5 0 0 0 0 0 0.7071 0.7071 0 0 0]; % both
x_des = [10.4732   -9.5139    4.8424   -0.0059    0.0058   -0.0003   -0.0220   -0.0008   -0.7708    0.7507   -0.0001   -0.0001   -0.0001];

u_des = [0 0 0 0 0 0];  % just 0 for now


% weightings
% [FIM_m, FIM_cx, FIM_cy, FIM_izz, r(3), v(3), q(3), w(3), u(6)], |W|=22
W_FIM = [100.0 0.0 0.0 0.5];
% W_FIM = [0.0 0.0 0.0 0.0]; 
W_x = [10, 10, 10, 5, 5, 5, 1, 1, 1, 1, 1, 1];
% W_x = zeros(1, 12);
W_u = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0];
% W_u = zeros(1, 6);
WN = eye(12)*100.0;


%% Create inputs to the planner
if USE_FIM == 1
  W = diag([W_FIM W_x, W_u]);
  if ISS == 1
    planner_input.x = repmat([x_sim'; 0.1*ones(45,1)]', N+1, 1);  % x0
  else
    planner_input.x = repmat([x_sim'; 0.1*ones(31,1)]', N+1, 1);  % x0
  end
  planner_input.u = repmat([ 0.1; 0.1; 0.1; 0.1; 0.1; 0.1]', N, 1); % matrix of horizon number of control inputs
  planner_input.W = W;
  planner_input.WN = WN;  % Weight for the terminal cost
  planner_input.y = repmat(zeros(1,22), N, 1);  % reference signal (unused)
  planner_input.yN = zeros(1,12);  % reference signal, terminal (unused)
else
  W = diag([W_x, W_u]);
  planner_input.x = repmat([x_sim']', N+1, 1);  % x0
  planner_input.u = repmat([ 0.1; 0.1; 0.1; 0.1; 0.1; 0.1]', N, 1); % matrix of horizon number of control inputs
  planner_input.W = W;
  planner_input.WN = WN;  % Weight for the terminal cost
  planner_input.y = repmat(zeros(1,18), N, 1);  % reference signal (unused)
  planner_input.yN = zeros(1,12);  % reference signal, terminal (unused)
end

% Run until KKT is reasonable
KKT_val = 1E-2;
planner_output = [];
iter = 1;
max_RTI_iter = 5;

tic
while (KKT_val > 1E-3 && iter <= max_RTI_iter)
  planner_input.od = repmat([th_hat, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...
                             x_des], N+1, 1);  % no u_des                   
  if USE_FIM == 1
    if ISS == 1
      planner_input.x0 = [x_sim(end,:), 0.1*ones(1,45)];  % latest ground truth value, add zeros because of the extra states
    else
      planner_input.x0 = [x_sim(end,:), zeros(1,31)];  % latest ground truth value, add zeros because of the extra states
    end
  else
    planner_input.x0 = x_sim(end,:);  % latest ground truth value, add zeros because of the extra states
  end
  
  if ISS == 1
    planner_output = RHP_6DOF(planner_input);  % this step runs receding horizon planning
  else
    planner_output = RHP(planner_input);  % this step runs receding horizon planning
  end
    
  planner_input.x = planner_output.x;  % initial guess shift
  planner_input.u = planner_output.u;
  KKT_val = planner_output.info.kktValue;
  fprintf('Iteration Number: %d, KKT: %d \n', iter, KKT_val);
    
  iter = iter + 1;
end
toc

planner_output.info;
x_plan = planner_output.x(:,1:13);
psi_plan = planner_output.x(:,14:end);  % psi propagation
u_plan = planner_output.u(:,1:6);


%% 3D animated visualization 
anim_tumble(x_plan(:,1:3), x_plan(:,7:10), x_plan(:,1:3));


%% Input plots
plot_inputs(u_plan);


%% FIM calc and plots
if ISS == 1
  sigma_diag = 1.0* ones(13, 1);
  dh_dx = eye(13);
else
  sigma_diag = 1.0* ones(9, 1);
  dh_dx = eye(9);
end

plot_info_content(psi_plan, sigma_diag, dh_dx, ISS);

%% Utility functions
function W_out = gamma_shifter(W, info)
  % Updates gamma according to a specified shifting pattern.
  % gamma is the first term
  gamma = W(1, 1);
  x = info{1};
  t = info{2};
  tau = 20;  % time constant for decay by a factor of e

  % adjust gamma for decent tracking
  % options: distance to goal; quality of tracking; change in parameter
  % estimates; hard cutoff after enough time

  % for now, it just ramps down as you get closer to x_g
  gamma = 10.0*exp(1)^(-1/tau*t);
  gamma = gamma + norm(x)^2 -1; % quaternion scalar element does not let this go to zero
  if gamma > 20
      gamma = 20;
  end  % limit max

  W(1, 1) = gamma;  % set gamma
  W_out = W;
end

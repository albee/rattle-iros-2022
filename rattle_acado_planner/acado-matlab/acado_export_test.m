%{
acado_export_test.m

Main code for running/ MATLAB - testing information weighted NMPC using ACADO 
The dynamics used are those of a 6 DoF rigid body with info gain enabled
for 3 dof parameters(mass, comx, comy, izz)

The state vector is X = [r(3)  v(3) q(4) w(3) psi(33)], see RHP.m.
Note that the augmented state psi has 6 + 9*3, or 33 elements,
discounting all the zeros that we dont want to waste time computing.

Inputs:
Correct file paths, see below.
Initialization parameters, see below.

Outputs:
MATLAB workspace variables for plotting

** The gamma shifter function that updates variable gamma is copied over from
ASTRA, but this has not been implemented in this script **
%}
%close all;
 clear all;
 
 addpath('../');
 addpath('NMPC_export');
 addpath('SIM_export');
 addpath('RHP_export');
 addpath('RHP_6DOF_export');

 
%% Initialization
rhp = 1; % {0, 1}: set to 1 if testing the receding horizon planner (RHP)
ISS = 0;  %{0, 1}: 1 indicates 6 DOF dynamics
PRINT_FIM = 0;  % if we want to see the FIM value on the screen
%!! Warning it uses the long format!!

Objective_value = [];
tf = 50.0;  % final time, s
time = 0;  % start time, s
if rhp == 1
    Ts = 0.3;  % length of time step
    N = 40;  % horizon length
else
    Ts = 0.2;  % length of time step
    N = 10;  % horizon length
end

sim_results = [];  % for logging sim data
control_ip = [];  % for logging control data 

if ISS == 0
    m_est = 18.95; I_xx_est = 0.2517; I_yy_est = 0.2517; I_zz_est = 0.2517;
else
    m_est = 9.85; I_xx_est = 0.15; I_yy_est = 0.14; I_zz_est = 0.16;
end

x_hat = [m_est; I_xx_est; I_yy_est; I_zz_est];
sigma  = ones(9,1);  % measurement covariance, ~10^-7 from EKF
inv_R = inv(diag(sigma)); %


%% System simulator variables
% state_sim = [0.5;0.5;0.5; 0;0;0; 0.2227177;0.0445435;0.4454354;0.8660254; 0;0;0.0]';  % |x| = 13 
% state_sim = [-1.5;-1.5;-2; 0.6030691; 0.3481821; 0.5509785; 0.4598907; 0;0;0; 0;0;0.0]';
state_sim = [0.0;0.0;0.0; 0;0;0; 0;0;0;1; 0.0;0.0;0.0]';
% some desired states for checking separate translation and rotation performance
x_des = [0.0 0.0 0.0  0 0 0  0 0 0 1  0 0 0]; % no state error
% x_des = [1 -2 0.0 0 0 0 0 0 0 1 0 0 0]; % only pos errors
%x_des = [0.0 0.0 0.0 0 0 0 0 0 0.7071 0.7071 0 0 0]; % only rot errors
%x_des = [1 -2 0.5 0 0 0 0 0 0.7071 0.7071 0 0 0]; % both
u_des = [0 0 0 0 0 0];  % just 0 for now


%% NMPC variables
% weight matrix settings
W_x = [10, 10, 10, 8, 8, 8, 1, 1, 1, 1, 1, 1];
W_u = [0.2, 0.2, 0.2, 0.2 0.2, 0.2];

if rhp == 1
    % [FIM_m, FIM_cx, FIM_cy, FIM_izz, r(3), v(3), q(3), w(3), u(6)], |W|=22
    W_FIM = [0 0 0 10.0]%[4000 1 1 1]%[2500 50 50 1]; 
    %W_FIM = [0.0 0.0 0.0 0.0];
   
    W = diag([W_FIM W_x, W_u]);
    input.W = W;
    input.x = repmat([state_sim';zeros(31,1)]', N+1, 1);
    input.y = repmat(zeros(1,22), N, 1);
else
    input.x = repmat([state_sim']', N+1, 1); 
    W = diag([W_x, W_u]);
    input.W = W;
    %input_NMPC.W = eye(18); % previous setting for controller weights
    input.y = repmat(zeros(1,18), N, 1);
end
input.u = repmat([ 0.1; 0.1; 0.1; 0.1; 0.1; 0.1]',N,1); % matrix of horizon number of control inputs


input.WN = eye(12)*150;                % Weight for the terminal cost
input.yN = zeros(1,12);               % reference for the minimizing function


%% FIM variables
% measurements v_x, v_y, w_z
dh_dx = eye(9);
         
F_local = zeros(N,1);
F_aggregate = 0;          % FIM over all horizons, corresponding to the applied (first) input of each MPC horizon
  

%% Main loop
%{
At each step, Acado's .od and .x0 variables must be updated before the MPC
can be called again.
%}
time_store = [];

if rhp == 1 
    for i  = 1:10
        input.od = repmat([m_est, I_xx_est, I_yy_est, I_zz_est, 0, 0, 0, 0, 0, 0, ...
                            x_des], N+1, 1);  % no u_des

        input.x0 = [state_sim(end,:), zeros(1,31)];  % latest ground truth value, add zeros because of the extra states

        if ISS == 1
            output = RHP_6DOF(input);  % this step runs receding horizon planning
        else
            output = RHP(input);  % this step runs receding horizon planning
        end

        output.info  % gives some info about the value of the objective after each MPC horizon



        control_ip = output.u;  % record applied u
        states = output.x;  % record ground truth dat
        input.x = states;       % latest state of simulation
        input.u = control_ip;          % first input from NMPC optimization
    end
    
end
time_store = 0:Ts:N*Ts;
    
% all the previous nmpc_test stuff, if needed.
nmpc_test_loop;

%% Misc plotting
visualize



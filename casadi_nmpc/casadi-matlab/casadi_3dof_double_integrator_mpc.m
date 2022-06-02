%{
Keenan Albee, Dec-11 2020
Double integrator (N)MPC using CasADi.
Originally based on CasADi example documentation and tutorials.

Inputs:
T = 2;        length of time horizon, s
N = 20;       number of control inputs over horizon

Outputs:
mpc_func_casadi: CasADi function
casadi_dynamics_mpc_dt: discrete dynamics at mpc dt.
casadi_dynamics_control_dt: disrete dynamics at control dt.
%}

function [mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt] = casadi_3dof_double_integrator_mpc(T, N, control_dt)
import casadi.*

DT = T/N;

%% Set up symbolic variables
% Continuous-time dynamics!
% x = [x1; x2; x3; x1d; x2d; x3d], m, m/s
% u = [u1; u2; u3], N

m = MX.sym('m', 1, 1);

A = [0, 0, 0, 1, 0, 0;
     0, 0, 0, 0, 1, 0;
     0, 0, 0, 0, 0, 1;
     zeros(3,6)
    ];
B = [zeros(3,3);
     1/m, 0,   0;
     0,   1/m, 0;
     0,   0,   1/m;
    ];

x = MX.sym('x', 6, 1); % States   
u = MX.sym('u', 3, 1); % Controls

%% Set up ODE
% double integrator dynamics
xdot = A*x + B*u;

f = Function('f',{x, u, m},{xdot},{'x', 'u', 'm'},{'ode'});
% f([0.2;0.001],0.0001)  % {x, u}


%% Create an integrator
% Integrator to discretize the system
intg_options = struct;
intg_options.tf = DT;  % delta-t
intg_options.simplify = true;
intg_options.number_of_finite_elements = 4;

% DAE problem structure
dae = struct;
dae.x = x;         % What are states?
dae.p = [u; m];         % What are parameters (=fixed during the integration horizon)?
dae.ode = f(x, u, m);  % Expression for the right-hand side

intg = integrator('intg', 'rk', dae, intg_options);

% x0 = [0;1];
% res = intg('x0',[0;1],'p',0);  % Easier to identify inputs, but bloated API
% res.xf;  % view final state

% (x,u)->(x_next)
res = intg('x0', x, 'p', [u; m]); % Evaluate with symbols
x_next = res.xf;  % symbolically defined
casadi_dynamics_mpc_dt = Function('casadi_dynamics_mpc_dt',{x, u, m}, {x_next},{'x', 'u', 'm'},{'x_next'});  % function call to integrate: continuous over timestep T/N

% One more for the control period
intg_options.tf = control_dt;  % delta-t
intg2 = integrator('intg', 'rk', dae, intg_options);
res2 = intg2('x0', x, 'p', [u; m]); % Evaluate with symbols
x_next2 = res2.xf;  % symbolically defined
casadi_dynamics_control_dt = Function('casadi_dynamics_control_dt',{x, u, m}, {x_next2},{'x', 'u', 'm'},{'x_next'});  % function call to integrate: continuous over timestep T/N


%% Multiple-shooting direct transcription optimal control using Opti interface
opti = casadi.Opti();  % the solver interface

% Variables (states and inputs)
x = opti.variable(6, N+1); % Decision variables for state trajetcory
u = opti.variable(3, N);

% Parameters
x0 = opti.parameter(6, 1);            % initial state x0 = x_k
x_des = opti.parameter(N+1, 6);  % des traj x_des_traj = N+1x6
u_mag = opti.parameter(3, 1);         % input limit
m = opti.parameter(1, 1);             % mass

%{
NB: x_des_traj must be sampled at same rate as MPC!!! (or interpolated)
Weighting matrices are defined over the enitre horizon length individually
for each state.

values ending in -N indicate terminal weight. Q is state; R is input.
%}
Q1 = opti.parameter(1);
Q2 = opti.parameter(1);
Q3 = opti.parameter(1);
Q4 = opti.parameter(1);
Q5 = opti.parameter(1);
Q6 = opti.parameter(1);
R1 = opti.parameter(1);
R2 = opti.parameter(1);
R3 = opti.parameter(1);
QN1 = opti.parameter(1);
QN2 = opti.parameter(1);
QN3 = opti.parameter(1);
QN4 = opti.parameter(1);
QN5 = opti.parameter(1);
QN6 = opti.parameter(1);
Q1_mat = eye(N)*Q1;
Q2_mat = eye(N)*Q2;
Q3_mat = eye(N)*Q3;
Q4_mat = eye(N)*Q4;
Q5_mat = eye(N)*Q5;
Q6_mat = eye(N)*Q6;
R1_mat = eye(N)*R1;
R2_mat = eye(N)*R2;
R3_mat = eye(N)*R3;

% l(x, t) + h(x, t), cost function
opti.minimize( (x(1,1:N)-x_des(1:N,1)')*Q1_mat*(x(1,1:N)-x_des(1:N,1)')' + ...  % state weight
               (x(2,1:N)-x_des(1:N,2)')*Q2_mat*(x(2,1:N)-x_des(1:N,2)')' + ...
               (x(3,1:N)-x_des(1:N,3)')*Q3_mat*(x(3,1:N)-x_des(1:N,3)')' + ...
               (x(4,1:N)-x_des(1:N,4)')*Q4_mat*(x(4,1:N)-x_des(1:N,4)')' + ...
               (x(5,1:N)-x_des(1:N,5)')*Q5_mat*(x(5,1:N)-x_des(1:N,5)')' + ...
               (x(6,1:N)-x_des(1:N,6)')*Q6_mat*(x(6,1:N)-x_des(1:N,6)')' + ...
               u(1,:)*R1_mat*u(1,:)' + u(2,:)*R2_mat*u(2,:)' + u(3,:)*R3_mat*u(3,:)' + ... % input weight
               (x(1,N+1)-x_des(N+1,1)')*QN1*(x(1,N+1)-x_des(N+1,1)')' + ... % terminal weight
               (x(2,N+1)-x_des(N+1,2)')*QN2*(x(2,N+1)-x_des(N+1,2)')' + ...
               (x(3,N+1)-x_des(N+1,3)')*QN3*(x(3,N+1)-x_des(N+1,3)')' + ...
               (x(4,N+1)-x_des(N+1,4)')*QN4*(x(4,N+1)-x_des(N+1,4)')' + ...
               (x(5,N+1)-x_des(N+1,5)')*QN5*(x(5,N+1)-x_des(N+1,5)')' + ...
               (x(6,N+1)-x_des(N+1,6)')*QN6*(x(6,N+1)-x_des(N+1,6)')');  

% Constraints            
opti.subject_to(x(:, 1) == x0);
for k=1:N
    opti.subject_to(x(:, k+1) == casadi_dynamics_mpc_dt( x(:, k), u(:, k), m));  % discrete time dynamics via integrator
end
opti.subject_to(-u_mag <= u <= u_mag);
% opti.subject_to(-0.2 <= x <= 0.2);

% Choose an NLP/QP solver
% qpsol_opts = struct('print_header',false,'print_iter',false,'print_info',false,'print_out',false);

qpsol_opts = struct('printLevel','none', 'print_out',false,'error_on_fail',true, 'print_problem', false);

% osqp_opts = struct('eps_rel', 1E-3, 'eps_abs', 1E-3, 'verbose', false, 'check_termination', 100, 'adaptive_rho_tolerance', 5);
% qpsol_opts = struct('print_out', false, 'error_on_fail',true, 'warm_start_primal', true, 'warm_start_dual', true, 'osqp', osqp_opts);

nlp_opts = struct('qpsol','qpoases','print_header',false,'print_iteration',false,'print_status',false,'print_out',false,'print_time',false,...
                     'max_iter', 5,'qpsol_options',qpsol_opts);
opti.solver('sqpmethod', nlp_opts);  % solver, plugin options, solver options

%% Convert Opti to a CasADi function and build MPC
% there is an SQP method and integrator embedded---differentiable
mpc_func_casadi = opti.to_function('mpc_func_casadi',...            % function name
                                   {x0, x_des, u_mag, m, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6},...
                                   {u(:,1), x(:,:)},...    % inputs and outputs
                                   {'x0', 'x_des_traj', 'u_mag', 'm', 'Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'R1', 'R2', 'R3', 'QN1', 'QN2', 'QN3', 'QN4', 'QN5', 'QN6'},...
                                   {'u0_opt', 'x_opt'});  % input and output names
end


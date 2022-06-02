%{
Keenan Albee, Aug-4 2021
Double integrator nominal robust (N)MPC using CasADi.

Inputs:
GEN_CODE = 1; 1 to generate code, 0 otherwise
T = 2;        length of time horizon, s
N = 20;       number of control inputs over horizon

Outputs:
mpc_func_casadi CasADi function
Generated code is sent to ../casadi_codegen

%}

function [tube_mpc_func_casadi, integrator_func_casadi] = casadi_ellipsoidal_tube_mpc(GEN_CODE, T, N)
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

% (x,u)->(x_next)
res = intg('x0', x, 'p', [u; m]); % Evaluate with symbols
x_next = res.xf;  % symbolically defined
integrator_func_casadi = Function('integrator_func_casadi', {x, u, m}, {x_next},{'x', 'u', 'm'},{'x_next'});  % function call to integrate: continuous over timestep T/N

% F([0.1;0.9],0.1)
% sim = integrator_func_casadi.mapaccum(N);  % take x0 and |u|xN matrix, output N states propagated forward


%% Multiple-shooting direct transcription optimal control using Opti interface
opti = casadi.Opti();  % the solver interface

% Variables (states and inputs)
x = opti.variable(6, N+1); % Decision variables for state trajetcory
u = opti.variable(3, N);

% Parameters (not optimized over)
x0 = opti.parameter(6, 1);       % initial state x0 = x_k.
x_des = opti.parameter(N+1, 6);  % ref trajectory, time as column
u_mag = opti.parameter(3, 1);  % maximum magnitude limits on control input
m = opti.parameter(1, 1);  % mass
Au = opti.parameter(6, 3);  % tightened input constraints from mRPI, Au matrix
bu = opti.parameter(6, 1);  % tightened input constraints from mRPI, bu matrix
AZ = opti.parameter(100, 6);  % initial condition constraint from mRPI, Ax matrix (unknown size!)
bZ = opti.parameter(100, 1);  % initial condition constraint from mRPI, bx matrix (unknown size!)

%{
NB: x_des_traj must be sampled at same rate as MPC!!! (or interpolated)
Weighting matrices are defined over the enitre horizon length individually
for each state.

QN indicates terminal weight. Q is state; R is input.
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
% "lasso" initial state constraint
for j=1:100
  opti.subject_to(AZ(j, :)*(x0 - x(:,1)) <= bZ(j));
end

% opti.subject_to(abs(x0 - x(:,1)) <= ones(6, 1)*0.008);

% discrete time dynamics via integrator
for k=1:N
   opti.subject_to( x(:, k+1) == integrator_func_casadi(x(:, k), u(:, k), m));
end

% tightened inputs
for j=1:6
  opti.subject_to(Au(j, :)*u <= bu(j));
end

% Choose an NLP/QP solver (NLP in this case)
% qpsol_opts = struct('print_header',false,'print_iter',false,'print_info',false,'print_out',false,'error_on_fail',true, 'verbose', true);

qpsol_opts = struct('printLevel', 'none', 'print_out',false, 'print_problem', false, 'print_time', false, 'error_on_fail',true);

% osqp_opts = struct('eps_rel', 1E-3, 'eps_abs', 1E-3, 'verbose', false, 'check_termination', 100, 'adaptive_rho_tolerance', 5);
% qpsol_opts = struct('print_out', false, 'error_on_fail',true, 'warm_start_primal', true, 'warm_start_dual', true, 'osqp', osqp_opts);

plugin_opts = struct('qpsol','qpoases','print_header',false,'print_iteration',false,'print_status',false,'print_out',false,'print_time',false,...
                     'max_iter', 5, 'qpsol_options', qpsol_opts);
opti.solver('sqpmethod', plugin_opts);  % solver, plugin options, solver options

%% Convert Opti to a CasADi function and build MPC
% there is an SQP method and integrator embedded---differentiable
nominal_mpc_func_casadi = opti.to_function('nominal_mpc_func_casadi',...            % function name
                                   {x0, x_des, u_mag, m, Au, bu, AZ, bZ, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6},...  % inputs
                                   {u(:, 1), x(:, 1), x(:, :)},...    % outputs
                                   {'x0', 'x_des', 'u_mag', 'm,', 'Au', 'bu', 'AZ', 'bZ', 'Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'R1', 'R2', 'R3', 'QN1', 'QN2', 'QN3', 'QN4', 'QN5', 'QN6'},...  % input names
                                   {'u0_mpc', 'x0_opt', 'x_opt'});  % output names

%%% want this interface:
%%% mpc_direct_tran(v_mag_, , w_bounds_);

%% Package ancillary controller
K_dr = MX.sym('K_dr', 3, 6);
u0_mpc = MX.sym('u0_mpc', 3, 1);
x0_mpc = MX.sym('x0_mpc', 6, 1);
u0_dr = MX.sym('u0_dr', 3, 1);
u0_opt = MX.sym('u0_dr', 3, 1);

[u0_mpc, x0_mpc, ~] = nominal_mpc_func_casadi(x0, x_des, u_mag, m, Au, bu, AZ, bZ, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6);

u0_dr = K_dr*(x0 - x0_mpc);
u0_opt = u0_mpc + u0_dr;

tube_mpc_func_casadi = Function('tube_mpc_func_casadi',...            % function name
                                   {x0, x_des, u_mag, m, Au, bu, AZ, bZ, K_dr, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6},...  % inputs
                                   {u0_opt, u0_mpc, u0_dr, x0_mpc},...    % outputs
                                   {'x0', 'x_des', 'u_mag', 'm,', 'Au', 'bu', 'AZ', 'bZ', 'K_dr', 'Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q6', 'R1', 'R2', 'R3', 'QN1', 'QN2', 'QN3', 'QN4', 'QN5', 'QN6'},...  % input names
                                   {'u0_opt', 'u0_mpc', 'u0_dr', 'x0_mpc'});  % output names

%% Code gen with embedded SQP solver
  
if GEN_CODE == 1
    disp('Generating C code and function export...')
    cd ../casadi_codegen
    FUNCTION_FILE = 'tube_mpc_func_casadi.casadi';
    tube_mpc_func_casadi.save(FUNCTION_FILE);
    copyfile(FUNCTION_FILE, '../../data/input/casadi-functions');
    
%     mpc_func_casadi.generate('casadi_3dof_double_integrator', struct('mex',true, 'with_header', true));
    CodeGen = CodeGenerator('casadi_robust_tube_mpc.c', struct('with_header', true));
    CodeGen.add(tube_mpc_func_casadi)
    CodeGen.add(integrator_func_casadi)

    CodeGen.generate();
    % mex casadi_3dof_double_integrator.c -DMATLAB_MEX_FILE
    disp('...done')
    cd ../casadi_matlab
    disp('Compile C code to a dll using: gcc -fPIC -shared gen.c -o gen.so')
end

end

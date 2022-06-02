%{
Generates CasADi serialized functions, for use by Python and C++
interfaces.
%}
close all;
clear all;

import casadi.*

addpath(genpath('../../../develop_utils/data-analysis/scripts'));
addpath(genpath('../../data/scripts'));

%% Create CasADi definitions
% Set timing parameters, get MPC
T = 5.0; % Time horizon, [s]
N = 5; % Number of control intervals over time horizon, [-]
DT = T/N;  % delta-t, [s]
control_dt = 0.2;
time_factor = round(DT/control_dt);

% ISS-TS1 values
% T = 2.0; % Time horizon, [s]
% N = 10; % Number of control intervals over time horizon, [-]
% DT = T/N;  % delta-t, [s]

[mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt] = casadi_3dof_double_integrator_mpc(T, N, control_dt);
[tube_mpc_func_casadi, ~] = casadi_3dof_double_integrator_tube_mpc(T, N);

%% Code gen and make serialized functions
import casadi.*

%***********************************************************
% Double integrator dynamics (internal controller timestep)
disp('Generating C code and function export...')
cd ../casadi-export
FUNCTION_FILE = 'casadi_dynamics_mpc_dt.casadi';
casadi_dynamics_mpc_dt.save(FUNCTION_FILE);


%%
%***********************************************************
%Robust Tube MPC
disp('Generating C code and function export...')
cd ../casadi-export
FUNCTION_FILE = 'tube_mpc_func_casadi.casadi';
tube_mpc_func_casadi.save(FUNCTION_FILE);
%     copyfile(FUNCTION_FILE, '../../data/input/casadi-functions');
    
CodeGenerator('blargh')
CodeGen = CodeGenerator('casadi_robust_tube_mpc.c', struct('with_header', true));
CodeGen.add(tube_mpc_func_casadi)
CodeGen.add(casadi_dynamics_mpc_dt)
CodeGen.generate();
% mex casadi_3dof_double_integrator.c -DMATLAB_MEX_FILE
disp('...done')
cd ../casadi-matlab
disp('Compile C code to a dll using: gcc -fPIC -shared gen.c -o gen.so')

%%
%***********************************************************
% Standard MPC
disp('Generating C code and function export...')
cd ../casadi-export
FUNCTION_FILE = 'mpc_func_casadi.casadi';
mpc_func_casadi.save(FUNCTION_FILE);
%     copyfile(FUNCTION_FILE, '../../data/input/casadi-functions');

CodeGen = CodeGenerator('casadi_mpc.c', struct('with_header', true));
CodeGen.add(mpc_func_casadi)
CodeGen.add(casadi_dynamics_mpc_dt)

CodeGen.generate();
% mex casadi_3dof_double_integrator.c -DMATLAB_MEX_FILE
disp('...done')
cd ../casadi-matlab
disp('Compile C code to a dll using: gcc -fPIC -shared gen.c -o gen.so')
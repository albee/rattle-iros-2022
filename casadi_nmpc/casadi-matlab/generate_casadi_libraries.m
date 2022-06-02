%{
Script to generate casadi_mpc.so and casadi_robust_tube_mpc.so
Must be run from casadi_definition directory!
%}

clear;

T = 1; % Time horizon, [s]
N = 5; % Number of control intervals over time horizon, [-]
DT = T/N;  % delta-t, [s]
% m = 9.583;  % mass, [kg]
% u_max = 0.4;  % input, [N]
% a_max = u_max/m;  % approximate max acceleration, [m/s^2]

GEN_CODE = 0;

% Generate c exports
[mpc_func_casadi, integrator_func_casadi] = casadi_3dof_double_integrator_tube_mpc(GEN_CODE, T, N);
[mpc_func_casadi_default, ~] = casadi_3dof_double_integrator(GEN_CODE, T, N);

if GEN_CODE == 1
  % Compile to shared library
  cd ../casadi_codegen
  system('gcc -fPIC -shared casadi_robust_tube_mpc.c -o casadi_robust_tube_mpc.so');
  system('gcc -fPIC -shared casadi_mpc.c -o casadi_mpc.so');
  cd ../
  disp('CasADi libraries ready!')
end

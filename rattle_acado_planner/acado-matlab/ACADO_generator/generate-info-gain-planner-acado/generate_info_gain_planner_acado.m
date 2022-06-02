%{ 

Script to generate mid-level receding horizon planner with ACADO.
6 DOF dynamics with products of inertia and center of mass offset, 
This script calls ACADO to generate MEX files of C++ NMPC code.
Edit and run this with correct paths to the ACADO folder 

Inputs:
Correct file path to ACADO Toolkit, see below.

Outputs:
RHP_export folder, containg the MEX version of this code

% MATLAB example (section 4.2.3): http://acado.sourceforge.net/doc/pdf/acado_matlab_manual.pdf
% Example for C integration: http://acado.sourceforge.net/doc/html/db/daf/cgt_getting_started.html

%}
clear all;
close all;

ISS = 1;  %{0, 1}: 1 indicates 6 DOF dynamics
USE_FIM = 1;  %{0, 1}: 1 creates FIM in the cost function (very complicated!)

ACADOtoolkit_PATH = '/usr/local/MATLAB/libraries/ACADOtoolkit';  % Set this to your ACADO Toolkit folder if not already on your MATLAB path!
addpath(genpath(ACADOtoolkit_PATH));
acadoSet('results_to_file', false); 

fprintf('%s\n',...
        'Generating planner with parameters:',...
        strcat({'ISS: '}, string(ISS)),...
        strcat({'USE_FIM: '}, string(USE_FIM)));


%% ACADO declarations and setup
% Declare all differential states, controls, parameters
DifferentialState r(3) v(3) q(4) w(3)  % q is scalar last! [x y z w]. 
if ISS==1 && USE_FIM==1
  DifferentialState psi(45);  % psi = 6+13*|theta|. (mass is reduced for simplicity)
else
  DifferentialState psi(31);  % psi is an "extended state" used for FIM calc (integrated)
end
acado_x = [r; v; q; w];
acado_psi = [psi];  % extended state for FIM computation

Control u(6);  % [F_x F_y F_z T_x T_y T_z], force is in body frame; torque is in body frame about original CM.
acado_u = u;

% roff is [x y z] CM offset from original CM, *_des is desired state
OnlineData mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3);  % online-updated parameters
OnlineData r_des(3) v_des(3) q_des(4) w_des(3);  % online-updated ref setpoint
% I = [I_xx I_xy I_xz;
%      I_xy I_yy I_yz;
%      I_xz I_yz I_zz];  % inertia tensor
acado_p = [mass; I_xx; I_yy; I_zz; I_xy; I_yz; I_xz; roff];

dt = 0.3;  % length of step
N = 40;  % length of horizon

if ISS==1
  sigma_diag = 1.0*ones(13, 1);  % measurement noise covariance, TODO: tune!
else
  sigma_diag = 1.0*ones(9, 1);  % measurement noise covariance, TODO: tune!
end

    
%% Dynamics, f(x, u, p, psi) 
% Note: includes psi to get integration
if ISS == 1
  if USE_FIM == 1
    f = dynamics(acado_x, acado_u, acado_p, acado_psi);
  else
    f = dynamics(acado_x, acado_u, acado_p);
  end
else
  if USE_FIM == 1
    f = dynamics_3DOF(acado_x, acado_u, acado_p, acado_psi);
  else
    f = dynamics_3DOF(acado_x, acado_u, acado_p);
  end
end


%% Cost function
if USE_FIM == 1
  [S, h, SN, hN] = get_weighting_matrices(r, v, q, w, r_des, v_des, q_des, w_des, u, acado_psi, sigma_diag, ISS);
else
  [S, h, SN, hN] = get_weighting_matrices(r, v, q, w, r_des, v_des, q_des, w_des, u);
end


%% Create planner
ocp = create_ocp(N, dt, S, h, SN, hN, u, f);


%% Export planner code
if ISS == 1
  cd '../../RHP_6DOF_export'
else
  cd '../../RHP_export'
end

ocp_export = acado.OCPexport(ocp);
ocp_export.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
ocp_export.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
ocp_export.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
ocp_export.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
ocp_export.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
ocp_export.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
ocp_export.set( 'HOTSTART_QP',                 'YES'             	);
ocp_export.set( 'LEVENBERG_MARQUARDT', 		 1e-5				);  % steps over horizon

% ocp_export.set( 'PRINTLEVEL', 'DEBUG');
% ocp_export.set( 'PRINT_COPYRIGHT', 'YES');

ocp_export.exportCode('./');  % export code and compile, TODO: fix for non-CM system!
copyfile(strcat(ACADOtoolkit_PATH, '/external_packages/qpoases'), './qpoases', 'f'); % supply path to ACADO toolkit

%%
if ISS == 1
  make_acado_solver('RHP_6DOF')  % place in subdirectory
else
  make_acado_solver('RHP')  % place in subdirectory
end
cd '../ACADO_generator';


%% Support functions
% DEPRECATED!
function dot = w_dot_calc(I, w, roff, u)
  %{
  w_dot

  Uses a trick to compute inverse---not supported by Acado
  %}
    detI = @(I) - I(3,3)*I(1,2)^2 + 2*I(1,2)*I(1,3)*I(2,3) - I(2,2)*I(1,3)^2 - I(1,1)*I(2,3)^2 + I(1,1)*I(2,2)*I(3,3);
    adjI = @(I)[    I(2,2)*I(3,3) - I(2,3)^2, I(1,3)*I(2,3) - I(1,2)*I(3,3), I(1,2)*I(2,3) - I(1,3)*I(2,2);
            I(1,3)*I(2,3) - I(1,2)*I(3,3),    I(1,1)*I(3,3) - I(1,3)^2, I(1,2)*I(1,3) - I(1,1)*I(2,3);
            I(1,2)*I(2,3) - I(1,3)*I(2,2), I(1,2)*I(1,3) - I(1,1)*I(2,3),    I(1,1)*I(2,2) - I(1,2)^2]; %invert I matrix. Acado expression gives error with matlab "inv", so had to define explicitly

    inv_I = adjI(I)/detI(I);
    dot = (inv_I)*(u(4:6) - cross(w,I*w) - cross(roff,u(1:3)));
end


% DEPRECATED!
function df_dx = quat_df_dx(q, w)
  % calculates df/dx for the quaternion portion, q, of the dynamics
  df_dx = [ 0     w(3) -w(2) w(1)  q(4) -q(3)  q(2);
           -w(3)   0    w(1) w(2)  q(3)  q(4) -q(1);
            w(2) -w(1)  0    w(3) -q(2)  q(1)  q(4);
           -w(1) -w(2) -w(3)  0   -q(1) -q(2) -q(3)];
end

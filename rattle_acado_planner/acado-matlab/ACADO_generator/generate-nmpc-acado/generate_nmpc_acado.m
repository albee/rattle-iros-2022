%{ 

Script to generate low level NMPC with ACADO.
6 DOF dynamics with products of inertia and center of mass offset.
This script calls ACADO to generate MEX files of C++ NMPC code.
Edit and run this with correct paths to the ACADO folder 

Inputs:
Correct file path to ACADO Toolkit, see below.

Outputs:
NMPC_export folder, containg the MEX version of this code.
%}

clear all; close all;

ACADOtoolkit_PATH = '/usr/local/MATLAB/libraries/ACADOtoolkit';  % Set this to your ACADO Toolkit folder if not already on your MATLAB path!

addpath(genpath(ACADOtoolkit_PATH));  
acadoSet('results_to_file', false); 

%% ACADO declarations and setup
% Declare all differential states, controls, parameters
DifferentialState r(3) v(3) q(4) w(3) % r(3): psi is used for FIM calc

Control u(6)  % [F_x F_y F_z T_x T_y T_z], force is in body frame; torque is in body frame about original CM.

OnlineData mass I_xx I_yy I_zz I_xy I_yz I_xz roff(3) r_des(3) v_des(3) q_des(4) w_des(3) u_des(6)

dt = 0.2;  % length of step
N = 10;  % length of horizon
    
%% Dynamics equations
I = [I_xx I_xy    I_xz;
     I_xy I_yy    I_yz;
     I_xz I_yz    I_zz];
 
% dynamic equations for 6DoF RBD with a frame offset from the COM

v_dot_and_w_dot = ddot_calc_3DOF(I, w, roff, mass, u);

v_dot = [v_dot_and_w_dot(1:2); 0];  % x,y translation
w_dot = [0; 0; v_dot_and_w_dot(3)];  % z rotation

f = [ dot(r); dot(v); dot(q); dot(w)] == ...
    [ v;               ...     % linear velocity @inertial frame, original CM
      Rotmat(q)*v_dot;   % linear acceleration @inertial frame, original CM  
      0.5*H_bar_T(q)*w; ...      % quaternion update, body frame wrt inertial frame
      w_dot];  % angular acceleration @body frame, original CM
   
%% Set up NMPC (ocp) solver
% W_mat = eye(11);  % state error, 13 DoF plus 1 for information term. q_err is compressed down to 1 metric, otherwise dim = 13
W_mat = eye(18);  % state error, for q [13] -1 + u[6] 
WN_mat = eye(12);  % terminal cost, 13 DoF--->12
W = acado.BMatrix(W_mat);
WN = acado.BMatrix(WN_mat);

ocp = acado.OCP(0.0, N*dt, N);  % start time 0, end time, number of intervals: OCP(tStart, tEnd, N)

ocp.minimizeLSQ( W,  [r-r_des; v-v_des; err_q(q,q_des); w-w_des; u-u_des] );  % Minimize error between states, 1 to avoid singularity

ocp.minimizeLSQEndTerm( WN, [r-r_des; v-v_des; err_q(q,q_des); w-w_des] );

% constraints
ocp.subjectTo( -0.35 <= u(1:3) <= 0.35);  % control constraints (forces)

ocp.subjectTo( -0.035 <= u(4:6) <= 0.035);  % control constraints (torques)
ocp.setModel(f);  % constraint from dynamic model

%% Export NMPC code
cd '../../NMPC_export'
mpc = acado.OCPexport( ocp );
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL4'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES'    	);
mpc.set( 'HOTSTART_QP',                 'YES'             	);
mpc.set( 'LEVENBERG_MARQUARDT', 		 1e-5				);  % steps over horizon

mpc.exportCode('../NMPC_export');  % export code and compile
copyfile(strcat(ACADOtoolkit_PATH, '/external_packages/qpoases'), './qpoases', 'f'); % supply path to ACADO toolkit
make_acado_solver('acado_NMPC_6DoF')  % place in subdirectory
cd '../ACADO_generator';

%% Export simulator
cd '../SIM_export'
acadoSet('problemname', 'my_sim');
sim = acado.SIMexport(dt); % sampling time
sim.setModel(f);
% sim.linkMatlabODE('ode_3DoF');

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        10      );
sim.exportCode( './' );
make_acado_integrator('sim_6DoF')  % place in subdirectory
cd '../ACADO_generator'

%% Support functions

% quaternion metric for cost function
function [error_quat] = err_q2(q,q_des)
%     error_quat = sqrt((q'*q_des)^2);
    error_quat = sqrt(2*(1 - sqrt((q'*q_des)^2)));
end

% quaternion error angle calculation for cost function
function [error_quat] = err_q(q,q_des)
    error_quat = invskew(Rotmat(q_des)'*Rotmat(q) - Rotmat(q)'*Rotmat(q_des))/(2*sqrt(1+tr(Rotmat(q_des)'*Rotmat(q))));
end
function [skew_v] = skew(vec)
    skew_v = [0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];  % Cross product matrix
end

function [vec] = invskew(A)
    vec = [A(3,2); A(1,3); A(2,1)];
end
function [Trace] = tr(A)
    Trace =  A(1,1) + A(2,2) + A(3,3);  % Trace
end

% rotation matrix for R_I_B: quaternion uses scalar last convention
function [R] = Rotmat(q)
    R = [q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)),          2*(q(1)*q(3)-q(2)*q(4));...
             2*(q(1)*q(2)-q(3)*q(4)),        -q(1)^2+q(2)^2-q(3)^2 + q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));...
             2*(q(1)*q(3)+q(2)*q(4)),        2*(q(2)*q(3)-q(1)*q(4)),          -q(1)^2-q(2)^2+q(3)^2+q(4)^2]';
end

% quaternion conversion matrix: quaternion uses scalar last convention,
% w_IB expressed in body frame
function out = H_bar_T(q)
    out = [ q(4) q(3) -q(2) -q(1);
              -q(3) q(4) q(1) -q(2);
              q(2) -q(1) q(4) -q(3)]';
end


function alpha = dot_w(I, w, roff, u)
  %{
  dot_w calculation from torque applied about original CM, and force
  applied at real CM, body frame

  Unfortunately, inverse does not work for ACADO online data.
  We could 1.pass the inverse of I, element by element,  as online data each time,
  2. As this is a simple diagonal matrix, directly use 1/diagonals.
  %}
  detI = @(I) - I(3,3)*I(1,2)^2 + 2*I(1,2)*I(1,3)*I(2,3) - I(2,2)*I(1,3)^2 - I(1,1)*I(2,3)^2 + I(1,1)*I(2,2)*I(3,3);
  
  %invert I matrix. Acado expression gives error with matlab "inv", so had to define explicitly
  adjI = @(I)[ I(2,2)*I(3,3) - I(2,3)^2,      I(1,3)*I(2,3) - I(1,2)*I(3,3), I(1,2)*I(2,3) - I(1,3)*I(2,2);
               I(1,3)*I(2,3) - I(1,2)*I(3,3), I(1,1)*I(3,3) - I(1,3)^2,      I(1,2)*I(1,3) - I(1,1)*I(2,3);
               I(1,2)*I(2,3) - I(1,3)*I(2,2), I(1,2)*I(1,3) - I(1,1)*I(2,3), I(1,1)*I(2,2) - I(1,2)^2]; 
  inv_I = adjI(I)/detI(I);
  alpha = (inv_I)*(u(4:6) - cross(w, I*w) - cross(roff, u(1:3)));
end

function ddot = ddot_calc_3DOF(I, w, roff, mass, u)
  %{
  3 DOF version of the complete RBD equations.

  ddot is [vdx vdy wdz]
  %}
  detI = @(I) - I(3,3)*I(1,2)^2 + 2*I(1,2)*I(1,3)*I(2,3) - I(2,2)*I(1,3)^2 - I(1,1)*I(2,3)^2 + I(1,1)*I(2,2)*I(3,3);
  
  %invert I matrix. Acado expression gives error with matlab "inv", so had to define explicitly
  adjI = @(I)[ I(2,2)*I(3,3) - I(2,3)^2,      I(1,3)*I(2,3) - I(1,2)*I(3,3), I(1,2)*I(2,3) - I(1,3)*I(2,2);
               I(1,3)*I(2,3) - I(1,2)*I(3,3), I(1,1)*I(3,3) - I(1,3)^2,      I(1,2)*I(1,3) - I(1,1)*I(2,3);
               I(1,2)*I(2,3) - I(1,3)*I(2,2), I(1,2)*I(1,3) - I(1,1)*I(2,3), I(1,1)*I(2,2) - I(1,2)^2]; 

  force = [u(1), u(2)]';
  torque = [u(6)]';
  cx = roff(1);
  cy = roff(2);
  Izz = I(3,3);
  m = mass;
  wz = w(3);

  U = [force; torque];  % generalized force matrix
  
  M = [m 0 -m*cy;
       0 mass m*cx;
       -m*cy m*cx Izz+m*(cx^2 + cy^2)];
     
  C = [-wz^2*cx*m;
       -wz^2*cy*m;
       0];
     
  inv_M = adjI(M)/detI(M);
  
  ddot = inv_M*(U - C);
end

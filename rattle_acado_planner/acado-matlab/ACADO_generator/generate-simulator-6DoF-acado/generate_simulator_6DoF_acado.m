%{ 
This script calls ACADO to generate MEX files of C++ system
simulator code. Edit and run this with correct paths to the ACADO folder 
ONLY if the dynamics need to be modified.

Inputs:
Correct file path to ACADO Toolkit, see below.

Outputs:
SIM_export folder, containg the MEX version of this code.
%}
clear all;

%addpath(genpath('/home/monica/ACADOtoolkit'));  % Set this to your ACADO Toolkit folder!
addpath(genpath(ACADOtoolkit_PATH));
acadoSet('results_to_file', false); 


%% ACADO declarations and setup
% Declare all differential states, controls, parameters
DifferentialState r(3) v(3) q(4) w(3)
OnlineData mass I_xx I_yy I_zz;  % online-updated values

Control u(6)  % [F_x F_y F_z T_x T_y T_z], force is in inertial frame; torque is in body frame
% Ts = 1/62.5;
dt = 0.3  % length of step


%% Set up simulator

I = [I_xx 0    0;
     0    I_yy 0;
     0    0    I_zz];
 
 
inv_I = [1/I_xx, 0, 0;
          0,    1/I_yy, 0;
          0,   0,      1/I_zz];
 
   
% dynamic equations for 6DoF RBD about CoM
f = [ dot(r);  dot(v);dot(q); dot(w)] == ...
    [ v;               ...
      rotmat(q)*u(1:3)/mass;     ...  % forces
      0.5*H_bar_T(q)*w; ...
      dot_w(I, inv_I, w, u(4:6))];   % torques
   
    
%% Export simulation code
cd '../../SIM_export'
acadoSet('problemname', 'my_sim');
sim = acado.SIMexport(dt); % sampling time
sim.setModel(f);

sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_RIIA5' );
sim.set( 'NUM_INTEGRATOR_STEPS',        10      );
sim.exportCode( './' );
make_acado_integrator('sim_6DoF')  % place in subdirectory
cd '../ACADO_generator'


%% Support functions
function out = H_bar_T(q)
  % quaternion conversion matrix: quaternion uses scalar last convention,
  % w_IB expressed in body frame
  out = [ q(4) q(3) -q(2) -q(1);
          -q(3) q(4) q(1) -q(2);
          q(2) -q(1) q(4) -q(3)]';
end


function out = dot_w(I, inv_I, w, tau)

      
 % dot_w calculation from xyz torque
  out = (-inv_I * cross(w, I*w) + inv_I*tau);
end
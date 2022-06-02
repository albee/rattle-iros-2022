%{
Keenan Albee, Oct-15 2020
Tumbling target dynamcis using CasADi.
See `propagate_tumble_dynamics` for an example.


Inputs:
GEN_CODE: {0, 1} 1 generates C code of CasADi dynamics.

Outputs:
newton_euler_dynamics(x, u, p, dt) - CasADi function
%}

function [newton_euler_dynamics] = casadi_6dof_newton_euler_dynamics(GEN_CODE)
  import casadi.*

  %% Set up symbolic variables
  % Continuous-time dynamics!
  % x = [x; y; z; xd; yd; zd; qx; qy; qz; qw; wx; wy; wz], [m] I, [m/s] I, [-] B wrt I, [rad/s] B frame
  % u = [Fx; Fy; Fz; Tx; Ty; Tz], [N], forces in inertial frame, torques in
  % body frame
  % p = [m; I_xx; I_yy; I_zz]  system mass, inertias

  x = MX.sym('x', 13, 1); % States   
  u = MX.sym('u', 6, 1); % Controls
  p = MX.sym('p', 4, 1); % Parameters
  tf = MX.sym('tf');  % final time of integration, determined by time horizons
  p_stacked = [u; p; tf];

  %% Set up ODE
  % See [Albee 2019], [Aghili 2008]
  rx = x(1);
  ry = x(2);
  rz = x(3);
  vx = x(4);
  vy = x(5);
  vz = x(6);
  qx = x(7);
  qy = x(8);
  qz = x(9);
  qw = x(10);
  wx = x(11);
  wy = x(12);
  wz = x(13);
  m = p(1);
  Ixx = p(2);
  Iyy = p(3);
  Izz = p(4);
  Tx = u(4);
  Ty = u(5);
  Tz = u(6);
  
  % quaternion dynamics  
  w = [wx; wy; wz];
  H = [ qw,  qz, -qy, -qx;
       -qz,  qw,  qx, -qy;
        qy, -qx,  qw, -qz]; % assumes scalar LAST! quaternion is B wrt I
  qd = 1/2*H'*w;

  % tumble angular velocity dynamics
%   phi = [px*wy*wz;
%          py*wx*wz;
%          pz*wx*wy];
%        
%   B = [1, 0, 0;
%        0, (1+py)/(1+px), 0;
%        0, 0, (1+pz)/(1-px)];
%      
%   wd = phi + B*(u(4:6)/Ixx);  % must correct to be per unit I_xx!!
  wd = [(Tx - (Izz - Iyy)*wy*wz)/Ixx;
        (Ty - (Ixx - Izz)*wz*wx)/Iyy;
        (Tz - (Iyy - Ixx)*wx*wy)/Izz];

  xdot_rot = [qd; wd];  % rotational dynamics
  
  % translational dynamics
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
  
  r = [rx; ry; rz];
  v = [vx; vy; vz];
  xdot_lin = A*[r; v] + B*u(1:3);  % linear dynamics
  
  xdot = [xdot_lin; xdot_rot];

  f = Function('f', {x, u, p}, {xdot}, {'x','u','p'}, {'ode'});  % continuous dynamics function
  
  %% Create an integrator
  % Integrator to discretize the system
  intg_options = struct;
  intg_options.tf = 1.0;  % final time, modified by input time
  intg_options.simplify = true;
  intg_options.number_of_finite_elements = 4;

  % DAE problem structure
  dae = struct;
  dae.x = x;         % What are states?
  dae.p = p_stacked;    % What are parameters (=fixed during the integration horizon)?
  dae.ode = f(x, u, p)*tf;  % Expression for the right-hand side

  intg = integrator('intg', 'rk', dae, intg_options);

  % x0 = [0;1];
  % res = intg('x0',[0;1],'p',0);  % Easier to identify inputs, but bloated API
  % res.xf;  % view final state

  % Simplify API to (x,u)->(x_next)
  res = intg('x0', x, 'p', p_stacked); % Evaluate with symbols
  x_next = res.xf;  % symbolically defined
  newton_euler_dynamics = Function('newton_euler_dynamics', {x, u, p, tf}, {x_next}, {'x', 'u', 'p', 'tf'}, {'x_next'});  % function call to integrate: continuous over timestep dt

  % sim = integrator_func_casadi.mapaccum(N);  % take x0 and |u|xN matrix, output N states propagated forward

  %% Code gen
  if GEN_CODE == 1
  %     mpc_func_casadi.generate('casadi_3dof_double_integrator', struct('mex',true, 'with_header', true));
      CodeGen = CodeGenerator('casadi_newton_euler_dynamics.c', struct('with_header', true));
      CodeGen.add(intg)
      CodeGen.add(newton_euler_dynamics)

      disp('Generating C code...')
      CodeGen.generate();
      % mex casadi_3dof_double_integrator.c -DMATLAB_MEX_FILE
      disp('...done')

  %     format long
  %     codegen_demo('mpc_func_casadi',x0)
  end
end


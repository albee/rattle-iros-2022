function[psi_dot] = calc_psidot(r, v, q, w, mass, I, roff, u, psi)
  %{
  Calculation of dx/dth propagation of psi, psi_dot (extended state). 6DOF!

  For more details, consult: 
  [1]A. D. Wilson, J. A. Schultz, A. R. Ansari, and T. D. Murphey,
  “Real-time trajectory synthesis for information maximization using sequential
   action control and least-squares estimation,” in 2015 IEEE/RSJ international
   conference on intelligent robots and systems (IROS), 2015, pp. 4935–4940.

  @param x, v, q, w: acado state symbolics
  @param mass, roff, I: acado parameter symbolics
  @param u: acado input symbolics
  @param psi: acado dx/dth state symbolic

  @return psi_dot: psi extended states
  %}  
  v_dot_and_w_dot = velocity_dynamics(mass, I, roff, w, u);
  v_dot = v_dot_and_w_dot(1:3);
  w_dot = v_dot_and_w_dot(4:6);
  
  % mult by R(q) is necessary (???), but applies torque inputs even
  % for zero orientation error, depending on the weight matrix W.
  % reduced dynamics for mass jacobians - !!!everything in body frame!!!
  f = [v;               ...       % linear velocity INERTIAL frame, original CM
       v_dot;   % linear acceleration BODY frame, original CM  
       0.5*H_bar_T(q)*w; ...      % quaternion update, BODY frame wrt INERTIAL frame
       w_dot];                    % BODY frame
  
  % Jacobians
  % df/dx == d/dx[dx/dt]
  df_x1 = simplify(jacobian(f(1:6), [r(1:3); v(1:3)]));  % translational (6x6)  TODO: this should actually use full state for 6DOF
  df_x2 = simplify(jacobian(f, [r; v; q; w]));  % entire state (13x13)
  disp('...df/dx calculated.')

  % df/dtheta == d/dtheta[dx/dt]
  df_theta_m = simplify(jacobian(f(1:6), mass));  % mass (6x1)
  df_theta_ixx = simplify(jacobian(f, I(1,1)));  % I_xx (13x1)
  df_theta_iyy = simplify(jacobian(f, I(2,2)));  % I_yy (13x1)
  df_theta_izz = simplify(jacobian(f, I(3,3)));  % I_zz (13x1)
  disp('...df/dtheta calculated.')

  % psi dots! (d/dt [dx/dtheta])
  psi_dot(1:6,1) = df_x1*psi(1:6) + df_theta_m;  %df/dmass (6x1)
%   psi_dot(5:13,1) = df_x2*psi(5:13) + df_theta_roff(:,1);  %df/d_cx (9x1)
%   psi_dot(14:22,1) = df_x2*psi(14:22) + df_theta_roff(:,2);  %df/d_cy (9x1)
  psi_dot(7:19,1) = df_x2*psi(7:19) + df_theta_ixx;  %df/dI_xx (13x1)
  psi_dot(20:32,1) = df_x2*psi(20:32) + df_theta_iyy;  %df/dI_yy (13x1)
  psi_dot(33:45,1) = df_x2*psi(33:45) + df_theta_izz;  %df/dI_zz (13x1)
  disp('...psi_dot calculated.')
  
%   psi_dot(1:6,1) = zeros(6,1);  %df/dmass (4x1)
%   psi_dot(7:19,1) = zeros(13,1);  %df/dI_xx (13x1)
%   psi_dot(20:32,1) = zeros(13,1);  %df/dI_yy (13x1)
%   psi_dot(33:45,1) = zeros(13,1);  %df/dI_zz (13x1)
end
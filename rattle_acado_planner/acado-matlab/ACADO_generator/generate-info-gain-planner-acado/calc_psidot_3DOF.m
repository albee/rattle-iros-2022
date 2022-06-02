function[psi_dot] = calc_psidot_3DOF(r, v, q, w, mass, I, roff, u, psi)
  %{
  Calculation of dx/dth propagation of psi, psi_dot (extended state). 3DOF!

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
  v_dot_and_w_dot = velocity_dynamics_3DOF(mass, I, roff, w, u);
  v_dot = [v_dot_and_w_dot(1:2); 0];  % x,y translation
  w_dot = [0; 0; v_dot_and_w_dot(3)];  % z rotation
  
  % mult by R(q) is necessary (???), but applies torque inputs even
  % for zero orientation error, depending on the weight matrix W.
  % reduced dynamics for mass jacobians - !!!everything in body frame!!!
  f = [v;               ...       % linear velocity INERTIAL frame, original CM
       v_dot;   % linear acceleration BODY frame, original CM  
       0.5*H_bar_T(q)*w; ...      % quaternion update, BODY frame wrt INERTIAL frame
       w_dot];                    % BODY frame

  % downselected dynamics for 3DOF
  f_3DOF = [f(1); ...  % x
            f(2); ...  % y
            f(4); ...  % vx
            f(5); ...  % vy
            0.5*w(3)*q(2);...  % qx
            -0.5*w(3)*q(1);... % qy
            0.5*w(3)*q(4);...  % qz
            -0.5*w(3)*q(3); ... % qw
            f(13)];  % wz
  
  % Jacobians (3DOF)
  % df/dx == d/dx[dx/dt]
  df_x1 = simplify(jacobian(f_3DOF(1:4), [r(1:2); v(1:2)]));  % translational (4x4)
  df_x2 = simplify(jacobian(f_3DOF, [r(1:2); v(1:2); q; w(3)]));  % entire 3DOF state (9x9)

  % df/dtheta == d/dtheta[dx/dt]
  df_theta_m = simplify(jacobian(f_3DOF(1:4), mass));  % mass (4x1)
  df_theta_roff = simplify(jacobian(f_3DOF, [roff(1); roff(2)]));  % cx, cy (13x2)
  df_theta_izz = simplify(jacobian(f_3DOF, I(3,3)));  % I_zz (13x1)

  % psi dots! (d/dt [dx/dtheta])
  psi_dot(1:4,1) = df_x1*psi(1:4) + df_theta_m;  %df/dmass (4x1)
%   psi_dot(5:13,1) = df_x2*psi(5:13) + df_theta_roff(:,1);  %df/d_cx (9x1)
%   psi_dot(14:22,1) = df_x2*psi(14:22) + df_theta_roff(:,2);  %df/d_cy (9x1)
  psi_dot(5:13,1) = 0.0;
  psi_dot(14:22,1) = 0.0;
  psi_dot(23:31,1) = df_x2*psi(23:31) + df_theta_izz;  %df/dI_zz (9x1)
end
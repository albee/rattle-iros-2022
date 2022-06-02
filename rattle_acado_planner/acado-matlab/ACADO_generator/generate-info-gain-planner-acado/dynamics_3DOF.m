function f = dynamics_3DOF(acado_x, acado_u, acado_p, acado_psi)
  %{
  Dynamics for 6DOF Newton-Euler

  @param:
  acado_x: state vector of acado vars
  acado_u: input vector of acado vars
  acado_p: parameter vector of acado vars

  @return:
  f : expression for differentiated state vector
  %}
  r = acado_x(1:3);
  v = acado_x(4:6);
  q = acado_x(7:10);
  w = acado_x(11:13);
  
  u = acado_u;

  mass = acado_p(1);
  I = [acado_p(2) acado_p(5) acado_p(7);
       acado_p(5) acado_p(3) acado_p(6);
       acado_p(7) acado_p(6) acado_p(4)];  % inertia tensor
  roff = acado_p(8:10);
  
  psi = acado_psi;
  
  % 3 DOF dynamics
  v_dot_and_w_dot = velocity_dynamics_3DOF(mass, I, roff, w, u);
  v_dot = [v_dot_and_w_dot(1:2); 0];  % x,y translation
  w_dot = [0; 0; v_dot_and_w_dot(3)];  % z rotation
  
  if nargin == 3  % no psi
    f = [ dot(r); dot(v); dot(q); dot(w)] == ...
        [ v;               ...       % linear velocity INERTIAL, original CM
          v_dot;   % linear acceleration BODY frame, original CM  
          0.5*H_bar_T(q)*w; ...      % quaternion update, BODY frame wrt INERTIAL frame
          w_dot; ...  % angular acceleration BODY frame, original CM
        ];
  elseif nargin == 4  % FIM version
    f = [ dot(r); dot(v); dot(q); dot(w); dot(psi)] == ...
      [ v;               ...       % linear velocity INERTIAL frame, original CM
        v_dot;   % linear acceleration BODY frame, original CM  
        0.5*H_bar_T(q)*w; ...      % quaternion update, BODY frame wrt inertial frame
        w_dot; ...  % angular acceleration BODY frame, original CM
        calc_psidot(r, v, q, w, mass, I, roff, u, psi)
      ];
  end
  

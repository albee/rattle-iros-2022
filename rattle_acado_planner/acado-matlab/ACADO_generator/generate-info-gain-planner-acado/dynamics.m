function f = dynamics(acado_x, acado_u, acado_p, acado_psi)
  %{
  Dynamics for 6DOF Newton-Euler

  @return f: expression for differentiated state vector
  %}
  disp('Generating system dynamics...')
  r = acado_x(1:3);
  v = acado_x(4:6);
  q = acado_x(7:10);
  w = acado_x(11:13);
  
  u = acado_u;
  
  mass = acado_p(1);
    
%   % offset CM and unknown principal axes
  I = [acado_p(2) acado_p(5) acado_p(7);
       acado_p(5) acado_p(3) acado_p(6);
       acado_p(7) acado_p(6) acado_p(4)];  % inertia tensor
%   roff = acado_p(8:10);
  roff = [acado_p(8); 0; 0];
  
  % coincident CM and known principal axes
%   I = [acado_p(2) 0 0;
%        0 acado_p(3) 0;
%        0 0 acado_p(4)];  % inertia tensor
%   roff = [0; 0; 0];
    
  if nargin == 4  % include psi
    psi = acado_psi;
  end
  
  % 6 DOF dynamics
  % dynamic equations for 6DOF RBD, including FIM psi-dot ("extended state")
  v_dot_and_w_dot = velocity_dynamics(mass, I, roff, w, u);
  v_dot = v_dot_and_w_dot(1:3);
  w_dot = v_dot_and_w_dot(4:6);
  disp('...velocity dynamics calculated.')
  
  if nargin == 3  % no psi
    f = [ dot(r); dot(v); dot(q); dot(w)] == ...
        [ v;               ...       % linear velocity @inertial frame, original CM
          v_dot;   % linear acceleration @BODY frame, original CM  
          0.5*H_bar_T(q)*w; ...      % quaternion update, body frame wrt inertial frame
          w_dot; ...  % angular acceleration @body frame, original CM
        ];  
  elseif nargin == 4  % include psi
    f = [ dot(r); dot(v); dot(q); dot(w); dot(psi)] == ...
        [ v;               ...       % linear velocity @inertial frame, original CM
          v_dot;   % linear acceleration @BODY frame, original CM  
          0.5*H_bar_T(q)*w; ...      % quaternion update, body frame wrt inertial frame
          w_dot; ...  % angular acceleration @body frame, original CM
          calc_psidot(r, v, q, w, mass, I, roff, u, psi)];
  end
  disp('...dynamics calculated.')
end
  

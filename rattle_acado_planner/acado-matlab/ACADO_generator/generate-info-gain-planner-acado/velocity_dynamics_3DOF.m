function xddot = dynamics_3DOF(mass, I, roff, w, u)
  %{
  3 DOF version of the complete RBD equations.

  Dynamics are in the classic form:
  M*xdd + C*xd = U;

  @param I: inertia tensor, acado symbolic
  @param w: angular velocity, acado symbolic
  @param roff: CM offset, acado symbolic
  @param mass: mass, acado symbolic
  @param u: inputs in BODY frame

  @return xddot
  xddot: state vector deriv, but only including vdot and wdot terms
        [vdx vdy wdz]'
  %}
  force = [u(1), u(2)]';
  torque = [u(6)]';
  cx = roff(1);
  cy = roff(2);
  Izz = I(3,3);
  m = mass;
  wz = w(3);

  U = [force; torque];  % generalized force matrix
  
  M = [m 0 -m*cy;
       0 m m*cx;
       -m*cy m*cx Izz+m*(cx^2 + cy^2)];
     
  C = [-wz^2*cx*m;
       -wz^2*cy*m;
       0];
     
  inv_M = inva(M);
%   inv_M = inv(M)
  
  xddot = inv_M*(U - C);  % [vdx vdy wdz]'
end
function xddot = velocity_dynamics(mass, I, roff, w, u)
  %{
  \dot{x} calculation from torque applied about original CM, and force
  applied at real CM, body frame.

  ACADO does not support matrix inversion, so the 3x3 matrix inverse is
  manually specified via the det() adj().

  Dynamics are in the classic form:
  M*xdd + C*xd = U;

  @return:
  xddot = state vector deriv, but only including the vdot and wdot terms
         [vdx vdy vdz wdx wdy wdz]
  %}  
  % cross product matrices (left hand side)
  roff_cross = [0, -roff(3), roff(2);
                roff(3), 0, -roff(1);
                -roff(2), roff(1), 0];
              
  w_cross = [0, -w(3), w(2);
             w(3), 0, -w(1);
            -w(2), w(1), 0];
          
  roff_vec = [roff(1), roff(2), roff(3)]';
  w_vec = [w(1), w(2), w(3)]';
  
  M11 = mass*eye(3);  % [3x3]
  M12 = -mass*roff_cross;  % [3x3]
  M21 = mass*roff_cross;  % [3x3]
  M22 = (I - mass*roff_cross*roff_cross);  % [3x3]
  
  force = [u(1), u(2), u(3)]';
  torque = [u(4), u(5), u(6)]';
    
  C1 = mass*w_cross*w_cross*roff_vec;
  C2 = w_cross*(I - mass*roff_cross*roff_cross)*w_vec;
          
  M = [M11, M12;
       M21, M22];  % spatial inertia matrix
     
  U = [force; torque];  % generalized force matrix
  
  C = [C1; C2];  % "Coriolis" rotational correction matrix
     
  inv_M = inva6(M);  % we need to invert M manually for ACADO. Verified to be correct.
  
%   inv_M
%   U
%   C
      
  xddot = (inv_M)*(U - C);  % [vdx vdy vdz wdx wdy wdz], using crazy block inversion trick
end
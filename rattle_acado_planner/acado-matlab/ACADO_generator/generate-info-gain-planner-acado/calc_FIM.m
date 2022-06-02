function [FIM_m, FIM_ixx, FIM_iyy, FIM_izz] = calc_FIM(psi, sigma_diag)
  %{
  Calculates Fisher Information for each parameter.
 
  Currently  measuring velocities. Add pos and quaternions soon
  
  3DoF: [rx, ry, vx, vy, qx, qy, qz, qw, wz]

  Mass has no effect on rotation so mass jacobian only use
  position dynamics to save extended (psi) states.

  For more details, consult: 
  [1]A. D. Wilson, J. A. Schultz, A. R. Ansari, and T. D. Murphey,
  “Real-time trajectory synthesis for information maximization using sequential
   action control and least-squares estimation,” in 2015 IEEE/RSJ international
   conference on intelligent robots and systems (IROS), 2015, pp. 4935–4940.

  @param ISS: {0, 1}: 1 indicates use ISS FIM [m Ixx Iyy Izz]

  @return FIM_*: FIM for parameter *
  %}
  
  %(1) gamma = dh_dx*psi + dh_dtheta

  dh_dx = eye(13);  % measurement func wrt x, assuming full state feedback (9x9)  
  % sigma_diag = ones(9, 1)*1e-7;     % measurement covariance, ~10^-7 from EKF
  sigma_inv = inv(diag(sigma_diag)); % 
  
  % gather extended states (|x| = 13 for 6DOF)
  psi_m = [psi(1:6); zeros(7,1)];  % add on missing ones for mass
  psi_ixx = psi(7:19);
  psi_iyy = psi(20:32);
  psi_izz = psi(33:45);

  dx_dtheta = [psi_m psi_ixx psi_iyy psi_izz];  % psi (13x4)
  
  % the infamous FIM!
  % (2) FI = gamma' * sigma_inv * gamma
  FIM = (dh_dx*dx_dtheta)'*sigma_inv*(dh_dx*dx_dtheta);  %(13x13) (13x4)' (13x13) (13x13) (13x4)
  FIM_m = FIM(1,1);
  FIM_ixx= FIM(2,2);
  FIM_iyy = FIM(3,3);
  FIM_izz = FIM(4,4);
  disp('...FIM contribution calculated.');
end
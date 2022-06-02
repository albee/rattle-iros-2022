%{
Use the z_poly_calc Python module using MATLAB's interface to call mRPI
calc.
%}
function [K_dr, Autight, butight, AZ, bZ] = calc_mRPI_python(mass, DT, noise, gains_dr)
  %% Python call, deprecated
  z_poly = py.z_poly_calc.ZPoly();
  
  % get discrete A and B matrices 
  W = [noise.r1, noise.r2, noise.r3, noise.v1, noise.v2, noise.v3]';
  W = mat2np(W);  
  U_max = mat2np([0.4, 0.4, 0.4]');  % input, [N]
  dt = double(DT);
  mass = double(mass);
  
  Q_LQR = [gains_dr.Q1, gains_dr.Q2, gains_dr.Q3, gains_dr.Q4, gains_dr.Q5, gains_dr.Q6];
  R_LQR = [gains_dr.R1, gains_dr.R2, gains_dr.R3];
  Q_LQR = py.numpy.diag(Q_LQR);
  R_LQR = py.numpy.diag(R_LQR);
  
  output = z_poly.calc_MRPI_and_K_dr(W, dt, U_max, mass, Q_LQR, R_LQR);
  
  K_dr = np2mat(output{1});
  Autight = np2mat(output{2});
  butight = np2mat(output{3});
  AZ = np2mat(output{4});
  bZ = np2mat(output{5});
end
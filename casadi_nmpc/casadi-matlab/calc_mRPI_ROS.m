%{
Call the Python implementation of the mRPI calculation. Uses ROS srv
interface.
mass: system mass
DT: timestep
W: a 6-vector noise struct
gains_dr: a struct of ancillary controller gains
%}

function [K_dr, Autight, butight, AZ, bZ] = calc_mRPI_ROS(u_mag, mass, DT, noise, gains_dr)
  %% set up the mRPI call    
  w = rosmessage('std_msgs/Float64MultiArray');
  w.Data = [noise.r1, noise.r2, noise.r3, noise.v1, noise.v2, noise.v3];
  w.Layout.Dim = [rosmessage('std_msgs/MultiArrayDimension'), rosmessage('std_msgs/MultiArrayDimension')];
  w.Layout.Dim(1).Size = 6;
  w.Layout.Dim(2).Size = 1;

  u_max = rosmessage('std_msgs/Float64MultiArray');
  u_max.Data = [u_mag(1), u_mag(2), u_mag(3)];  % [x y z] F max
  u_max.Layout.Dim = [rosmessage('std_msgs/MultiArrayDimension'), rosmessage('std_msgs/MultiArrayDimension')];
  u_max.Layout.Dim(1).Size = 3;
  u_max.Layout.Dim(2).Size = 1;
  
  Q_pos_anc = gains_dr.Q1;
  Q_vel_anc = gains_dr.Q4;
  R_anc = gains_dr.R1;
  
  % create srv "message"...even though it's a srv it still uses msg format
  req = rosmessage('rattle_msgs/RattleSrvMRPI');
  req.W = w;
  req.UMax = u_max;
  req.Dt = DT;
  req.Mass = mass;
  req.QPosAnc = Q_pos_anc;
  req.QVelAnc = Q_vel_anc;
  req.RAnc = R_anc;

  z_poly_calc_srv = rossvcclient('/mrpi');
  % res = z_poly_calc_srv(w, u_max, dt, mass, Q_pos_anc, Q_vel_anc, R_anc);  % srv call

  %% call the mRPI service
  res = call(z_poly_calc_srv, req);
  res
  K_dr = Float64MultiArray2Mat(res.K);
  Autight = Float64MultiArray2Mat(res.Au);
  butight = Float64MultiArray2Mat(res.Bu);
  AZ = Float64MultiArray2Mat(res.AZ);
  bZ = Float64MultiArray2Mat(res.BZ);
end

function mat = Float64MultiArray2Mat(msg)
  %{ 
  Convert float64multiarray to matlab matrix
  %}
  sz = [msg.Layout.Dim.Size];
  data = msg.Data;
  mat = reshape(data, sz(2), sz(1))';  % required for weird array ordering by std_msgs
end


%  % default test values
%   wx = 0.02;
%   wy = 0.02;
%   wz = 0.02;
%   wdx = 0.02;
%   wdy = 0.02;
%   wdz = 0.02;

%   dt = 0.2;
%   mass = 10.0;
%   Q_pos_anc = 5;
%   Q_vel_anc = 50;
%   R_anc = 0.1;
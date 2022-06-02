function out = H_bar_T(q)
  % quaternion conversion matrix: quaternion uses scalar last convention,
  % w_IB expressed in body frame
  out = [ q(4) q(3) -q(2) -q(1);
          -q(3) q(4) q(1) -q(2);
          q(2) -q(1) q(4) -q(3)]';
end

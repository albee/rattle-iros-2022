function [error_quat] = err_q(q, q_des)
  % quaternion error angle calculation for cost function
  % output: euler angle error
    error_quat = invskew(rotmat(q_des)'*rotmat(q) - rotmat(q)'*rotmat(q_des))/(2*sqrt(1+tr(rotmat(q_des)'*rotmat(q))));
end
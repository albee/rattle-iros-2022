function [error_quat] = err_q2(q, q_des)
  % quaternion metric for cost function
  %     error_quat = sqrt((q'*q_des)^2);
    error_quat = sqrt(2*(1 - sqrt((q'*q_des)^2)));
end
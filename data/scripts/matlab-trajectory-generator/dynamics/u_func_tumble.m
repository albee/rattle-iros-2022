%{
Inputs
t - time
u - suggested input history

u_hist and t_hist MUST be set in the workspace!

Outputs
u - input for time t  [Fx; Fy; Fz; Tx; Ty; Tz], [N], forces in inertial frame, torques in body frame
%}
function u_out = u_func_tumble(t)
  global u_hist;
  global t_hist;
  [~, idx] = min(abs(t - t_hist));
  u = u_hist(idx, :);
  u_out = u';
%     u_out = [0.1, 0, 0, 0, 0, 0];
end
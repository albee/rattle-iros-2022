function [S, h, SN, hN] = get_weighting_matrices(r, v, q, w, r_des, v_des, q_des, w_des, u, psi, sigma_diag, ISS)
  %{
  Create weighting matrices the optimization problem.

  @param ISS: {0, 1}, 1 indidcates ISS FIM dynamics
  %}
  if nargin == 12  % FIM version
    if ISS == 1
      W_mat = eye(22);  % state error and information weighting (4 + 12 + 6)
      WN_mat = eye(12);  % terminal cost, 13 DoF

      [FIM_m, FIM_ixx, FIM_iyy, FIM_izz] = calc_FIM(psi, sigma_diag);  % get FIM from psi (df/dt) propagation
      eul = err_q(q, q_des);

      % [FIM_m, FIM_cx, FIM_cy, FIM_izz, r(3), v(3), q(3), w(3), u(6)], |W|=22
      % Minimize inverse FIM + error between states, 1 to avoid singularity
      S = acado.BMatrix(W_mat);  % this MUST be a BMatrix or it cannot be set
      h = [1/(FIM_m + 1.0), 1/(FIM_ixx + 1.0), 1/(FIM_iyy + 1.0), 1/(FIM_izz + 1.0), ...
           r(1)-r_des(1), r(2)-r_des(2), r(3)-r_des(3), ...
           v(1)-v_des(1), v(2)-v_des(2), v(3)-v_des(3), ...
           eul(1), eul(2), eul(3), ...
           w(1)-w_des(1), w(2)-w_des(2), w(3)-w_des(3), ...
           u(1), u(2), u(3), u(4), u(5), u(6)]';
      ref = zeros(1, 22);

      % [r(3), v(3), q(3), w(3)], |W|=12
      SN = acado.BMatrix(WN_mat);
      hN = [r(1)-r_des(1), r(2)-r_des(2), r(3)-r_des(3), ...
            v(1)-v_des(1), v(2)-v_des(2), v(3)-v_des(3), ...
            eul(1), eul(2), eul(3), ...
            w(1)-w_des(1), w(2)-w_des(2), w(3)-w_des(3)]';
      refN = zeros(1, 12);
    else
      W_mat = eye(22);  % state error and information weighting (4 + 12 + 6)
      WN_mat = eye(12);  % terminal cost, 13 DoF

      [FIM_m, FIM_cx, FIM_cy, FIM_izz] = calc_FIM_3DOF(psi, sigma_diag);  % get FIM from psi (df/dt) propagation
      eul = err_q(q, q_des);

      % [FIM_m, FIM_cx, FIM_cy, FIM_izz, r(3), v(3), q(3), w(3), u(6)], |W|=22
      % Minimize inverse FIM + error between states, 1 to avoid singularity
      S = acado.BMatrix(W_mat);  % this MUST be a BMatrix or it cannot be set
      h = [1/(FIM_m + 1.0), 1/(FIM_cx + 1.0), 1/(FIM_cy + 1.0), 1/(FIM_izz + 1.0), ...
           r(1)-r_des(1), r(2)-r_des(2), r(3)-r_des(3), ...
           v(1)-v_des(1), v(2)-v_des(2), v(3)-v_des(3), ...
           eul(1), eul(2), eul(3), ...
           w(1)-w_des(1), w(2)-w_des(2), w(3)-w_des(3), ...
           u(1), u(2), u(3), u(4), u(5), u(6)]';
      ref = zeros(1, 22);

      % [r(3), v(3), q(3), w(3)], |W|=12
      SN = acado.BMatrix(WN_mat);
      hN = [r(1)-r_des(1), r(2)-r_des(2), r(3)-r_des(3), ...
            v(1)-v_des(1), v(2)-v_des(2), v(3)-v_des(3), ...
            eul(1), eul(2), eul(3), ...
            w(1)-w_des(1), w(2)-w_des(2), w(3)-w_des(3)]';
      refN = zeros(1, 12);
    end
  else  % no FIM version
    W_mat = eye(18);  % state error and information weighting
    WN_mat = eye(12);  % terminal cost, 13 DoF

    eul = err_q(q, q_des);

    % [FIM_m, FIM_cx, FIM_cy, FIM_izz, r(3), v(3), q(3), w(3), u(6)], |W|=22
    % Minimize inverse FIM + error between states, 1 to avoid singularity
    S = acado.BMatrix(W_mat);
    h = {r(1)-r_des(1), r(2)-r_des(2), r(3)-r_des(3), ...
         v(1)-v_des(1), v(2)-v_des(2), v(3)-v_des(3), ...
         eul(1), eul(2), eul(3), ...
         w(1)-w_des(1), w(2)-w_des(2), w(3)-w_des(3), ...
         u(1), u(2), u(3), u(4), u(5), u(6)};
    ref = zeros(1, 18);

    % [r(3), v(3), q(3), w(3)], |W|=12
    SN = acado.BMatrix(WN_mat);
    hN = {r(1)-r_des(1), r(2)-r_des(2), r(3)-r_des(3), ...
          v(1)-v_des(1), v(2)-v_des(2), v(3)-v_des(3), ...
          eul(1), eul(2), eul(3), ...
          w(1)-w_des(1), w(2)-w_des(2), w(3)-w_des(3)};
    refN = zeros(1, 12);
  end
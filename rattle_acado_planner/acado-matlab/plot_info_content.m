function plot_info_content(psi_plan, sigma_diag, dh_dx, ISS)
  %{
  Using psi, Sigma, and dh_dx, find the information content of a
  trajectory. Returns diagonal of the Fisher Information matrix.

  @param psi_plan: [31 x N] of psi states

  Optional plot of info data.
  %}
  % measurements (?) v_x, v_y, w_z
  inv_sigma = inv(diag(sigma_diag));

  FIM_diag_hist = zeros(4, size(psi_plan, 1));
  FIM_diag_total = zeros(4, 1);  % predicted FIM over horizon

  if ISS == 0
    for k = 1:size(psi_plan, 1)
      psi = psi_plan(k, :);
      psi_matrix_col1 = [psi(1:4) zeros(1,5)]';  % reduced state mass

      psi_matrix = [ psi_matrix_col1 psi(5:13)' psi(14:22)' psi(23:31)'];  % [mass cx, cy, inertia] (9x4)

      FIM = (dh_dx*psi_matrix)'*inv_sigma*(dh_dx*psi_matrix);  % (4x4)
      FIM_diag_hist(:, k) = diag(FIM);
      FIM_diag_total = FIM_diag_total + diag(FIM);
    end
  else
    for k = 1:size(psi_plan, 1)
      psi = psi_plan(k, :);
      psi_matrix_col1 = [psi(1:6) zeros(1,7)]';  % reduced state mass

      psi_matrix = [ psi_matrix_col1 psi(7:19)' psi(20:32)' psi(33:45)'];  % [mass Ixx Iyy Izz] (13x4)

      FIM = (dh_dx*psi_matrix)'*inv_sigma*(dh_dx*psi_matrix);  % (4x4)
      FIM_diag_hist(:, k) = diag(FIM);
      FIM_diag_total = FIM_diag_total + diag(FIM);
    end
  end

  if ISS == 0
    figure;
    subplot(2, 2, 1)
    plot(FIM_diag_hist(1, :))  % mass
    title('$m$')
    subplot(2, 2, 2)
    plot(FIM_diag_hist(2, :))  % c_x
    title('$c_x$')
    subplot(2, 2, 3)
    plot(FIM_diag_hist(3, :))  % c_y
    title('$c_y$')
    subplot(2, 2, 4)
    plot(FIM_diag_hist(4, :))  % I_zz
    title('$I_{zz}$')
  else
    figure;
    subplot(2, 2, 1)
    plot(FIM_diag_hist(1, :))  % mass
    title('$m$')
    subplot(2, 2, 2)
    plot(FIM_diag_hist(2, :))  % I_xx
    title('$I_{xx}$')
    subplot(2, 2, 3)
    plot(FIM_diag_hist(3, :))  % I_yy
    title('$I_{yy}$')
    subplot(2, 2, 4)
    plot(FIM_diag_hist(4, :))  % I_zz
    title('$I_{zz}$')

  figure
  plot(0:0.3:12, FIM_diag_hist(4, :))  % I_zz
  title('$I_{zz}$')
  ylabel('Information Content')
  xlabel('Time $[s]$')
end
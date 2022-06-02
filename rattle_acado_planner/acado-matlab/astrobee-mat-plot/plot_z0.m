function plot_z0(AZ, bZ, X_hist, x0_mpc_hist, X_des_hist)
  %{
  Plot the initial state constraint for a single axis (2D phase plane plot).
  %}
  close all;
  figure;
  hold on;
  
  xyz = 1;  % {0=x, 1=y, 2=z}
  
  mRPI = Polyhedron('A', AZ(:, [1,4]), 'b', bZ);
  
  rx = X_hist(1+xyz, :);
  rdx = X_hist(4+xyz, :);
  
  len = length(X_des_hist);
  x0_mpc_hist = x0_mpc_hist(:, 2:len);  % get rid of start/end erroneous values
  
  r0x_mpc = x0_mpc_hist(1+xyz, :);
  r0dx_mpc = x0_mpc_hist(4+xyz, :);
  
  rx_des = X_des_hist(1+xyz, :);
  rdx_des = X_des_hist(4+xyz, :);
  
  % Plot all polytopes
  for idx=1:1:size(x0_mpc_hist, 2)
    h1 = plot_polytope_offset(mRPI, x0_mpc_hist([1+xyz, 4+xyz], idx)');
  end
  
  ax1 = plot(rx, rdx, 'Color',  'red');  % real state
  ax2 = plot(rx_des, rdx_des, ':', 'Color',  'black');  % desired state
  ax3 = plot(r0x_mpc, r0dx_mpc, 'Color', 'green');  % mpc state
  legend([ax1, ax2, ax3, h1], {'$\mathbf{x}_{real}$', '$\mathbf{x}_{des}$', '$\mathbf{x}_{mpc}$', 'RPI'});
    
  axis([-0.5, 0.5, -0.1, 0.1]);
  title('Phase Plot of Z-Axis Tube MPC Tracking');
  ylabel('$\dot{x}$ [m/s]');
  xlabel('$x$ [m]');
end

function handle = plot_polytope_offset(polytope, offset)
  %{
  Plot 2D polytope with offset
  %}
  verts = polytope.V + offset;
  verts_idx = convhull(verts);
  verts = verts(verts_idx, :);
  handle = patch(verts(:, 1), verts(:, 2), [0,0,0]+0.8)
end
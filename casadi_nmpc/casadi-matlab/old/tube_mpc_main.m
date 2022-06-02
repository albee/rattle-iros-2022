uc_bound = ones(4,3);
prepare_tube_mpc;

%% Prepare for results
x_stored = [];
u_stored = [];
x_stored = mpc.x';  % initial state (stored as a row vector for plotting)

%% Simulate for the Nf timesteps of the ref traj: the first reference is the current state
for t_idx = 1:1:Nf  % indices of the ref traj to solve for
        
    % Control
    if strcmp(CONTROL,'TUBE_MPC')
        u_t_idx = mpc.mpc_robust_tube(t_idx);  % start on t_idx
    elseif strcmp(CONTROL, 'MPC')
        u_t_idx = mpc.mpc_direct_tran(t_idx);  % start on t_idx
    end
    
    % Dynamics
%     [x_next_true, ~] = mpc.propagate_dynamics(F); % run MODEL dynamics 
    [x_next_true] = mpc.propagate_dynamics_uncertain(u_t_idx);  % run simulated TRUE dynamics   
    mpc.x = x_next_true;  % update the TRUE state
    % Save results
    x_stored = [x_stored;
                x_next_true'];
            
    u_stored = [u_stored;
                u_t_idx'];
end

%% Plot result

%% Tube
% figure; hold on;
% set(0,'DefaultAxesFontName', 'CMU Serif')
% xlabel('$x_1$', 'Interpreter', 'latex', 'FontSize', 24)
% ylabel('$\dot{x}_1$',  'Interpreter', 'latex','FontSize', 24)
% grid
% title(['Double Integrator Run, MPC+Ancillary Controller'], 'FontSize', 24)
%     
% tube_idx = convhull(x_stored(:,1:2));
% tube_bound = x_stored(tube_idx, :);  % [N, size(tube_idx)] of hull pts
% patch(tube_bound(:,1),tube_bound(:,2),'blue', 'FaceAlpha',.4);
% scatter(x_stored(:,1), x_stored(:,2));
% % x = [0 0 tf tf];
% % y = [-w_mag w_mag w_mag -w_mag];
% % p=patch(x,y,'r','FaceAlpha',.2);
% legend('Approximate Tube','Trajectory Point','Noise Level', 'FontSize',24);

%% 1D state history
% if strcmp(DIMS, '1D') == 1
%     figure; hold on;
%     set(0,'DefaultAxesFontName', 'CMU Serif')
%     xlabel('Time [s]', 'FontSize', 24)
%     ylabel('State', 'FontSize', 24)
%     grid
%     title(['X-axis State History'], 'FontSize', 24)
% 
%     plot(t_stored(2:end), [x_ref_hist(1:end,1), x_ref_hist(1:end,2)]);  % ref traj
%     plot(t_stored(2:end), [x_stored(2:end, 1), x_stored(2:end, 2)], 'LineWidth', 2.0);  % state traj
%     x = [0 0 tf tf];
%     y = [-w_mag w_mag w_mag -w_mag];
%     % p=patch(x,y,'r','FaceAlpha',.2);
%     legend('x1_{ref}','x1d_{ref}','x1','x1d', 'FontSize',24);
% end
% 
if strcmp(DIMS, '3D')
    %% State history, 1
    figure; hold on;
    set(0,'DefaultAxesFontName', 'CMU Serif')
    xlabel('Time [s]', 'FontSize', 24)
    ylabel('State', 'FontSize', 24)
    grid
    title(['X-axis State History'], 'FontSize', 24)
        
    plot(t_stored(2:end), [x_ref_hist(1:end,1), x_ref_hist(1:end,4)]);  % ref traj
    plot(t_stored(2:end), [x_stored(2:end, 1), x_stored(2:end, 4)], 'LineWidth', 2.0);  % state traj
    x = [0 0 tf tf];
    y = [-w_mag w_mag w_mag -w_mag];
    % p=patch(x,y,'r','FaceAlpha',.2);
    legend('x1_{ref}','x1d_{ref}','x1','x1d', 'FontSize',24);

    %% State history, 2
    figure; hold on;
    set(0,'DefaultAxesFontName', 'CMU Serif')
    xlabel('Time [s]', 'FontSize', 24)
    ylabel('State', 'FontSize', 24)
    grid
    title(['Y-axis State History'], 'FontSize', 24)
        
    plot(t_stored(2:end), [x_ref_hist(1:end,2), x_ref_hist(1:end,5)]);  % ref traj
    plot(t_stored(2:end), [x_stored(2:end, 2), x_stored(2:end, 5)], 'LineWidth', 2.0);  % state traj
    x = [0 0 tf tf];
    y = [-w_mag w_mag w_mag -w_mag];
    % p=patch(x,y,'r','FaceAlpha',.2);
    legend('x2_{ref}','x2d_{ref}','x2','x2d', 'FontSize',24);
    
    %% State history, 3
    figure; hold on;
    set(0,'DefaultAxesFontName', 'CMU Serif')
    xlabel('Time [s]', 'FontSize', 24)
    ylabel('State', 'FontSize', 24)
    grid
    title(['Z-axis State History'], 'FontSize', 24)
        
    plot(t_stored(2:end), [x_ref_hist(1:end,3), x_ref_hist(1:end,6)]);  % ref traj
    plot(t_stored(2:end), [x_stored(2:end, 3), x_stored(2:end, 6)], 'LineWidth', 2.0);  % state traj
    x = [0 0 tf tf];
    y = [-w_mag w_mag w_mag -w_mag];
    % p=patch(x,y,'r','FaceAlpha',.2);
    legend('x3_{ref}','x3d_{ref}','x3','x3d', 'FontSize',24);
end
% 
% %% Input history
% figure; hold on;
% set(0,'DefaultAxesFontName', 'CMU Serif')
% grid;
% ylim([-mpc.um(1)*2, mpc.um(1)*2]);
% title(['Input'], 'Interpreter', 'latex', 'FontSize', 24)    
% plot(t_stored(2:end), u_stored,'LineWidth',2.0);
% x = [0 0 tf tf];
% y = [-u_constraint(1) u_constraint(1) u_constraint(1) -u_constraint(1)];
% p=patch(x,y,'r','FaceAlpha',0.2);
% xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 24)
% ylabel('Input [-]',  'Interpreter', 'latex','FontSize', 24)
% legend('$u_1$','$u_2$','$u_3$''Input Bound', 'Interpreter', 'latex', 'FontSize',24);

% %% Animate result
% r0_mat = [x_stored(:,1) x_stored(:,3) zeros(size(x_stored, 1), 1)];
% % theta_mat = x_stored(:,3);
% theta_mat = zeros(size(x_stored,1),1);
% anim_FK3(r0_mat, theta_mat);
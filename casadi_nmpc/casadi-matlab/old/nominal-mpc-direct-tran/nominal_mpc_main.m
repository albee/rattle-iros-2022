%% Set up dynamics
close all
clear all

REF_TRAJ = 'SINUSOID';  % ['SINUSOID or 'STEP']
% t0: x0, u0 --> t1: x1, u1 ... tN-1: xN-1, uN-1 --> tf: xN
tf = 30.0;

% timing and horizons
dt = .2;
N = 10;  % number of timesteps ahead for MPC
Nf = ceil(tf/dt) + 1;  % number of timesteps in total; |u| is 1 less

% ref traj
x_ref_hist = zeros(Nf, 6);
x_ref_hist = create_ref_traj(dt, tf, '3D', REF_TRAJ);

x_stored = [];
u_stored = [];
t_stored = x_ref_hist(:,1);
t_stored = [-dt; t_stored];
x_ref_hist = x_ref_hist(:,2:7);

% parameters
u_mag = 0.2;
v_mag = 0.1;
Q_mag = 20.0;
R_mag = 1.0;
mass = 9.58;
w_bounds = zeros(6,1);

x_cur = x_ref_hist(1,1:6)';  % take initial state as start of ref traj

%% Simulate for the Nf timesteps of the ref traj: the first reference is the current state
for t_idx = 1:1:Nf  % indices of the ref traj to solve for
        
    % Control
    u_t_idx = mpc_direct_tran(x_cur, t_idx, dt, N, Nf, x_ref_hist, u_mag, v_mag, Q_mag, R_mag, mass, w_bounds);  % start on t_idx
%     u_t_idx = mpc.mpc_direct_tran(t_idx);  % start on t_idx
    
    % Dynamics
    [x_cur, ~] = propagate_dynamics_3DOF(dt, mass, x_cur, u_t_idx); % run MODEL dynamics 
%     [x_next_true] = mpc.propagate_dynamics_uncertain(u_t_idx);  % run simulated TRUE dynamics   

    % Save results
    x_stored = [x_stored;
                x_cur'];
            
    u_stored = [u_stored;
                u_t_idx'];
end

x_ref_hist = x_ref_hist(1:Nf, :);

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
%% State history, 1
figure;
subplot(1,3,1); hold on;
set(0,'DefaultAxesFontName', 'CMU Serif')
xlabel('Time [s]', 'FontSize', 24)
ylabel('State', 'FontSize', 24)
grid
title(['X-axis State History'], 'FontSize', 24)

plot(t_stored(2:end), [x_ref_hist(1:end,1), x_ref_hist(1:end,4)]);  % ref traj
plot(t_stored(2:end), [x_stored(1:end, 1), x_stored(1:end, 4)], 'LineWidth', 2.0);  % state traj
% x = [0 0 tf tf];
% y = [-w_mag w_mag w_mag -w_mag];
% p=patch(x,y,'r','FaceAlpha',.2);
legend('$x_{1,ref}$','$\dot{x}_{1,ref}$','$x_1$','$\dot{x}_{1}$', 'Interpreter', 'latex','FontSize',24);

%% State history, 2
subplot(1,3,2); hold on;
set(0,'DefaultAxesFontName', 'CMU Serif')
xlabel('Time [s]', 'FontSize', 24)
ylabel('State', 'FontSize', 24)
grid
title(['Y-axis State History'], 'FontSize', 24)

plot(t_stored(2:end), [x_ref_hist(1:end,2), x_ref_hist(1:end,5)]);  % ref traj
plot(t_stored(2:end), [x_stored(1:end, 2), x_stored(1:end, 5)], 'LineWidth', 2.0);  % state traj
% x = [0 0 tf tf];
% y = [-w_mag w_mag w_mag -w_mag];
% p=patch(x,y,'r','FaceAlpha',.2);
legend('$x_{2,ref}$','$\dot{x}_{2,ref}$','$x_2$','$\dot{x}_{2}$', 'Interpreter', 'latex','FontSize',24);

%% State history, 3
subplot(1,3,3); hold on;
set(0,'DefaultAxesFontName', 'CMU Serif')
xlabel('Time [s]', 'FontSize', 24)
ylabel('State', 'FontSize', 24)
grid
title(['Z-axis State History'], 'FontSize', 24)

plot(t_stored(2:end), [x_ref_hist(1:end,3), x_ref_hist(1:end,6)]);  % ref traj
plot(t_stored(2:end), [x_stored(1:end, 3), x_stored(1:end, 6)], 'LineWidth', 2.0);  % state traj
% x = [0 0 tf tf];
% y = [-w_mag w_mag w_mag -w_mag];
% p=patch(x,y,'r','FaceAlpha',.2);
legend('$x_{3,ref}$','$\dot{x}_{3,ref}$','$x_3$','$\dot{x}_{3}$', 'Interpreter', 'latex','FontSize',24);

%% Input history
figure; hold on;
set(0,'DefaultAxesFontName', 'CMU Serif')
grid;
ylim([-0.4*2, 0.4*2]);
title(['Input'], 'Interpreter', 'latex', 'FontSize', 24)    
plot(t_stored(2:end), u_stored,'LineWidth',2.0);
x = [0 0 tf tf];
y = [-0.4 0.4 0.4 -0.4];
p=patch(x,y,'r','FaceAlpha',0.2);
xlabel('Time [s]', 'Interpreter', 'latex', 'FontSize', 24)
ylabel('Input [N]',  'Interpreter', 'latex','FontSize', 24)
legend('$u_1$','$u_2$','$u_3$','Input Bound', 'Interpreter', 'latex', 'FontSize',24);
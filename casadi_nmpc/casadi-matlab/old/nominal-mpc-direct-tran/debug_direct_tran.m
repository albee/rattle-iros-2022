% Run direct transcription unit tests

% parameters
u_mag = 0.2;
v_mag = 0.1;
Q_mag = 20.0;
R_mag = 1.0;
mass = 9.58; % kg
w_bounds = zeros(6,1);

%% Pos input on x3
x_cur = [11.0; 0.0; 0.095; 0; 0; 0];
t_idx = 1;
dt = .2;
N = 10;  % number of timesteps ahead for MPC
Nf = ceil(tf/dt) + 1;  % number of timesteps in total; |u| is 1 less

% Mimic C++ testing
x_ref_hist = zeros(10000, 6);
x_ref_hist(1:Nf, 3) = 0.095;

u_test1 = mpc_direct_tran(x_cur, t_idx, dt, N, Nf, x_ref_hist, u_mag, v_mag, Q_mag, R_mag, mass, w_bounds)  % start on t_idx

%% Neg input on x3
x_cur = [0; 0.0; 0.105; 0; 0; 0];
t_idx = 1;
dt = .2;
N = 10;  % number of timesteps ahead for MPC
Nf = ceil(tf/dt) + 1;  % number of timesteps in total; |u| is 1 less

% Mimic C++ testing
x_ref_hist = zeros(10000, 6);
x_ref_hist(1:Nf, 3) = 0.1;

u_test2 = mpc_direct_tran(x_cur, t_idx, dt, N, Nf, x_ref_hist, u_mag, v_mag, Q_mag, R_mag, mass, w_bounds)  % start on t_idx

%% x3 max neg
x_cur = [0; 0.0; 0.105; 0; 0; 0];
t_idx = 1;
dt = .2;
N = 10;  % number of timesteps ahead for MPC
Nf = ceil(tf/dt) + 1;  % number of timesteps in total; |u| is 1 less

% Mimic C++ testing
x_ref_hist = zeros(10000, 6);
% x_ref_hist(1:Nf, 3) = 0.1;

u_test3 = mpc_direct_tran(x_cur, t_idx, dt, N, Nf, x_ref_hist, u_mag, v_mag, Q_mag, R_mag, mass, w_bounds)  % start on t_idx
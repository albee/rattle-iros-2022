%{
This is a caller function around the robust_tube_mpc class, required for
MATLAB codegen.
%}
function tube_mpc_caller()
    %% Set up dynamics
    CONTROL = 'TUBE_MPC';  % ['TUBE_MPC, or 'MPC']
    REF_TRAJ = 'SINUSOID';  % ['SINUSOID or 'STEP']
    DIMS = '3D';  % ['1D', '3D', or Limon]

    % t0: x0, u0 --> t1: x1, u1 ... tN-1: xN-1, uN-1 --> tf: xN
    tf = 10.0;
    dt = .1;
    Nf = ceil(tf/dt) + 1;  % number of timesteps in total; |u| is 1 less
    N = 20;  % number of timesteps ahead for MPC

    w_mag = 0.0;  % uncertainty magnitude cutoff
    Q_mag = 10000.0;
    R_mag = 1.0;
    m1 = 10.0;

    if strcmp(DIMS, '1D') == 1
        n = 2;  % state vector dimension
        m = 1;  % input vector dimension
        u_constraint = [1.0]*10.0;  % [u1_abs; u2_abs; u3_abs] 
        x_constraint = [1.0; 1.0]*10.0;  % [x1_abs; x2_abs; x3_abs; x1d_abs; x2d_abs; x3d_abs]
        mpc = robust_tube_mpc(DIMS);
    elseif strcmp(DIMS, '3D') == 1
        n = 6;  % state vector dimension
        m = 3;  % input vector dimension
        u_constraint = [1.0; 1.0; 1.0]*10.0;  % [u1_abs; u2_abs; u3_abs] 
        x_constraint = [1.0; 1.0; 1.0; 1.0; 1.0; 1.0]*10.0;  % [x1_abs; x2_abs; x3_abs; x1d_abs; x2d_abs; x3d_abs]
        mpc = robust_tube_mpc(DIMS);
    end

    %% Create a reference trajectory
    x_ref_hist = create_ref_traj(dt, tf, DIMS, REF_TRAJ);
    t_stored = x_ref_hist(:,1);
    t_stored = [-dt; t_stored];
    x_ref_hist = x_ref_hist(:,2:2+n-1);

    %% Set constraints and uncertainty level
    mpc.w_mag = w_mag;
    mpc.xn = x_constraint;  % nominal x constraints
    mpc.um = u_constraint;  % nominal state constraints
    mpc.dt = dt;
    mpc.N = N;
    mpc.Nf = Nf;
    mpc.m = m;
    mpc.n = n;
    mpc.m1 = 10.0;
    mpc.Q_mag = Q_mag;
    mpc.R_mag = R_mag;
    mpc.x_ref_hist = x_ref_hist;  % reference state
    mpc.x = x_ref_hist(1,1:n)';  % initial state is first state of ref traj

    if strcmp(CONTROL, 'TUBE_MPC')
        mpc.prepare_nominal();
        mpc.prepare_ancillary();  % shrink for tube MPC
    elseif strcmp(CONTROL, 'MPC')
        mpc.prepare_nominal();
    end

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
end
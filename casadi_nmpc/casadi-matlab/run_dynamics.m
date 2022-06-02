%{
Simulate the dynamics and controller response over a time horizon.
%}
function sim_hist = run_dynamics(x0, u_mag, m, Autight, butight, AZ, bZ, K_dr, gains_mpc, gains_dr, noise, x_des_traj, DT, N, traj_dt, control_dt, num_timesteps, ...
    mpc_func_casadi, tube_mpc_func_casadi, casadi_dynamics_mpc_dt, casadi_dynamics_control_dt, USE_TUBE_MPC)
    %{
    Run the simulation dynamics and controller
    %}
    x_test = x0;

    X_hist = x0;
    U_hist = [];
    U_mpc_hist = [];
    U_dr_hist = [];
    x0_mpc_hist = [];
    t_hist = [0];
    X_des_hist = x_des_traj(1:num_timesteps+1, :)';
    computation_hist = [];

    % for tube MPC
    u_mpc = 0;
    u_dr = 0;
    x0_mpc = 0;

    setpoint_step = DT/traj_dt;
    N_required = setpoint_step*N+1;
    
    Q1 = gains_mpc.Q1;
    Q2 = gains_mpc.Q2;
    Q3 = gains_mpc.Q3;
    Q4 = gains_mpc.Q4;
    Q5 = gains_mpc.Q5;
    Q6 = gains_mpc.Q6;
    R1 = gains_mpc.R1;
    R2 = gains_mpc.R2;
    R3 = gains_mpc.R3;
    QN1 = gains_mpc.QN1;
    QN2 = gains_mpc.QN2;
    QN3 = gains_mpc.QN3;
    QN4 = gains_mpc.QN4;
    QN5 = gains_mpc.QN5;
    QN6 = gains_mpc.QN6;

    for i=1:1:num_timesteps  % 10*T*N timesteps---leave extra T horizon for MPC
      x_des_traj_N = x_des_traj(i:setpoint_step:i+N_required, :);
      tic;

      if USE_TUBE_MPC
          % Tube MPC
          [dm_u0_opt, dm_u0_mpc, dm_u0_dr, dm_x0_mpc] = tube_mpc_func_casadi(x_test, x_des_traj_N, u_mag, m, Autight, butight, AZ, bZ, K_dr, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6);
          u = full(dm_u0_opt); u_mpc = full(dm_u0_mpc); u_dr = full(dm_u0_dr); x0_mpc = full(dm_x0_mpc);
      else
        % Standard MPC
         u = full(mpc_func_casadi(x_test, x_des_traj_N, u_mag, m, Q1, Q2, Q3, Q4, Q5, Q6, R1, R2, R3, QN1, QN2, QN3, QN4, QN5, QN6));  % solve control for i-th instant over DT time
      end

      % History recording
      computation_hist(i) = toc;
      t_hist(end+1) = i*control_dt;
      U_hist(:,i) = u;
      X_hist(:,end+1) = x_test;

      U_mpc_hist(:,i) = u_mpc;
      U_dr_hist(:,i) = u_dr;
      x0_mpc_hist(:,end+1) = x0_mpc;
      abs(x_test - x0_mpc);

      % simulate system      
      w = [unifrnd([-noise.r1, -noise.r2, -noise.r3, -noise.v1, -noise.v2, -noise.v3],...
                   [noise.r1, noise.r2, noise.r3, noise.v1, noise.v2, noise.v3])]';
      
      x_test = full(casadi_dynamics_control_dt(x_test, u, m)) + w;  % no noise
    end
    
    fprintf("mean computation time was %f seconds \n", mean(computation_hist));
    
    sim_hist.X_hist = X_hist;
    sim_hist.U_hist = U_hist;
    sim_hist.t_hist = t_hist;
    sim_hist.U_mpc_hist = U_mpc_hist;
    sim_hist.U_dr_hist = U_dr_hist;
    sim_hist.x0_mpc_hist = x0_mpc_hist;
    sim_hist.computation_hist = computation_hist;
    sim_hist.X_des_hist = X_des_hist;
end

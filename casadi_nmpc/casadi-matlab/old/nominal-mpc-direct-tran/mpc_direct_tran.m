%{
Perform robust MPC, direct transcription.
%}
      
%         K_lqr;  % ancillary controller gain
%         w_mag;  % max magnitude of uncertainty
%         x_ref;  % reference value
       
%     function prepare_ancillary(self)
%         %--- Ancillary controller ---%
%         Q = self.Q_mag*eye(self.n);
%         R = self.R_mag*eye(self.m);
%         [K, ~, ~] = dlqr(self.A, self.B, Q, R, zeros(self.n, self.m));
%         self.K_lqr = -K;
%         
%         self.shrink_constraints();  % shrink constraints for tube MPC
%     end
%    
%     % Shrink the constraints for the selected disturbance rejection term
%     function um_shrunk = shrink_constraints(self)
%         max_w = self.w_mag*ones(self.n,1);
%         K_dr = self.K_lqr;
%         Zmax = K_dr*max_w;
%         self.u_constraint = self.u_constraint - abs(Zmax);
%     end

%     %% Ancillary controller
%     %{
%     Given the state estimate and goal, give the desired forces and
%     torques
%     %}
%     function F_T = lqr_calc_input(self, t_idx)
%         n = self.n;
%         
%         x_ref = self.x_ref_hist(t_idx,1:n)';
%         x_err = self.x - x_ref;  % predicted reference state
%         F_T = self.K_lqr*x_err;
%     end
    
%     %{
%     Calculate nominal MPC with tightened constraints, then add ancillary
%     controller correction.
%     %}
%     function u_t_idx = mpc_robust_tube(self, t_idx)
%         u_t_idx = mpc_direct_tran(self, t_idx);
%         u_t_idx = u_t_idx + lqr_calc_input(self, t_idx);
%     end
    
    %% Direct transcription
    %{
    Compute MPC inputs by direct transcription.
    Assumes the state estimate has been set!
    
    Inputs:
    [current state, time index, dt, horizon length, ref traj length, ref
    traj, input magnitude, velocity magnitude, state weight, input weight,
    mass, uncertainty bounds]
    %}    
function u_t_idx = mpc_direct_tran(x_cur, t_idx, dt, N, Nf, x_ref_hist, u_mag, v_mag, Q_mag, R_mag, mass, w_bounds)    
    n = 6;  % state size
    m = 3;  % input size
    m1 = mass;  % mass, kg
    u_constraint = [1.0; 1.0; 1.0]*u_mag;
    x_constraint = [100.0; 100.0; 100.0; v_mag; v_mag; v_mag];  % are these actually being enforced?

    [A, B] = get_dynamics_for_dt(dt, m1);

    state_horz_dim = n*N;  % size of state decision variable vector
    input_horz_dim = m*N;  % size of input decision variable vector
    dec_dim = state_horz_dim + input_horz_dim;  % size of total decision variable vector
    % decision vector is of form [x_horz ; u_horz]


    %---Cost function---
    % min [x-x_ref]'*Q*[x-x_ref] + u'*R*u
    % min 0.5*x'*H*x + f'x
    H_Q = ones(1,state_horz_dim)*Q_mag;  % penalize states
    H_R = ones(1,input_horz_dim)*R_mag;      % penalize inputs
    H_diag = [H_Q, H_R];
    H = diag(H_diag);

    f = zeros(dec_dim, 1);

    for i = 0:(N-1)  % for the full horizon...
        if i+t_idx > Nf  % (use final state if horizon exceeds ref traj)
%                 fprintf("%d %d\n", i, t0_idx);
            x_des = x_ref_hist(Nf,1:n)';  % get state reference...
        else
            x_des = x_ref_hist(i+t_idx,1:n)';  % get state reference...
        end
        f((n*i + 1) : (n*i+n)) = -x_des*Q_mag;  % ...and set a penalty for missing this reference (state only)
    end


    %---Equality constraints---
    % A*x = b
    Aeq_state = zeros(state_horz_dim, state_horz_dim);
    Aeq_input = zeros(state_horz_dim, input_horz_dim);
    beq = zeros(state_horz_dim, 1);
    % Dynamics constraints
    % First constraint is set
    % -x_1 + B*u_0 = -A*x_0
    Aeq_state(1:n, 1:n) = -eye(n,n);
    Aeq_input(1:n, 1:m) = B;
    beq(1:n, 1) = -A*x_cur;
    % These constraints depend on dec vars
    for i = 1:(N-1)
        % x_n+ = A*x_n
        Aeq_state((n*i + 1):(n*i + n), (n*(i-1) + 1):(n*(i-1) + n)) = A;
        Aeq_state((n*i + 1):(n*i + n), (n*(i-1) + 1+n):(n*(i-1) + n+n)) = -eye(n,n);

        % +B*u_n
        Aeq_input((n*i + 1):(n*i + n), (m*(i) + 1):(m*(i) + m)) = B;
    end

    % Assemble equality matrix
    Aeq = [Aeq_state, Aeq_input];
    beq = [beq];

    %---Inequality constraints---
    % A*x <= b
    % Actuator saturation inequality constraint
    Aineq_input = [-eye(input_horz_dim);
                   eye(input_horz_dim)];
    bineq_input = repmat(u_constraint,N*2,1);

    % Half-plane inequality constraint
    Aineq_state = [eye(state_horz_dim);
                  -eye(state_horz_dim)];
    bineq_state = repmat(x_constraint, N*2,1);

    % Assemble inequality matrix
    Aineq = [ zeros(input_horz_dim*2, state_horz_dim), Aineq_input;
              Aineq_state, zeros((state_horz_dim)*2, input_horz_dim)];
    bineq = [bineq_input;
             bineq_state];

    options =  optimoptions(@quadprog,'Display','off','Algorithm','active-set');
%     options =  optimoptions(@quadprog,'Display','off','Algorithm','interior-point-convex');


    %{
    min 0.5 x'*H*x + f'*x
    s.t.
    A*x = b
    A*x <= b
    %}
    x0 = zeros(dec_dim,1);  % active set initial guess
    dec_vec = quadprog(H,f,Aineq,bineq,Aeq,beq,[],[],x0,options);  % [x0; x_horz; u_horz]
    U = dec_vec(end + 1-input_horz_dim:end);
    u_t_idx = U(1:m); % inputs for this timestep
end

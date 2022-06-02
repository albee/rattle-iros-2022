%{
Propagates forward dynamics, computes robust MPC control, and performs
plotting

Current configuration: 1D or 3D control.
%}

classdef robust_tube_mpc    
    properties (SetAccess = public)
        dt;  % timestep between steps for discrete dynamics
        Nf;  % total timesteps in ref traj
        N;  % timesteps ahead for MPC optimization
        Q_mag;  % state weighting matrix diagonal values
        R_mag;  % input weighting matrix diagonal values
        m1;  % mass (kg)
        x_ref_hist;  % setpoints
        x;  % current SYSTEM state (MODEL receives perfect SYSTEM estimates)

        w_mag;  % max magnitude of uncertainty
        t_idx;  
        z;  % measurement
        x_ref;  % reference value
        
        xn;
        um;
        
        A;
        B;
        C = eye(6);  % full state feedback
        
        n = 6;  % state vector dimension
        m = 3;  % input vector dimension
        u_constraint = [1.0; 1.0; 1.0]*8.0;  % [u1_abs; u2_abs; u3_abs] state constraints for the n states
        x_constraint = [1.0; 1.0; 1.0; 1.0; 1.0; 1.0]*10.0;  % [x1_abs; x2_abs; x3_abs; x1d_abs; x2d_abs; x3d_abs] actuation constraints for the m inputs
        
        K_lqr;  % ancillary controller gain
        
        uc_bound;
        Zmax;
    end
    
    methods 
    %% Setup
    % Constructor
    function self = robust_tube_mpc(x_ref_hist, dt, Nf, N, Q_mag, R_mag, m1)
%         if strcmp(DIMS, '1D') == 1
%             self.n = 2;  % state vector dimension
%             self.m = 1;  % input vector dimension
%             self.u_constraint = [1.0]*10.0;  % [u1_abs; u2_abs; u3_abs] 
%             self.x_constraint = [1.0; 1.0]*10.0;  % [x1_abs; x2_abs; x3_abs; x1d_abs; x2d_abs; x3d_abs]
%         elseif strcmp(DIMS, '3D') == 1

        self.x_ref_hist = x_ref_hist;  % reference state
        self.x = x_ref_hist(1,1:self.n)';  % initial state is first state of ref traj
        self.dt = dt;
        self.Nf = Nf;
        self.N = N;
        self.Q_mag = Q_mag;
        self.R_mag = R_mag;
        self.m1 = m1;
        
        [self.A, self.B] = get_dynamics_for_dt(dt, m1);
        
%             case '1D'
%             % x = [x1 x1d]
% 
%             self.A = [1, dt;
%                       0, 1];
%             self.B = [dt^2/(2*m1);
%                       dt/m1];
%             self.C = [1, 0;
%                       0, 1];  % full state feedback
              
%             case '3D'
            % x = [x1 x2 x3 x1d x2d x3d]
    end
    
    % Set dynamics based on dt zero-order hold
    function [A, B] = get_dynamics_for_dt(dt, m1)
        A = [1, 0, 0, dt, 0, 0;
          0, 1, 0, 0, dt, 0;
          0, 0, 1, 0, 0, dt;
          0, 0, 0, 1, 0, 0;
          0, 0, 0, 0, 1, 0;
          0, 0, 0, 0, 0, 1];

        B = [dt^2/(2*m1),   0,               0;
                  0,               dt^2/(2*m1),   0;
                  0,               0,               dt^2/(2*m1);
                  dt/m1, 0,               0;
                  0,               dt/m1, 0;
                  0,               0,               dt/m1];
    end
    
    function self = prepare_ancillary(self)
        %--- Ancillary controller ---%
        Q = self.Q_mag*eye(self.n);
        R = self.R_mag*eye(self.m);
        [K, ~, ~] = dlqr(self.A, self.B, Q, R, zeros(self.n, self.m));
        self.K_lqr = -K;
        
        [self, Zmax] = self.shrink_constraints();
%         self.shrink_constraints_simple();  % shrink constraints for tube MPC
    end
   
    % Shrink the constraints for the selected disturbance rejection term
    function um_shrunk = shrink_constraints_simple(self)
        max_w = self.w_mag*ones(self.n,1);
        K_dr = self.K_lqr;
        Zmax = K_dr*max_w;
        self.u_constraint = self.u_constraint - abs(Zmax);
    end
    
    % Shrink the constraints for the selected disturbance rejection term
    function [self, Zmax] = shrink_constraints(self)
        max_w = [self.uc_bound(1,:), self.uc_bound(3,:)]'
        K_dr = self.K_lqr;
        Zmax = K_dr*max_w;
        self.u_constraint = self.u_constraint - abs(Zmax);
        self.Zmax = abs(Zmax);
    end

    %% Ancillary controller
    %{
    Given the state estimate and goal, give the desired forces and
    torques
    %}
    function F_T = lqr_calc_input(self, t_idx)
        n = self.n;
        
        x_ref = self.x_ref_hist(t_idx,1:n)';
        x_err = self.x - x_ref;  % predicted reference state
        F_T = self.K_lqr*x_err;
    end
    
    %{
    Calculate nominal MPC with tightened constraints, then add ancillary
    controller correction.
    %}
    function u_t_idx = mpc_robust_tube(self, t_idx)
        u_t_idx = mpc_direct_tran(self, t_idx);
        u_t_idx = u_t_idx + lqr_calc_input(self, t_idx);
    end
    
    %% Direct transcription
    %{
    Compute MPC inputs by direct transcription.
    Assumes the state estimate has been set!
    %}    
   function u_t_idx = mpc_direct_tran(self, t_idx)       
        n = self.n;  % state size
        m = self.m;  % input size
        N = self.N;  % horizon length
        Nf = self.Nf;  % ref traj length
        A = self.A;
        B = self.B;
        x_ref_hist = self.x_ref_hist;
        u_constraint = self.u_constraint;
        x_constraint = self.x_constraint;
        Q_mag = self.Q_mag;
        R_mag = self.R_mag;
                      
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
        beq(1:n, 1) = -A*self.x;
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

        %{
        min 0.5 x'*H*x + f'*x
        s.t.
        A*x = b
        A*x <= b
        %}
        x0 = zeros(dec_dim,1);  % active set initial guess
        dec_vec = quadprog(H,f,Aineq,bineq,Aeq,beq,[],[],x0,options);  % [x0; x_horz; u_horz]
        U = dec_vec(end + 1-input_horz_dim:end);
        u_t_idx = U(1:self.m); % inputs for this timestep
   end
   
    %% Dynamics
    % Run the nominal forward dynamics MODEL, do NOT update state
    function [x_FD, z_FD] = propagate_dynamics(self, uk)
        A = self.A; B = self.B; C = self.C; xk = self.x;
        x_FD = A*xk + B*uk;
        z_FD = C*xk;
    end
    
    % Run the uncertain forward dynamics MODEL, do NOT update state.
    % Uncertain.
    function x_FD = propagate_dynamics_uncertain(self, uk)     
        A = self.A; B = self.B; C = self.C; xk = self.x; w_mag = self.w_mag;
        
        % Create Gaussian zero mean white noise that lives in the 0.1 inf-norm
        % ball
        w = normrnd(0, w_mag, [self.n,1]);
        w(w>w_mag) = w_mag;
        w(w<-w_mag) = -w_mag;

        x_FD = A*xk + B*uk + w;
    end
    
    % Calculate the metric between x and x_ref
    function dist = calc_dist(self)
        % Euclidean L2
        dist = norm(self.x_ref - self.x);
    end
    end
end

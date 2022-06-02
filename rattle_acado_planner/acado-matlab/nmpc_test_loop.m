%{
previous nmpc_test code, which runs the exported optimization problem as a
controller with simulated dynamics in the loop
%}
if rhp == 0
    while time <= tf
        % use states from simulation of the previous step, and the last estimated mass
          input.od = repmat([m_est, I_xx_est, I_yy_est, I_zz_est, 0, 0, 0, 0, 0, 0, ...
                            x_des, u_des], N+1, 1);  % latest param estimate for NMPC online data
          input.x0 = [state_sim(end,:)];  % latest ground truth value
          output = acado_NMPC_6DoF(input);  % this step runs MPC

        output.info;  % gives some info about the value of the objective after each MPC horizon
        Objective_value  = [Objective_value; output.info.objValue];

        % forward shifting: use NMPC's predicted next state results 
        input.x = [output.x(2:end,:); output.x(end,:)]; 
        input.u = [output.u(2:end,:); output.u(end,:)];


        %% ACADO Dynamics Simulation
        % prepare inputs and state for forward dynamics sim
        input_sim.x = state_sim(end,:).';       % latest state of simulation
        input_sim.u = output.u(1,:).';          % first input from NMPC optimization
        input_sim.od = [m_est, I_xx_est, I_yy_est, I_zz_est]';
        % Apply NMPC input and evaluated forward dynamics over timestep dt
        output_sim = sim_6DoF(input_sim);

        %% Store Data
        control_ip = [control_ip; output.u(1,:)];  % record applied u
        states = [states;
                     output_sim.value'];  % record ground truth dat

        time = time + Ts;

        time_store = [time_store; time];
    %     store_results = [store_results;time, [m_est,I_xx_est,I_yy_est,I_zz_est], P', vels_tilde_next', input_NMPC.W(1,1)];
    end
    time_store = [time_store; time];
end


%% Utility functions
function [error_quat] = err_q(q,q_des)
    error_quat = invskew(Rotmat(q_des)'*Rotmat(q) - Rotmat(q)'*Rotmat(q_des))/(2*sqrt(1+tr(Rotmat(q_des)'*Rotmat(q))));
end


function [skew_v] = skew(vec)
    skew_v = [0 -vec(3) vec(2) ; vec(3) 0 -vec(1) ; -vec(2) vec(1) 0 ];  % Cross product matrix
end


function [vec] = invskew(A)
    vec = [A(3,2); A(1,3); A(2,1)];
end


function [Trace] = tr(A)
    Trace =  A(1,1) + A(2,2) + A(3,3);  % Trace
end


% Rotation matrix for R_I_B: quaternion uses scalar last convention
function [R] = Rotmat(q)
  R = [q(1)^2-q(2)^2-q(3)^2+q(4)^2, 2*(q(1)*q(2)+q(3)*q(4)),          2*(q(1)*q(3)-q(2)*q(4));...
       2*(q(1)*q(2)-q(3)*q(4)),        -q(1)^2+q(2)^2-q(3)^2 + q(4)^2, 2*(q(2)*q(3)+q(1)*q(4));...
       2*(q(1)*q(3)+q(2)*q(4)),        2*(q(2)*q(3)-q(1)*q(4)),          -q(1)^2-q(2)^2+q(3)^2+q(4)^2]';
end


% Provides the error quaternion.
function [error_quat] = err_q2(q,q_des)
  error_quat = sqrt(2*(1 - abs(dot(q, q_des))));
end


function W_out = gamma_shifter(W, info)
  % Updates gamma according to a specified shifting pattern.
  % gamma is the first term
  gamma = W(1, 1);
  x = info{1};
  t = info{2};
  tau = 20;  % time constant for decay by a factor of e

  % adjust gamma for decent tracking
  % options: distance to goal; quality of tracking; change in parameter
  % estimates; hard cutoff after enough time

  % for now, it just ramps down as you get closer to x_g
  gamma = 10.0*exp(1)^(-1/tau*t);
  gamma = gamma + norm(x)^2 -1; % quaternion scalar element does not let this go to zero
  if gamma > 20
      gamma = 20;
  end  % limit max

  W(1, 1) = gamma;  % set gamma
  W_out = W;
end



% 
%  if rhp == 1
%         % FIM calc over hoirzon using output.x
%         psi = output.x(:,14:end);  % Grab psi propagation. Actually dx/d_theta is a jacobian, with columns: [dx/dmass dx/dIzz]. It has been vectorized.
%         for k = 1:N+1
%           psi_matrix_col1 = [psi(1:4) zeros(1,5)]'; % reduced state mass
% 
%           psi_matrix = [ psi_matrix_col1 psi(5:13)' psi(14:22)' psi(23:31)']; % cx, cy, inertia
%           
%           FIM = (dh_dx*psi_matrix)'*inv_R*(dh_dx*psi_matrix)  ;  % H' * inv_R * H = [1 x 1]
%           if PRINT_FIM == 1
%               format long
%               FIM
%           end
%           tr_F_local(k)  = trace(FIM);
%         end
%         F_aggregate = [F_aggregate; F_aggregate(end) + tr_F_local(2)];  % only add on the information actually gained
%    end
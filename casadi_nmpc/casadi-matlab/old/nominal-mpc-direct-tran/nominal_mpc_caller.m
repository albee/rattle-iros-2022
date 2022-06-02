%{
This is a caller function around the robust_tube_mpc class, required for
MATLAB codegen.

mpc
x
t_idx
%}
function u_t_idx = nominal_mpc_caller(x_cur, t_idx, dt, N, Nf, x_ref_hist)
    %% Simulate for the Nf timesteps of the ref traj: the first reference is the current state
    %u_t_idx = mpc.mpc_robust_tube(t_idx);  % start on t_idx

    u_t_idx = mpc_direct_tran(x_cur, t_idx, dt, N, Nf, x_ref_hist);  % start on t_idx
end
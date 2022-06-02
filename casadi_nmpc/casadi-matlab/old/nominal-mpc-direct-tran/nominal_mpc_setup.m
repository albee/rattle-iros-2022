%{
This is a wrapper function around the robust_tube_mpc class, required for
MATLAB codegen.

mpc
x
t_idx
%}
function setup = nominal_mpc_setup(x_ref_hist, dt, Nf, N, Q_mag, R_mag, m1)
    % Generate an mpc object
    setup = [x_ref_hist, dt, Nf, N, Q_mag, R_mag, m1];
end
# robust-tube-mpc

A robust tube MPC with a simple LQR ancillary controller and a nominal MPC using
quadprog as its optimization backend.

Note that the robust tube MPC has been stripped down to accomodate MATLAB
codegen.

## Usage

`tube_mpc_main` is the main script to run a robust tube instance.
`nominal_mpc_main` is the main script to run a nominal MPC instance.
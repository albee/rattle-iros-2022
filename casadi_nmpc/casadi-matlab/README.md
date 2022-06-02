# casadi_matlab

Keenan Albee, 08.04.21

CasADi-based linear robust tube MPC. This is the CasADi definition of a robust
MPC controller, a standard MPC controller, different sets of discrete dynamics,
and some test scripts. Controlllers designed here are exported as .casadi files,
which can be read in from the CasADi C++ interface.

## Code

- `run_simulation.m`: runs robust MPC simulation tests.
- `run_simulation_ISS_TS1.m`: above w/ ROAM-1 settings.
- `casadi_3dof_double_integrator.m`: the non-robust standard MPC
- `casadi_3dof_double_integrator_z0.m`: the robust nominal MPC
- `z_poly_calc.m` supplementary Rakovic mRPI calculation, needed for robust nominal MPC constraints (historical)
- `get_gains.m` get gain values
- `get_noise.m` get noise values

- `generate_casadi_functions`: export functions for casadi C++ use (preferred)
- `generate casadi_libraries`: generate C source (standalone) to compile into C++ project

## Usage

* Generate code using `generate_casadi_functions.m`.

* Generate standalone shared libraries using `generate_casadi_libraries.m`.

* Run a simulation using `run_simulation.m`. This creates a robust MPC and runs
it on a double integrator model.

### z_poly_calc

Polytope calculation is performed using a separate ROS node, which must be launched.

### Ref Traj Generation

`create_ref_traj.m` creates one of a variety of reference trajectories
`des_traj_to_dlr_format.m` converts des_traj to DLR-formatted Chaser ref trajs in DATA_DLR_FORMAT

### Monte Carlo Testing, Unit Testing

- `unit_test_casadi.m`
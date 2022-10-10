# param_est
This package contains  the `param_est` node for inertial parameter estimation.

Performs sequential parameter estimation using velocity, acceleration and omega measurements from `/gnc/ekf` and control inputs from `inv_fam/appliedFandT` (published by the inv_fam node).

Estimates and their covariances are published to topic `/mob/inertia_est` 

## Usage
Setting `use_vels_meas` (line 185 of param_estimate) uses the linear velocity measurements along with angular velocity and linear acceleration measurements. 
Setting `simplified_dynamics` (line 186) ignores CoM offsets and products of inertia in the dynamics and estimates the remaining parameters.

TODO: standalone usage
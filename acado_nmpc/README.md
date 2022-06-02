#acado_nmpc_rattle

DEPRECATED FOR RESWARM USE!!!

Runs a non-linear model predictive controller. 

The controller will then launch with the simulation, and start publishing forces and torques to `gnc/ctl/command` when it starts receiving reference poses, twists and wrenches.

####To Do
* decrease motion limits or increase iterations (or acado will give Nan) in case of large divergence from the reference
* ensuring safety while tracking - is the tube MPC better?

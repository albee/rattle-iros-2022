# rattle_acado_planner
A receding-horizon motion planner, based on ACADO's NMPC RTI method.

This ROS package has an ACADO-exported planner contained in `RHP_export/`.
A node wrapper is written in `rattle_acado_planner.cpp`, `rattle_acado_planner`.

`acado-matlab/ACADO_generator/` contains MATLAB definitions of the ACADO planner and controller and are output to
`acado-matlab/RHP_export/`. The ready-to-use exported code must be copied up to `acado-export/RHP_export` when ready to compile.


## Usage
Launch individual nodes using  `rosrun acado_rhp_rattle [node]`


## Planner Notes
*NOTE: `acado_dummy_file.c` contains missing variable definitions and must be included for Acado to compile!*

Each uses the newest mass and inertia values from topic `/mob/inertia`. 

# rattle_acado_planner
A receding-horizon motion planner, based on ACADO's NMPC RTI method.

This ROS package has an ACADO-exported planner contained in `NMPC_export/`.
A node wrapper is written in `rattle_acado_planner_node.cpp`, with core planner implementation in `rattle_acado_planner.cpp`.

- `acado-matlab/ACADO_generator/` contains MATLAB definitions of the ACADO planner and controller, as well as a sim tester.

- `acado-matlab/NMPC_export/`. The ready-to-use exported code must be copied up to `acado-export/'''` when ready to compile.

- `acado-matlab/RHP_6DOF_export/`. '''

- `acado-matlab/RHP_export/`. '''

- `acado-matlab/SIM_export/`. '''


## Usage
Launch individually using  `roslaunch rattle_acado_planner rattle_acado_planner.launch`.
Consult README in `acado-matlab` for additional information.

Note that a 6DOF or 3DOF version can be toggled using the CMake command `USE_ISS_VERSION`.


## Planner Notes
*NOTE: `acado_dummy_file.c` contains missing variable definitions and must be included for Acado to compile!*

Each uses the newest mass and inertia values from topic `/mob/inertia`. 

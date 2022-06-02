# rattle_coordinator
 
(Previously test_session_tools)

This package contains high-level scripts for coordinating RATTLE's components:

(1) Info-aware motion planner: acado_nmpc_rattle
(2) Online-updateable robust controller: casadi_nmpc
(3) Global motion planner: rattle_rrt
(4) Parameter estimator: param_est

* run_rattle.py : (in development) runs specific instances of RATTLE tests
* rattle_coordinator.py : the main node monitoring RATTLE progress

## Usage

* Standalone:
1. `roslaunch rattle_coordinator rattle_coordinator.launch`
2. `rosrun rattle_coordinator run_rattle.py [test number]`

* Multi-node testing:
1. `roslaunch rattle_coordinator debug.launch [rviz:={false, true}]`
2. `rosrun rattle_coordinator run_rattle.py <rattle_test_name>` (to use debug interface) OR
`rosrun execute_asap pub_gds_topics.py [--sim] [--ground] [--test] <reswarm_test_number>` (to use execute_asap interface)

## Behavior



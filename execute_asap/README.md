# execute_ASAP

A set of tools that handle high-level coordination of Astrobee science experiments.

execute_asap.py : high-level coordinating script (mainly startup and shutdown)
asap_primary.py : primary Astrobee Python command script
asap_secondary.py : secondary Astrobee Python command script

The `execute_asap` nodelet listens to incoming GDS ROS parameters that define a test. The parameters are
1) whether the test is being run in simulation are on hardware (--sim)
2) whether the test is on the ISS or on the ground (--ground)
3) the test number. For now, these parameters are defined as

`/asap/gds_sim`  (String)
`/asap/gds_ground`   (String)
`/asap/gds_test_num`   (int).

Some additional GDS options are checked for real-time test changes,

`/asap/gds_role` (String)
`/asap/gds_roam_bagger` (int).

## Usage

To run a test in the simulator, use your desired TEST_NUMBER as an argument with the `pub_gds_topics.py` script. This script sets the parameters that GDS eventually will set. `execute_asap.py` listens to these parameters, launches nodelets, and hands it off to individual *roles*, in this case
primary and secondary. `asap_primary.py` and `asap_secondary.py` set some initial conditions and test-specific parameters; real-time logic throughout a test is handled by *coordinator nodelets*, `coordinator_nodelet.cc` which is the main C++ test commanding coordinator.

Run using: rosrun execute_asap pub_gds_topics.py [--ground] [--sim] test_number, e.g.,

`rosrun execute_asap pub_gds_topics.py --ground --sim 1` : to run test 1 on the ground for sim

`[-g, --ground]` : Run a ground test. Default to ISS.

`[-s, --sim]` : Run a simulation test. Defaults to hardware.

## Adding a Node/Nodelet

New nodes/nodelets can be created just like any normal ROS node. Usually, each node will go in its own package.
Nodes are intended to run persistently, executing a main ROS loop while waiting for publishers or srv requests.

1. Nodes should be added to `asap_primary_*.launch`. Choose either the MLP or LLP for hardware use (does not matter for simulation),

```XML
<node pkg="z_poly_calc" type="z_poly_calc.py" name="z_poly_calc"
    required="false" respawn="false"
    output="$(arg output)"/>
```

2. Nodes should also be added to `asap_config.py` to enable shutdown requests,

```Python
NODE_LIST_SIM_PRIMARY = ["primary_coordinator", "casadi_nmpc", ...
```

Note: replace `primary` with `secondary` above to use the secondary Astrobee.

## Config Files

Located in `config/`, these parameters pulled in at startup by `asap_*.launch`.

`asap_config.py` contains configuration parameters for `execute_asap`.

## Canceling a Test

To end a test and kill nodelets, run the script with a -1:

`rosrun execute_asap pub_gds_topics.py --ground --sim -1` : to cancel tests on the ground for sim.

This will stop all ROS bag recording and will also kill the TumbleDock nodelets. The `execute_asap` nodelet will still run, offering the opportunity to run another test and launch nodelets again.

## Robot Namespacing

For the simulator, each robot defines its role in `execute_asap.py` via a robot name argument that is passed in the `execute_asap` node launch process. For hardware, `execute_asap` subscribes to the topic `/robot_name` to define its role. Individual nodes might need logic to handle topics on a different namespace.

## Topic Recording

rosbags are automatically started for a new test. See `asap_config.py` for bag and many other configuration options.

## Simulator Startup

Start up the Astrobee sim with '/bumble' and '/honey' for the ISS:

`roslaunch astrobee sim_reswarm.launch rviz:=true`

You can set additional parameters on what to launch in `sim_reswarm.launch`.

## Adding Tests

You can add tests in the `coordinator`, consult the README.

## Logging

logging levels are set in astrobee/resources/logging/config

# execute_asap/launch

## Behavior 

The `execute_asap` node launches nodes/nodelets specific to the experiment upon recieving a test number from GDS.
ALL nodes/nodelest are launched upon receiveing a valid test number, and each is designed to enter their main ROS loop, waiitng for instruction from
the `coordinator`.

Either `asap_primary.launch` or `asap_secondary.launch` is called on test startup.
From here, specific launch files for the robot's MLP and LLP actually launch each nodelet.
Each nodelet is launched as a standalone nodelet, which offers the ability to kill and re-launch them for each test.

Upon a -1 test command, all nodes are killed. They are restarted upon receiving a new test command that is not -1.


## Launch Files

* `asap_astrobee.launch`: Runs `execute_asap` command node. Persistently runs, handles startup, shutdown, recording, etc.

* `asap_primary.launch`: Called to start up MLP and LLP nodes (including `coordinator`) by `execute_asap`, primary.

* `asap_secondary.launch`: Called to start up MLP and LLP nodes (including `coordinator`) by `execute_asap`, secondary.

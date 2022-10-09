# rrt_high_level
This nodelet is a high-level RRT, implemented in C++. Currently, it includes a kinodynamic planning component too, but this
can be dropped for time efficiency (we'll assume the RRT trajectory is feasible and let the mid-level planner handle traj opt...).

Acknowledgment to the Georgia Tech Robojackets team for the original template upon which the RRT has evolved.

## ROS Usage
Write to the topic RRT_PARAMS_TOPIC, `rattle/rrt/params` ONCE using the `RRTParamsMsg`, defined in `msg/`. For now, params are just `start_state` and `goal_state`.
(See `rrt_path_request()` in `rrt_nodelet.cc`.)

Monitor `rattle/rrt/path` for the `RRTPathMsg`, defined in `msg/`. A full state path will be provided.


## Usage
`main.cpp` contains usage examples. 

After compilation, an additional executable `test-rrt` is avaiable. Calling `test-rrt` will run some predefined RRT operations.

Executable:

`rosrun rrt_high_level test-rrt`


## Behavior
- Most actual computation is defined in the template `tree.h`. 
- State spaces are implemented by inheriting the abstract `state_space.h`.

Library:
Include `librrt_high_level` in your code, and call as follows:

```
std::shared_ptr<rrt_ad::StateSpace13D<rrt_ad::Vec13>> state_space_ptr =
    std::make_shared<rrt_ad::StateSpace13D<rrt_ad::Vec13>> (min_bounds, max_bounds, mass, ixx, iyy, izz);

std::shared_ptr<rrt_ad::Tree<rrt_ad::Vec13>> rrt =
tree_for_13d(state_space_ptr,
    start, goal, max_iterations, is_ASC_enabled, TIMESTEP, MAX_TIMESTEP,
    goal_bias, goal_max_dist, remember_actions);

// logf(state_space_ptr->get_dist(start, goal));
// logf(state_space_ptr->find_quat_rand());
// logf(state_space_ptr->is_state_valid(goal));
// logf(state_space_ptr->find_x_rand());

// run
std::vector<rrt_ad::Node<rrt_ad::Vec13>> path;
result = rrt->build_RRT();
path = rrt->get_path_nodes();
```

The RRT is also wrapped by a cpp ROS nodelet, `rattle_rrt_nodelet.cpp`.


## Visualize
Visualize:
`rosrun rrt_high_level rrt_plotter.py`

or use the MATLAB `visualize_rrt` script in the `data/` folder. Posearray can also be viewed from RViz.
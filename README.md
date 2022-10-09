# rattle-iros-2022
Parametric information-aware motion planning using the [RATTLE method](https://ieeexplore.ieee.org/document/9851849), as published in IROS/RA-L 2022.

## What is this?
RATTLE is a set of modules that perform parametric information-aware motion planning; in other words, RATTLE creates motion plans for robots to follow that try to be both *useful for learning about system unknowns* while also *attempting to reach a goal/minimize fuel use*. Uniquely, RATTLE includes capabilities to *use learned system information on-the-fly*, allowing the robot to better understand its own system model, while reasoning at a control level about how cautious it should be. Please see [the paper](https://ieeexplore.ieee.org/document/9851849) for more detail.

RATTLE is implemented here for a 6DOF free-flying robot, though its interfaces and methods are more general to robotic systems with parameteric unknowns in their system dynamics. Specifically, RATTLE has been tested on the International Space Station using the [Astrobee robots](https://github.com/nasa/astrobee).

## Requirements
For coordinated use of all packages with the Astrobee sim Ubuntu 16.04 is required (possibly 18.04 or 20.04 depending on Astrobee sim support) with a standard
[ROS](http://wiki.ros.org/ROS/Installation) installation. 

- [CasADi 3.5.5](https://github.com/casadi/casadi/releases/tag/3.5.5)
```
git clone https://github.com/casadi/casadi --branch 3.5.5
cd casadi && mkdir build && cd build
cmake ..
make -j2
make install
```
- [ACADO](https://acado.github.io/) (bundled)

- Pytope (bundled)

- Bullet (bundled)

- Astrobee custom msgs (bundled)

- Luajit (likley already on your system)

Note: Some of RATTLE's packages have baked-in dependencies on some Astrobee flight software classes, namely, ff_nodelet. A few extra dependencies are required 
## ROS Packages
RATTLE's functions are implemented here as separate ROS packages:

- Global planning
    - `rattle_rrt`: A "long-horizon" sampling-based planner that also accounts for, in this case, translational dynamics.

- Local information-aware planning
    - `rattle_acado_planner`: A mid-level "local" planner that also considers information content of planned trajectories. Implemented using ACADO.

- Low-level robust control
    - `casadi_nmpc` : A robust tube MPC control implementation, for the translational dynamics.
    - `z_poly_calc`: An mRPI (minimum robust postiviely invariant set) calculator, using Rakovic's method.

- Online parameter estimation
    - `inv_fam`: TODO get Monica's permission!
    - `param_est`: TODO get Monica's permission!

- ROS
    - `rattle_msgs`: Custom msg types used by these packages.
    - `rattle_coordinator`: The coordinator node that runs eveything and oversees execution. Uses a messy version of [ASAP](https://github.com/albee/ASAP), which has shiny cleaned up interfaces now.
    - `execute_asap`: A manager node that launches all other nodes. Useful if hardware testing, especially using Astrobee.
    - `data`: Miscellaneous I/O.

## Installation

### Dependencies
```
apt update && apt install libluajit-5.1-dev
```

### (optional) Astrobee simulation environment
Please consult The [Astrobee](https://github.com/nasa/astrobee) repository and follow their detailed installation instructions to set up the Astrobee simulation.
The simulation will essentially live in a ROS workspace devoted to it.

### RATTLE packages
Within a desired ROS workspace (such as the Astrobee simulation):

```
git clone https://github.com/albee/rattle-iros-2022
mv -rf rattle-iros-2022 src
catkin init
source ./devel/setup.bash
catkin build
```

This will build the RATTLE packages, which can be used standalone, or coordinate as a whole with the aid of a simulation environment (see below).

Note: strange catkin build errors for `ff_msgs` involving empy can be resolved using:
`catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m`



## Usage
RATTLE's packages can be used standalone, or as a coordinated whole as in the examples in the [paper](https://ieeexplore.ieee.org/document/9851849).
To run as a whole, RATTLE requires a simulation environment to respond to and provide message inputs/outputs. This repository is configured to interact with
the [Astrobee simulation environment](https://github.com/nasa/astrobee), on which RATTLE's modules should be placed.

Most packages separate ROS wrappers over core algorithms for standalone use; please consult individual READMEs in each package.
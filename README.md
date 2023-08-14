# rattle-iros-2022
Parametric information-aware motion planning using the [RATTLE method](https://ieeexplore.ieee.org/document/9851849), as published in IROS/RA-L 2022.

<p align="center">
  <img src="/img/astrobee.png" />
</p>

## What is this?
RATTLE is a set of modules that perform parametric information-aware motion planning; in other words, RATTLE creates motion plans for robots to follow that try to be both *useful for learning about system unknowns* while also *attempting to reach a goal/minimize fuel use*. Uniquely, RATTLE includes capabilities to *use learned system information on-the-fly*, allowing the robot to better understand its own system model, while reasoning at a control level about how cautious it should be. Please see [the paper](https://ieeexplore.ieee.org/document/9851849) for more detail.

RATTLE is implemented here for a 6DOF free-flying robot, though its interfaces and methods are more general to robotic systems with parameteric unknowns in their system dynamics. Specifically, RATTLE has been tested on the International Space Station using the [Astrobee robots](https://github.com/nasa/astrobee). A detailed account of code deployment and results of the flight experiments is given in [this recently submitted paper](https://arxiv.org/pdf/2301.01319.pdf).

<p align="center">
  <img src="/img/astrobee_rattle_sim.gif" />

  <img src="/img/astrobee_rattle_iss.gif" />
  
  <em>RATTLE running and replanning in a simulation environment (top), and demonstrating a trajectory with weighted information content on the ISS (bottom).</em>
</p>

## Requirements
For coordinated use of all packages with the Astrobee sim Ubuntu 16.04 is required (possibly 18.04 or 20.04 depending on Astrobee sim support) with a standard
[ROS](http://wiki.ros.org/ROS/Installation) installation. 

- [CasADi 3.5.5](https://github.com/casadi/casadi/releases/tag/3.5.5)
```
git clone https://github.com/casadi/casadi --branch 3.5.5
cd casadi && mkdir build && cd build  # this is the top-level directory
# GOTO line 614, and `set(WITH_LAPACK ON)`
# GOTO line 626, and `set(WITH_QPOASES ON)`
# (if you can find a cleaner way of specifying this please let us know!)
cmake ..
make -j2
make install  # You might need sudo
```

- [flann](https://github.com/flann-lib/flann)
```
apt install libflann-dev
```

- Misc. Python Dependencies
```
pip3 install matplotlib scipy numpy pycddlib
```

- gmp-dev
```
apt install libgmp-dev
```

- [ACADO](https://acado.github.io/) (bundled)

- Pytope (bundled)

- Bullet (bundled)

- Astrobee custom msgs and classes (bundled)

- Luajit (likley already on your system)
```
apt install libluajit-5.1-dev
```

- Autograd
```
pip3 install autograd
```

- pycddlib
```
pip3 install pycddlib
```

Note: Some of RATTLE's packages have baked-in dependencies on some Astrobee flight software classes, namely, ff_nodelet. A few extra dependencies are required, via the `astrobee` packages added here.


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
    - `inv_fam`: A package to estimate the post-saturation forces and torques, for use by the inertial parameter estimator
    - `param_est`: A sequential inertial parameter estimator - this implementation considers rigid body dynamics and estimates the mass and principal moments of inertia.

- ROS
    - `rattle_msgs`: Custom msg types used by these packages.
    - `rattle_coordinator`: The coordinator node that runs eveything and oversees execution. Uses a messy version of [ASAP](https://github.com/albee/ASAP), which has shiny cleaned up interfaces now.
    - `execute_asap`: A manager node that launches all other nodes. Useful if hardware testing, especially using Astrobee.
    - `data`: Miscellaneous I/O.

## Installation

### Dependencies
See above.

### RATTLE packages
Create a ROS workspace and set up the packages within it:

```
mkdir rattle-ws
cd rattle-ws
git clone https://github.com/albee/rattle-iros-2022
mv rattle-iros-2022 src
catkin init
catkin build
source ./devel/setup.bash
```

This will build the RATTLE packages, which can be used standalone, or coordinated as a whole with the aid of a simulation environment (see below).
Individual RATTLE use is best demonstrated using the `rattle_coordinator` package (consult the README).

Note: strange catkin build errors for `ff_msgs` involving empy can be resolved using:
`catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m`


### (optional) Astrobee simulation environment
Please consult The [Astrobee](https://github.com/nasa/astrobee) repository and follow their detailed installation instructions to set up the Astrobee simulation.
The simulation essentially lives in a ROS workspace devoted to it and Astrobee's core FSW packages. After setting up the sim, you should copy `rattle-ws`'s packages to the src subdirectory,

```
# in rattle-ws:
mv rattle-ws/src $ASTROBEE-WS/src
```

This will overwrite the default `astrobee` subdirectory: this is okay, since this will allow you to have access to the special `sim_rattle.launch` launchfile sequence.

## Usage
RATTLE's packages can be used standalone, or as a coordinated whole as in the examples in the [paper](https://ieeexplore.ieee.org/document/9851849).
To run as a whole, RATTLE requires a simulation environment to respond to and provide message inputs/outputs. This repository is configured to interact with
the [Astrobee simulation environment](https://github.com/nasa/astrobee), on which RATTLE's modules should be placed.

Most packages separate ROS wrappers over core algorithms for standalone use; please consult individual READMEs in each package.

Coordinated use is covered in `execute_asap/README.md`, which uses an implementation of the Astrobee [ASAP](https://github.com/albee/ASAP) testing interface. Launching the sim with RATTLE's default configuration is performed using:

```
roslaunch astrobee sim_rattle.launch rviz:=true dds:=false world:=iss
```
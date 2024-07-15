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
    - `inv_fam`: Astrobee's Force Allocation Module (or FAM) computes the nozzle opening angles required to apply commanded wrenches. The inv_fam package estimates the true (post-saturation) forces and torques from nozzle openings, for use by the inertial parameter estimator
    - `param_est`: A sequential inertial parameter estimator - this implementation considers rigid body dynamics and estimates the mass and principal moments of inertia.

- ROS
    - `rattle_msgs`: Custom msg types used by these packages.
    - `rattle_coordinator`: The coordinator node that runs eveything and oversees execution. Uses a messy version of [ASAP](https://github.com/albee/ASAP), which has shiny cleaned up interfaces now.
    - `execute_asap`: A manager node that launches all other nodes. Useful if hardware testing, especially using Astrobee.
    - `data`: Miscellaneous I/O.

## Installation

### Dependencies
See [above](###requirements).

### RATTLE packages
Create a ROS workspace and set up the packages within it:

```
mkdir rattle-ws
cd rattle-ws
catkin init
git clone https://github.com/albee/rattle-iros-2022
mv rattle-iros-2022 src/rattle
```

### Astrobee Simulation Setup

This implementation of RATTLE features tight integration with the Astrobee simulation environment. Get a compatible version:

```
export ASTROBEE_WS=${HOME}/rattle-ws/
git clone https://github.com/nasa/astrobee.git $ASTROBEE_WS/src/astrobee
pushd $ASTROBEE_WS/src/astrobee
git checkout v0.16.1
git submodule update --init --depth 1 description/media
popd
```

Now, build Astrobee's dependencies:

```bash
sudo apt update
sudo apt upgrade

pushd $ASTROBEE_WS
cd src/astrobee/scripts/setup
./add_ros_repository.sh
sudo apt-get update
cd debians
./build_install_debians.sh
cd ../
./install_desktop_packages.sh
sudo rosdep init
rosdep update
popd
```

Finally, configure and run catkin to build both the Astrobee sim and TRACE:

```bash
pushd $ASTROBEE_WS
./src/astrobee/scripts/configure.sh -l -F -D

# This step is very important! If your paths are wrong, you will not be able to build Astrobee packages.
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:+"$CMAKE_PREFIX_PATH:"}${ASTROBEE_WS}/src/astrobee/cmake"
catkin build -j2
```

Don't forget to source your workspace!

```bash
source $ASTROBEE_WS/devel/setup.bash
```

This will build the RATTLE packages, which can be used standalone, or coordinated as a whole with the aid of the Astrobee simulation environment (see below).
Individual RATTLE use is best demonstrated using the `rattle_coordinator` package (consult the README).


## Usage
RATTLE's packages can be used standalone, or as a coordinated whole as in the examples in the [paper](https://ieeexplore.ieee.org/document/9851849).
To run as a whole, RATTLE requires a simulation environment to respond to and provide message inputs/outputs, which you should have installed [above](###astrobee-simulation-setup).

### ISS-Like Commanding Environment
Consult `execute_asap/README.md`, which uses an implementation of the Astrobee [ASAP](https://github.com/albee/ASAP) testing interface. Launching the sim with RATTLE's default configuration is performed using:

```
roslaunch astrobee sim_rattle.launch rviz:=true dds:=false world:=iss
```

### RATTLE Commanding Environment

Consult `rattle_coordinator/README.md`, which uses a separate RATTLE commanding interface for a different set of tests.

- When first starting or switching between sim environments be sure to reset accelerometer bias for both Astrobees:

```bash
rosrun executive teleop_tool -ns "bumble/" -reset_bias
rosrun executive teleop_tool -ns "queen/" -reset_bias
```

### Standalone Usage

Most packages separate ROS wrappers from core algorithms for standalone use; please consult individual READMEs in each package to adapt for use with
other simulation frameworks.



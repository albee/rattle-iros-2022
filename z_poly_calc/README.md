# z_poly_calc
A node providing a ROS service to calculate robust invariant sets (RPI).


## What is this? Tube MPC explanation
Two main components are required for a Tube MPC: (1) a nominal MPC planner that operates using restricted constraints and (2) an ancillary tube controller that operates on top of the MPC.

Nominal trajectory setpoints (including state and input) are provided to the nominal MPC planner, which provides a set of *nominal inputs*, v, and *nominal states*, z. Note that the nominal state need not correspond to the current true state. Time horizons, constraints, Q and R weightings, and an uncertainty bound are also specified. Tightened limits are produced and an ancillary controller gain is calculated. Finally, the nominal MPC solution is computed over a time horizon, and is used in computed the ancillary control. The ancillary and nominal control are added to provide input.

The nominal MPC planner requires an approximation of the ``robust positively invariant set," also known as the Z set (hence the name of this node), which helps to ensure that the future system states will not evolve beyond this set. This node computes this set for a specified set of dynamics.


## Requirements
`z_poly_calc`'s main dependency is Pytope, which can be installed using:

```
pip3 install pytope
```

Pytope is also distributed here directly within the source directory. Acknowledgment to Tor Aksel N. Heirung, who
created this useful software.

Pytope has some additional requirements, mainly for Python3 ROS compatibility and libcdd:
```
pip3 install matplotlib scipy numpy pycddlib
apt-get install libgmp-dev
```

## Testing
```
rosrun casadi_nmpc z_poly_calc.py
rosrun casadi_nmpc service_call_tester.py
```

should trigger a service call to `z_poly_calc`.
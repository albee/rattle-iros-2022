# z_poly_calc

A node providing a ROS service to calculate robust invariant sets (RPI).

## Tube MPC Explanation

Two main components are required for a Tube MPC: (1) a nominal MPC planner that operates using restricted constraints and (2) an ancillary tube controller that operates on top of the MPC.

Nominal trajectory setpoints (including state and input) are provided to the nominal MPC planner, which provides a set of *nominal inputs*, v, and *nominal states*, z. Note that the nominal state need not correspond to the current true state. Time horizons, constraints, Q and R weightings, and an uncertainty bound are also specified. Tightened limits are produced and an ancillary controller gain is calculated. Finally, the nominal MPC solution is computed over a time horizon, and is used in computed the ancillary control. The ancillary and nominal control are added to provide input.

## Requirements for mRPI

Pytope has some requirements, mainly for Python3 ROS compatibility and libcdd.

Notes and Useful commands:
Debian OS packages are often compatible; 16.04 derives from stretch
`apt-get download $DEB:$ARCH`

For source:
Download everything
`pip3 download --no-binary ":all:" --dest "./" $PKG` (works best if also from a 16.04 system)
Install everything
`pip3 install $PKG -f ./ --no-index`

For wheels:
`pip download --index-url=https://www.piwheels.org/simple --platform linux_armv7l --no-deps -r requirements.txt`
https://stackoverflow.com/questions/60182080/install-wheel-file-on-off-line-machine-which-has-different-processor
`pip3 download --dest "./" $PKG -i https://www.piwheels.org/simple --no-deps`

export LC_ALL=C needed for pip to work

## Set up pip3
`sudo dpkg -i $PACKAGE`
* python-pip-whl (deb)[https://packages.ubuntu.com/xenial/all/python-pip-whl/download
* python3-wheel (deb)[https://packages.ubuntu.com/xenial/all/python3-wheel/download]
* python3-pip (deb)[https://packages.ubuntu.com/xenial/all/python3-pip/download]
* python3-setuptools(deb)[https://packages.ubuntu.com/xenial/all/python3-setuptools/download]

## Set up ROS Kinetic compatibility for Python3
`sudo dpkg -i $PACKAGE`
* python3-yaml (deb)[https://packages.debian.org/stretch/armhf/python3-yaml/download]
* python3-roman(deb)[https://packages.ubuntu.com/xenial/all/python3-roman/download]
* python3-dateutil(deb)[https://packages.ubuntu.com/xenial/python3-dateutil]
* python3-docutils(deb)[https://packages.ubuntu.com/xenial/all/python3-docutils/download]
* python3-pyparsing(deb)[https://packages.ubuntu.com/xenial/all/python3-pyparsing/download]
* python3-catkin-pkg-modules(deb)[https://index.ros.org/d/python3-catkin-pkg-modules/]
* python3-rospkg-modules(deb)[https://index.ros.org/d/python3-rospkg-modules/]

## Set up special Pytope dependencies
`sudo dpkg -i $PACKAGE`
`pip3 install $PKG -f ./ --no-index`
* python3-numpy (deb)[https://packages.debian.org/stretch/python3-numpy]
* python3-decorator(deb)[https://packages.debian.org/stretch/python3-decorator]
* python3-scipy (deb)[https://packages.debian.org/stretch/python3-scipy]

* python3-tk (deb)[https://packages.ubuntu.com/xenial/python3-tk]
* python3-cycler (deb)[https://packages.ubuntu.com/xenial/python3-cycler]
* python3-tz (deb)[https://packages.ubuntu.com/xenial/python3-tz]

* libgmpxx4ldbl (deb) [https://packages.ubuntu.com/xenial/libgmpxx4ldbl]
* libgmp10 (deb) [https://packages.ubuntu.com/search?keywords=libgmp10]
* libgmp-dev (GNU multiprecision library) (deb)[https://packages.ubuntu.com/xenial/libgmp-dev]

* cython (tar-src)[https://packages.ubuntu.com/xenial/amd64/cython/download]
* pycddlib (tar-src)[https://pypi.org/project/pycddlib/#files]
* matplotlib (tar-src)[https://pypi.org/project/matplotlib/#files] currently not used!


Test for LLP: can we run `rosrun casadi_nmpc z_poly_calc.py` and
`rosrun casadi_nmpc service_call_tester.py` successfully?

## Astrobee python3
Astrobee has the following Python3 packages installed:
* libpython3-stdlib:armhf
* libpython3.5:armhf
* libpython3.5-minimal:armhf
* libpython3.5-stdlib:armhf
* python3
* python3-minimal

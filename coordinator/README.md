# coordinator

High-level test logic for controlling the ReSWARM experiments. Test calls are made
using `execute_asap`, and the test run is operated from coordinator.

`coordinator` nodelet : Handles operations between the planner, estimator, controller, etc. Versions exist for the primary and secondary Astrobees.

## Usage

See `execute_asap` README for usage instructions.

## Details

* A base class is defined in `coordinator_base.tpp`. `primary.h` extends `CoordinatorBase` and implements virtual
test functions. 

* The resulting nodelet, instantiated by`primary_nodelet.cc` is the `primary_coordinator`, which takes in test numbers on `/reswarm/test_number`, and sends out
`/reswarm/status` msgs which command other nodes. 

* A number of methods are defined in `primary_*.hpp` files for specific tests. 
  e.g., see `primary_ooa_methods.hpp` for test commanding methods specific to on-orbit assembly tests.

Replace `primary` with `secondary` above for a second Astrobee.
The nodelet_plugins.xml file had to be defined for this nodelet.
The coordinator::CoordinatorNodelet class had to be created, extending FreeFlyerNodelet.


## Adding Publishers, Subscribers, and Services to Coordinator

New publishers, subscribers, and services monitored/advertised by `primary_coordinator` are created in `primary_nodelet.cc`,

```C++
  sub_casadi_status_ = nh->subscribe<reswarm_msgs::ReswarmCasadiStatus>("reswarm/casadi_nmpc/status", 5,
    boost::bind(&PrimaryNodelet::casadi_status_callback, this, _1));
```

with declarations in `primary.h`. Callback definitions may be defined in any desired `primary_*_methods.hpp`.


## Adding a New Test

1. Create a `primary_*_methods.hpp` file (or use an existing one). This will be the header-only definition of specific methods for your test number. Add
this new .hpp to `primary_nodelet.cc`,

```C++
  #include "coordinator/primary_*_methods.hpp"
```
.

2. Add an entry method for your test in `primary_*_methods.hpp`, for example,

```C++
void PrimaryNodelet::RunTest0(ros::NodeHandle *nh){
    NODELET_INFO_STREAM("[PRIMARY_COORD]: Congratulations, you have passed quick checkout. " 
    "May your days be blessed with only warnings and no errors.");
    ros::Duration(5.0).sleep();

    NODELET_DEBUG_STREAM("[PRIMARY COORD]: ...test complete!");
    base_reswarm_status_.test_finished = true;
};
```

Also be sure to add the declaration to the headers,

* `coordinator.h`:
```C++
virtual void RunTest0(ros::NodeHandle *nh) {};
```

* `primary.h`:
```C++
void RunTest0(ros::NodeHandle *nh) override;
```
.

3. Finally, make sure your method actually gets called when its test number is received by adding it to 
the main `primary_coordinator` loop in `coordinator_base.tpp`, 

```C++
void CoordinatorBase<T>::Run(ros::NodeHandle *nh) {
  ...
  if (base_reswarm_status_.test_number == 0) {
    RunTest0(nh);
  }
```
.

4. (Optional) You may need to enable test number filtering for your test number from `execute_asap.py`. Verify the `test_num_okay()` function.


## Astrobee setpoint state vector convention for ReSWARM (with time)
[t x y z xd yd zd qx qy qz qw wx wy wz xdd ydd zdd wxd wyd wzd
...
]

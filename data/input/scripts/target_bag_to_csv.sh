#!/bin/bash

# Astrobee FSW topics
rostopic echo -b $1 -p /gnc/ekf > $2/ekf.csv
rostopic echo -b $1 -p /gnc/ctl/command > $2/ctl_command.csv
rostopic echo -b $1 -p /hw/pmc/command > $2/pmc_command.csv
rostopic echo -b $1 -p /gnc/ctl/setpoint > $2/ctl_setpoint.csv

# TD status topic
rostopic echo -b $1 -p /td/status > $2/td_status.csv

# Tube-MPC topics
rostopic echo -b $1 -p /td/tube_mpc/debug > $2/mpc_debug.csv
rostopic echo -b $1 -p /td/tube_mpc/traj > $2/mpc_traj.csv



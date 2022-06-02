# info_rich_traj

Nodelet for calculating (or loading from file .csv) a maximum Fisher Information path between two points. Currently adapted to the ISS JEM and the granite table. 

The offline computed trajectories are stored in `data/input/info-trajectories`, named as excitation_env_test_num.csv, where env can be either 'grd' or 'iss'. The nodelet retrieves this trajectory and publish the poses, twists and wrenches to topics `gnc/ctl/setpoint` for the default Astrobee PID (or  `info_rich_traj/pose`,  `info_rich_traj/twist` and  `info_rich_traj/wrench` at each time step in case of a custom controller).



### usage
Waits for `info_send_traj` from topic `reswarm/status` to be true before publishing the trajectory. After the trajectory is sent, publishing is stopped and `info_sent_traj` from`reswarm/info_traj_status` is set to true.

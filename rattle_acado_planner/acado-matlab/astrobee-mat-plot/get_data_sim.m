function [ROSBAG_PATH, ROSBAG_NAME] = get_data_sim(robot_role)
  %{
  which_bee: {chaser, target}
  %}
  if robot_role == "chaser"
    ROSBAG_PATH = "../../data/roam-bags-sim/2021-05-31/honey/delayed/";

    % debugging body traj
    ROSBAG_NAME = "20210531_1956_roam_test77_chaser_0.bag";
    ROSBAG_NAME = "20210531_2146_roam_test77_chaser_0.bag";

    % debugging body traj
    ROSBAG_PATH = "../roam-bags-sim/2021-06-01/honey/delayed/";
    ROSBAG_NAME = "20210601_1952_roam_test77_chaser_0.bag";  % what happens if it's too fast!
    % ROSBAG_NAME = "20210601_1958_roam_test77_chaser_0.bag";  % same update rate
    % ROSBAG_NAME = "20210601_2001_roam_test77_chaser_0.bag";  % 0.03 rad/s difference
    % ROSBAG_NAME = "20210601_2022_roam_test77_chaser_0.bag";

    ROSBAG_PATH = "../../data/roam-bags-sim/2021-06-02/honey/delayed/";
    ROSBAG_NAME = "20210602_2221_roam_test77_chaser_0.bag";

    ROSBAG_PATH = "../../data/roam-bags-sim/2021-06-03/honey/delayed/";
    ROSBAG_NAME = "20210603_1527_roam_test5_chaser_0.bag";

    ROSBAG_PATH = "../../data/roam-bags-sim/2021-06-04/honey/delayed/";
    ROSBAG_NAME = "20210604_1440_roam_test14_chaser_0.bag";  % online_update_mode debug: updates
    % ROSBAG_NAME = "20210604_1456_roam_test14_chaser_0.bag";  % online_update_mode debug: NO updates
    ROSBAG_NAME = "20210604_1552_roam_test14_chaser_0.bag";  % 33 sec wait
    ROSBAG_NAME = "20210604_1653_roam_test14_chaser_0.bag";  % pause added
    ROSBAG_NAME = "20210604_1730_roam_test14_chaser_0.bag";  % first working online update
    ROSBAG_NAME = "20210604_1755_roam_test14_chaser_0.bag";  % velocity is fixed
    ROSBAG_NAME = "20210604_2001_roam_test14_chaser_0.bag";
    % ROSBAG_NAME = "20210604_1418_roam_test16_chaser_0.bag";  % tube_mpc tuning
    % ROSBAG_NAME = "20210604_1259_roam_test15_chaser_0.bag";  % tube_mpc tuning

    % Frontiers results
    ROSBAG_PATH = "../../data/roam-bags/frontiers-results/honey/delayed/";
    ROSBAG_NAME = "20210622_1657_roam_test21211112_chaser.bag";  % The frontiers bag
    % ROSBAG_NAME = "20210622_1703_roam_test21211112_chaser.bag";  % online_update_mode
    % ROSBAG_NAME = "20210622_1709_roam_test21211112_chaser.bag";  % online_update_mode
    % ROSBAG_NAME = "20210622_1647_roam_test21211122_chaser.bag";  % online_update_mode
        
  elseif robot_role == "target"
    % tumble examples
    ROSBAG_PATH = "../bags-sim/2021-05-31/bumble/delayed/";
    ROSBAG_NAME = "20210531_1255_roam_test2_target_0.bag";
    ROSBAG_NAME = "20210531_1314_roam_test2_target_0.bag";
    
  elseif robot_role == "primary"
    % ReSWARM sim
    ROSBAG_PATH = "../../data/reswarm-bags-sim/";
    ROSBAG_NAME = "20211022_1722_test16_primary_0.bag";
  
  elseif robot_role == "secondary"
    % ReSWARM sim
    ROSBAG_PATH = "../../data/reswarm-bags-sim/";

  end
end
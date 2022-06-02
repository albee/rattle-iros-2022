function [ROSBAG_PATH, ROSBAG_NAME] = get_data_iss_08_19_21(which_bee)
  %{
  which_bee: {chaser, target}
  %}
  if which_bee == "primary"
    ROSBAG_PATH = "../../data/reswarm-bags/iss-test1-08-19-21/primary/bags/";

    % robust MPC
%     ROSBAG_NAME = "20210819_1454_test14_primary.bag";
    ROSBAG_NAME = "20210819_1620_test14_primary.bag";
    
    % standard MPC
%     ROSBAG_NAME = "20210819_1500_test15_primary.bag";
%     ROSBAG_NAME = "20210819_1629_test15_primary.bag";

     % lqrrrt tracking
%     ROSBAG_NAME = "20210819_1646_test7_primary.bag";
%     ROSBAG_NAME = "20210819_1651_test8_primary.bag";

    % info-aware
%     ROSBAG_NAME = "20210819_1551_test10_primary.bag";
%     ROSBAG_NAME = "20210819_1559_test11_primary.bag";
%     ROSBAG_NAME = "20210819_1604_test12_primary.bag";

    % full tests
%     ROSBAG_NAME = "20210819_1608_test13_primary.bag";
%     ROSBAG_NAME = "20210819_1608_test13_primary.bag";
    
  elseif which_bee == "secondary"
    disp("An Astrobee hardware failure prevented two-Astrobee ops. No data from secondary.");
  end
end
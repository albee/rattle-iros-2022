function [ROSBAG_PATH, ROSBAG_NAME] = get_data_granite_lab_testing_05_27_21(which_bee)
  %{
  which_bee: {chaser, target}
  %}
  if which_bee == "chaser"
    % the best 2
    ROSBAG_NAME = "441_test_2021-05-03-16-56-12_chaser.bag";
    % ROSBAG_NAME = "411_test_2021-05-03-17-16-25_chaser.bag"

    ROSBAG_PATH = "../../data/roam-bags/granite-lab-testing-05-27-21/bsharp/";

    % the best 2
    ROSBAG_NAME = "20210527_204415_roam_test3_chaser_0.bag";
    % ROSBAG_NAME = "20210527_205132_roam_test3_chaser_0.bag";
    % ROSBAG_NAME = "20210527_205623_roam_test3_chaser_0.bag";

%     ROSBAG_NAME = "20210527_221738_roam_test12_chaser_0.bag";

%     ROSBAG_NAME = "20210527_215924_roam_test5_chaser_0.bag";

    % ROSBAG_NAME = "20210527_221010_roam_test8_chaser_0.bag";
    % ROSBAG_NAME = "20210527_221342_roam_test8_chaser_0.bag";
    
    
  
%     ROSBAG_PATH = "../bags/granite-lab-testing-06-07-21/chaser/";
%     ROSBAG_NAME = "20210607_2009_roam_test3_chaser_0.bag";
%     ROSBAG_NAME = "20210607_2113_roam_test15_chaser_0.bag"  % standard MPC
%     % ROSBAG_NAME = "20210607_2115_roam_test16_chaser_0.bag"  % tube MPC
%     ROSBAG_NAME = "20210607_2015_roam_test5_chaser_0.bag"; 
    
  elseif which_bee == "target"
    ROSBAG_PATH = "../../data/roam-bags/granite-lab-testing-05-27-21/wannabee/";
    ROSBAG_NAME = "412_test_2021-05-03-16-33-20_target.bag";
    ROSBAG_NAME = "7_test_2021-05-03-15-03-20_target.bag";
    ROSBAG_NAME = "7_test_2021-05-03-15-03-20_target.bag";
    ROSBAG_NAME = "411_test_2021-05-03-17-16-26_target.bag";

    % ROSBAG_NAME = "412_test_2021-05-03-16-33-20_target.bag";
    % ROSBAG_NAME = "441_test_2021-05-03-16-56-14_target.bag";
    % ROSBAG_NAME = "442_test_2021-05-03-16-04-09_target.bag";
    % ROSBAG_NAME = "412_test_2021-05-03-15-34-46_target.bag";

    % Large target localization deviations
    % ROSBAG_NAME = "412_test_2021-05-03-16-33-20_target.bag";
    % ROSBAG_NAME = "412_test_2021-05-03-15-56-14_target.bag"
    % ROSBAG_NAME = "411_test_2021-05-03-17-16-26_target.bag";
    % ROSBAG_NAME = "411_test_2021-05-03-17-13-11_target.bag";
    % ROSBAG_NAME = "411_test_2021-05-03-16-50-45_target.bag";
  end
end
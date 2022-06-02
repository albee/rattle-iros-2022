function [ROSBAG_PATH, ROSBAG_NAME] = get_data_iss_05_03_21(which_bee)
  %{
  which_bee: {chaser, target}
  %}
  if which_bee == "chaser"
    ROSBAG_PATH = "../../data/roam-bags/iss-test1-05-03-21/chaser/bags/";
    
    
    % ROSBAG_NAME = "4_test_2021-05-03-14-31-58_chaser.bag";
    % ROSBAG_NAME = "3_test_2021-05-03-14-25-50_chaser.bag";
    
    % 412s
%    ROSBAG_NAME = "412_test_2021-05-03-15-28-37_chaser.bag";  % bad one, jerky
%    ROSBAG_NAME = "412_test_2021-05-03-15-34-45_chaser.bag";  
    % ROSBAG_NAME = "412_test_2021-05-03-16-33-20_chaser.bag";  % no data
    
    % bad mpc tracking (overtuned?)
%     ROSBAG_NAME = "12_test_2021-05-03-15-19-08_chaser.bag";
%     ROSBAG_NAME = "442_test_2021-05-03-16-04-10_chaser.bag";

    % best bags for full pipeline
    ROSBAG_NAME = "412_test_2021-05-03-15-56-13_chaser.bag";  % good one
    ROSBAG_NAME = "411_test_2021-05-03-17-16-25_chaser.bag";
    
    ROSBAG_NAME = "4_test_2021-05-03-14-31-58_chaser.bag";
    
  elseif which_bee == "target"
    ROSBAG_PATH = "../../data/roam-bags/iss-test1-05-03-21/target/bags/";
    ROSBAG_NAME = "412_test_2021-05-03-16-33-20_target.bag";
    ROSBAG_NAME = "7_test_2021-05-03-15-03-20_target.bag";
    ROSBAG_NAME = "7_test_2021-05-03-15-03-20_target.bag";
%     ROSBAG_NAME = "411_test_2021-05-03-17-16-26_target.bag";

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
    
    ROSBAG_NAME = "412_test_2021-05-03-15-56-14_target.bag";  % good one
    ROSBAG_NAME = "411_test_2021-05-03-17-16-26_target.bag";
    ROSBAG_NAME = "2_test_2021-05-03-14-15-48_target.bag";
    ROSBAG_NAME = "412_test_2021-05-03-15-28-38_target.bag"
  end
end
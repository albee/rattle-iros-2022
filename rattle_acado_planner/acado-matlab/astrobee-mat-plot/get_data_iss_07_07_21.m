function [ROSBAG_PATH, ROSBAG_NAME] = get_data_iss_07_07_21(which_bee)
  %{
  which_bee: {chaser, target}
  %}
  if which_bee == "chaser"
    ROSBAG_PATH = "";
    ROSBAG_NAME = "";
    disp('No chaser data available (target debug test)!');
    
  elseif which_bee == "target"
    ROSBAG_PATH = "../../data/roam-bags/iss-debug-07-07-21/";
    ROSBAG_NAME = "20210707_1612_roam_test1_target.bag";
    ROSBAG_NAME = "20210707_1615_mob_nav_nocrew_roam_map_verification.bag";
    ROSBAG_NAME = "20210707_1639_roam_test2_target.bag";  % most important one for understanding results
  end
end
function [ROSBAG_PATH, ROSBAG_NAME] = get_data_granite_lab_testing_10_21_21(robot_role)
  %{
  robot_role: {primary, secondary}
  %}
  if robot_role == "primary"
    ROSBAG_PATH = "../../data/reswarm-bags/granite-lab-testing-10-21-21/primary/";
    ROSBAG_NAME = "20211021_2128_test16_primary_0.bag";
  end
end
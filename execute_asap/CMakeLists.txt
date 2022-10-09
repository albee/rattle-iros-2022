cmake_minimum_required(VERSION 3.0.0)
project(execute_asap)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ff_msgs
  ff_hw_msgs
  rattle_msgs
  data
)

catkin_package(
 LIBRARIES
 DEPENDS
  rospy
  std_msgs
  ff_msgs
  ff_hw_msgs
  rattle_msgs
  data
)

#install_launch_files()

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)
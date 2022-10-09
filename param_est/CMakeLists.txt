cmake_minimum_required(VERSION 3.0.0)
project(param_est)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_runtime
  message_generation
  geometry_msgs
)

add_message_files(
   FILES
   Params.msg
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
    rospy
    std_msgs
    message_runtime
    message_generation
)

# install_launch_files()
 
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)

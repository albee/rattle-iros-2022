# cmake_minimum_required(VERSION 2.8.3)
project(rattle_coordinator)

find_package(catkin REQUIRED
  COMPONENTS rospy std_msgs message_generation
)

add_message_files(
   FILES
   TestInstruct.msg
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/ DESTINATION share/${PROJECT_NAME}/src USE_SOURCE_PERMISSIONS)

install_launch_files()

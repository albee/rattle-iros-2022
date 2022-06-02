project(info_rich_traj)

find_package(PkgConfig REQUIRED) 

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy std_msgs
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)

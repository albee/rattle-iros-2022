cmake_minimum_required(VERSION 3.0.0)
project(inv_fam)

find_package(catkin REQUIRED COMPONENTS 
  rospy 
  std_msgs 
  geometry_msgs 
  message_generation
)


add_message_files(
   FILES
     ForwardFAM.msg
     InverseFAM.msg
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
    geometry_msgs 
    message_runtime
    ff_msgs
    ff_hw_msgs
    rattle_msgs
)
 

#install_launch_files()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)

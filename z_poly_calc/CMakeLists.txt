cmake_minimum_required(VERSION 3.0.0)
project(z_poly_calc)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    ff_msgs
    rattle_msgs
    rattle_acado_planner
)

catkin_package(
 LIBRARIES
 DEPENDS
    rospy
    std_msgs
    ff_msgs
    rattle_msgs
    rattle_acado_planner
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/ DESTINATION share/${PROJECT_NAME}/src USE_SOURCE_PERMISSIONS)
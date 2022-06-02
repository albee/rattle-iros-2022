project(coordinator)

catkin_package(
  LIBRARIES
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_msgs
    geometry_msgs
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Astrobee-esque way of linking libraries and adding include directories. All files in /src get added to TARGET by default.
# create_library(TARGET PrimaryNodelet
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ff_common ff_nodelet msg_conversions ff_flight
#   INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
#   DEPS ff_msgs ff_hw_msgs
#   ADD_SRCS ${CMAKE_CURRENT_SOURCE_DIR})  # additional source files

# Primary Nodelet
add_library(PrimaryNodelet src/primary_nodelet.cc)
target_link_libraries(PrimaryNodelet ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ff_common ff_nodelet msg_conversions ff_flight)
target_include_directories(PrimaryNodelet PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)  # src added for templated functions
add_dependencies(PrimaryNodelet ff_msgs ff_hw_msgs)

# Secondary Nodelet
add_library(SecondaryNodelet src/secondary_nodelet.cc)
target_link_libraries(SecondaryNodelet ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ff_common ff_nodelet msg_conversions ff_flight)
target_include_directories(SecondaryNodelet PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)  # src added for templated functions
add_dependencies(SecondaryNodelet ff_msgs ff_hw_msgs)

install(TARGETS PrimaryNodelet SecondaryNodelet DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.tpp")

install_launch_files()

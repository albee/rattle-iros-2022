project(data)

find_package(catkin REQUIRED COMPONENTS rosbag)

catkin_package(
  LIBRARIES
  DEPENDS
  CATKIN_DEPENDS
)

# install to hardware rospkg directory (will be found by rospkg)
install(DIRECTORY input/ DESTINATION share/${PROJECT_NAME}/input)

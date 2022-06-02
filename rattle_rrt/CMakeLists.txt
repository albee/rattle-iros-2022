# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

message("-- RRTNodelet and rrt_high_level added...")

project(rattle_rrt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations  -Werror=return-type")
# set(DCMAKE_BUILD_TYPE=Debug)

add_subdirectory(bullet3)

################################
# ROS/Catkin
#################################

# Declare the message files to be built
add_message_files(
  FILES
  RRTParams.msg
  TwistArray.msg
  WrenchArray.msg
  ellipsoid.msg
  ellipsoidArray.msg
)

# Actually generate the language-specific message/service files
generate_messages(
  DEPENDENCIES
  std_msgs geometry_msgs
)

# Astrobee CMake command to build messages: we are NOT using this method! Results in rrt_high_level target getting used
# create_msg_targets(DIR msg
#   DEPS std_msgs geometry_msgs geometry_msgs)

catkin_package(
  LIBRARIES
  DEPENDS
  CATKIN_DEPENDS
    message_runtime
    message_generation
    eigen_conversions
    eigen3
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
    ff_msgs
)

install_launch_files()

#################################
# Normal CMake C++ (using Astrobee CMake Commands)
#################################

# Astrobee-esque way of linking libraries and adding include directories. All files in /src get added to TARGET by default.
# NB: CMAKE_CURRENT_LIST_DIR will get the current directory of CMakeLists.txt, rather than just the top level
# NB2: must specify FULL shared library name for linking to work this way
# NB3: casadi is installed from source by the local user. See casadi_nmpc/README.md.
# NB4: manually specify the default source directory with DIR
# create_library(TARGET rattle_rrt
#   LIBS ${catkin_LIBRARIES} ff_common ff_nodelet msg_conversions
#   INC  ${EIGEN3_INCLUDE_DIR}
#   ADD_SRCS src/quaternion.cpp src/rigidBodyDynamics.cpp  # additional sources
# )

# create_library(TARGET rattle_rrt_nodelet
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ff_common ff_nodelet msg_conversions rattle_rrt  # these libraries are created higher up in the CMake hierarchy
#   INC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
#   DEPS ff_msgs ff_hw_msgs
# )

message("-- [BULLET]: added.")

add_library(rattle_rrt src/quaternion.cpp src/rigidBodyDynamics.cpp src/state_space_4d.cpp 
  src/state_space_6d_iss.cpp src/bullet_collision_checker.cpp)
target_link_libraries(rattle_rrt ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} ${Boost_LIBRARIES} ff_common ff_nodelet msg_conversions ff_flight BulletCollision BulletDynamics LinearMath)
target_include_directories(rattle_rrt PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include bullet3/src)

add_library(rattle_rrt_nodelet src/rattle_rrt_nodelet.cpp)
target_link_libraries(rattle_rrt_nodelet rattle_rrt)
target_include_directories(rattle_rrt_nodelet PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} include bullet3/src)
add_dependencies(rattle_rrt_nodelet ff_msgs ff_hw_msgs)

add_executable(test-rrt test/test_rrt.cpp)
target_link_libraries(test-rrt rattle_rrt)

add_executable(test-bullet test/test_bullet.cpp)
target_link_libraries(test-bullet rattle_rrt)

install(TARGETS rattle_rrt rattle_rrt_nodelet DESTINATION lib)
install(TARGETS test-rrt DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.tpp")

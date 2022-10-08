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
cmake_minimum_required(VERSION 3.0.0)
project(casadi_nmpc)
find_package(catkin REQUIRED)

find_package(PkgConfig)
pkg_search_module(casadi casadi) # required for cross-compile, can't use find_package

# codegen is not required when using the serial interface
#add_subdirectory(casadi-export)

message("--  CASADI_LIBRARY_DIRS: ${CASADI_LIBRARY_DIRS}")
message("--  CASADI_LIBRARIES: ${CASADI_LIBRARIES}")
message("--  CASADI_INCLUDEDIR: ${CASADI_INCLUDEDIR}")

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

#---casadi_nmpc---
# Astrobee-esque way of linking libraries and adding include directories. All files in /src get added to TARGET by default.
# NB1: CMAKE_CURRENT_LIST_DIR will get the current directory of CMakeLists.txt, rather than just the top level
# NB2: must specify FULL shared library name for linking to work this way
# NB3: casadi is installed from source by the local user. See casadi_nmpc/README.md.

### Codegen version
# create_library(TARGET casadi_nmpc
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi ff_common ff_nodelet msg_conversions ff_flight casadi_robust_tube_mpc casadi_mpc
#   INC casadi ${CMAKE_CURRENT_SOURCE_DIR}/../traj_gen_dlr/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../data/include ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/include
#   DEPS ff_msgs ff_hw_msgs
#   ADD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../data/src/csv_read.cc ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/src/backup_controller.cc)

### Serialized version
# create_library(TARGET casadi_nmpc
#   LIBS ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi ff_common ff_nodelet msg_conversions ff_flight
#   INC casadi ${CMAKE_CURRENT_SOURCE_DIR}/../traj_gen_dlr/include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
#     ${CMAKE_CURRENT_SOURCE_DIR}/../data/include ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/include
#   DEPS ff_msgs ff_hw_msgs
#   ADD_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/../data/src/csv_read.cc ${CMAKE_CURRENT_SOURCE_DIR}/../backup_controller/src/backup_controller.cc)

add_library(casadi_nmpc src/casadi_nmpc_nodelet.cc src/casadi_nmpc_utils.cc src/pd_attitude_controller.cc src/casadi_nmpc_ros.cc src/casadi_nmpc_core.cc)
# target_link_libraries(casadi_nmpc ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi ${CASADI_LIBRARY_DIRS} ff_common ff_nodelet msg_conversions ff_flight)
# target_include_directories(casadi_nmpc PUBLIC casadi ${CASADI_INCLUDEDIR} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
target_link_libraries(casadi_nmpc ${catkin_LIBRARIES} ${EIGEN_LIBRARIES} casadi ff_common ff_nodelet msg_conversions ff_flight)
target_include_directories(casadi_nmpc PUBLIC casadi ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} include)
add_dependencies(casadi_nmpc ff_msgs ff_hw_msgs)

install(TARGETS casadi_nmpc DESTINATION lib)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.tpp")

# install_launch_files()

# Enable gtest unit tests
enable_testing()
add_subdirectory(tests)

# install_launch_files()
# install(TARGETS backup_controller DESTINATION lib)



# cmake_minimum_required(VERSION 2.8.3)
project(acado_nmpc_rattle)

######################
## ROS Requirements ##
######################
## Compile as C++2020, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
 find_package(catkin REQUIRED COMPONENTS
   roslaunch
   roscpp
   std_msgs tf
   message_generation
   message_runtime
 )


## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES nmpc_acado_rattle_lib
  DEPENDS nmpc_acado_rattle_lib
)


###########
## Build ##
###########

## nmpc_acado node

## Compile ACADO shared library
add_library(nmpc_acado_rattle_lib
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/Bounds.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/Constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/CyclingManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/Indexlist.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/MessageHandling.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/QProblem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/QProblemB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/SubjectTo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/Utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/acado_qpoases_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/acado_integrator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/acado_solver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/acado_auxiliary_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/acado_dummy_file.c  # acado is missing these definitions!
)

target_include_directories(nmpc_acado_rattle_lib PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/INCLUDE/EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/NMPC_export/qpoases/SRC  # why is it in src? who knows...
)

add_dependencies(nmpc_acado_rattle_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# node
add_executable(nmpc_acado_rattle src/nmpc_acado_rattle.cpp)

link_directories(NMPC_export)
target_link_libraries(nmpc_acado_rattle nmpc_acado_rattle_lib ${catkin_LIBRARIES} ff_common msg_conversions ff_flight)
add_dependencies(nmpc_acado_rattle
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
         ${catkin_EXPORTED_TARGETS}
)


###########
## Install ##
###########

install(TARGETS nmpc_acado_rattle_lib DESTINATION lib)
install(TARGETS nmpc_acado_rattle DESTINATION bin)

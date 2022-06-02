# cmake_minimum_required(VERSION 2.8.3)
project(rattle_acado_planner)

option(USE_ISS_VERSION "Use ISS 6DOF Version" ON)

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

add_message_files(
  FILES
  NMPCInstruct.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
  LIBRARIES 
  DEPENDS 
)


# NB: We have to manually specify everything because acado has lots of junk in the codegen.
if(NOT USE_ISS_VERSION)
  message(" -- [acado] Using ground Acado!")
  # ACADO shared library export (from MATLAB, 3 DOF)
  # NOTE: ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_dummy_file.c  # acado is missing these definitions!
  # file(GLOB_RECURSE acado_3DOF_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/*.c*")
  # list(REMOVE_ITEM acado_3DOF_srcs "${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_solver_mex.c")  # boo, mex
  # file(GLOB_RECURSE acado_3DOF_incs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/*.h*")
  # add_library(rattle_acado_planner_export ${acado_3DOF_srcs})

  add_library(rattle_acado_planner_export
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/Bounds.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/Constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/CyclingManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/Indexlist.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/MessageHandling.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/QProblem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/QProblemB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/SubjectTo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/Utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_qpoases_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_integrator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_solver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_auxiliary_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_dummy_file.c  # acado is missing these definitions!
  )

  # target_include_directories(rattle_acado_planner_export PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
  target_include_directories(rattle_acado_planner_export PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/INCLUDE/EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/qpoases/SRC
  )

  # ACADO wrapper (3 DOF)
  add_library(rattle_acado_planner src/rattle_acado_planner.cpp)
  target_link_libraries(rattle_acado_planner rattle_acado_planner_export ff_common msg_conversions ff_flight)
  target_include_directories(rattle_acado_planner PUBLIC acado-export/RHP_export include)
else()
  message(" -- [acado] Using ISS Acado!")
  # ACADO shared library export (from MATLAB, 6 DOF)
  # NOTE: ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_export/acado_dummy_file.c  # acado is missing these definitions!
  # file(GLOB_RECURSE acado_6DOF_srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/*.c*")
  # list(REMOVE_ITEM acado_6DOF_srcs "${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/acado_solver_mex.c")  # boo, mex
  # message(" -- [acado] srcs are: "${acado_6DOF_srcs})
  # file(GLOB_RECURSE acado_6DOF_incs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/*.h*")
  # message(" -- [acado] incs are: "${acado_6DOF_incs})
  # add_library(rattle_acado_planner_export ${acado_6DOF_srcs})
  # target_include_directories(rattle_acado_planner_export PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  add_library(rattle_acado_planner_export
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/Bounds.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/Constraints.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/CyclingManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/Indexlist.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/MessageHandling.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/QProblem.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/QProblemB.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/SubjectTo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/Utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/acado_qpoases_interface.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/acado_integrator.c
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/acado_solver.c
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/acado_auxiliary_functions.c
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/acado_dummy_file.c  # acado is missing these definitions!
  )

  target_include_directories(rattle_acado_planner_export PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/INCLUDE
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/INCLUDE/EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/acado-export/RHP_6DOF_export/qpoases/SRC
  )

  # ACADO wrapper (6 DOF)
  add_library(rattle_acado_planner src/rattle_acado_planner.cpp)
  target_link_libraries(rattle_acado_planner rattle_acado_planner_export ff_common msg_conversions ff_flight)
  target_include_directories(rattle_acado_planner PUBLIC acado-export/RHP_6DOF_export include)
endif(NOT USE_ISS_VERSION)
unset(USE_ISS_VERSION CACHE)


# ACADO tester
add_executable(test-acado src/test_acado.cpp)
target_link_libraries(test-acado rattle_acado_planner_export rattle_acado_planner ff_common msg_conversions ff_flight)
target_include_directories(test-acado PUBLIC include)


# Node wrapper
add_executable(rattle_acado_planner_node src/rattle_acado_planner_node.cpp)
target_link_libraries(rattle_acado_planner_node rattle_acado_planner_export rattle_acado_planner ff_common msg_conversions ff_flight)
target_include_directories(rattle_acado_planner_node PUBLIC include)
add_dependencies(rattle_acado_planner_node ff_msgs ff_hw_msgs reswarm_msgs)


install(TARGETS rattle_acado_planner_export
        DESTINATION share/${PROJECT_NAME}/lib)

install(TARGETS rattle_acado_planner
        DESTINATION share/${PROJECT_NAME}/lib)

install(TARGETS rattle_acado_planner_node
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION share/${PROJECT_NAME}/bin)

install(TARGETS test-acado
        PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
        DESTINATION share/${PROJECT_NAME}/bin)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.tpp")

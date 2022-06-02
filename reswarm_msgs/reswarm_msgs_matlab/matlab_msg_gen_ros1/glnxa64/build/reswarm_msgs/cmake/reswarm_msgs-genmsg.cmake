# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "reswarm_msgs: 9 messages, 1 services")

set(MSG_I_FLAGS "-Ireswarm_msgs:/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg;-Istd_msgs:/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg;-Igeometry_msgs:/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg;-Istd_msgs:/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(reswarm_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiDebug.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiDebug.msg" "geometry_msgs/Wrench:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray:geometry_msgs/Vector3:std_msgs/Float64:geometry_msgs/Point:std_msgs/Header:std_msgs/MultiArrayLayout:std_msgs/String"
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusSecondary.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusSecondary.msg" ""
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/srv/ReswarmSrvMRPI.srv" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/srv/ReswarmSrvMRPI.srv" "std_msgs/Float64MultiArray:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmInfoStatus.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmInfoStatus.msg" ""
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmPlannerStatus.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmPlannerStatus.msg" ""
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmUCBoundStatus.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmUCBoundStatus.msg" ""
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusPrimary.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusPrimary.msg" ""
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmTestNumber.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmTestNumber.msg" ""
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmMsgMRPI.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmMsgMRPI.msg" "std_msgs/Float64MultiArray:std_msgs/MultiArrayDimension:std_msgs/Bool:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiStatus.msg" NAME_WE)
add_custom_target(_reswarm_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "reswarm_msgs" "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiStatus.msg" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiDebug.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Wrench.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Vector3.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmPlannerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmInfoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusSecondary.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmUCBoundStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusPrimary.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmTestNumber.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmMsgMRPI.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Bool.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)

### Generating Services
_generate_srv_cpp(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/srv/ReswarmSrvMRPI.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
)

### Generating Module File
_generate_module_cpp(reswarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(reswarm_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(reswarm_msgs_generate_messages reswarm_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiDebug.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusSecondary.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/srv/ReswarmSrvMRPI.srv" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmInfoStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmPlannerStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmUCBoundStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusPrimary.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmTestNumber.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmMsgMRPI.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_cpp _reswarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reswarm_msgs_gencpp)
add_dependencies(reswarm_msgs_gencpp reswarm_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reswarm_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiDebug.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Wrench.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Vector3.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/geometry_msgs/cmake/../msg/Point.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Header.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/String.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmPlannerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmInfoStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusSecondary.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmUCBoundStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusPrimary.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmTestNumber.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmMsgMRPI.msg"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Bool.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)
_generate_msg_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)

### Generating Services
_generate_srv_py(reswarm_msgs
  "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/srv/ReswarmSrvMRPI.srv"
  "${MSG_I_FLAGS}"
  "/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/usr/local/MATLAB/R2021a/sys/ros1/glnxa64/ros1/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
)

### Generating Module File
_generate_module_py(reswarm_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(reswarm_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(reswarm_msgs_generate_messages reswarm_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiDebug.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusSecondary.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/srv/ReswarmSrvMRPI.srv" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmInfoStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmPlannerStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmUCBoundStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmStatusPrimary.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmTestNumber.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmMsgMRPI.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/albee/workspaces/astrobee-ws-reswarm/freeflyer-reswarm/develop/reswarm_msgs/reswarm_msgs_matlab/matlab_msg_gen_ros1/glnxa64/src/reswarm_msgs/msg/ReswarmCasadiStatus.msg" NAME_WE)
add_dependencies(reswarm_msgs_generate_messages_py _reswarm_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(reswarm_msgs_genpy)
add_dependencies(reswarm_msgs_genpy reswarm_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS reswarm_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/reswarm_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reswarm_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(reswarm_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(reswarm_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs)
  install(CODE "execute_process(COMMAND \"/home/albee/.matlab/R2021a/ros1/glnxa64/venv/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/reswarm_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reswarm_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(reswarm_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(reswarm_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()

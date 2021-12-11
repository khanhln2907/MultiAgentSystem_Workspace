# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tip: 4 messages, 0 services")

set(MSG_I_FLAGS "-Itip:/home/qingchen/catkin_ws/src/tip/msg;-Iqualisys:/home/qingchen/catkin_ws/src/qualisys/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tip_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" NAME_WE)
add_custom_target(_tip_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tip" "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" NAME_WE)
add_custom_target(_tip_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tip" "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" "tip/UnicycleInfoStruct:std_msgs/Header"
)

get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" NAME_WE)
add_custom_target(_tip_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tip" "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" NAME_WE)
add_custom_target(_tip_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tip" "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip
)
_generate_msg_cpp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip
)
_generate_msg_cpp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip
)
_generate_msg_cpp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip
)

### Generating Services

### Generating Module File
_generate_module_cpp(tip
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tip_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tip_generate_messages tip_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_cpp _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_cpp _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" NAME_WE)
add_dependencies(tip_generate_messages_cpp _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" NAME_WE)
add_dependencies(tip_generate_messages_cpp _tip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tip_gencpp)
add_dependencies(tip_gencpp tip_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tip_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tip
  "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip
)
_generate_msg_eus(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip
)
_generate_msg_eus(tip
  "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip
)
_generate_msg_eus(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip
)

### Generating Services

### Generating Module File
_generate_module_eus(tip
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tip_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tip_generate_messages tip_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_eus _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_eus _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" NAME_WE)
add_dependencies(tip_generate_messages_eus _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" NAME_WE)
add_dependencies(tip_generate_messages_eus _tip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tip_geneus)
add_dependencies(tip_geneus tip_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tip_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip
)
_generate_msg_lisp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip
)
_generate_msg_lisp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip
)
_generate_msg_lisp(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip
)

### Generating Services

### Generating Module File
_generate_module_lisp(tip
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tip_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tip_generate_messages tip_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_lisp _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_lisp _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" NAME_WE)
add_dependencies(tip_generate_messages_lisp _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" NAME_WE)
add_dependencies(tip_generate_messages_lisp _tip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tip_genlisp)
add_dependencies(tip_genlisp tip_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tip_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tip
  "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip
)
_generate_msg_nodejs(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip
)
_generate_msg_nodejs(tip
  "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip
)
_generate_msg_nodejs(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip
)

### Generating Services

### Generating Module File
_generate_module_nodejs(tip
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tip_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tip_generate_messages tip_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_nodejs _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_nodejs _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" NAME_WE)
add_dependencies(tip_generate_messages_nodejs _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" NAME_WE)
add_dependencies(tip_generate_messages_nodejs _tip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tip_gennodejs)
add_dependencies(tip_gennodejs tip_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tip_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tip
  "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip
)
_generate_msg_py(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg"
  "${MSG_I_FLAGS}"
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip
)
_generate_msg_py(tip
  "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip
)
_generate_msg_py(tip
  "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip
)

### Generating Services

### Generating Module File
_generate_module_py(tip
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tip_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tip_generate_messages tip_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/ControlMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_py _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoMsg.msg" NAME_WE)
add_dependencies(tip_generate_messages_py _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/Vector.msg" NAME_WE)
add_dependencies(tip_generate_messages_py _tip_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/qingchen/catkin_ws/src/tip/msg/UnicycleInfoStruct.msg" NAME_WE)
add_dependencies(tip_generate_messages_py _tip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tip_genpy)
add_dependencies(tip_genpy tip_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tip_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tip
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET qualisys_generate_messages_cpp)
  add_dependencies(tip_generate_messages_cpp qualisys_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tip_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tip
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET qualisys_generate_messages_eus)
  add_dependencies(tip_generate_messages_eus qualisys_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tip_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tip
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET qualisys_generate_messages_lisp)
  add_dependencies(tip_generate_messages_lisp qualisys_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tip_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tip
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET qualisys_generate_messages_nodejs)
  add_dependencies(tip_generate_messages_nodejs qualisys_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tip_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tip
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET qualisys_generate_messages_py)
  add_dependencies(tip_generate_messages_py qualisys_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tip_generate_messages_py std_msgs_generate_messages_py)
endif()

# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "internal: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iinternal:/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(internal_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" NAME_WE)
add_custom_target(_internal_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "internal" "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(internal
  "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/internal
)

### Generating Services

### Generating Module File
_generate_module_cpp(internal
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/internal
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(internal_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(internal_generate_messages internal_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" NAME_WE)
add_dependencies(internal_generate_messages_cpp _internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(internal_gencpp)
add_dependencies(internal_gencpp internal_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS internal_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(internal
  "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/internal
)

### Generating Services

### Generating Module File
_generate_module_eus(internal
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/internal
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(internal_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(internal_generate_messages internal_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" NAME_WE)
add_dependencies(internal_generate_messages_eus _internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(internal_geneus)
add_dependencies(internal_geneus internal_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS internal_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(internal
  "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/internal
)

### Generating Services

### Generating Module File
_generate_module_lisp(internal
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/internal
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(internal_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(internal_generate_messages internal_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" NAME_WE)
add_dependencies(internal_generate_messages_lisp _internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(internal_genlisp)
add_dependencies(internal_genlisp internal_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS internal_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(internal
  "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/internal
)

### Generating Services

### Generating Module File
_generate_module_nodejs(internal
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/internal
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(internal_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(internal_generate_messages internal_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" NAME_WE)
add_dependencies(internal_generate_messages_nodejs _internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(internal_gennodejs)
add_dependencies(internal_gennodejs internal_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS internal_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(internal
  "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/internal
)

### Generating Services

### Generating Module File
_generate_module_py(internal
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/internal
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(internal_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(internal_generate_messages internal_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/cc/ee106a/fa24/class/ee106a-adc/final_project/src/internal/msg/BoardMove.msg" NAME_WE)
add_dependencies(internal_generate_messages_py _internal_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(internal_genpy)
add_dependencies(internal_genpy internal_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS internal_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/internal
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(internal_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/internal
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(internal_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/internal
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(internal_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/internal)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/internal
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(internal_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/internal)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/internal\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/internal
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(internal_generate_messages_py std_msgs_generate_messages_py)
endif()

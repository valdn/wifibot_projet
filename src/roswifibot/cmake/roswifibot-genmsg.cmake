# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "roswifibot: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iroswifibot:/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(roswifibot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" NAME_WE)
add_custom_target(_roswifibot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roswifibot" "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(roswifibot
  "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswifibot
)

### Generating Services

### Generating Module File
_generate_module_cpp(roswifibot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswifibot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(roswifibot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(roswifibot_generate_messages roswifibot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" NAME_WE)
add_dependencies(roswifibot_generate_messages_cpp _roswifibot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswifibot_gencpp)
add_dependencies(roswifibot_gencpp roswifibot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswifibot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(roswifibot
  "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswifibot
)

### Generating Services

### Generating Module File
_generate_module_eus(roswifibot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswifibot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(roswifibot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(roswifibot_generate_messages roswifibot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" NAME_WE)
add_dependencies(roswifibot_generate_messages_eus _roswifibot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswifibot_geneus)
add_dependencies(roswifibot_geneus roswifibot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswifibot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(roswifibot
  "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswifibot
)

### Generating Services

### Generating Module File
_generate_module_lisp(roswifibot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswifibot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(roswifibot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(roswifibot_generate_messages roswifibot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" NAME_WE)
add_dependencies(roswifibot_generate_messages_lisp _roswifibot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswifibot_genlisp)
add_dependencies(roswifibot_genlisp roswifibot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswifibot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(roswifibot
  "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswifibot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(roswifibot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswifibot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(roswifibot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(roswifibot_generate_messages roswifibot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" NAME_WE)
add_dependencies(roswifibot_generate_messages_nodejs _roswifibot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswifibot_gennodejs)
add_dependencies(roswifibot_gennodejs roswifibot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswifibot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(roswifibot
  "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswifibot
)

### Generating Services

### Generating Module File
_generate_module_py(roswifibot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswifibot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(roswifibot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(roswifibot_generate_messages roswifibot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/wifibot/Documents/wifibotprojet/src/roswifibot/msg/Status.msg" NAME_WE)
add_dependencies(roswifibot_generate_messages_py _roswifibot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roswifibot_genpy)
add_dependencies(roswifibot_genpy roswifibot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roswifibot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswifibot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roswifibot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(roswifibot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswifibot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/roswifibot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(roswifibot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswifibot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roswifibot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(roswifibot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswifibot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/roswifibot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(roswifibot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswifibot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswifibot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roswifibot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(roswifibot_generate_messages_py std_msgs_generate_messages_py)
endif()

cmake_minimum_required(VERSION 2.8.3)
project(asv_ros_revamp)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GPS.msg
  AHRS.msg
  SENSOR.msg
  NAVIGATION.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  controller_settings.srv
  controller_inputs.srv
  thruster_control.srv
  guidance_inputs.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES asv_ros_revamp
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(asv_ros_revamp
#   src/${PROJECT_NAME}/asv_ros_revamp.cpp
# )

add_library(serial src/common/serial.cpp)
target_link_libraries(serial ${roscpp_LIBRARIES})

add_library(gps src/sensors/gps.cpp src/common/serial.cpp)
target_link_libraries(gps ${roscpp_LIBRARIES})

add_library(ahrs src/sensors/ahrs.cpp src/common/serial.cpp)
target_link_libraries(ahrs ${roscpp_LIBRARIES})

add_library(utm src/navigation/utm.cpp)
target_link_libraries(utm ${roscpp_LIBRARIES})

add_library(controllers src/control/controllers.cpp)
target_link_libraries(controllers ${roscpp_LIBRARIES})

add_library(matrix src/guidance/matrix.cpp)
target_link_libraries(matrix ${roscpp_LIBRARIES})

add_library(guidance src/guidance/guidance.cpp src/guidance/matrix.cpp)
target_link_libraries(guidance ${roscpp_LIBRARIES})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(asv_ros_revamp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(asv_ros_revamp_node src/asv_ros_revamp_node.cpp)

add_executable(data_acq src/data_acquisition/data_acq.cpp  src/sensors/gps.cpp src/sensors/ahrs.cpp src/common/serial.cpp)
target_link_libraries(data_acq ${catkin_LIBRARIES})
add_dependencies(data_acq asv_ros_revamp_generate_messages_cpp)

add_executable(safety_node src/safety/safety_node.cpp)
target_link_libraries(safety_node ${catkin_LIBRARIES})
add_dependencies(safety_node asv_ros_revamp_generate_messages_cpp)

add_executable(navigation_node src/navigation/navigation_node.cpp src/navigation/utm.cpp)
target_link_libraries(navigation_node ${catkin_LIBRARIES}) 
add_dependencies(navigation_node asv_ros_revamp_generate_messages_cpp)

add_executable(control_node src/control/control_node.cpp src/control/controllers.cpp)
target_link_libraries(control_node ${catkin_LIBRARIES}) 
add_dependencies(control_node asv_ros_revamp_generate_messages_cpp)

add_executable(thruster_node src/actuators/thruster_node.cpp src/common/serial.cpp)
target_link_libraries(thruster_node ${catkin_LIBRARIES}) 
add_dependencies(thruster_node asv_ros_revamp_generate_messages_cpp)

add_executable(guidance_node src/guidance/guidance_node.cpp src/guidance/guidance.cpp src/guidance/matrix.cpp)
target_link_libraries(guidance_node ${catkin_LIBRARIES}) 
add_dependencies(guidance_node asv_ros_revamp_generate_messages_cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(asv_ros_revamp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(asv_ros_revamp_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS asv_ros_revamp asv_ros_revamp_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_asv_ros_revamp.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

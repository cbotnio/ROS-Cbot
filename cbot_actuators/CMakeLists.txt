cmake_minimum_required(VERSION 2.8.3)
project(cbot_actuators)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  cbot_ros_msgs
  cbot_common
  genmsg
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/actuators.cfg
)

include_directories(
  include
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp cbot_ros_msgs cbot_common
  INCLUDE_DIRS include
)

add_executable(thruster_node ${PROJECT_SOURCE_DIR}/src/thruster_node.cpp ${HEADER_FILES})
add_dependencies(thruster_node cbot_ros_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(thruster_node ${LIBS} ${catkin_LIBRARIES})
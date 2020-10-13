cmake_minimum_required(VERSION 2.8.3)
project(cbot_monitoring)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  cbot_ros_msgs
  genmsg
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/safety.cfg
)

include_directories(
  include
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp cbot_ros_msgs rospy
  INCLUDE_DIRS include
)

add_executable(safety_node ${PROJECT_SOURCE_DIR}/src/safety_node.cpp ${HEADER_FILES})
add_dependencies(safety_node cbot_ros_msgs_generate_messages_cpp ${PROJECT_NAME}_gencfg)
target_link_libraries(safety_node ${LIBS} ${catkin_LIBRARIES})


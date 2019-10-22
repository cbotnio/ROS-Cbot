cmake_minimum_required(VERSION 2.8.3)
project(cbot_monitoring)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  cbot_ros_msgs
  genmsg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp cbot_ros_msgs
  INCLUDE_DIRS include
)

add_executable(safety_monitor_node ${PROJECT_SOURCE_DIR}/src/safety_monitor_node.cpp ${HEADER_FILES})
add_dependencies(safety_monitor_node cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(safety_monitor_node ${LIBS} ${catkin_LIBRARIES})


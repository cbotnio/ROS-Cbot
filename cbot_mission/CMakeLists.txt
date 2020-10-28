cmake_minimum_required(VERSION 2.8.3)
project(cbot_mission)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  cbot_ros_msgs
  nav_msgs 
  geometry_msgs
  genmsg
  dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS roscpp cbot_ros_msgs nav_msgs geometry_msgs
  INCLUDE_DIRS include
)

add_library(utm ${PROJECT_SOURCE_DIR}/src/utm.cpp)
add_dependencies(utm cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(utm ${LIBS} ${catkin_LIBRARIES})

add_executable(waypoint_publisher ${PROJECT_SOURCE_DIR}/src/publishPath.cpp ${HEADER_FILES})
add_dependencies(waypoint_publisher cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(waypoint_publisher ${LIBS} ${catkin_LIBRARIES} utm)
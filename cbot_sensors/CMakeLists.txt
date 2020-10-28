cmake_minimum_required(VERSION 2.8.3)
project(cbot_sensors)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  cbot_ros_msgs
  cbot_common
  genmsg
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cbot_common_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS roscpp cbot_ros_msgs cbot_common
  INCLUDE_DIRS include
)

add_library(simple_gps ${PROJECT_SOURCE_DIR}/src/GPS/simple_gps.cpp)
add_dependencies(simple_gps cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(simple_gps ${LIBS} ${catkin_LIBRARIES} ${cbot_common_LIBRARIES})

add_library(gps ${PROJECT_SOURCE_DIR}/src/GPS/gps.cpp)
add_dependencies(gps cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(gps ${LIBS} ${catkin_LIBRARIES} ${cbot_common_LIBRARIES} simple_gps)

add_library(vnav ${PROJECT_SOURCE_DIR}/src/AHRS/vnav.cpp)
add_dependencies(vnav cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(vnav ${LIBS} ${catkin_LIBRARIES} ${cbot_common_LIBRARIES})

add_library(ahrs ${PROJECT_SOURCE_DIR}/src/AHRS/ahrs.cpp)
add_dependencies(ahrs cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(ahrs ${LIBS} ${catkin_LIBRARIES} ${cbot_common_LIBRARIES} vnav)


add_executable(sensors_node ${PROJECT_SOURCE_DIR}/src/sensor_node.cpp ${HEADER_FILES})
add_dependencies(sensors_node cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(sensors_node ${LIBS} ${catkin_LIBRARIES} ${cbot_common_LIBRARIES} gps ahrs)


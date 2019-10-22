cmake_minimum_required(VERSION 2.8.3)
project(cbot_guidance)

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

add_library(matrix ${PROJECT_SOURCE_DIR}/src/matrix.cpp)
add_dependencies(matrix cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(matrix ${LIBS} ${catkin_LIBRARIES})

add_library(guidance ${PROJECT_SOURCE_DIR}/src/guidance.cpp)
add_dependencies(guidance cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(guidance ${LIBS} ${catkin_LIBRARIES})

add_executable(guidance_node ${PROJECT_SOURCE_DIR}/src/guidance_node.cpp ${HEADER_FILES})
add_dependencies(guidance_node cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(guidance_node ${LIBS} ${catkin_LIBRARIES} guidance matrix)


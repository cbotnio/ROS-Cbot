cmake_minimum_required(VERSION 2.8.3)
project(cbot_common)

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
  CATKIN_DEPENDS
  roscpp
  cbot_ros_msgs
)

add_library(serial ${PROJECT_SOURCE_DIR}/src/interfaces/serial.cpp ${HEADER_FILES})
add_dependencies(serial cbot_ros_msgs_generate_messages_cpp)
target_link_libraries(serial ${LIBS} ${catkin_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS roscpp cbot_ros_msgs
  INCLUDE_DIRS include
  LIBRARIES serial
)

install(TARGETS serial 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".git" EXCLUDE)
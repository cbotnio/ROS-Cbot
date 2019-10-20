cmake_minimum_required(VERSION 2.8.6)
project(cbot_ros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs genmsg)

add_message_files(
  DIRECTORY sensors
  FILES
  AHRS.msg
  GPS.msg
)

add_message_files(
  DIRECTORY monitoring
  FILES
  SensorsStatus.msg
)


add_service_files(
  DIRECTORY srv
  FILES
  ControllerInputs.srv
  ControllerSettings.srv
  GuidanceInputs.srv
  ThrusterControl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs
#  DEPENDS message_runtime
)

cmake_minimum_required(VERSION 2.8.6)
project(cbot_ros_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs genmsg)

add_message_files(
  DIRECTORY sensors/msg
  FILES
  AHRS.msg
  GPS.msg
)


add_message_files(
  DIRECTORY actuators/msg
  FILES
  ThrusterInputs.msg
  ThrusterCMDM.msg
)

add_message_files(
  DIRECTORY controller/msg
  FILES
  ControllerInputs.msg
)

add_message_files(
  DIRECTORY monitoring/msg
  FILES
  SensorsStatus.msg
)

add_service_files(
  DIRECTORY guidance/srv
  FILES
  GuidanceInputs.srv
)


add_service_files(
  DIRECTORY mission_parser/srv
  FILES
  MissInputs.srv
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

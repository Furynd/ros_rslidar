cmake_minimum_required(VERSION 2.8.3)
project(rslidar_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)

add_message_files(
  DIRECTORY msg
  FILES
  rslidarPacket.msg
  rslidarScan.msg
  ThreePointData.msg
  objectDetection.msg
)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

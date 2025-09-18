cmake_minimum_required(VERSION 2.8.3)
project(messages)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
)

add_message_files(
  FILES
  MyOdom.msg
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_package()

cmake_minimum_required(VERSION 2.8.3)
project(pa10_controller_interface)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  pa10controller
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

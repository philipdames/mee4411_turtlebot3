cmake_minimum_required(VERSION 2.8.3)
project(phase1)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs visualization_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


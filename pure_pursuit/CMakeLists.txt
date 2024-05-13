cmake_minimum_required(VERSION 3.0.2)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phase3
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


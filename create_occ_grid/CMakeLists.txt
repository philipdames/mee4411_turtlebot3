cmake_minimum_required(VERSION 3.0.2)
project(create_occ_grid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs tf2_ros visualization_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


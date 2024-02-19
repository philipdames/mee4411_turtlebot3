cmake_minimum_required(VERSION 3.0.2)
project(wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2_ros
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs tf2_ros
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  src/wheel_odometry/wheel_odometry.py
  src/wheel_odometry/tb3_kinematics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

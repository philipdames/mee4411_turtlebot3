cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  occupancy_grid
  sensor_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs occupancy_grid sensor_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  nodes/icp_localization
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

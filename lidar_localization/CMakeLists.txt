cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  occupancy_grid
  tb3_simulation
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs visualization_msgs occupancy_grid tb3_simulation
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  nodes/icp_localization
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

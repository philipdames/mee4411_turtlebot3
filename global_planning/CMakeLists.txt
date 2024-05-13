cmake_minimum_required(VERSION 3.0.2)
project(global_planning)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  occupancy_grid
  rospy
  tb3_simulation
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES global_planning
#  CATKIN_DEPENDS geometry_msgs nav_msgs occupancy_grid rospy tb3_simulation
#  DEPENDS system_lib
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  nodes/prm_planner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

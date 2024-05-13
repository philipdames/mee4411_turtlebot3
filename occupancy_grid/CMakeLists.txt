cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid)

cmake_policy(SET CMP0148 OLD)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  std_msgs
  visualization_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS nav_msgs std_msgs visualization_msgs
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS 
  nodes/create_occupancy_grid
  nodes/block_vis
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    tests/test_map_conversions.py
    DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
endif()

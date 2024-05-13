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
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_localization
#  CATKIN_DEPENDS geometry_msgs nav_msgs occupancy_grid sensor_msgs
#  DEPENDS system_lib
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
  nodes/lidar_localization
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(tests/test_transform2d_utils.py
                      DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

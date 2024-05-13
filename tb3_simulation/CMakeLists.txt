cmake_minimum_required(VERSION 3.0.2)
project(tb3_simulation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  tf2_ros
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tb3_simulation
#  CATKIN_DEPENDS geometry_msgs rospy tf2_ros
#  DEPENDS system_lib
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  nodes/repub_simulated_sensors
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
catkin_add_nosetests(tests/test_transform2d_utils.py
                    DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS})
endif()

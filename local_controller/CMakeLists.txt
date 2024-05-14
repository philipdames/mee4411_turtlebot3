cmake_minimum_required(VERSION 3.0.2)
project(local_controller)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  tf
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phase3
#  CATKIN_DEPENDS geometry_msgs nav_msgs rospy
#  DEPENDS system_lib
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS 
  nodes/pure_pursuit
  nodes/dnn_controller
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    tests/test_pure_pursuit.py
    DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
endif()
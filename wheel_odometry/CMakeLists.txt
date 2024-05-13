cmake_minimum_required(VERSION 3.0.2)
project(wheel_odometry)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  tf2_ros
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs tf2_ros
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS 
  nodes/wheel_odometry
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(
    tests/test_kinematics.py
    DEPENDENCIES ${${PROJECT_NAME}_EXPORTED_TARGETS}
  )
endif()
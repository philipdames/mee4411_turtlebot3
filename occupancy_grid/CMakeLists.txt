cmake_minimum_required(VERSION 3.0.2)
project(occupancy_grid)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  visualization_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs tf2_ros visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/create_occupancy_grid nodes/block_vis
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/map_conversions.test)
endif()

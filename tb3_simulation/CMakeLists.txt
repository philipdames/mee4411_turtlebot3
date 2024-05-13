cmake_minimum_required(VERSION 3.0.2)
project(tb3_simulation)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-frounding-math ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS roscpp nav_msgs rosbag sensor_msgs tf message_generation)

  find_package(Armadillo REQUIRED)
  find_package(CGAL REQUIRED)
  find_package(LAPACK REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  FILES
  ScanPair.msg
  PoseStampedNamed.msg
  PoseStampedNamedArray.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs sensor_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_simlib
  CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs rosbag tf message_runtime
)

###########
## Build ##
###########
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS}
)

add_library(laser_simlib
  src/LaserSimulator.cc
)
add_dependencies(laser_simlib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(laser_simlib 
  ${CGAL_LIBRARY}
  ${catkin_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${LAPACK_LIBRARIES} 
)

add_executable(laser_simulator
  src/laser_simulator.cc
)
add_dependencies(laser_simulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(laser_simulator
  laser_simlib
  ${catkin_LIBRARIES}
)

add_executable(make_scan_pairs
  src/make_scan_pairs.cpp
)
add_dependencies(make_scan_pairs
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(make_scan_pairs
  laser_simlib
  ${catkin_LIBRARIES}
)

add_executable(pose_aggregator
  src/pose_aggregator_node.cpp
)
add_dependencies(pose_aggregator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(pose_aggregator
  ${catkin_LIBRARIES}
)

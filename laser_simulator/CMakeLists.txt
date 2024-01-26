cmake_minimum_required(VERSION 3.0.2)
project(laser_simulator)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-frounding-math ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS roscpp nav_msgs rosbag sensor_msgs tf message_generation)

add_message_files(FILES ScanPair.msg PoseStampedNamed.msg PoseStampedNamedArray.msg)

generate_messages(DEPENDENCIES nav_msgs sensor_msgs geometry_msgs)

find_package(Armadillo REQUIRED)
find_package(CGAL REQUIRED)
find_package(LAPACK REQUIRED)

catkin_package(INCLUDE_DIRS include
   LIBRARIES laser_simlib
   CATKIN_DEPENDS roscpp nav_msgs sensor_msgs geometry_msgs rosbag tf
   message_runtime)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
  ${CGAL_INCLUDE_DIRS})

add_library(laser_simlib src/LaserSimulator.cc)
add_dependencies(laser_simlib ${PROJECT_NAME}_gencpp)

target_link_libraries(laser_simlib ${CGAL_LIBRARY})

target_link_libraries(laser_simlib
  ${ARMADILLO_LIBRARIES} ${LAPACK_LIBRARIES} ${roscpp_LIBRARIES})

add_executable(laser_simulator src/laser_simulator.cc)
target_link_libraries(laser_simulator laser_simlib ${catkin_LIBRARIES})
add_dependencies(laser_simulator ${PROJECT_NAME}_gencpp)

add_executable(make_scan_pairs src/make_scan_pairs.cpp)
target_link_libraries(make_scan_pairs laser_simlib ${catkin_LIBRARIES})
add_dependencies(make_scan_pairs ${PROJECT_NAME}_gencpp)

add_executable(pose_aggregator src/pose_aggregator_node.cpp)
target_link_libraries(pose_aggregator ${catkin_LIBRARIES})
add_dependencies(pose_aggregator ${PROJECT_NAME}_gencpp)

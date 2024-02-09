cmake_minimum_required(VERSION 3.0.2)
project(kinematic_sim)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-frounding-math ${CMAKE_CXX_FLAGS}")

find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf2 tf2_ros nav_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES playermap
  CATKIN_DEPENDS 
    roscpp
    tf2
    tf2_ros
    nav_msgs
    geometry_msgs
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_library(playermap src/map.c src/rosmap.cpp)
target_link_libraries(playermap ${catkin_LIBRARIES})

add_executable(kinematic_sim src/kinematic_sim.cc)
target_link_libraries(kinematic_sim playermap ${catkin_LIBRARIES})

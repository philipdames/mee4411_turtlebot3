cmake_minimum_required(VERSION 2.8.3)
project(mee4411_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

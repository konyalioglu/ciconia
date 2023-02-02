cmake_minimum_required(VERSION 3.0.2)
project(ciconia_indoor_test_launch)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros
)



catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  mavros
)


include_directories(include ${catkin_INCLUDE_DIRS})
link_directories()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")



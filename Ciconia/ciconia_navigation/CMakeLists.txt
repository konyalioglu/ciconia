cmake_minimum_required(VERSION 3.0.2)
project(ciconia_navigation)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
)
find_package(gazebo REQUIRED)



catkin_python_setup()


catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  gazebo_msgs
)




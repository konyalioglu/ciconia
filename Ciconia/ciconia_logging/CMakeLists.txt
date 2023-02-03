cmake_minimum_required(VERSION 3.0.2)
project(ciconia_logging)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  ciconia_msgs
)


catkin_python_setup()


catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  ciconia_msgs
)




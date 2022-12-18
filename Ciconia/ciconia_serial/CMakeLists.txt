cmake_minimum_required(VERSION 3.0.2)
project(ciconia_serial)



find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  ciconia_msgs
)


catkin_python_setup()


catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  ciconia_msgs
)




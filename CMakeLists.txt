cmake_minimum_required(VERSION 3.0.2)
project(ciconia)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  xsens_msgs
)


catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime xsens_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_message_files(
  FILES
  rc_messages.msg
)


catkin_install_python(PROGRAMS
  scripts/control.py 
  scripts/physics.py
  scripts/processImage.py
  scripts/utils.py
  scripts/SensorModels/xsensSensorModel.py
  scripts/Controllers/model_predictive_controller.py 
  scripts/Controllers/lqr.py 
  scripts/Controllers/pid.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



cmake_minimum_required(VERSION 3.0.2)
project(ciconia)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  xsens_msgs
)
find_package(gazebo REQUIRED)


catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  message_runtime
  xsens_msgs
)


add_message_files(
  FILES
  rc_messages.msg
)


catkin_python_setup()


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(joint_plugin SHARED plugins/src/joint_plugin.cpp)
target_link_libraries(joint_plugin ${GAZEBO_LIBRARIES})




#catkin_install_python(PROGRAMS
#  scripts/control.py 
#  scripts/physics.py
#  scripts/processImage.py
#  scripts/utils.py
#  scripts/SensorModels/xsensSensorModel.py
#  scripts/Controllers/model_predictive_controller.py 
#  scripts/Controllers/lqr.py 
#  scripts/Controllers/pid.py 
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)



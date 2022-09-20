cmake_minimum_required(VERSION 3.0.2)
project(ciconia)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  xsens_msgs
  gazebo_msgs
)
find_package(gazebo REQUIRED)


add_message_files(
  FILES
  rcMessages.msg
  quadrotorControl.msg
  transitionControl.msg
  flightControl.msg
  estimatedStates.msg
)


catkin_python_setup()


generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
 )


catkin_package(CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  message_generation
  message_runtime
  xsens_msgs
  gazebo_msgs
)


include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_library(joint_plugin SHARED plugins/src/joint_plugin.cpp)
target_link_libraries(joint_plugin ${GAZEBO_LIBRARIES})

add_library(gazeboStatePublisherPlugin SHARED plugins/src/gazeboStatePublisherPlugin.cpp)
target_link_libraries(gazeboStatePublisherPlugin ${GAZEBO_LIBRARIES})


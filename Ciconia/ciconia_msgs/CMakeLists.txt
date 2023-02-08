cmake_minimum_required(VERSION 3.0.2)
project(ciconia_msgs)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)


add_message_files(
  FILES
  rcMessages.msg
  motorSignal.msg
  quadrotorControl.msg
  transitionControl.msg
  flightControl.msg
  estimatedStates.msg
  altPIDControl.msg
  altMPCControl.msg
)


generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
 )


catkin_package(CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  message_generation
  message_runtime

)


include_directories(include ${catkin_INCLUDE_DIRS})



cmake_minimum_required(VERSION 3.0.2)
project(lander_junior_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs std_srvs genmsg)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES ActuatorCommands.msg AttitudeState.msg Force.msg PositionalState.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs std_srvs)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lander_junior_ros
  CATKIN_DEPENDS roscpp rospy std_msgs std_srvs)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(librobotcontrol SHARED IMPORTED GLOBAL)
set_target_properties(librobotcontrol PROPERTIES IMPORTED_LOCATION /usr/lib/librobotcontrol.so)

add_subdirectory(src)

#add_executable(lander_junior_ros 
#  src/Servo.cpp
#  src/ESC.cpp
#  src/AttitudeControl.py
#  src/PositionControl.py
#  src/StateEstimateVicon.py
#  src/TakeoffServer.py
#)

#target_link_libraries(lander_junior_ros ${catkin_LIBRARIES} robotcontrol)

#add_dependencies(lander_junior_ros lander_junior_ros_generate_messages_cpp)

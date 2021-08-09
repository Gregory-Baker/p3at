cmake_minimum_required(VERSION 3.0.2)
project(vr_pan_tilt_autophat)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vr_pan_tilt_autophat
#  CATKIN_DEPENDS geometry_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



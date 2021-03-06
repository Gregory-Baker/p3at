cmake_minimum_required(VERSION 3.0.2)
project(p3at_base)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager diagnostic_updater geometry_msgs hardware_interface p3at_msgs 
  realtime_tools roscpp roslaunch roslint rosserial_server sensor_msgs std_msgs
  topic_tools
)

find_package(Boost REQUIRED COMPONENTS thread chrono)

catkin_package()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(p3at_node
  src/p3at_base.cpp
  src/p3at_hardware.cpp)
target_link_libraries(p3at_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(p3at_node p3at_msgs_gencpp rosserial_msgs_gencpp)


install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

roslaunch_add_file_check(launch/base.launch)
roslint_cpp()
roslint_add_test()


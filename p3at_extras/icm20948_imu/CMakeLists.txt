cmake_minimum_required(VERSION 3.0.2)
project(icm20948_imu)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
	nodes/icm20948_node
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


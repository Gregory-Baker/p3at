cmake_minimum_required(VERSION 3.0.2)
project(p3at_bringup)


find_package(catkin REQUIRED COMPONENTS
  robot_upstart
)

find_package(catkin REQUIRED)

catkin_package()
catkin_python_setup()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/install
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

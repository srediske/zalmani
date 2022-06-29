cmake_minimum_required(VERSION 3.0.2)

project(zalmani_commander)

find_package(catkin REQUIRED
  COMPONENTS
    rospy
)

find_package(Boost REQUIRED python)

catkin_python_setup()

catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/commander
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

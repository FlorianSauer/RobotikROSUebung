cmake_minimum_required(VERSION 2.8.3)
project(beleg)

find_package(catkin REQUIRED COMPONENTS roscpp rospy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp rospy
  DEPENDS
)

## other directories
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

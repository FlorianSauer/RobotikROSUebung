 cmake_minimum_required(VERSION 2.8.3)
project(prediction)

find_package(OpenCV 3.2 REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy roscpp sensor_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS rospy roscpp sensor_msgs numpy cv_bridge
  DEPENDS
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


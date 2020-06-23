cmake_minimum_required(VERSION 3.0.2)
project(rs750_viz)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
)

################################################################################
# Declare catkin configuration

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

################################################################################
# Install

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

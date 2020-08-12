cmake_minimum_required(VERSION 3.0.2)
project(rs750_controller)

################################################################################
# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find dependent catkin packages

find_package(catkin REQUIRED COMPONENTS
    control_toolbox
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf
)

################################################################################
# Setup python modules

catkin_python_setup()

################################################################################
# Dynamic reconfigure

generate_dynamic_reconfigure_options(
    cfg/SailController.cfg
)

################################################################################
# Declare catkin configuration

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        rs750_base
    CATKIN_DEPENDS
        control_toolbox
        dynamic_reconfigure
        geometry_msgs
        hardware_interface
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
)

################################################################################
# Build

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

# Library
add_library(rs750_controller
    src/hardware.cpp
)
target_link_libraries(rs750_controller ${catkin_LIBRARIES})

################################################################################
# Install

install(TARGETS
    rs750_controller
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/rs750_controller/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
    nodes/sail_controller
    nodes/sail_controller_config
    scripts/twist_translate.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.0.2)
project(rs750_controller)

################################################################################
# Preprocessor options
option(CURIO_WITH_PYTHON_HAL "Use the Python HAL" OFF)

if(CURIO_WITH_PYTHON_HAL)
    add_definitions(-DCURIO_USE_PYTHON_HAL=1)
else()
    add_definitions(-DCURIO_USE_PYTHON_HAL=0)
endif()

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
    cfg/CourseController.cfg
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
    src/course_controller.cpp
    src/hardware.cpp
)
target_link_libraries(rs750_controller ${catkin_LIBRARIES})

# Executable
add_executable(course_controller_node
    src/course_controller_node.cpp
)
target_link_libraries(course_controller_node rs750_controller ${catkin_LIBRARIES})
add_dependencies(course_controller_node rs750_controller)

################################################################################
# Install

install(TARGETS
    rs750_controller
    course_controller_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/rs750_controller/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

catkin_install_python(PROGRAMS
    nodes/course_controller
    nodes/sail_controller
    nodes/sail_controller_config
    scripts/twist_translate.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

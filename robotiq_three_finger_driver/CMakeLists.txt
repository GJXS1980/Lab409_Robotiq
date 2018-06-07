cmake_minimum_required(VERSION 2.6.0)

# By default, don't install out to system.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)"
    FORCE)
endif()

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME robotiq_three_finger_driver)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

pods_install_python_script(robotiq_tcp_node python/pyrobotiqhand/robotiq_tcp_node.py)
pods_install_python_script(robotiq_command python/pyrobotiqhand/robotiq_command.py)
pods_install_python_script(robotiq_command_indiv python/pyrobotiqhand/robotiq_command_indiv.py)


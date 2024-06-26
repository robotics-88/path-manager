cmake_minimum_required(VERSION 3.0.2)
project(path_manager)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  roslib
  std_msgs
  tf2_ros
  pcl_ros
  livox_ros_driver
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_manager
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_manager_node
  src/path_manager_node.cpp
)

add_library(path_manager 
  src/path_manager.cpp
)

target_link_libraries(path_manager_node
  path_manager
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# install nodes
install(TARGETS path_manager_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

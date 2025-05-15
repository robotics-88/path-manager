cmake_minimum_required(VERSION 3.5)
project(path_manager)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(messages_88 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(PCL REQUIRED)
find_package(task_manager REQUIRED)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

add_library(path_manager SHARED
  src/path_manager.cpp
)

ament_target_dependencies(path_manager
  rclcpp
  geometry_msgs
  messages_88
  nav_msgs
  sensor_msgs
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  livox_ros_driver2
  task_manager
)


add_executable(path_manager_node
  src/path_manager_node.cpp
)

target_link_libraries(path_manager_node path_manager)

install(TARGETS
  path_manager
  path_manager_node
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

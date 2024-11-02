cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(avoid_wall)

# Find the Ignition_Transport library
find_package(ignition-transport11 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(${CMAKE_BINARY_DIR})
include_directories(${tf2_INCLUDE_DIRS})

if (EXISTS "${CMAKE_SOURCE_DIR}/lidar_node.cc")
  add_executable(lidar_node lidar_node.cc)
  ament_target_dependencies(lidar_node
     tf2_geometry_msgs
   )
  target_link_libraries(lidar_node ignition-transport${IGN_TRANSPORT_VER}::core ${tf2_LIBRARIES})
endif()

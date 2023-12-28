cmake_minimum_required(VERSION 3.14)
project(raylib_viz)

find_package(autoware_cmake REQUIRED)
autoware_package()

# find dependencies
find_package(raylib REQUIRED)

include_directories(
  include
  SYSTEM
  ${raylib_INCLUDE_DIRS}
)

ament_auto_add_library(raylib_viz SHARED
  src/node.cpp
  src/window.cpp
  src/viewer3d.cpp
  src/topic_plugin/rlights.cpp
)

target_link_libraries(raylib_viz
  ${raylib_LIBRARIES}
)

rclcpp_components_register_node(raylib_viz
  PLUGIN "RaylibViz"
  EXECUTABLE raylib_viz_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  shaders
)

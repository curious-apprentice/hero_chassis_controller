cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  controller_interface
  dynamic_reconfigure
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  realtime_tools
  roscpp
  tf
)

catkin_package(
  CATKIN_DEPENDS
    control_toolbox
    controller_interface
    dynamic_reconfigure
    geometry_msgs
    hardware_interface
    nav_msgs
    pluginlib
    realtime_tools
    roscpp
    tf
)

# Add the controller library, plugin export, and tests with their implementations.

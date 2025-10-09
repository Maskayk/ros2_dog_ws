cmake_minimum_required(VERSION 3.8)
project(dog_controllers)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dog_leg_controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  hardware_interface
)

pluginlib_export_plugin_description_file(controller_interface dog_controllers_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

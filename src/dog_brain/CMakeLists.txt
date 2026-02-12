cmake_minimum_required(VERSION 3.8)
project(dog_brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Зависимости ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# --- Библиотека (чистый C++, без ROS) ---
add_library(dog_brain_lib
  src/leg_kinematics.cpp
  src/gait_generator.cpp
  src/foot_trajectory.cpp
  src/body_controller.cpp
)
target_include_directories(dog_brain_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Executable (ROS-нода) ---
add_executable(trot_node src/trot_node.cpp)
target_link_libraries(trot_node dog_brain_lib)
ament_target_dependencies(trot_node rclcpp std_msgs geometry_msgs sensor_msgs)

# --- Установка ---
install(TARGETS dog_brain_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS trot_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()

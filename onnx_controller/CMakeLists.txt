cmake_minimum_required(VERSION 3.8)
project(onnx_controller)

# Use ament_cmake and related conventions only
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(onnx_interfaces REQUIRED)
find_package(onnx_inference REQUIRED)

# C++ standard and compiler options
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# MCAP Builder (imported manually via CMake FetchContent)
include(FetchContent)
FetchContent_Declare(
  mcap_builder
  GIT_REPOSITORY https://github.com/olympus-robotics/mcap_builder.git
  GIT_TAG main
)
FetchContent_MakeAvailable(mcap_builder)

# Add executable target
add_executable(controller
  src/controller.cpp
  src/motor_crc.cpp
  src/robot_interface.cpp
)

# Include paths
target_include_directories(controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/onnx_controller>
  $<INSTALL_INTERFACE:include/onnx_controller>
)

# Link external and local libraries
# Assuming onnx_actor and mcap are built targets or available via dependencies
target_link_libraries(controller
  onnx_inference::onnx_actor
  onnxruntime
  mcap
)

# Declare package dependencies
ament_target_dependencies(controller
  rclcpp
  unitree_go
  sensor_msgs
  onnx_interfaces
)

# Install target
install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# Optionally install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()